/*
 * GANGSTILAR.c
 *
 * Created: 4/8/2016 3:03:52 PM
 *  Author: chrma018
 */ 

#include "gangstilar.h"

uint8_t n = 12; //index g�ngcykel startv�rde m/2 ska g�ras snyggare
float last_h = 11;

double_float Adjust_L_And_th(float thin)
{
	uint8_t back= 0;
	uint8_t negative = 0;
	float pi=3.14;
	float l0 = 7;
	float thlimits[11] = {0,12,12,12,12,12,211,12,12,1/2}; // dessa �r ju uppenbarligen fel ;)SISTA VAR PI/2
	float R=l0/thin;
	float lout=10101;
	float thout=10101;
	
	if(thin<-pi/2)
	{
		back = 1;
		negative = 1;
		thin = pi + thin;
	}
	else if(thin<0)
	{
		negative=1;
		thin = -thin;
	}
	else if(thin>pi/2)
	{
		back= 1;
		thin=pi-thin;
	}
	
	for(int i=0; i<10; i++)
	{
		if(thin<=thlimits[i])
		{
			lout = l0*(1-1/10*i);
			thout = lout/R;
		}
	}
	if((lout = 10101))
	{
		lout= 0;
		thin = thlimits[10];
	}
	
	if((negative=1))
	{
		thout = -thout;
	}
	else if((back=1))
	{
		lout=-lout;
	}
	
	return create_double_float(lout,thout);
}

//ger max m�jlig speed f�r inmatat theta.
int max_speed(float theta, int sgn_theta)
{
	int speed=0;
	//Gamla v�rden p� thlimits = {0.57,0.48,0.38,0.3,0.2,0.11,0}
	float thlimits[7] = {0.53,0.44,0.34,0.26,0.16,0.07,0};//thetamax f�r olika speeds 0->5
	
	for(int i = 0; theta * sgn_theta <= thlimits[i] & i <= 6; i++)
	{
		speed = i;
	}
	return speed;
}
	
//Generar tripod g�ng
triple_float Tripod(float x, float s, float h, uint8_t m, uint8_t n)
{
	float lift = 3; //h�jd som ben lyfts i sving
	float y = 0;
	float z = - last_h;
	
	if(n > 2 * m) //ser till att det blir cykliskt
	{
		n -= 2 * m;
	}

	if(n <= m)//st�dfas
	{
		y = s/2 - s * n/m;
		z= - h;
	}
	else if( n <= m + 2)//benlyft i slut av st�dfas, k�rs 2 cykler f�r garantera liftoff
	{
		y = - s/2; 
		z = lift - h;
	}
	else if( n <=  2 * m - 4)//f�r ben fram�t mot startpos
	{
		z = lift - h;
		y = s/2;
	}
	else if( n <= 2 * m - 1)//s�nker ben n�ra mark 
	{
		z = 1 - h; 
		y = s/2;
	}
	else if(n == 2 * m )//s�nk ner ben i marken
	{
		y = s/2; 
		z = - h;
	}
	
	return create_triple_float(x,y,z);
}


int sgn = 0;
// speed = hastighet fram/bak [-6,6], th = sv�ng[-0.57, 0.57], h = h�jd fr�n mark(11)
//Undvik att �ndra v�rde f�r h n�r robot st�r p� marken (ska fixas)
//fixa mjuk �verg�ng f�r h och l(g�r globala) 
void Walk_Half_Cycle(int speed, float th, float h) 
{
if (speed > 4)
{
	speed = 4;
}
	
	//l�gg in begr av max/min h, justera l efter h.
	
if(h > 16)
{
	h=16;
}
if(h < 6)
{
	h=6;
}
	float l = 13;
	int sgn_speed = (speed >= 0) - (speed < 0) ;
	int sgn_theta = (th >= 0) - (th < 0) ;
	int m_speed = max_speed(th, sgn_theta); //m_speed alltid possitiv
	uint8_t m = 24; //delsekvenser per halv cykel
	uint8_t walk_break = 1;
	float corner_pitch = 8; //f�rskjutning av arbetsomr�de i y-led f�r h�rnben
	
	triple_float kar_p1;
	triple_float kar_p2;
	triple_float cyl1;
	triple_float cyl2;
	triple_float cyl3;
	triple_float cyl4;
	triple_float cyl5;
	triple_float cyl6;
	
	if(th * sgn_theta > 0.57) // minskar theta om f�r stor
	{
		th=0.57 * sgn_theta;
	}
	  
	if(sgn_speed * speed > m_speed ) //hastighetsbegr�nsing ): s�nker speed om f�r h�g, kan �ndras s� att speed 6 alltid = h�gsta realiserbara.
	{
		speed = m_speed * sgn_speed;
	}
	

	//justerar servospeed INTE KLART
	unsigned int speed_theta = 352 - 53 * (m_speed - sgn_speed*speed);//= max -Y*(speed- m_speed), alt tabell ;  //fast v�rde 0x0100 
	unsigned int speed_lift  = 336 - 10 * (m_speed - sgn_speed*speed);//fast v�rde 0x010 
	
	//justerar stegl�ngd
	float s =  2.2 * speed; 
	
	//skickar hastighetsbegr�nsningar till servon.
	Send_Inner_P1_Velocity(speed_theta);
	Send_Middle_P1_Velocity(speed_lift);
	Send_Outer_P1_Velocity(speed_lift);
	Send_Inner_P2_Velocity(speed_theta);
	Send_Middle_P2_Velocity(speed_lift);
	Send_Outer_P2_Velocity(speed_lift);
	
		float h_step = (h - last_h)/m; //f�r stegning fixa bort eller l�gg in i nedanst�ende lopp (g�r till for)
	
	//g�ngloop, utf�r en halv g�ngfas och stannar n�r st�djande ben �r i mitten av arbetsomr�de
	 while( walk_break || ( n != m/2 && n != 3 * m/2 ))
	{
		walk_break = 0;
		
		//stegvis h�jdjustering
		last_h += h_step;
		
		kar_p1 = Tripod(l, s, last_h, m, n); //kart koord f�r par 1
		kar_p2 = Tripod(l, s, last_h, m, n + m); //kart koord f�r par 2
		
		if(th == 0) //beh�ver ej g� via cyl koord vid rak g�ng.
		{
			//par 1 f�r pos
			Send_Leg4_Kar(kar_p1.a, kar_p1.b, kar_p1.c); 
			Send_Leg1_Kar(kar_p1.a, kar_p1.b + corner_pitch, kar_p1.c);
			Send_Leg5_Kar(kar_p1.a, kar_p1.b - corner_pitch, kar_p1.c);
			
			//par 2 f�r pos
			Send_Leg3_Kar(kar_p2.a, kar_p2.b, kar_p2.c); 
			Send_Leg2_Kar(kar_p2.a, kar_p2.b + corner_pitch, kar_p2.c);
			Send_Leg6_Kar(kar_p2.a, kar_p2.b - corner_pitch, kar_p2.c);
		}
		else //om sv�ng
		{
			//koord f�r par 1 omvandlas till cyl
			triple_float cyl1 = Kar_To_Cyl_Leg1(kar_p1.a, kar_p1.b + corner_pitch, kar_p1.c);
			triple_float cyl4 = Kar_To_Cyl_Leg4(kar_p1.a, kar_p1.b, kar_p1.c);
			triple_float cyl5 = Kar_To_Cyl_Leg5(kar_p1.a, kar_p1.b - corner_pitch, kar_p1.c);
			
			//koord f�r par 2 omvandlas till cyl
			triple_float cyl3 = Kar_To_Cyl_Leg3(kar_p2.a, kar_p2.b, kar_p2.c);
			triple_float cyl2 = Kar_To_Cyl_Leg2(kar_p2.a, kar_p2.b + corner_pitch, kar_p2.c);
			triple_float cyl6 = Kar_To_Cyl_Leg6(kar_p2.a, kar_p2.b - corner_pitch, kar_p2.c);
			
			//if h�r ska tas bort, �ndra betydelse/range f�r N i tripod.
			if(n <= m) //st�dfas par 1, svingfas par 2
			{
				//Stegvis rotation p� p1
				cyl1.b = cyl1.b +sgn_speed*(- th/2 + n * th/m);
				cyl4.b = cyl4.b +sgn_speed*(- th/2 + n * th/m);
				cyl5.b = cyl5.b +sgn_speed*(- th/2 + n * th/m);
				
				//startl�ge f�r P2
				cyl3.b = cyl3.b - sgn_speed*th/2;
				cyl2.b = cyl2.b - sgn_speed*th/2;
				cyl6.b = cyl6.b - sgn_speed*th/2;
			}
			else //swingfas par1, st�dfas par 2
			{
				//rotation till startl�ge P1
				cyl1.b = cyl1.b -sgn_speed* th/2 ;
				cyl4.b = cyl4.b - sgn_speed* th/2 ;
				cyl5.b = cyl5.b - sgn_speed*th/2 ;
				
				//Stegvis rotation p� p2
				cyl3.b = cyl3.b + sgn_speed * (- 3 * th/2 + n * th/m);  
				cyl2.b = cyl2.b + sgn_speed * (- 3 * th/2 + n * th/m);
				cyl6.b = cyl6.b + sgn_speed * (- 3 * th/2 + n * th/m);
			}
			//skicka ut cyl koord
			Send_Leg4_Cyl(cyl4.a, cyl4.b, cyl4.c);
			Send_Leg1_Cyl(cyl1.a, cyl1.b, cyl1.c);
			Send_Leg5_Cyl(cyl5.a, cyl5.b, cyl5.c);
			Send_Leg3_Cyl(cyl3.a, cyl3.b, cyl3.c);
			Send_Leg2_Cyl(cyl2.a, cyl2.b, cyl2.c);
			Send_Leg6_Cyl(cyl6.a, cyl6.b, cyl6.c);
		}
		
		if(n < 2 * m)
		{
		n += 1; //stegar upp index
		}
		else
		{
			n = 1; //nollar index
		}
		
		_delay_ms(5); // =5 vore logiskt med olika delay f�r kart o cyl.
	}
	last_h = h; //nya h blir gamla
}


//test, endast rak g�ng
//Crab people, Crab people, tastes like crab, talks like people.
//g�r ny copy fr�n walk_half_cycle.
void Walk_Half_Crab_Cycle(int speed, float th, float h, float l)
{
	th=0;
	if(speed > 6 ) //hastighetsbegr�nsing
	{
		speed = 6;
	}
	
	uint8_t m = 24; //delsekvenser per halv cykel, minska vid "overdrive"
	float corner_pitch = 8;
	
	triple_float kar_p1;
	triple_float kar_p2;
	
	triple_float cyl1;
	triple_float cyl2;
	triple_float cyl3;
	triple_float cyl4;
	triple_float cyl5;
	triple_float cyl6;
	
	//justerar servospeed 
	unsigned int speed_theta = 0x0060 ;//+ 0x0010 * speed; //0x0020 lagom f�r speed 1
	unsigned int speed_lift = 0x0100 ;//+ 0x0030 * speed; // 0x0060 lagom f�r speed 1
	
	//justerar stegl�ngd
	uint8_t s =  2.2 * speed; //var 1 + 2* speed innnan steg inom gr�ns nu f�r speed = 6?
	
	Send_Inner_P1_Velocity(speed_theta);
	Send_Middle_P1_Velocity(speed_lift);
	Send_Outer_P1_Velocity(speed_lift);
	Send_Inner_P2_Velocity(speed_theta);
	Send_Middle_P2_Velocity(speed_lift);
	Send_Outer_P2_Velocity(speed_theta);
	
	for (uint8_t i = 1; i <= m; i++)
	{
		kar_p1 = Tripod(l, s, h, m, n); //kart koord f�r par 1
		kar_p2 = Tripod(l, s, h, m, n + m); //kart koord f�r par 2
		
		if(th == 0) //beh�ver ej g� via cyl koord vid rak g�ng.
		{
			//par 1 f�r pos
			Send_Leg4_Kar(kar_p1.a - kar_p1.b, 0, kar_p1.c); 
			Send_Leg1_Kar(kar_p1.a + kar_p1.b, 0, kar_p1.c);
			Send_Leg5_Kar(kar_p1.a + kar_p1.b, 0, kar_p1.c);
			
			//par 2 f�r pos
			Send_Leg3_Kar(kar_p2.a + kar_p2.b, 0, kar_p2.c); 
			Send_Leg2_Kar(kar_p2.a - kar_p2.b, 0, kar_p2.c);
			Send_Leg6_Kar(kar_p2.a - kar_p2.b, 0, kar_p2.c);
		}
		else //om sv�ng
		{
			//koord f�r par 1 omvandlas till cyl
			triple_float cyl1 = Kar_To_Cyl_Leg1(kar_p1.a, kar_p1.b + corner_pitch, kar_p1.c);
			triple_float cyl4 = Kar_To_Cyl_Leg4(kar_p1.a, kar_p1.b, kar_p1.c);
			triple_float cyl5 = Kar_To_Cyl_Leg5(kar_p1.a, kar_p1.b - corner_pitch, kar_p1.c);
			
			//koord f�r par 2 omvandlas till cyl
			triple_float cyl3 = Kar_To_Cyl_Leg3(kar_p2.a, kar_p2.b, kar_p2.c);
			triple_float cyl2 = Kar_To_Cyl_Leg2(kar_p2.a, kar_p2.b + corner_pitch, kar_p2.c);
			triple_float cyl6 = Kar_To_Cyl_Leg6(kar_p2.a, kar_p2.b - corner_pitch, kar_p2.c);
			
			//if h�r ska tas bort, �ndra betydelse/range f�r N i tripod.
			if(n <= m) //st�dfas par 1, svingfas par 2
			{
				//Stegvis rotation p� p1
				cyl1.b = cyl1.b -th/2 + th/m*n;
				cyl4.b = cyl4.b -th/2 + th/m*n;
				cyl5.b = cyl5.b -th/2 + th/m*n;
				
				//startl�ge f�r P2
				cyl3.b = cyl3.b - th/2;
				cyl2.b = cyl2.b - th/2;
				cyl6.b = cyl6.b - th/2;
			}
			else //swingfas par1, st�dfas par 2
			{
				//rotation till startl�ge P1
				cyl1.b = cyl1.b - th/2 ;
				cyl4.b = cyl4.b - th/2 ;
				cyl5.b = cyl5.b - th/2 ;
				
				//Stegvis rotation p� p2
				cyl3.b = cyl3.b - 3 * th/2 + n * th/m;  
				cyl2.b = cyl2.b - 3 * th/2 + n * th/m;
				cyl6.b = cyl6.b - 3 * th/2 + n * th/m;
			}
			//skicka ut cyl koord
			Send_Leg4_Cyl(cyl4.a, cyl4.b, cyl4.c);
			Send_Leg1_Cyl(cyl1.a, cyl1.b, cyl1.c);
			Send_Leg5_Cyl(cyl5.a, cyl5.b, cyl5.c);
			Send_Leg3_Cyl(cyl3.a, cyl3.b, cyl3.c);
			Send_Leg2_Cyl(cyl2.a, cyl2.b, cyl2.c);
			Send_Leg6_Cyl(cyl6.a, cyl6.b, cyl6.c);
		}
		
		if(n < 2 * m)
		{
		n += 1; //stegar upp index
		}
		else
		{
			n = 1; //nollar index
		}
		
		_delay_ms(5); //dumpa? vore logiskt med olika delay f�r kart o cyl.
	}
}




