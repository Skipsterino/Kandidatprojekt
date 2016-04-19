/*
 * GANGSTILAR.c
 *
 * Created: 4/8/2016 3:03:52 PM
 *  Author: chrma018
 */ 

#include "gangstilar.h"

uint8_t n = 12; //index gångcykel startvärde m/2 ska göras snyggare
float last_h = 11;

double_float Adjust_L_And_th(float thin)
{
	uint8_t back= 0;
	uint8_t negative = 0;
	float pi=3.14;
	float l0 = 7;
	float thlimits[11] = {0,12,12,12,12,12,211,12,12,1/2}; // dessa är ju uppenbarligen fel ;)SISTA VAR PI/2
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

//ger max möjlig speed för inmatat theta.
int max_speed(float theta, int sgn_theta)
{
	int speed=0;
	//Gamla värden på thlimits = {0.57,0.48,0.38,0.3,0.2,0.11,0}
	float thlimits[7] = {0.53,0.44,0.34,0.26,0.16,0.07,0};//thetamax för olika speeds 0->5
	
	for(int i = 0; theta * sgn_theta <= thlimits[i] & i <= 6; i++)
	{
		speed = i;
	}
	return speed;
}
	
//Generar tripod gång
triple_float Tripod(float x, float s, float h, uint8_t m, uint8_t n)
{
	float lift = 3; //höjd som ben lyfts i sving
	float y = 0;
	float z = - last_h;
	
	if(n > 2 * m) //ser till att det blir cykliskt
	{
		n -= 2 * m;
	}

	if(n <= m)//stödfas
	{
		y = s/2 - s * n/m;
		z= - h;
	}
	else if( n <= m + 2)//benlyft i slut av stödfas, körs 2 cykler för garantera liftoff
	{
		y = - s/2; 
		z = lift - h;
	}
	else if( n <=  2 * m - 4)//för ben framåt mot startpos
	{
		z = lift - h;
		y = s/2;
	}
	else if( n <= 2 * m - 1)//sänker ben nära mark 
	{
		z = 1 - h; 
		y = s/2;
	}
	else if(n == 2 * m )//sänk ner ben i marken
	{
		y = s/2; 
		z = - h;
	}
	
	return create_triple_float(x,y,z);
}


int sgn = 0;
// speed = hastighet fram/bak [-6,6], th = sväng[-0.57, 0.57], h = höjd från mark(11)
//Undvik att ändra värde för h när robot står på marken (ska fixas)
//fixa mjuk övergång för h och l(gör globala) 
void Walk_Half_Cycle(int speed, float th, float h) 
{
if (speed > 4)
{
	speed = 4;
}
	
	//lägg in begr av max/min h, justera l efter h.
	
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
	float corner_pitch = 8; //förskjutning av arbetsområde i y-led för hörnben
	
	triple_float kar_p1;
	triple_float kar_p2;
	triple_float cyl1;
	triple_float cyl2;
	triple_float cyl3;
	triple_float cyl4;
	triple_float cyl5;
	triple_float cyl6;
	
	if(th * sgn_theta > 0.57) // minskar theta om för stor
	{
		th=0.57 * sgn_theta;
	}
	  
	if(sgn_speed * speed > m_speed ) //hastighetsbegränsing ): sänker speed om för hög, kan ändras så att speed 6 alltid = högsta realiserbara.
	{
		speed = m_speed * sgn_speed;
	}
	

	//justerar servospeed INTE KLART
	unsigned int speed_theta = 352 - 53 * (m_speed - sgn_speed*speed);//= max -Y*(speed- m_speed), alt tabell ;  //fast värde 0x0100 
	unsigned int speed_lift  = 336 - 10 * (m_speed - sgn_speed*speed);//fast värde 0x010 
	
	//justerar steglängd
	float s =  2.2 * speed; 
	
	//skickar hastighetsbegränsningar till servon.
	Send_Inner_P1_Velocity(speed_theta);
	Send_Middle_P1_Velocity(speed_lift);
	Send_Outer_P1_Velocity(speed_lift);
	Send_Inner_P2_Velocity(speed_theta);
	Send_Middle_P2_Velocity(speed_lift);
	Send_Outer_P2_Velocity(speed_lift);
	
		float h_step = (h - last_h)/m; //för stegning fixa bort eller lägg in i nedanstående lopp (gör till for)
	
	//gångloop, utför en halv gångfas och stannar när stödjande ben är i mitten av arbetsområde
	 while( walk_break || ( n != m/2 && n != 3 * m/2 ))
	{
		walk_break = 0;
		
		//stegvis höjdjustering
		last_h += h_step;
		
		kar_p1 = Tripod(l, s, last_h, m, n); //kart koord för par 1
		kar_p2 = Tripod(l, s, last_h, m, n + m); //kart koord för par 2
		
		if(th == 0) //behöver ej gå via cyl koord vid rak gång.
		{
			//par 1 får pos
			Send_Leg4_Kar(kar_p1.a, kar_p1.b, kar_p1.c); 
			Send_Leg1_Kar(kar_p1.a, kar_p1.b + corner_pitch, kar_p1.c);
			Send_Leg5_Kar(kar_p1.a, kar_p1.b - corner_pitch, kar_p1.c);
			
			//par 2 får pos
			Send_Leg3_Kar(kar_p2.a, kar_p2.b, kar_p2.c); 
			Send_Leg2_Kar(kar_p2.a, kar_p2.b + corner_pitch, kar_p2.c);
			Send_Leg6_Kar(kar_p2.a, kar_p2.b - corner_pitch, kar_p2.c);
		}
		else //om sväng
		{
			//koord för par 1 omvandlas till cyl
			triple_float cyl1 = Kar_To_Cyl_Leg1(kar_p1.a, kar_p1.b + corner_pitch, kar_p1.c);
			triple_float cyl4 = Kar_To_Cyl_Leg4(kar_p1.a, kar_p1.b, kar_p1.c);
			triple_float cyl5 = Kar_To_Cyl_Leg5(kar_p1.a, kar_p1.b - corner_pitch, kar_p1.c);
			
			//koord för par 2 omvandlas till cyl
			triple_float cyl3 = Kar_To_Cyl_Leg3(kar_p2.a, kar_p2.b, kar_p2.c);
			triple_float cyl2 = Kar_To_Cyl_Leg2(kar_p2.a, kar_p2.b + corner_pitch, kar_p2.c);
			triple_float cyl6 = Kar_To_Cyl_Leg6(kar_p2.a, kar_p2.b - corner_pitch, kar_p2.c);
			
			//if här ska tas bort, ändra betydelse/range för N i tripod.
			if(n <= m) //stödfas par 1, svingfas par 2
			{
				//Stegvis rotation på p1
				cyl1.b = cyl1.b +sgn_speed*(- th/2 + n * th/m);
				cyl4.b = cyl4.b +sgn_speed*(- th/2 + n * th/m);
				cyl5.b = cyl5.b +sgn_speed*(- th/2 + n * th/m);
				
				//startläge för P2
				cyl3.b = cyl3.b - sgn_speed*th/2;
				cyl2.b = cyl2.b - sgn_speed*th/2;
				cyl6.b = cyl6.b - sgn_speed*th/2;
			}
			else //swingfas par1, stödfas par 2
			{
				//rotation till startläge P1
				cyl1.b = cyl1.b -sgn_speed* th/2 ;
				cyl4.b = cyl4.b - sgn_speed* th/2 ;
				cyl5.b = cyl5.b - sgn_speed*th/2 ;
				
				//Stegvis rotation på p2
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
		
		_delay_ms(5); // =5 vore logiskt med olika delay för kart o cyl.
	}
	last_h = h; //nya h blir gamla
}


//test, endast rak gång
//Crab people, Crab people, tastes like crab, talks like people.
//gör ny copy från walk_half_cycle.
void Walk_Half_Crab_Cycle(int speed, float th, float h, float l)
{
	th=0;
	if(speed > 6 ) //hastighetsbegränsing
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
	unsigned int speed_theta = 0x0060 ;//+ 0x0010 * speed; //0x0020 lagom för speed 1
	unsigned int speed_lift = 0x0100 ;//+ 0x0030 * speed; // 0x0060 lagom för speed 1
	
	//justerar steglängd
	uint8_t s =  2.2 * speed; //var 1 + 2* speed innnan steg inom gräns nu för speed = 6?
	
	Send_Inner_P1_Velocity(speed_theta);
	Send_Middle_P1_Velocity(speed_lift);
	Send_Outer_P1_Velocity(speed_lift);
	Send_Inner_P2_Velocity(speed_theta);
	Send_Middle_P2_Velocity(speed_lift);
	Send_Outer_P2_Velocity(speed_theta);
	
	for (uint8_t i = 1; i <= m; i++)
	{
		kar_p1 = Tripod(l, s, h, m, n); //kart koord för par 1
		kar_p2 = Tripod(l, s, h, m, n + m); //kart koord för par 2
		
		if(th == 0) //behöver ej gå via cyl koord vid rak gång.
		{
			//par 1 får pos
			Send_Leg4_Kar(kar_p1.a - kar_p1.b, 0, kar_p1.c); 
			Send_Leg1_Kar(kar_p1.a + kar_p1.b, 0, kar_p1.c);
			Send_Leg5_Kar(kar_p1.a + kar_p1.b, 0, kar_p1.c);
			
			//par 2 får pos
			Send_Leg3_Kar(kar_p2.a + kar_p2.b, 0, kar_p2.c); 
			Send_Leg2_Kar(kar_p2.a - kar_p2.b, 0, kar_p2.c);
			Send_Leg6_Kar(kar_p2.a - kar_p2.b, 0, kar_p2.c);
		}
		else //om sväng
		{
			//koord för par 1 omvandlas till cyl
			triple_float cyl1 = Kar_To_Cyl_Leg1(kar_p1.a, kar_p1.b + corner_pitch, kar_p1.c);
			triple_float cyl4 = Kar_To_Cyl_Leg4(kar_p1.a, kar_p1.b, kar_p1.c);
			triple_float cyl5 = Kar_To_Cyl_Leg5(kar_p1.a, kar_p1.b - corner_pitch, kar_p1.c);
			
			//koord för par 2 omvandlas till cyl
			triple_float cyl3 = Kar_To_Cyl_Leg3(kar_p2.a, kar_p2.b, kar_p2.c);
			triple_float cyl2 = Kar_To_Cyl_Leg2(kar_p2.a, kar_p2.b + corner_pitch, kar_p2.c);
			triple_float cyl6 = Kar_To_Cyl_Leg6(kar_p2.a, kar_p2.b - corner_pitch, kar_p2.c);
			
			//if här ska tas bort, ändra betydelse/range för N i tripod.
			if(n <= m) //stödfas par 1, svingfas par 2
			{
				//Stegvis rotation på p1
				cyl1.b = cyl1.b -th/2 + th/m*n;
				cyl4.b = cyl4.b -th/2 + th/m*n;
				cyl5.b = cyl5.b -th/2 + th/m*n;
				
				//startläge för P2
				cyl3.b = cyl3.b - th/2;
				cyl2.b = cyl2.b - th/2;
				cyl6.b = cyl6.b - th/2;
			}
			else //swingfas par1, stödfas par 2
			{
				//rotation till startläge P1
				cyl1.b = cyl1.b - th/2 ;
				cyl4.b = cyl4.b - th/2 ;
				cyl5.b = cyl5.b - th/2 ;
				
				//Stegvis rotation på p2
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
		
		_delay_ms(5); //dumpa? vore logiskt med olika delay för kart o cyl.
	}
}




