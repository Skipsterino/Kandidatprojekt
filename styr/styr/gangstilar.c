/*
 * GANGSTILAR.c
 *
 * Created: 4/8/2016 3:03:52 PM
 *  Author: jenma 421 (Gimmie creed) [chrma018]
 */ 

#include "gangstilar.h"

uint8_t n = 12; //index gångcykel startvärde m/2 ska göras snyggare
float last_height = 11;

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
	float thlimits[7] = {0.57,0.48,0.38,0.3,0.2,0.11,0};//thetamax för olika speeds 0->5
	
	for(speed = 0; (theta * sgn_theta <= thlimits[speed]) && (speed <= 6); speed++) // rpova parantes och &&
	{
	}
	return speed;
}
	
//Generar tripod gång
triple_float Tripod(float x, float stroke, float height, uint8_t m, uint8_t n)
{
	float lift = 3; //höjd som ben lyfts i sving
	float y = 0;
	float z = - height;
	
	if(n > 2 * m) //ser till att det blir cykliskt
	{
		n -= 2 * m;
	}

	if(n <= m)//stödfas
	{
		y = stroke/2 - stroke * n/m;
		z= - height;
	}
	else if( n <= m + 2)//benlyft i slut av stödfas, körs 2 cykler för garantera liftoff
	{
		y = - stroke/2; 
		z = lift - height;
	}
	else if( n < 3 * m/2) // pausar ben centrerat i svingfas 
	{
		y = 0;
		z = lift - height;
	}
	else if( n <=  2 * m - 4)//för ben framåt mot startpos
	{
		y = stroke/2;
		z = lift - height;
	}
	else if( n <= 2 * m - 1)//sänker ben nära mark 
	{
		y = stroke/2;
		z = 1 - height; 
	}
	else if(n == 2 * m )//sänk ner ben i marken
	{
		y = stroke/2; 
		z = - height;
	}
	return create_triple_float(x,y,z);
}

// speed = hastighet fram/bak [-6,6], th = sväng[-0.57, 0.57], h = höjd från mark(11) HÖJDJUSTERING UR DRIFT
//fixa mjuk övergång för h och l(gör globala) 
void Walk_Half_Cycle(int speed, float th, float height) 
{
	if( speed == 0  && th ==0 && (height - last_height) == 0) // gör inget
	{
		return;
	}
	
if(height > 16)
{
	height = 16;
}
if(height < 6)
{
	height = 6;
}

	height = 11;
	float l = 13;
	int sgn_speed = (speed >= 0) - (speed < 0) ;
	int sgn_theta = (th >= 0) - (th < 0) ;
	int m_speed = max_speed(th, sgn_theta); //m_speed alltid possitiv
	uint8_t m = 24; //delsekvenser per halv cykel (per stödfas)
	uint8_t walk_break = 1;
	float corner_pitch = 8; //förskjutning av arbetsområde i y-led för hörnben
	float height_step = (height - last_height)/m; //delar upp höjdändring i steg.
	triple_float kar_p1;
	triple_float kar_p2;
	triple_float cyl1;
	triple_float cyl2;
	triple_float cyl3;
	triple_float cyl4;
	triple_float cyl5;
	triple_float cyl6;
	
	if(speed * sgn_speed > 6)
	{
		speed = 6 * sgn_speed;
	}
	//test, begränsar maxvinkel efter speed ist för omvänt
	float thlimits[7] = {0.56,0.47,0.37,0.3,0.2,0.11,0};
	float th_max = 0;	
	if( th * sgn_theta > (th_max =thlimits[speed * sgn_speed]))
	{
		th = sgn_theta * th_max;
	}
	
	/*
	if(th * sgn_theta > 0.57) // minskar theta om för stor
	{
		th=0.57 * sgn_theta;
	}
	  
	if(sgn_speed * speed > m_speed ) //hastighetsbegränsing ): sänker speed om för hög, kan ändras så att speed 6 alltid = högsta realiserbara.
	{
		speed = m_speed * sgn_speed;
	}
	*/

	//justerar servospeed efter theta/speed INTE KLART
	unsigned int speed_drive = 352- 53 * (m_speed - sgn_speed * speed);//fast värde 0x0100 
	unsigned int speed_lift  = 336 - 10 * (m_speed - sgn_speed * speed);//fast värde 0x0100 
	
	//justerar steglängd
	float stroke =  2.2 * speed; 
	
	//skickar hastighetsbegränsningar till servon.
	Send_Inner_P1_Velocity(speed_drive);
	Send_Middle_P1_Velocity(speed_lift);
	Send_Outer_P1_Velocity(speed_lift);
	Send_Inner_P2_Velocity(speed_drive);
	Send_Middle_P2_Velocity(speed_lift);
	Send_Outer_P2_Velocity(speed_lift);
	
	
	if( speed == 0  && th ==0 ) //justera höjd utan att gå
	{
		for(uint8_t i = 0; i <= m; i++)
		{
			last_height += height_step;
				
			kar_p1 = Tripod(l, stroke, last_height, m, n); //kart koord för par 1
			kar_p2 = Tripod(l, stroke, last_height, m, n + m); //kart koord för par 2
				
			//par 1 får pos
			Send_Leg4_Kar(kar_p1.a, kar_p1.b, kar_p1.c);
			Send_Leg1_Kar(kar_p1.a, kar_p1.b + corner_pitch, kar_p1.c);
			Send_Leg5_Kar(kar_p1.a, kar_p1.b - corner_pitch, kar_p1.c);
				
			//par 2 får pos
			Send_Leg3_Kar(kar_p2.a, kar_p2.b, kar_p2.c);
			Send_Leg2_Kar(kar_p2.a, kar_p2.b + corner_pitch, kar_p2.c);
			Send_Leg6_Kar(kar_p2.a, kar_p2.b - corner_pitch, kar_p2.c);
				
			_delay_ms(5); // =5 delay för kart
		}
		return;
	}
		
	//gångloop, utför en halv gångfas och stannar när stödjande ben är i mitten av arbetsområde
	 while( walk_break || ( n != m/2 && n != 3 * m/2 ))
	{
		walk_break = 0;
		
		//stegvis höjdjustering
		last_height += height_step;
		
		kar_p1 = Tripod(l, stroke, last_height, m, n); //kart koord för par 1
		kar_p2 = Tripod(l, stroke, last_height, m, n + m); //kart koord för par 2
		
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
			_delay_ms(5); // =5 delay för kart
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
				cyl1.b = cyl1.b + sgn_speed * (- th/2 + n * th/m);
				cyl4.b = cyl4.b + sgn_speed * (- th/2 + n * th/m);
				cyl5.b = cyl5.b + sgn_speed * (- th/2 + n * th/m);
				
				//mot startläge  för P2
				cyl3.b = cyl3.b + sgn_speed * ( th/2 - n * th/m);
				cyl2.b = cyl2.b + sgn_speed * ( th/2 - n * th/m);
				cyl6.b = cyl6.b + sgn_speed * ( th/2 - n * th/m);
			}
			else //swingfas par1, stödfas par 2
			{
				//rotation till mot startläge P1
				cyl1.b = cyl1.b + sgn_speed * ( th/2 - (n - m) * th/m);
				cyl4.b = cyl4.b + sgn_speed * ( th/2 - (n - m) * th/m);
				cyl5.b = cyl5.b + sgn_speed * ( th/2 - (n - m) * th/m);
				
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
			_delay_ms(3); // =3 Delay för cyl
		}
		
		if(n < 2 * m)
		{
		n += 1; //stegar upp index
		}
		else
		{
			n = 1; //nollar index
		}	
	}
	last_height = height; //ny höjd blir gamal
}

//test, endast rak gång
//Crab people, Crab people, tastes like crab, talks like people.
//gör ny copy från walk_half_cycle.
void Walk_Half_Crab_Cycle(int speed, float height) 
{
		if( speed == 0  && (height - last_height) == 0) // gör inget
		{
			return;
		}
		
	//lägg in begr av max/min h, justera l efter h.
	
if(height > 16)
{
	height = 16;
}
if(height < 6)
{
	height = 6;
}
	float l = 13;
	int sgn_speed = (speed >= 0) - (speed < 0) ;
	int m_speed = 6;
	uint8_t m = 24; //delsekvenser per halv cykel
	uint8_t walk_break = 1;
	float corner_pitch = 8; //förskjutning av arbetsområde i y-led för hörnben
	float height_step = (height - last_height)/m; //delar upp höjdändring i steg.
	
	triple_float kar_p1;
	triple_float kar_p2;
	
	if(sgn_speed * speed > m_speed ) //hastighetsbegränsing ): sänker speed om för hög, kan ändras så att speed 6 alltid = högsta realiserbara.
	{
		speed = m_speed * sgn_speed;
	}
	
	//justerar servospeed efter theta/speed INTE KLART
	unsigned int speed_drive = 352 - 53 * (m_speed - sgn_speed * speed); //fast värde 0x0100 
	unsigned int speed_lift  = 336 - 10 * (m_speed - sgn_speed * speed);//fast värde 0x0100 
	
	//justerar steglängd
	float stroke =  2.2 * speed; 
	
	//skickar hastighetsbegränsningar till servon.
	Send_Inner_P1_Velocity(speed_drive);
	Send_Middle_P1_Velocity(speed_lift);
	Send_Outer_P1_Velocity(speed_lift);
	Send_Inner_P2_Velocity(speed_drive);
	Send_Middle_P2_Velocity(speed_lift);
	Send_Outer_P2_Velocity(speed_lift);
	
	//gångloop, utför en halv gångfas och stannar när stödjande ben är i mitten av arbetsområde
	 while( walk_break || ( n != m/2 && n != 3 * m/2 ))
	{
		walk_break = 0;
		
		//stegvis höjdjustering
		last_height += height_step;
		
		kar_p1 = Tripod(l, stroke, last_height, m, n); //kart koord för par 1
		kar_p2 = Tripod(l, stroke, last_height, m, n + m); //kart koord för par 2
		
		//par 1 får pos
		Send_Leg4_Kar(kar_p1.a - kar_p1.b, 0, kar_p1.c); 
		Send_Leg1_Kar(kar_p1.a + kar_p1.b, corner_pitch, kar_p1.c);
		Send_Leg5_Kar(kar_p1.a + kar_p1.b, - corner_pitch, kar_p1.c);
			
		//par 2 får pos
		Send_Leg3_Kar(kar_p2.a + kar_p2.b, 0, kar_p2.c); 
		Send_Leg2_Kar(kar_p2.a - kar_p2.b, corner_pitch, kar_p2.c);
		Send_Leg6_Kar(kar_p2.a - kar_p2.b, - corner_pitch, kar_p2.c);
			
		if(n < 2 * m)
		{
		n += 1; //stegar upp index
		}
		else
		{
			n = 1; //nollar index
		}
		
		_delay_ms(5); // =5 
	}
	
	last_height = height; //ny höjd blir gammal
}
/*tankar xjustering
kör liknande som height med new och old, andra värde i steg när benpar är i svingas, båda
par är ju det under en loop :)
*/


// speed = hastighet fram/bak [-6,6], th = sväng[-0.57, 0.57], h = höjd från mark(11)
// långsam version av Walk_Half_Hycle, utför samma rörelse, men långsamt.

//OBS höjdjustering kan ge felaktigt beteende! justera parsamt.
void Walk_Half_Safe_Cycle(int speed, float th, float height) 
{
	if( speed == 0  && th ==0 && (height - last_height) == 0) // gör inget
		{
			return;
		}
	//justera in begr av max/min h, justera l efter h.
	
if(height > 14)
{
	height = 14;
}
if(height < 8)
{
	height = 8;
}

	float l = 13;
	int sgn_speed = (speed >= 0) - (speed < 0) ;
	int sgn_theta = (th >= 0) - (th < 0) ;
	int m_speed = max_speed(th, sgn_theta); //m_speed alltid possitiv
	uint8_t m = 24; //delsekvenser per halv cykel (per stödfas)
	uint8_t walk_break = 1;
	float corner_pitch = 8; //förskjutning av arbetsområde i y-led för hörnben
	float height_step = (height - last_height)/m; //delar upp höjdändring i steg.
	
		if(speed * sgn_speed  > 6)
	{
		speed = 6 * sgn_speed;
	}
	
	triple_float kar_p1;
	triple_float kar_p2;
	triple_float cyl1;
	triple_float cyl2;
	triple_float cyl3;
	triple_float cyl4;
	triple_float cyl5;
	triple_float cyl6;
	
	//test, begränsar maxvinkel efter speed ist för omvänt
	float thlimits[7] = {0.57,0.48,0.38,0.3,0.2,0.11,0};
	float th_max = 0;	
	if( th * sgn_theta > (th_max =thlimits[speed * sgn_speed]))
	{
		th = sgn_theta * th_max;
	}
	
	/*
	if(th * sgn_theta > 0.57) // minskar theta om för stor
	{
		th=0.57 * sgn_theta;
	}
	  
	if(sgn_speed * speed > m_speed ) //hastighetsbegränsing ): sänker speed om för hög, kan ändras så att speed 6 alltid = högsta realiserbara.
	{
		speed = m_speed * sgn_speed;
	}
	*/

	//justerar servospeed efter theta/speed INTE KLART
	unsigned int speed_drive = 0x0100;//352- 53 * (m_speed - sgn_speed * speed);//fast värde 0x0100 
	unsigned int speed_lift  = 0x0100;//336 - 10 * (m_speed - sgn_speed * speed);//fast värde 0x0100 
	
	//justerar steglängd
	float stroke =  2.2 * speed; 
	
	//skickar hastighetsbegränsningar till servon.
	Send_Inner_P1_Velocity(speed_drive);
	Send_Middle_P1_Velocity(speed_lift);
	Send_Outer_P1_Velocity(speed_lift);
	Send_Inner_P2_Velocity(speed_drive);
	Send_Middle_P2_Velocity(speed_lift);
	Send_Outer_P2_Velocity(speed_lift);
	
	
	 if( speed == 0  && th ==0 ) //justera höjd utan att gå	
	 {
		for(uint8_t i = 0; i <= m; i++)
		{
			last_height += height_step;
					
			kar_p1 = Tripod(l, stroke, last_height, m, n); //kart koord för par 1
			kar_p2 = Tripod(l, stroke, last_height, m, n + m); //kart koord för par 2
					
			//par 1 får pos
			Send_Leg4_Kar(kar_p1.a, kar_p1.b, kar_p1.c);
			Send_Leg1_Kar(kar_p1.a, kar_p1.b + corner_pitch, kar_p1.c);
			Send_Leg5_Kar(kar_p1.a, kar_p1.b - corner_pitch, kar_p1.c);
					
			//par 2 får pos
			Send_Leg3_Kar(kar_p2.a, kar_p2.b, kar_p2.c);
			Send_Leg2_Kar(kar_p2.a, kar_p2.b + corner_pitch, kar_p2.c);
			Send_Leg6_Kar(kar_p2.a, kar_p2.b - corner_pitch, kar_p2.c);
			_delay_ms(50); // =5 delay för kart
		}
		return;
	}
			
	//gångloop, utför en halv gångfas och stannar när stödjande ben är i mitten av arbetsområde
	 while( walk_break || ( n != m/2 && n != 3 * m/2 ))
	{
		walk_break = 0;
		
		//stegvis höjdjustering
		last_height += height_step;
		
		kar_p1 = Tripod(l, stroke, last_height, m, n); //kart koord för par 1
		kar_p2 = Tripod(l, stroke, last_height, m, n + m); //kart koord för par 2
		
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
			_delay_ms(50); // =5 vore logiskt med olika delay för kart o cyl.
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
				cyl1.b = cyl1.b + sgn_speed * (- th/2 + n * th/m);
				cyl4.b = cyl4.b + sgn_speed * (- th/2 + n * th/m);
				cyl5.b = cyl5.b + sgn_speed * (- th/2 + n * th/m);
				
				//startläge för P2
				cyl3.b = cyl3.b - sgn_speed * th/2;
				cyl2.b = cyl2.b - sgn_speed * th/2;
				cyl6.b = cyl6.b - sgn_speed * th/2;
			}
			else //swingfas par1, stödfas par 2
			{
				//rotation till startläge P1
				cyl1.b = cyl1.b - sgn_speed * th/2 ;
				cyl4.b = cyl4.b - sgn_speed * th/2 ;
				cyl5.b = cyl5.b - sgn_speed * th/2 ;
				
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
			_delay_ms(48); // =5 vore logiskt med olika delay för kart o cyl.
		}
		
		if(n < 2 * m)
		{
		n += 1; //stegar upp index
		}
		else
		{
			n = 1; //nollar index
		}	
	}
	last_height = height; //ny höjd blir gamal
}
