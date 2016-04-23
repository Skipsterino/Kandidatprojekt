/*
 * GANGSTILAR.c
 *
 * Created: 4/8/2016 3:03:52 PM
 *  Author: jenma 421 (Gimmie creed) [chrma018]
 */ 

#include "gangstilar.h"
uint8_t m = 24; //delsekvenser per halv cykel (per stödfas)
uint8_t n = 12; //index gångcykel startvärde m/2 ska göras snyggare
float last_height = 11;
float last_speed = 0;
float last_theta = 0;
float theta_max = 0;


//LP-filtrerar input för att undvika våldsamheter Anropa limit_theta o limit_speed här, döp till LP_And_Limit_output, o begränsa höjd. Sätter även theta_max
float LP_Filter_And_Limit_Input(float speed, int sgn_speed, float theta, int sgn_theta, float height)
{
	speed = Limit_Speed(speed, sgn_speed);
	double_float thetas = Limit_Theta(speed, sgn_speed, theta, sgn_theta);
	height = Limit_Height(height);
	theta = thetas.a;
	theta_max = thetas.b;
	
	float speed_diff = speed - last_speed;
	float theta_diff = theta - last_theta;
	float height_diff = height - last_height;
	
	if(speed_diff > 3)
	{
		speed = last_speed + 3;
	}
	else if(speed_diff < -3)
	{
		speed = last_speed - 3;
	}
	
	if(theta_diff > 0.29)
	{
		theta = last_theta + 0.29;
	}
	else if(theta_diff < -0.29)
	{
		theta = last_theta - 0.29;
	}
	
	if(height_diff > 2)
	{
		height = last_height + 2;
	}
	else if(height_diff < -2)
	{
		height = last_height -2;
	}
	
	last_speed = speed;
	last_theta = theta;
	return height;
}
			

//ger max möjlig speed för inmatat theta. Används ej
int max_speed(float theta, int sgn_theta)
{
	int speed=0;
	float thlimits[7] = {0.57,0.47,0.37,0.3,0.2,0.11,0};//thetamax för olika speeds 0->5//
	
	for(speed = 0; (theta * sgn_theta <= thlimits[speed]) && (speed <= 6); speed++) // rpova parantes och &&
	{
	}
	return speed;
}

//Justerar servospeed efter förhåladne mellan theta o thetamax
void Adjust_Servo_Speed(float theta, int sgn_theta)
{                                                                                    
	//justerar servospeed ÄNDRA SKALFAKTOR !!
	unsigned int speed_drive =  250 + 500 * (sgn_theta * theta - theta_max);// 330 250
	unsigned int speed_lift  =  320 + 430 * (sgn_theta * theta - theta_max);//300
 
	//skickar hastighetsbegränsningar till servon.
	Send_Inner_P1_Velocity(speed_drive);
	Send_Middle_P1_Velocity(speed_lift);
	Send_Outer_P1_Velocity(speed_lift);
	Send_Inner_P2_Velocity(speed_drive);
	Send_Middle_P2_Velocity(speed_lift);
	Send_Outer_P2_Velocity(speed_lift);
}	

//justerar höjd stegvis, BORDE JUSTERA SERVOSPEED SPeedat upp
void Adjust_Height(float l, float height_step, float corner_pitch)
{
	//begränsa height här om för stor/liten
	for(uint8_t i = 0; i < m/4; i++)
	{
		last_height = last_height + 4 * height_step;
		
		triple_float kar_p1 = Tripod(l, 0, last_height, n); //kart koord för par 1
		triple_float kar_p2 = Tripod(l, 0, last_height, n + m); //kart koord för par 2
		
		Send_Legs_Kar(kar_p1, kar_p2, corner_pitch);
		_delay_ms(5); // =5 delay för kart provar mer speed
	}
	return;
}

//skickar ut kartesiska koordinater till ben, trycker ner ensamt söden extra (anpassad för tripod)
void Send_Legs_Kar(triple_float kar_p1, triple_float kar_p2, float corner_pitch)
{
	//extra push ner i marken för att kompensera för att ensamt ben sviktar mer än ett par.
	float left_boost = 0;
	float rigth_boost = 0;
	//tilldelar boost till ensamt ben.
	if(n <= m)
	{
		rigth_boost = 0;
	}
	else
	{
		left_boost = 0;
	}
	//par 1 får pos
	Send_Leg4_Kar(kar_p1.a, kar_p1.b, kar_p1.c - rigth_boost); 
	Send_Leg1_Kar(kar_p1.a, kar_p1.b + corner_pitch, kar_p1.c);
	Send_Leg5_Kar(kar_p1.a, kar_p1.b - corner_pitch, kar_p1.c);
			
	//par 2 får pos
	Send_Leg3_Kar(kar_p2.a, kar_p2.b, kar_p2.c - left_boost); 
	Send_Leg2_Kar(kar_p2.a, kar_p2.b + corner_pitch, kar_p2.c);
	Send_Leg6_Kar(kar_p2.a, kar_p2.b - corner_pitch, kar_p2.c);
}

//Roterar och skickar ut koordinater till ben ev: fixa så den lyfter ett av (beror på rotationsriktning) hörnbenen extra
void Rotate_And_Send_Legs(triple_float kar_p1, triple_float kar_p2, float corner_pitch, int sgn_speed,float theta)
{			
	//koord för par 1 omvandlas till cyl
	triple_float cyl1 = Kar_To_Cyl_Leg1(kar_p1.a, kar_p1.b + corner_pitch, kar_p1.c);
	triple_float cyl4 = Kar_To_Cyl_Leg4(kar_p1.a, kar_p1.b, kar_p1.c);
	triple_float cyl5 = Kar_To_Cyl_Leg5(kar_p1.a, kar_p1.b - corner_pitch, kar_p1.c);
	
	//koord för par 2 omvandlas till cyl
	triple_float cyl3 = Kar_To_Cyl_Leg3(kar_p2.a, kar_p2.b, kar_p2.c);
	triple_float cyl2 = Kar_To_Cyl_Leg2(kar_p2.a, kar_p2.b + corner_pitch, kar_p2.c);
	triple_float cyl6 = Kar_To_Cyl_Leg6(kar_p2.a, kar_p2.b - corner_pitch, kar_p2.c);
	
	if(n <= m) //stödfas par 1, svingfas par 2
	{
		//Stegvis rotation på p1
		cyl1.b = cyl1.b + sgn_speed * (- theta/2 + n * theta/m);
		cyl4.b = cyl4.b + sgn_speed * (- theta/2 + n * theta/m);
		cyl5.b = cyl5.b + sgn_speed * (- theta/2 + n * theta/m);
		
			//Stegvis rotation på p2
		cyl3.b = cyl3.b + sgn_speed * ( theta/2 - n * theta/m);
		cyl2.b = cyl2.b + sgn_speed * ( theta/2 - n * theta/m);
		cyl6.b = cyl6.b + sgn_speed * ( theta/2 - n * theta/m);
	}
	else //swingfas par1, stödfas par 2
	{
			//Stegvis rotation på p1
		cyl1.b = cyl1.b + sgn_speed * ( theta/2 - (n - m) * theta/m);
		cyl4.b = cyl4.b + sgn_speed * ( theta/2 - (n - m) * theta/m);
		cyl5.b = cyl5.b + sgn_speed * ( theta/2 - (n - m) * theta/m);
		//Stegvis rotation på p2
		cyl3.b = cyl3.b + sgn_speed * (- 3 * theta/2 + n * theta/m);  
		cyl2.b = cyl2.b + sgn_speed * (- 3 * theta/2 + n * theta/m);  
		cyl6.b = cyl6.b + sgn_speed * (- 3 * theta/2 + n * theta/m);  
	}
	//skicka ut cyl koord
	Send_Leg4_Cyl(cyl4.a, cyl4.b, cyl4.c);
	Send_Leg1_Cyl(cyl1.a, cyl1.b, cyl1.c);
	Send_Leg5_Cyl(cyl5.a, cyl5.b, cyl5.c);
	Send_Leg3_Cyl(cyl3.a, cyl3.b, cyl3.c);
	Send_Leg2_Cyl(cyl2.a, cyl2.b, cyl2.c);
	Send_Leg6_Cyl(cyl6.a, cyl6.b, cyl6.c);
}			



//begränsar maxhastighet.
float Limit_Speed(float speed, int sgn_speed)
{
	if(speed * sgn_speed > 6)  //utförs beräkninge som float?
	{
		speed = 6 * sgn_speed;
	}
	return speed;
}

float Limit_Height(float height)
{
	if(height > 14)
	{
		height = 14;
	}
	if(height < 8)
	{
		height = 8;
	}
	return height;
}

//Begränsar theta baserat på speed "steglös" Returnerar max tillåtna theta.
double_float Limit_Theta(float speed, int sgn_speed, float theta, int sgn_theta )
{
	speed = speed * sgn_speed;
	int speed_int = speed;
	float speed_dec = speed - speed_int;
	float thlimits[7] = {0.56,0.46,0.36,0.3,0.2,0.11,0};
	float thlin[7] = {-0.09,-0.1,-0.06,-0.1,-0.09,-0.11,0};
	
	//th_max beräknas med linjärsering mellan heltal av speed.
	float th_max = thlimits[speed_int] + speed_dec * thlin[speed_int];
	
	if( theta * sgn_theta > th_max )
	{
		theta = sgn_theta * th_max;
	}
	return create_double_float(theta, th_max);
}

//Generar tripod gång
triple_float Tripod(float x, float stroke, float height, uint8_t n)
{
	float lift = 2.5; //höjd som ben lyfts i sving
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
	else if( n <= 2 * m - 2) // för ben frammmåt mot startpos
	{
		y = -stroke/2 + (n-m) * stroke/m;
		z = lift - height;
	}
//	else if( n <=  2 * m - 2)//för ben framåt mot startpos
//	{
//		y = stroke/2;
//		z = lift - height;
//	}
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
void Walk_Half_Cycle(float speed, float theta, float height) 
{
	return;
	//justeringar
	//height = 11;//tilfällig steloperation
	float l = 13; //fötters förskjuting från kropp i x-led
	float corner_pitch = 8; //förskjutning av arbetsområde i y-led för hörnben
	
	int sgn_speed = (speed >= 0) - (speed < 0) ;
	int sgn_theta = (theta >= 0) - (theta < 0) ;
	uint8_t walk_break = 1;	
	
	//filtrerade o begränsade värden kommer som globala last_...sätter th_max och heigth step också.
	height = LP_Filter_And_Limit_Input(speed, sgn_speed, theta, sgn_theta, height); 
	//sätter parametrar till LP-filtrerade
	speed = last_speed;
	theta = last_theta;
	float height_step = (height - last_height)/m;
	
	if( speed == 0  && theta ==0 && height_step == 0) // gör inget
	{
		return;
	}

	float stroke =  2.1 * speed; //steglängd 2.2

	Adjust_Servo_Speed(theta, sgn_theta);
	
	if( speed == 0  && theta ==0 ) //justera höjd utan att gå
	{
		Adjust_Height(l, height_step, corner_pitch);
		return;
	}
	
	//gångloop, utför en halv gångfas och stannar när stödjande ben är i mitten av arbetsområde
	 while( walk_break || ( n != m/2 && n != 3 * m/2 ))
	{
		walk_break = 0;
		
		//stegvis höjdjustering
		last_height += height_step;
		triple_float kar_p1 = Tripod(l, stroke, last_height, n); //kart koord för par 1
		triple_float kar_p2 = Tripod(l, stroke, last_height, n + m); //kart koord för par 2
		
		if(theta == 0) //behöver ej gå via cyl koord vid rak gång.
		{
			Send_Legs_Kar(kar_p1, kar_p2, corner_pitch);
			if(walk_break || ( n+1 != m/2 && n+1 != 3 * m/2 ))
			{
				_delay_ms(5); // =5 delay för kart skippar på sista loopen, pga tids som mainloop tar TEST
			}
		}
		else //om sväng
		{
			Rotate_And_Send_Legs(kar_p1, kar_p2, corner_pitch, sgn_speed, theta);
				if(walk_break || ( n+1 != m/2 && n+1 != 3 * m/2 ))
			{
				_delay_ms(3); // =3 Delay för cyl skippar på sista loopen, pga tids som mainloop tar TEST
			}
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
}

//test, endast rak gång, ej testad med olika höjder.
//Crab people, Crab people, tastes like crab, talks like people.
void Walk_Half_Crab_Cycle(float speed) 
{
	if( speed == 0 ) // gör inget
	{
		return;
	}
	

	//justeringar
	float l = 13; //fötters förskjuting från kropp i x-led
	float corner_pitch = 8; //förskjutning av arbetsområde i y-led för hörnben
	
	int sgn_speed = (speed >= 0) - (speed < 0) ;
	uint8_t walk_break = 1;	
	speed = Limit_Speed(speed, sgn_speed);
	
	float stroke = speed * 0.8; //steglängd 
	unsigned int speed_drive =  150;
	unsigned int speed_lift  =  250 + 30 * (sgn_speed * speed - 6);//fast värde 0x0100 //336 - 10 * //280
 
	//skickar hastighetsbegränsningar till servon.
	Send_Inner_P1_Velocity(speed_drive);
	Send_Middle_P1_Velocity(speed_lift);
	Send_Outer_P1_Velocity(speed_lift);
	Send_Inner_P2_Velocity(speed_drive);
	Send_Middle_P2_Velocity(speed_lift);
	Send_Outer_P2_Velocity(speed_lift);
	
	float left_boost = 0;
	float rigth_boost = 0;
	if(sgn_speed > 0)
	{
		
	}
	
		if( last_height != 11)
	{
		Adjust_Height(l, (11-last_height)/m, corner_pitch);
	}
	
	//gångloop, utför en halv gångfas och stannar när stödjande ben är i mitten av arbetsområde
	 while( walk_break || ( n != m/2 && n != 3 * m/2 ))
	{
		walk_break = 0;
		
		triple_float kar_p1 = Tripod(l, stroke, last_height, n); //kart koord för par 1
		triple_float kar_p2 = Tripod(l, stroke, last_height, n + m); //kart koord för par 2
		
		//par 1 får pos
		Send_Leg4_Kar(kar_p1.a - kar_p1.b, 0, kar_p1.c); 
		Send_Leg1_Kar(kar_p1.a + kar_p1.b, corner_pitch, kar_p1.c);
		Send_Leg5_Kar(kar_p1.a + kar_p1.b, - corner_pitch, kar_p1.c);
			
		//par 2 får pos
		Send_Leg3_Kar(kar_p2.a + kar_p2.b, 0, kar_p2.c); 
		Send_Leg2_Kar(kar_p2.a - kar_p2.b, corner_pitch, kar_p2.c);
		Send_Leg6_Kar(kar_p2.a - kar_p2.b, - corner_pitch, kar_p2.c);
		
		if(walk_break || ( n+1 != m/2 && n+1 != 3 * m/2 ))
		{
			_delay_ms(10); // =5 delay för kart skippar på sista loopen, pga tids som mainloop tar TEST
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
	
}

// speed = hastighet fram/bak [-6,6], th = sväng[-0.57, 0.57], h = höjd från mark(11)
// långsam version av Walk_Half_Hycle, utför samma rörelse, men långsamt.

//OBS höjdjustering kan ge felaktigt beteende! justera parsamt.
void Walk_Half_Safe_Cycle(float speed, float theta, float height) 
{
	//justeringar
	//height = 11;//tilfällig steloperation
	float l = 13; //fötters förskjuting från kropp i x-led
	float corner_pitch = 8; //förskjutning av arbetsområde i y-led för hörnben
	
	int sgn_speed = (speed >= 0) - (speed < 0) ;
	int sgn_theta = (theta >= 0) - (theta < 0) ;
	uint8_t walk_break = 1;	
	
	//filtrerade o begränsade värden kommer som globala last_...sätter th_max och heigth step också.
	height = LP_Filter_And_Limit_Input(speed, sgn_speed, theta, sgn_theta, height); 
	//sätter parametrar till LP-filtrerade
	speed = last_speed;
	theta = last_theta;
	float height_step = (height - last_height)/m;
	
	if( speed == 0  && theta ==0 && height_step == 0) // gör inget
	{
		return;
	}

	float stroke =  2.1 * speed; //steglängd 2.2

	Adjust_Servo_Speed(theta, sgn_theta);
	
	if( speed == 0  && theta ==0 ) //justera höjd utan att gå
	{
		Adjust_Height(l, height_step, corner_pitch);
		return;
	}
	
	//gångloop, utför en halv gångfas och stannar när stödjande ben är i mitten av arbetsområde
	 while( walk_break || ( n != m/2 && n != 3 * m/2 ))
	{
		walk_break = 0;
		
		//stegvis höjdjustering
		last_height += height_step;
		triple_float kar_p1 = Tripod(l, stroke, last_height, n); //kart koord för par 1
		triple_float kar_p2 = Tripod(l, stroke, last_height, n + m); //kart koord för par 2
		
		if(theta == 0) //behöver ej gå via cyl koord vid rak gång.
		{
			Send_Legs_Kar(kar_p1, kar_p2, corner_pitch);
			if(walk_break || ( n+1 != m/2 && n+1 != 3 * m/2 ))
			{
				_delay_ms(10); // =5 delay för kart skippar på sista loopen, pga tids som mainloop tar TEST
			}
		}
		else //om sväng
		{
			Rotate_And_Send_Legs(kar_p1, kar_p2, corner_pitch, sgn_speed, theta);
				if(walk_break || ( n+1 != m/2 && n+1 != 3 * m/2 ))
			{
				_delay_ms(8); // =3 Delay för cyl skippar på sista loopen, pga tids som mainloop tar TEST
			}
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
}