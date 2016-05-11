/*
 * GANGSTILAR.c
 *
 * Created: 8 april 3:03:52 PM
 *  Author: jenma421 
 */ 

#include "gangstilar.h"


uint8_t support_l = 24; //läng på stödfas
uint8_t swing_l = 12; //längd på svingfas 12
uint8_t cycle_length = 36; //cykelängd = support_l + swing_l
uint8_t n_1 = 12; //index för benpar 1
uint8_t n_2 = 30; //index för benpar 2
float last_height = 11; //höjd senaste avslutade cykel
float last_speed = 0; //speed senaste avslutade cykel
float last_theta = 0; //theta senaste avslutade cykel
float theta_max = 0;
uint8_t p1_down = 0; //benpar 1 mot mark
uint8_t p2_down = 1; //benpar 2 mot mark


//OBS tilfällig ändring i send legs kar!

void Adjust_Stance_Climbed(char direction) //me no like 
{
	if(direction == 'u')
	{
		last_height = 14-6.3;
		height = 14-6.3;
	}
	else 
	{
		last_height = 14;
		height = 14;
	}
	n_1=support_l/2 + cycle_length/2; 
}


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
	
	/*if(theta_diff > 0.54)
	{
		theta = last_theta + 0.54;
	}
	else if(theta_diff < -0.54)
	{
		theta = last_theta - 0.54;
	}*/
	
	if(height_diff > 2)
	{
		height = last_height + 2;
	}
	else if(height_diff < - 4)
	{
		height = last_height - 4;
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
triple_float Adjust_Servo_Speed(float theta, int sgn_theta, int8_t leg_down)
{    
	int speed_inner ;
	int speed_middle;
	int speed_outer;
	
	if(leg_down == 1)
			{
	speed_inner =  280 + 200 * (sgn_theta * theta - theta_max);// 250 + 500 * 
	speed_middle  = 100; 
	speed_outer  =  100;
			}     
			else // if(theta*sgn_theta <=0.20)
			{                                                                   
	//justerar servospeed ÄNDRA SKALFAKTOR !!
	 speed_inner =  280 + 200 * (sgn_theta * theta - theta_max);// 250 + 500 * 
	 speed_middle  = 350 + 340 * (sgn_theta * theta - theta_max);//320 + 430 *//220
	 speed_outer  =  380 + 300 * (sgn_theta * theta - theta_max);//320 + 430 *//250
			}
		/*else
		{
	 speed_inner =  30 + 160 * (sgn_theta * theta - theta_max);// 250 + 500 * 
	 speed_middle  = 350 + 340 * (sgn_theta * theta - theta_max);//320 + 430 //220
	 speed_outer  =  380 + 300 * (sgn_theta * theta - theta_max);//320 + 430 //250
		}	*/
		
 return create_triple_float(speed_inner, speed_middle, speed_outer);
}	

//justerar höjd stegvis, BORDE JUSTERA SERVOSPEED tveksamt om vi vill ha denna.
void Adjust_Height(float l, float height_step, float corner_pitch)
{
	//begränsa height här om för stor/liten
	for(uint8_t i = 0; i < cycle_length/2; i++)
	{
		//last_height += height_step;
		
		//triple_float kar_p1 = Tripod(l, 0, last_height, n); //kart koord för par 1
		//triple_float kar_p2 = Tripod(l, 0, last_height, n_2); //kart koord för par 2  
		
		//Send_Legs_Kar(kar_p1, kar_p2, corner_pitch, , p2_down);
		_delay_ms(5); // =5 delay för kart provar mer speed
	}
	return;
}

void Send_Legs_Kar(triple_float kar_p1, triple_float kar_p2, float corner_pitch, triple_float speed_p1, triple_float speed_p2) 
{
	int sgn_speed = (last_speed >= 0) - (last_speed < 0);
	float scale = 1 + 0.2*sgn_speed;
	
	Send_Leg5_Kar_And_Velocity(kar_p1.a, kar_p1.b - corner_pitch, kar_p1.c, speed_p1.a, speed_p1.b, speed_p1.c); 
	Send_Leg1_Kar_And_Velocity(kar_p1.a, kar_p1.b + corner_pitch, kar_p1.c, speed_p1.a, speed_p1.b, speed_p1.c);	
	Send_Leg3_Kar_And_Velocity(kar_p2.a, kar_p2.b, kar_p2.c, speed_p2.a, speed_p2.b, speed_p2.c); 
	Send_Leg4_Kar_And_Velocity(kar_p1.a, kar_p1.b, kar_p1.c, speed_p1.a, speed_p1.b, speed_p1.c); 	//OBS!! ta bort efter servofix?? <------------------------------------------------------------------------
	 
	Send_Leg2_Kar_And_Velocity(kar_p2.a, kar_p2.b + corner_pitch, kar_p2.c, speed_p2.a, speed_p2.b, speed_p2.c);
	Send_Leg6_Kar_And_Velocity(kar_p2.a, kar_p2.b - corner_pitch, kar_p2.c, speed_p2.a, speed_p2.b, speed_p2.c);
}

//Roterar och skickar ut koordinater till ben ev: fixa så den lyfter ett av (beror på rotationsriktning) hörnbenen extra
void Rotate_And_Send_Legs(triple_float kar_p1, triple_float kar_p2, float corner_pitch, int sgn_speed, float theta, triple_float speed_p1, triple_float speed_p2)
{			
	//koord för par 1 omvandlas till cyl
	triple_float cyl4 = Kar_To_Cyl_Leg4(kar_p1.a, kar_p1.b, kar_p1.c); 
	triple_float cyl1 = Kar_To_Cyl_Leg1(kar_p1.a , kar_p1.b + corner_pitch, kar_p1.c); //testar att ha mindre x på hörnben
	triple_float cyl5 = Kar_To_Cyl_Leg5(kar_p1.a , kar_p1.b - corner_pitch, kar_p1.c);
	
	//koord för par 2 omvandlas till cyl
	triple_float cyl3 = Kar_To_Cyl_Leg3(kar_p2.a, kar_p2.b, kar_p2.c); 
	triple_float cyl2 = Kar_To_Cyl_Leg2(kar_p2.a  , kar_p2.b + corner_pitch, kar_p2.c);
	triple_float cyl6 = Kar_To_Cyl_Leg6(kar_p2.a , kar_p2.b - corner_pitch, kar_p2.c);
	
	if(n_1 <= support_l) 
	{
		//Stegvis rotation på p1 stöd
		cyl1.b = cyl1.b + sgn_speed * (- theta/2 + n_1* theta/support_l);
		cyl4.b = cyl4.b + sgn_speed * (- theta/2 + n_1* theta/support_l);
		cyl5.b = cyl5.b + sgn_speed * (- theta/2 + n_1* theta/support_l);
	}
	else 
	{
			//Stegvis rotation på p1 sving
		cyl1.b = cyl1.b + sgn_speed * ( theta/2 - (n_1 - support_l) * theta/swing_l);
		cyl4.b = cyl4.b + sgn_speed * ( theta/2 - (n_1 - support_l) * theta/swing_l);
		cyl5.b = cyl5.b + sgn_speed * ( theta/2 - (n_1 - support_l) * theta/swing_l);  
	}
	
	if(n_2 <= support_l) 
	{
			//Stegvis rotation på p2 stöd
			cyl3.b = cyl3.b + sgn_speed * (- theta/2 + n_2 * theta/support_l);
			cyl2.b = cyl2.b + sgn_speed * (- theta/2 + n_2 * theta/support_l);
			cyl6.b = cyl6.b + sgn_speed * (- theta/2 + n_2 * theta/support_l);
	}
	else
	{
			//Stegvis rotation på p2 sving
			cyl3.b = cyl3.b + sgn_speed * ( theta/2 - (n_2 - support_l) * theta/swing_l);
			cyl2.b = cyl2.b + sgn_speed * ( theta/2 - (n_2 - support_l) * theta/swing_l);
			cyl6.b = cyl6.b + sgn_speed * ( theta/2 - (n_2 - support_l) * theta/swing_l);
	}

	//skicka ut cyl koord
	Send_Leg4_Cyl_And_Velocity(cyl4.a, cyl4.b, cyl4.c, speed_p1.a, speed_p1.b, speed_p1.c);
	Send_Leg1_Cyl_And_Velocity(cyl1.a, cyl1.b, cyl1.c, speed_p1.a, speed_p1.b, speed_p1.c);
	Send_Leg5_Cyl_And_Velocity(cyl5.a, cyl5.b, cyl5.c, speed_p1.a, speed_p1.b, speed_p1.c);
	Send_Leg3_Cyl_And_Velocity(cyl3.a, cyl3.b, cyl3.c, speed_p2.a, speed_p2.b, speed_p2.c);
	Send_Leg2_Cyl_And_Velocity(cyl2.a, cyl2.b, cyl2.c, speed_p2.a, speed_p2.b, speed_p2.c);
	Send_Leg6_Cyl_And_Velocity(cyl6.a, cyl6.b, cyl6.c, speed_p2.a, speed_p2.b, speed_p2.c);
}			



//begränsar maxhastighet.
float Limit_Speed(float speed, int sgn_speed)
{
	if( ((last_height <= 9) || (last_height >= 12)) && (speed * sgn_speed > 5))
	{
		speed = 5 * sgn_speed;
	}
	else if(speed * sgn_speed > 6)  //utförs beräkninge som float?
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
	else if(height < 7)
	{
		height = 7;
	}
	return height;
}

//Begränsar theta baserat på speed "steglös" Returnerar max tillåtna theta.
double_float Limit_Theta(float speed, int sgn_speed, float theta, int sgn_theta )
{
	speed = speed * sgn_speed;
	int speed_int = speed;
	float speed_dec = speed - speed_int;
	float thlimits[7] = {0.57,0.45,0.35,0.33,0.23,0.13,0};//{0.56,0.46,0.36,0.3,0.2,0.11,0};{0.33,0.33,0.30,0.3,0.2,0.11,0};
	float thlin[7] =    {-0.12,-0.1,-0.02,-0.1,-0.1,-0.13,0};
	
	//th_max beräknas med linjärsering mellan heltal av speed.
	float th_max = thlimits[speed_int] + speed_dec * thlin[speed_int];
	
	if( theta * sgn_theta > th_max )
	{
		theta = sgn_theta * th_max;
	}
	return create_double_float(theta, th_max);
}


//Generar tripod gång testar bwta >½ 16 steg svingfas
triple_float Tripod(float x, float stroke, float height,float lift, uint8_t n)
{
	
	float y = 0;
	float z = 0;
	
	if(n <= 2 ) //delat upp bensänkingen i steg
	{
		y = stroke/2 - stroke * n/support_l;
		z= - height;//lift/2 - n * lift/4 - height;
	}
	else if(n <= support_l - 1)//stödfas
	{
		y = stroke/2 - stroke * n/support_l;
		z= - height;
	}
	else if( n <= support_l)//börjar lyfta ben med fortsatt benrörelse bakåt
	{
		y = stroke/2 - stroke * n/support_l;
		z =   lift - height;
	}
	else if( n <= cycle_length - 1) // för ben frammmåt mot startpos
	{
		y = -stroke/2 + (n - support_l) * stroke/swing_l;
		z = lift - height;
	}
	else if( n <= cycle_length) // för ben frammmåt mot startpos
	{
		y = -stroke/2 + (n - support_l) * stroke/swing_l;
		z=  lift - height;  // + (cycle_length - n) * lift/4 - height;
	}
	
	return create_triple_float(x,y,z);
}




//testar överlappande svingfas, strypt o ingen höj/sänk
void Walk_Half_Cycle(float speed, float theta, float height)
{

	float l = 12;//13 låg //fötters förskjuting från kropp i x-led OBS orginal = 13, numera 12
	float corner_pitch = 4; //förskjutning av arbetsområde i y-led för hörnben 4
	
	int sgn_speed = (speed >= 0) - (speed < 0) ;
	int sgn_theta = (theta >= 0) - (theta < 0) ;
	uint8_t walk_break = 1;
	
	//filtrerade o begränsade värden kommer som globala last_...sätter th_max och heigth step också.
	height = LP_Filter_And_Limit_Input(speed, sgn_speed, theta, sgn_theta, height);
	//sätter parametrar till LP-filtrerade
	speed = last_speed;
	theta = last_theta;
	sgn_speed = (speed >= 0) - (speed < 0) ;
	sgn_theta = (theta >= 0) - (theta < 0) ;
	float height_step = 2 * (height - last_height)/cycle_length;
	float lift = 1.2 + sgn_speed*speed/12; //benens lyfthöjd justeras efter speed 1.2 + sgn_speed*speed/5; 
	float stroke =  1.8 * speed; //steglängd 2.2
	
	if( (speed*sgn_speed < 0.2 ) && (theta * sgn_theta < 0.01) && (height_step == 0)) // gör inget
	{
		return;
	}
	
	if( (speed*sgn_speed < 0.2 ) && (theta * sgn_theta < 0.01) ) //justera höjd utan att gå
	{
		stroke = 0;
		lift = 0;
	}
	
	//gångloop, utför en halv gångfas och stannar när stödjande ben är i mitten av arbetsområde
	while( walk_break || ( n_1 != support_l/2 && n_2 !=  support_l/2))
	{
		walk_break = 0;
			if(n_1 < cycle_length)
		{
			n_1 += 1; //stegar upp index
		}
		else
		{
			n_1 = 1; //nollar index
		}
		
		//stegvis höjdjustering
		last_height += height_step;
		
		n_2= n_1 + cycle_length/2; //barnpar 2 förskjuts en halv fas
		
		if(n_2 > cycle_length) //ser till att det blir cykliskt
		{
			n_2 -= cycle_length;
		}
		
		if(n_1 <= 4 )
		{
			p1_down =1;
		}
		else
		{
			p1_down = 0;
		}
				
		if((n_2 <= 4))
		{
			p2_down =1;
		}
		else
		{
			p2_down = 0;
		}
		
		triple_float kar_p1;
		triple_float kar_p2;
		triple_float speed_p1 = Adjust_Servo_Speed(theta, theta_max, p1_down);
		triple_float speed_p2 = Adjust_Servo_Speed(theta, theta_max, p2_down);
		
	
		
		if(theta * sgn_theta < 0.01) //behöver ej gå via cyl koord vid rak gång.
		{
			kar_p1 = Tripod(l, stroke, last_height, lift, n_1); //kart koord för par 1
			kar_p2 = Tripod(l, stroke, last_height, lift, n_2); //kart koord för par 2
			
			Send_Legs_Kar(kar_p1, kar_p2, corner_pitch, speed_p1, speed_p2);
			
			if( walk_break || (n_1 != support_l/2 && n_2 != support_l/2))
			{
			_delay_ms(7); // =9 delay för kart skippar på sista loopen, pga tids som mainloop tar TEST
			}
		}
		else //om sväng
		{
			lift = 1.5 + theta * sgn_theta * 1.2; //1.5 + theta * sgn_theta *1.8;
			kar_p1 = Tripod(l, stroke, last_height, lift, n_1); //kart koord för par 1
			kar_p2 = Tripod(l, stroke, last_height, lift, n_2); //kart koord för par 2
			
			Rotate_And_Send_Legs(kar_p1, kar_p2, corner_pitch, sgn_speed, theta, speed_p1, speed_p2);
			
			if( walk_break || ( n_1 != support_l/2 && n_2  !=  support_l/2))
			{
			_delay_ms(5); // 7 Delay för cyl skippar på sista loopen, pga tids som mainloop tar TEST
			}
					
		}
	}
	last_height = height;
}

//Crab 2.0 - The servokiller
void Walk_Half_Crab_Cycle(int8_t speed)// höger är possitivt
{
	float l = 12;//13 låg //fötters förskjuting från kropp i x-led OBS orginal = 13, numera 12
	float corner_pitch = 4; //förskjutning av arbetsområde i y-led för hörnben 8
	int sgn_speed = (speed >= 0) - (speed < 0) ;
	uint8_t walk_break = 1;
	float stroke =6 * sgn_speed;  //steglängd 2.2

	if(speed == 0) // gör inget
	{
		return;
	}
	
	if(last_height != 11)
	{
		Walk_Half_Cycle(0,0,11);
	}
	//gångloop, utför en halv gångfas och stannar när stödjande ben är i mitten av arbetsområde
	while( walk_break || ( n_1 != support_l/2 && n_2 !=  support_l/2))
	{
		walk_break = 0;
			if(n_1 < cycle_length)
		{
			n_1 += 1; //stegar upp index
		}
		else
		{
			n_1 = 1; //nollar index
		}
		
		n_2= n_1 + cycle_length/2; //barnpar 2 förskjuts en halv fas
		
		if(n_2 > cycle_length) //ser till att det blir cykliskt
		{
			n_2 -= cycle_length;
		}
		
		if(n_1 <= 4 )
		{
			p1_down =1;
		}
		else
		{
			p1_down = 0;
		}
				
		if((n_2 <= 4))
		{
			p2_down =1;
		}
		else
		{
			p2_down = 0;
		}
		
		triple_float kar_p1;
		triple_float kar_p2;
		
		//servospeeds
		float speed_p1_middle = 310- 80 * p1_down;
		float speed_p1_outer = 310 - 80 * p1_down;
		float speed_p2_middle = 310 - 80 * p2_down;
		float speed_p2_outer = 310 - 80 * p2_down;

		float lift =1.4; 
	
			kar_p1 = Tripod(l, stroke, last_height, lift, n_1); //kart koord för par 1
			kar_p2 = Tripod(l, stroke, last_height, lift, n_2); //kart koord för par 2
			
			Send_Leg5_Kar_And_Velocity(1 + kar_p1.a - kar_p1.b, - corner_pitch, kar_p1.c, 100, speed_p1_middle, speed_p1_outer); 	
			Send_Leg3_Kar_And_Velocity(1 + kar_p2.a - kar_p2.b, 0, kar_p2.c, 100, speed_p2_middle, speed_p2_outer); 
			Send_Leg4_Kar_And_Velocity(1 + kar_p1.a +  kar_p1.b, 0, kar_p1.c, 100, speed_p1_middle, speed_p1_outer); 	
			Send_Leg1_Kar_And_Velocity(1 + kar_p1.a -  kar_p1.b, corner_pitch, kar_p1.c, 100, speed_p1_middle, speed_p1_outer); 
			Send_Leg2_Kar_And_Velocity(1 + kar_p2.a + kar_p2.b, corner_pitch, kar_p2.c, 100, speed_p2_middle, speed_p2_outer);
			Send_Leg6_Kar_And_Velocity(1 + kar_p2.a + kar_p2.b, - corner_pitch, kar_p2.c, 100, speed_p2_middle, speed_p2_outer);
			
			if( walk_break || (n_1 != support_l/2 && n_2 != support_l/2))
			{
			_delay_ms(7); 
			}
		}
}