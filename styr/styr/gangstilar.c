/*
 *        File: gangstilar.c
 *     Version: 1.0
 * Last edited: 18/5/2016 
 *	   Authors: jenma421
 */ 

#include "gangstilar.h"

uint8_t support_l = 24; //l�ngd p� st�dfas
uint8_t swing_l = 12; //l�ngd p� svingfas 
uint8_t cycle_length = 36; //cykel�ngd = support_l + swing_l
uint8_t n_1 = 12; //index f�r benpar 1
uint8_t n_2 = 30; //index f�r benpar 2
float last_height = 11; //h�jd senaste avslutade cykel
float last_speed = 0; //speed senaste avslutade cykel
float last_theta = 0; //theta senaste avslutade cykel
float theta_max = 0; //max till�tna theta med aktuell speed
uint8_t p1_down = 0; //benpar 1 mot mark
uint8_t p2_down = 1; //benpar 2 mot mark
float dance_x = 0; 
float dance_y = 0; 

void Adjust_Stance_Climbed(char direction)
{
	if(direction == 'u')
	{
		last_height = 14-6.3;
	}
	else 
	{
		last_height = 14;
	}
	n_1=support_l/2 + cycle_length/2; 
}

//LP-filtrerar input f�r att undvika v�ldsamheter. S�tter �ven theta_max
float LP_Filter_And_Limit_Input(float speed, int sgn_speed, float theta, int sgn_theta, float height, float dx, int sgn_dx)
{
	speed = Limit_Speed(speed, sgn_speed, dx, sgn_dx);
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

//Justerar servospeed efter f�rh�llande mellan theta o thetamax
triple_float Calc_Servo_Speed(float theta, int sgn_theta, int8_t leg_down, float dx, int sgn_dx)
{    
	int speed_inner ;
	int speed_middle;
	int speed_outer;
	
	if(leg_down == 1)
		{
		speed_inner =  270 + 170 * (sgn_theta * theta - theta_max);
		speed_middle  = 100 + 10 * dx * sgn_dx;
		speed_outer  =  100 + 20 * dx * sgn_dx;
		}     
		else 
		{                                                                   
		speed_inner =  270 + 170 * (sgn_theta * theta - theta_max);
		speed_middle  = 270 + 280 * (sgn_theta * theta - theta_max) + 40 * dx * sgn_dx;
		speed_outer  =  270  + 260 * (sgn_theta * theta - theta_max) + 40 * dx * sgn_dx;
		}
		
 return create_triple_float(speed_inner, speed_middle, speed_outer);
}	

//Skickar kartersiska koordianter och hastigheter till ben
void Send_Legs_Kar(triple_float kar_p1, triple_float kar_p2, float corner_pitch, triple_float speed_p1, triple_float speed_p2, float dx_p1, float dx_p2) 
{
	Send_Leg5_Kar_And_Velocity(kar_p1.a + dx_p1, kar_p1.b - corner_pitch, kar_p1.c, speed_p1.a, speed_p1.b, speed_p1.c); 
	Send_Leg1_Kar_And_Velocity(kar_p1.a + dx_p1, kar_p1.b + corner_pitch, kar_p1.c, speed_p1.a, speed_p1.b, speed_p1.c);	
	Send_Leg3_Kar_And_Velocity(kar_p2.a + dx_p2, kar_p2.b, kar_p2.c, speed_p2.a, speed_p2.b, speed_p2.c); 
	Send_Leg4_Kar_And_Velocity(kar_p1.a - dx_p1, kar_p1.b, kar_p1.c, speed_p1.a, speed_p1.b, speed_p1.c); 
	Send_Leg2_Kar_And_Velocity(kar_p2.a - dx_p2, kar_p2.b + corner_pitch, kar_p2.c, speed_p2.a, speed_p2.b, speed_p2.c);
	Send_Leg6_Kar_And_Velocity(kar_p2.a - dx_p2, kar_p2.b - corner_pitch, kar_p2.c, speed_p2.a, speed_p2.b, speed_p2.c);
}

//Roterar och skickar ut koordinater till ben ev: fixa s� den lyfter ett av (beror p� rotationsriktning) h�rnbenen extra
void Rotate_And_Send_Legs(triple_float kar_p1, triple_float kar_p2, float corner_pitch, int sgn_speed, float theta, triple_float speed_p1, triple_float speed_p2, float dx_p1, float dx_p2)
{			
	//koord f�r par 1 omvandlas till cyl
	triple_float cyl4 = Kar_To_Cyl_Leg4(kar_p1.a - dx_p1, kar_p1.b, kar_p1.c); 
	triple_float cyl1 = Kar_To_Cyl_Leg1(kar_p1.a + dx_p1, kar_p1.b + corner_pitch, kar_p1.c);
	triple_float cyl5 = Kar_To_Cyl_Leg5(kar_p1.a + dx_p1, kar_p1.b - corner_pitch, kar_p1.c);
	
	//koord f�r par 2 omvandlas till cyl
	triple_float cyl3 = Kar_To_Cyl_Leg3(kar_p2.a + dx_p2, kar_p2.b, kar_p2.c); 
	triple_float cyl2 = Kar_To_Cyl_Leg2(kar_p2.a - dx_p2, kar_p2.b + corner_pitch, kar_p2.c);
	triple_float cyl6 = Kar_To_Cyl_Leg6(kar_p2.a - dx_p2, kar_p2.b - corner_pitch, kar_p2.c);
	
	if(n_1 <= support_l) 
	{
		//Stegvis rotation p� p1 st�dfas
		cyl1.b = cyl1.b + sgn_speed * (- theta/2 + n_1 * theta/support_l);
		cyl4.b = cyl4.b + sgn_speed * (- theta/2 + n_1 * theta/support_l);
		cyl5.b = cyl5.b + sgn_speed * (- theta/2 + n_1 * theta/support_l);
	}
	else 
	{
			//Stegvis rotation p� p1 svingfas
		cyl1.b = cyl1.b + sgn_speed * ( theta/2 - (n_1 - support_l) * theta/swing_l);
		cyl4.b = cyl4.b + sgn_speed * ( theta/2 - (n_1 - support_l) * theta/swing_l);
		cyl5.b = cyl5.b + sgn_speed * ( theta/2 - (n_1 - support_l) * theta/swing_l);  
	}
	
	if(n_2 <= support_l) 
	{
			//Stegvis rotation p� p2 st�dfas
			cyl3.b = cyl3.b + sgn_speed * (- theta/2 + n_2 * theta/support_l);
			cyl2.b = cyl2.b + sgn_speed * (- theta/2 + n_2 * theta/support_l);
			cyl6.b = cyl6.b + sgn_speed * (- theta/2 + n_2 * theta/support_l);
	}
	else
	{
			//Stegvis rotation p� p2 svingfas
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

//begr�nsar maxhastighet.
float Limit_Speed(float speed, int sgn_speed,  float dx, int sgn_dx)
{
	if( ((last_height <= 9) || (last_height >= 12)) && (speed * sgn_speed > 5))
	{
		speed = 5 * sgn_speed;
	}
	else if(speed * sgn_speed > ( 6 - dx * sgn_dx/4))  
	{
		speed = 6 * sgn_speed - dx * sgn_dx/4;
	}
	return speed;
}

//Begr�nsar h�jd
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

//Begr�nsar theta baserat p� speed, stegl�s Returnerar max till�tna theta och begr�nsat theta
double_float Limit_Theta(float speed, int sgn_speed, float theta, int sgn_theta )
{
	speed = speed * sgn_speed;
	int speed_int = speed;
	float speed_dec = speed - speed_int;
	float thlimits[7] = {0.57,0.45,0.35,0.33,0.23,0.13,0.04};
	float thlin[7] =    {-0.12,-0.1,-0.02,-0.1,-0.1,-0.09,0};
	
	//th_max ber�knas med linj�rsering mellan heltal av speed.
	float th_max = thlimits[speed_int] + speed_dec * thlin[speed_int];
	
	if( theta * sgn_theta > th_max )
	{
		theta = sgn_theta * th_max;
	}
	return create_double_float(theta, th_max);
}

//Generar tripod g�ng 
triple_float Tripod(float x, float stroke, float height,float lift, uint8_t n)
{
	float y = 0;
	float z = 0;
	
	if(n <= support_l - 1)//st�dfas
	{
		y = stroke/2 - stroke * n/support_l;
		z= - height;
	}
	else if( n <= support_l)//b�rjar lyfta ben med fortsatt benr�relse bak�t
	{
		y = stroke/2 - stroke * n/support_l;
		z = lift - height;
	}
	else if( n <= cycle_length - 1) // f�r ben frammm�t mot startpos
	{
		y = -stroke/2 + (n - support_l) * stroke/swing_l;
		z = lift - height;
	}
	
	return create_triple_float(x,y,z);
}

//G�r en halv cykel
void Walk_Half_Cycle(float speed, float theta, float height, float dx)
{

	float l = 12; //f�tters f�rskjuting fr�n kropp i x-led 
	float corner_pitch = 4; //f�rskjutning av arbetsomr�de i y-led f�r h�rnben 
	
	int sgn_dx = (dx >= 0) - (dx < 0);
	int sgn_speed = (speed >= 0) - (speed < 0);
	int sgn_theta = (theta >= 0) - (theta < 0);
	uint8_t walk_break = 1;
	
	if( dx * sgn_dx > 6)
	{
		dx = 6 * sgn_dx;
	}
	
	//filtrerade o begr�nsade v�rden kommer som globala last_
	height = LP_Filter_And_Limit_Input(speed, sgn_speed, theta, sgn_theta, height, dx, sgn_dx);
	//s�tter parametrar till LP-filtrerade
	speed = last_speed;
	theta = last_theta;
	sgn_speed = (speed >= 0) - (speed < 0) ;
	sgn_theta = (theta >= 0) - (theta < 0) ;
	float height_step = 2 * (height - last_height)/cycle_length;
	float lift = 1.2 + sgn_speed * speed/12;//benens lyfth�jd justeras efter speed 
	float stroke =  1.8 * speed; 
	float dx_p1 = 0;
	float dx_p2 = 0;
	triple_float kar_p1;
	triple_float kar_p2;
	triple_float speed_p1;
	triple_float speed_p2;
	
	if( (speed*sgn_speed < 0.2 ) && (theta * sgn_theta < 0.01) && (height_step == 0) &&(dx * sgn_dx < 0.5)) // g�r inget
	{
		return;
	}
	
	if( (speed*sgn_speed < 0.2 ) && (theta * sgn_theta < 0.01)&&(dx * sgn_dx < 0.5) ) //justera h�jd utan att g�
	{
		stroke = 0;
		lift = 0;
		dx = 0;
	}
	
	//g�ngloop, utf�r en halv g�ngfas och stannar n�r st�djande ben �r i mitten av arbetsomr�de
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
		
		//stegvis h�jdjustering
		last_height += height_step;
		n_2= n_1 + cycle_length/2; //benpar 2 f�rskjuts en halv fas
		
		if(n_2 > cycle_length) //ser till att det blir cykliskt
		{
			n_2 -= cycle_length;
		}
		
		if(n_1 <= 4 )
		{
			p1_down =1; //ber�ttar om benpar ska s�ttas ner
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
	
		if(n_1 <= support_l)
		{
			dx_p1 = (n_1-support_l/2) * dx/support_l;
		}
		else
		{
			dx_p1 = (support_l + swing_l/2 - n_1) * dx/swing_l;
		}
		
		if(n_2 <= support_l)
		{
			dx_p2 = (n_2-support_l/2) * dx/support_l;
		}
		else
		{
			dx_p2 = (support_l + swing_l/2 - n_2) * dx/swing_l;
		}
	
		speed_p1 = Calc_Servo_Speed(theta, sgn_theta, p1_down, dx, sgn_dx);
		speed_p2 = Calc_Servo_Speed(theta, sgn_theta, p2_down, dx, sgn_dx);

		if(theta * sgn_theta < 0.01) //beh�ver ej g� via cyl koord vid rak g�ng.
		{
			kar_p1 = Tripod(l, stroke, last_height, lift, n_1); //kart koord f�r par 1
			kar_p2 = Tripod(l, stroke, last_height, lift, n_2); //kart koord f�r par 2
			
			Send_Legs_Kar(kar_p1, kar_p2, corner_pitch, speed_p1, speed_p2, dx_p1, dx_p2);
			
			if( walk_break || (n_1 != support_l/2 && n_2 != support_l/2))
			{
			_delay_ms(7); //f�r kart skippar p� sista loopen, pga tid som mainloop tar 
			}
		}
		else //om sv�ng
		{
			lift = 1.5 + theta * sgn_theta * 1.2; 
			kar_p1 = Tripod(l, stroke, last_height, lift, n_1); //kart koord f�r par 1
			kar_p2 = Tripod(l, stroke, last_height, lift, n_2); //kart koord f�r par 2
			
			Rotate_And_Send_Legs(kar_p1, kar_p2, corner_pitch, sgn_speed, theta, speed_p1, speed_p2, dx_p1, dx_p2);
			
			if( walk_break || ( n_1 != support_l/2 && n_2  !=  support_l/2))
			{
			_delay_ms(5); //Delay f�r cyl skippar p� sista loopen, pga tids som mainloop tar TEST
			}
		}
	}
	last_height = height;
}

//G�r en halv cykel crabg�ng
void Walk_Half_Crab_Cycle(int8_t speed)// h�ger �r possitivt
{
	
	float l = 12;//f�tters f�rskjuting fr�n kropp i x-led 
	float corner_pitch = 4; //f�rskjutning av arbetsomr�de i y-led f�r h�rnben 
	int sgn_speed = (speed >= 0) - (speed < 0) ;
	uint8_t walk_break = 1;
	float stroke =speed;  //stegl�ngd 
	if(speed * sgn_speed <2)
	{
		return;
	}
	if(speed* sgn_speed > 6)
	{
		speed = 6* sgn_speed;
	}

	if(speed == 0) // g�r inget
	{
		return;
	}
	//justerar h�jd om den �r p� o�nskad niv�
	if(last_height != 11)
	{
		Walk_Half_Cycle(0,0,11,0);
	}
	//g�ngloop, utf�r en halv g�ngfas och stannar n�r st�djande ben �r i mitten av arbetsomr�de
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
		
		n_2= n_1 + cycle_length/2; //barnpar 2 f�rskjuts en halv fas
		
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
	
			kar_p1 = Tripod(l, stroke, last_height, lift, n_1); //kart koord f�r par 1
			kar_p2 = Tripod(l, stroke, last_height, lift, n_2); //kart koord f�r par 2
			
			Send_Leg5_Kar_And_Velocity( kar_p1.a - kar_p1.b, - corner_pitch, kar_p1.c, 100, speed_p1_middle, speed_p1_outer); 	
			Send_Leg3_Kar_And_Velocity( kar_p2.a - kar_p2.b, 0, kar_p2.c, 100, speed_p2_middle, speed_p2_outer); 
			Send_Leg4_Kar_And_Velocity( kar_p1.a +  kar_p1.b, 0, kar_p1.c, 100, speed_p1_middle, speed_p1_outer); 	
			Send_Leg1_Kar_And_Velocity( kar_p1.a -  kar_p1.b, corner_pitch, kar_p1.c, 100, speed_p1_middle, speed_p1_outer); 
			Send_Leg2_Kar_And_Velocity( kar_p2.a + kar_p2.b, corner_pitch, kar_p2.c, 100, speed_p2_middle, speed_p2_outer);
			Send_Leg6_Kar_And_Velocity( kar_p2.a + kar_p2.b, - corner_pitch, kar_p2.c, 100, speed_p2_middle, speed_p2_outer);
			
			if( walk_break || (n_1 != support_l/2 && n_2 != support_l/2))
			{
			_delay_ms(7); 
			}
		}
}

void Dance(float x, float y)
{
	float l = 12;//f�tters f�rskjuting fr�n kropp i x-led OBS orginal = 13, numera 12
	float corner_pitch = 4; //f�rskjutning av arbetsomr�de i y-led f�r h�rnben 4
	int speed_inner_dance= 180;
	int speed_middle_dance= 200;
	int speed_outer_dance  = 200;
	float max_r = 7; 
	float max_step= 0.3; 
	float height_step = 0.5; 
		
	// h�jdjustering
	float height; 
	if(last_height>12)
	{
		height=last_height-height_step; 
	}
	if(last_height<10)
	{
		height=last_height+height_step; 
	}
	else 
	{
		height=11;
	}
	last_height = height; 
		
	// l�gpass 
	float direction_x = x-dance_x; 
	float direction_y = y-dance_y; 
	float diff= sqrt(pow(direction_x,2) + pow(direction_y,2));
	if(diff>max_step)
	{
		dance_x = dance_x + direction_x/diff*max_step;
		dance_y = dance_y + direction_y/diff*max_step;
	}
	else
	{
	dance_x = dance_x + direction_x; 
	dance_y = dance_y + direction_y;
		
	}
		
	//Begr�nsningar i radie;
	dance_r = sqrt(pow(dance_y,2) + pow(dance_x,2));
	if(dance_r>max_r)
	{
		dance_x= dance_x/dance_r*max_r;
		dance_y= dance_y/dance_r*max_r;
	}
		
	Send_Leg1_Kar_And_Velocity(l+dance_x, -dance_y + corner_pitch, -height, speed_inner_dance, speed_middle_dance, speed_outer_dance);
	Send_Leg2_Kar_And_Velocity(l-dance_x, -dance_y + corner_pitch, -height, speed_inner_dance, speed_middle_dance, speed_outer_dance);
	Send_Leg3_Kar_And_Velocity(l+dance_x, -dance_y, -height, speed_inner_dance, speed_middle_dance, speed_outer_dance);
	Send_Leg4_Kar_And_Velocity(l-dance_x, -dance_y, -height, speed_inner_dance, speed_middle_dance, speed_outer_dance);
	Send_Leg5_Kar_And_Velocity(l+dance_x, -dance_y - corner_pitch, -height, speed_inner_dance, speed_middle_dance, speed_outer_dance);
	Send_Leg6_Kar_And_Velocity(l-dance_x, -dance_y - corner_pitch, -height, speed_inner_dance, speed_middle_dance, speed_outer_dance);
	_delay_ms(15);
	
}

void Dance_Cyl(float r, float th)
{
	float x = r *cos(th);
	float y = r * sin(th); 
	Dance(x,y);
}

void Victory_Dance(void)
{
	float pi = 3.14159265359;
	
	//fixa r�relsem�nster 
	for(float i = 1; i< 10; ++i)
	{
		Dance_Cyl(6, 2*pi*0.01);
	}
	for(float i = 1; i< 200; ++i)
	{
		Dance_Cyl(6, 2*pi*0.01*i);
	}
	for(float i = 1; i< 20; ++i)
	{
		Dance(0,6-6*0.05*i);
	}
	for(float i = 1; i< 20; ++i)
	{
		Dance(0,-6*0.05*i);
	}
	for(float i = 1; i< 20; ++i)
	{
		Dance(0,-6+6*0.05*i);
	}
	for(float i = 1; i< 20; ++i)
	{
		Dance(0, 6*0.05*i);
	}
	for(float i = 1; i< 10; ++i)
	{
		Dance_Cyl(0, 2*pi*0.01);
	}
}

