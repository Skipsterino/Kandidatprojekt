/*
 * GANGSTILAR.c
 *
 * Created: 4/8/2016 3:03:52 PM
 *  Author: jenma 421 (Gimmie creed) [chrma018]
 */ 

#include "gangstilar.h"
uint8_t m = 24; //l'ng st;dfas 24
uint8_t swing_l = 12; //l�ngd p� svingfas 12
uint8_t cycle_length = 36;

int n = 12; //index f�r benpar
int n_2 = 30;

float last_height = 11;
float last_speed = 0;
float last_theta = 0;
float theta_max = 0;

triple_float servofuck_adjust_p1;
triple_float servofuck_adjust_p2;
float sideways_fuck;
//unsigned int speed_inner;;
//unsigned int speed_middle;
//unsigned int speed_outer;
int8_t p1_down = 0;
int8_t p2_down = 1;

void init_fuck()
{
	sideways_fuck = 0;
	servofuck_adjust_p1 = create_triple_float(0.8 + sideways_fuck,  sideways_fuck, -0.3 + sideways_fuck);
	servofuck_adjust_p2 = create_triple_float(0.2, -0.15, -0.26);
}

	
	
//LP-filtrerar input f�r att undvika v�ldsamheter Anropa limit_theta o limit_speed h�r, d�p till LP_And_Limit_output, o begr�nsa h�jd. S�tter �ven theta_max
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
			

//ger max m�jlig speed f�r inmatat theta. Anv�nds ej
int max_speed(float theta, int sgn_theta)
{
	int speed=0;
	float thlimits[7] = {0.57,0.47,0.37,0.3,0.2,0.11,0};//thetamax f�r olika speeds 0->5//
	
	for(speed = 0; (theta * sgn_theta <= thlimits[speed]) && (speed <= 6); speed++) // rpova parantes och &&
	{
	}
	return speed;
}

//Justerar servospeed efter f�rh�ladne mellan theta o thetamax
triple_float Adjust_Servo_Speed(float theta, int sgn_theta, int8_t leg_down)
{    
	int speed_inner ;
	int speed_middle;
	int speed_outer;
	
	if(leg_down == 1)
			{
	speed_inner =  180 + 200 * (sgn_theta * theta - theta_max);// 250 + 500 * 
	speed_middle  = 90; 
	speed_outer  =  90;
			}     
			else if(theta*sgn_theta <=0.20)
			{                                                                   
	//justerar servospeed �NDRA SKALFAKTOR !!
	 speed_inner =  180 + 200 * (sgn_theta * theta - theta_max);// 250 + 500 * 
	 speed_middle  = 350 + 340 * (sgn_theta * theta - theta_max);//320 + 430 *//220
	 speed_outer  =  250 + 300 * (sgn_theta * theta - theta_max);//320 + 430 *//250
			}
		else
		{
	 speed_inner =  70 + 160 * (sgn_theta * theta - theta_max);// 250 + 500 * 
	 speed_middle  = 280 + 340 * (sgn_theta * theta - theta_max);//320 + 430 *//220
	 speed_outer  =  240 + 300 * (sgn_theta * theta - theta_max);//320 + 430 *//250
		}	
		
 return create_triple_float(speed_inner, speed_middle, speed_outer);
}	

//justerar h�jd stegvis, BORDE JUSTERA SERVOSPEED SPeedat upp
void Adjust_Height(float l, float height_step, float corner_pitch)
{
	//begr�nsa height h�r om f�r stor/liten
	for(uint8_t i = 0; i < cycle_length/2; i++)
	{
		last_height += height_step;
		
		//triple_float kar_p1 = Tripod(l, 0, last_height, n); //kart koord f�r par 1
		//triple_float kar_p2 = Tripod(l, 0, last_height, n_2); //kart koord f�r par 2  
		
		//Send_Legs_Kar(kar_p1, kar_p2, corner_pitch, , p2_down);
		_delay_ms(5); // =5 delay f�r kart provar mer speed
	}
	return;
}

//skickar ut kartesiska koordinater till ben, trycker ner ensamt s�den extra (anpassad f�r tripod) //testar att ha mindre x p� h�rnben
void Send_Legs_Kar(triple_float kar_p1, triple_float kar_p2, float corner_pitch, triple_float speed_p1, triple_float speed_p2) 
{
	Send_Leg5_Kar_And_Velocity(kar_p1.a, kar_p1.b - corner_pitch, kar_p1.c + servofuck_adjust_p1.c, speed_p1.a, speed_p1.b, speed_p1.c); 	
	Send_Leg3_Kar_And_Velocity(kar_p2.a, kar_p2.b, kar_p2.c + servofuck_adjust_p2.a, speed_p2.a, speed_p2.b, speed_p2.c); 
	Send_Leg4_Kar_And_Velocity(kar_p1.a, kar_p1.b, kar_p1.c + servofuck_adjust_p1.a, speed_p1.a, speed_p1.b, speed_p1.c); 	
	Send_Leg1_Kar_And_Velocity(kar_p1.a, kar_p1.b + corner_pitch, kar_p1.c + servofuck_adjust_p1.b, speed_p1.a, speed_p1.b, speed_p1.c); 
	Send_Leg2_Kar_And_Velocity(kar_p2.a, kar_p2.b + corner_pitch, kar_p2.c + servofuck_adjust_p2.b, speed_p2.a, speed_p2.b, speed_p2.c);
	Send_Leg6_Kar_And_Velocity(kar_p2.a, kar_p2.b - corner_pitch, kar_p2.c + servofuck_adjust_p2.c, speed_p2.a, speed_p2.b, speed_p2.c);
}

//Roterar och skickar ut koordinater till ben ev: fixa s� den lyfter ett av (beror p� rotationsriktning) h�rnbenen extra
void Rotate_And_Send_Legs(triple_float kar_p1, triple_float kar_p2, float corner_pitch, int sgn_speed, float theta, triple_float speed_p1, triple_float speed_p2)
{			
	//koord f�r par 1 omvandlas till cyl
	triple_float cyl4 = Kar_To_Cyl_Leg4(kar_p1.a, kar_p1.b, kar_p1.c + servofuck_adjust_p1.a); 
	triple_float cyl1 = Kar_To_Cyl_Leg1(kar_p1.a , kar_p1.b + corner_pitch, kar_p1.c + servofuck_adjust_p1.b); //testar att ha mindre x p� h�rnben
	triple_float cyl5 = Kar_To_Cyl_Leg5(kar_p1.a , kar_p1.b - corner_pitch, kar_p1.c + servofuck_adjust_p1.c);
	
	//koord f�r par 2 omvandlas till cyl
	triple_float cyl3 = Kar_To_Cyl_Leg3(kar_p2.a, kar_p2.b, kar_p2.c + servofuck_adjust_p2.a); 
	triple_float cyl2 = Kar_To_Cyl_Leg2(kar_p2.a  , kar_p2.b + corner_pitch, kar_p2.c + servofuck_adjust_p2.b);
	triple_float cyl6 = Kar_To_Cyl_Leg6(kar_p2.a , kar_p2.b - corner_pitch, kar_p2.c + servofuck_adjust_p2.c);
	
	if(n <= m) 
	{
		//Stegvis rotation p� p1 st�d
		cyl1.b = cyl1.b + sgn_speed * (- theta/2 + n * theta/m);
		cyl4.b = cyl4.b + sgn_speed * (- theta/2 + n * theta/m);
		cyl5.b = cyl5.b + sgn_speed * (- theta/2 + n * theta/m);
	}
	else 
	{
			//Stegvis rotation p� p1 sving
		cyl1.b = cyl1.b + sgn_speed * ( theta/2 - (n - m) * theta/swing_l);
		cyl4.b = cyl4.b + sgn_speed * ( theta/2 - (n - m) * theta/swing_l);
		cyl5.b = cyl5.b + sgn_speed * ( theta/2 - (n - m) * theta/swing_l);  
	}
	
	if(n_2 <= m) 
	{
			//Stegvis rotation p� p2 st�d
			cyl3.b = cyl3.b + sgn_speed * (- theta/2 + n_2 * theta/m);
			cyl2.b = cyl2.b + sgn_speed * (- theta/2 + n_2 * theta/m);
			cyl6.b = cyl6.b + sgn_speed * (- theta/2 + n_2 * theta/m);
	}
	else
	{
			//Stegvis rotation p� p2 sving
			cyl3.b = cyl3.b + sgn_speed * ( theta/2 - (n_2 - m) * theta/swing_l);
			cyl2.b = cyl2.b + sgn_speed * ( theta/2 - (n_2 - m) * theta/swing_l);
			cyl6.b = cyl6.b + sgn_speed * ( theta/2 - (n_2 - m) * theta/swing_l);
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
float Limit_Speed(float speed, int sgn_speed)
{
	if(speed * sgn_speed > 6)  //utf�rs ber�kninge som float?
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
	if(height < 7)
	{
		height = 7;
	}
	return height;
}

//Begr�nsar theta baserat p� speed "stegl�s" Returnerar max till�tna theta.
double_float Limit_Theta(float speed, int sgn_speed, float theta, int sgn_theta )
{
	speed = speed * sgn_speed;
	int speed_int = speed;
	float speed_dec = speed - speed_int;
	float thlimits[7] = {0.33,0.33,0.30,0.3,0.2,0.11,0};//{0.56,0.46,0.36,0.3,0.2,0.11,0};
	float thlin[7] =    {0.0,0.03, 0,-0.1,-0.09,-0.11,0};
	
	//th_max ber�knas med linj�rsering mellan heltal av speed.
	float th_max = thlimits[speed_int] + speed_dec * thlin[speed_int];
	
	if( theta * sgn_theta > th_max )
	{
		theta = sgn_theta * th_max;
	}
	return create_double_float(theta, th_max);
}


//Generar tripod g�ng testar bwta >� 16 steg svingfas
triple_float Tripod(float x, float stroke, float height,float lift, uint8_t n)
{
	
	float y = 0;
	float z = 0;
	
	if(n <= 2 ) //delat upp bens�nkingen i steg
	{
		y = stroke/2 - stroke * n/m;
		z= - height;//lift/2 - n * lift/4 - height;
	}
	else if(n <= m-1)//st�dfas
	{
		y = stroke/2 - stroke * n/m;
		z= - height;
	}
	else if( n <= m)//b�rjar lyfta ben med fortsatt benr�relse bak�t
	{
		y = stroke/2 - stroke * n/m;
		z =   lift - height;
	}
	else if( n <= m +swing_l - 1) // f�r ben frammm�t mot startpos
	{
		y = -stroke/2 + (n-m) * stroke/swing_l;
		z = lift - height;
	}
	else if( n <= m +swing_l) // f�r ben frammm�t mot startpos
	{
		y = -stroke/2 + (n-m) * stroke/swing_l;
		z=  lift/2 - height;  // + ((m + swing_l) - n) * lift/4 - height;
	}
	
	return create_triple_float(x,y,z);
}




//testar �verlappande svingfas, strypt o ingen h�j/s�nk
void Walk_Half_Cycle(float speed, float theta, float height)
{
	//theta = 0;
	//height = 11;//7 l�g 11 normal 14 h�g
	//justeringar

	float l = 12;//13 l�g //f�tters f�rskjuting fr�n kropp i x-led OBS orginal = 13, numera 12
	float corner_pitch = 4; //f�rskjutning av arbetsomr�de i y-led f�r h�rnben 8
	
	int sgn_speed = (speed >= 0) - (speed < 0) ;
	int sgn_theta = (theta >= 0) - (theta < 0) ;
	uint8_t walk_break = 1;
	
	//filtrerade o begr�nsade v�rden kommer som globala last_...s�tter th_max och heigth step ocks�.
	height = LP_Filter_And_Limit_Input(speed, sgn_speed, theta, sgn_theta, height);
	//s�tter parametrar till LP-filtrerade
	speed = last_speed;
	theta = last_theta;
	float height_step = 2 * (height - last_height)/cycle_length;
	
	if( (speed*sgn_speed < 0.2 ) && (theta*sgn_theta < 0.01) && (height_step == 0)) // g�r inget
	{
		return;
	}

	float stroke =  1.8 * speed; //stegl�ngd 2.2


	if( speed == 0  && theta ==0 ) //justera h�jd utan att g�
	{
		Adjust_Height(l, height_step, corner_pitch);
		return;
	}
	
	//g�ngloop, utf�r en halv g�ngfas och stannar n�r st�djande ben �r i mitten av arbetsomr�de
	while( walk_break || ( n  != m/2 && n  !=  m/2 + (m + swing_l)/2))
	{
		walk_break = 0;
			if(n < m + swing_l)
		{
			n += 1; //stegar upp index
		}
		else
		{
			n = 1; //nollar index
		}
		
		//stegvis h�jdjustering
		last_height += height_step;
		
		n_2= n+(m + swing_l)/2;
		
		if(n_2 > m + swing_l) //ser till att det blir cykliskt
		{
			n_2 -= (m + swing_l);
		}
		
		if(n <= 4 )
		{
			p1_down =1;
		}
		else
		{
			p1_down = 0;
		}
				
		if((n_2<= 4))
		{
			p2_down =1;
		}
		else
		{
			p2_down = 0;
		}
		
		triple_float kar_p1;
		triple_float kar_p2;
		triple_float speed_p1 = Adjust_Servo_Speed(theta, theta_max ,p1_down);
		triple_float speed_p2 = Adjust_Servo_Speed(theta, theta_max ,p2_down);
		float lift =1 + sgn_speed*speed/6 ;
		
		if(theta*sgn_theta < 0.01) //beh�ver ej g� via cyl koord vid rak g�ng.
		{
			kar_p1 = Tripod(l, stroke, last_height, lift, n); //kart koord f�r par 1
			kar_p2 = Tripod(l, stroke, last_height, lift, n_2); //kart koord f�r par 2
			
			Send_Legs_Kar(kar_p1, kar_p2, corner_pitch, speed_p1, speed_p2);
			
			if( walk_break || ( n  != m/2 && n  !=  m/2 + (m + swing_l)/2))
			{
			_delay_ms(13); // =5 delay f�r kart skippar p� sista loopen, pga tids som mainloop tar TEST
			}
		}
		else //om sv�ng
		{
			lift = 1.5 + theta * sgn_theta *1.5;
			kar_p1 = Tripod(l, stroke, last_height, lift, n); //kart koord f�r par 1
			kar_p2 = Tripod(l, stroke, last_height, lift, n_2); //kart koord f�r par 2
			
			Rotate_And_Send_Legs(kar_p1, kar_p2, corner_pitch, sgn_speed, theta, speed_p1, speed_p2);
			
			if( walk_break || ( n  != m/2 && n  !=  m/2 + (m + swing_l)/2))
			{
			_delay_ms(11); // =3 Delay f�r cyl skippar p� sista loopen, pga tids som mainloop tar TEST
			}
					
		}
	}
}


void Walk_Half_Crab_Cycle(float speed) 
{
	return;
	if( speed == 0 ) // g�r inget
	{
		return;
	}
	

	//justeringar
	float l = 13; //f�tters f�rskjuting fr�n kropp i x-led
	float corner_pitch = 8; //f�rskjutning av arbetsomr�de i y-led f�r h�rnben
	
	int sgn_speed = (speed >= 0) - (speed < 0) ;
	uint8_t walk_break = 1;	
	speed = Limit_Speed(speed, sgn_speed);
	
	float stroke = speed * 0.8; //stegl�ngd 
	unsigned int speed_drive =  150;
	unsigned int speed_lift  =  250 + 30 * (sgn_speed * speed - 6);//fast v�rde 0x0100 //336 - 10 * //280
 
	//skickar hastighetsbegr�nsningar till servon.
	Send_Inner_P1_Velocity(speed_drive);
	Send_Middle_P1_Velocity(speed_lift);
	Send_Outer_P1_Velocity(speed_lift);
	Send_Inner_P2_Velocity(speed_drive);
	Send_Middle_P2_Velocity(speed_lift);
	Send_Outer_P2_Velocity(speed_lift);
	
	
		if( last_height != 11)
	{
		Adjust_Height(l, (11-last_height)/m, corner_pitch);
	}
	
	//g�ngloop, utf�r en halv g�ngfas och stannar n�r st�djande ben �r i mitten av arbetsomr�de
	 while( walk_break || ( n != m/2 && n != 3 * m/2 ))
	{
		walk_break = 0;
		
		triple_float kar_p1 = Tripod(l, stroke, last_height,2, n); //kart koord f�r par 1
		triple_float kar_p2 = Tripod(l, stroke, last_height,2, n + m); //kart koord f�r par 2
		
		//par 1 f�r pos
		Send_Leg4_Kar(kar_p1.a - kar_p1.b, 0, kar_p1.c); 
		Send_Leg1_Kar(kar_p1.a + kar_p1.b, corner_pitch, kar_p1.c);
		Send_Leg5_Kar(kar_p1.a + kar_p1.b, - corner_pitch, kar_p1.c);
			
		//par 2 f�r pos
		Send_Leg3_Kar(kar_p2.a + kar_p2.b, 0, kar_p2.c); 
		Send_Leg2_Kar(kar_p2.a - kar_p2.b, corner_pitch, kar_p2.c);
		Send_Leg6_Kar(kar_p2.a - kar_p2.b, - corner_pitch, kar_p2.c);
		
		if(walk_break || ( n+1 != m/2 && n+1 != 3 * m/2 ))
		{
			_delay_ms(10); // =5 delay f�r kart skippar p� sista loopen, pga tids som mainloop tar TEST
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
