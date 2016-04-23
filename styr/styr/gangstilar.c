/*
 * GANGSTILAR.c
 *
 * Created: 4/8/2016 3:03:52 PM
 *  Author: jenma 421 (Gimmie creed) [chrma018]
 */ 

#include "gangstilar.h"
uint8_t m = 24; //delsekvenser per halv cykel (per st�dfas)
uint8_t n = 12; //index g�ngcykel startv�rde m/2 ska g�ras snyggare
float last_height = 11;
float last_speed = 0;
float last_theta = 0;
float theta_max = 0;


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
	
	if(speed_diff > 4)
	{
		speed = last_speed + 4;
	}
	else if(speed_diff < -4)
	{
		speed = last_speed - 4;
	}
	
	if(theta_diff > 0.3)
	{
		theta = last_theta + 0.3;
	}
	else if(theta_diff < -0.3)
	{
		theta = last_theta - 0.3;
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
	float thlimits[7] = {0.57,0.48,0.38,0.3,0.2,0.11,0};//thetamax f�r olika speeds 0->5//
	
	for(speed = 0; (theta * sgn_theta <= thlimits[speed]) && (speed <= 6); speed++) // rpova parantes och &&
	{
	}
	return speed;
}

//Justerar servospeed efter f�rh�ladne mellan theta o thetamax
void Adjust_Servo_Speed(float theta, int sgn_theta)
{                                                                                    
	//justerar servospeed �NDRA SKALFAKTOR !!
	unsigned int speed_drive =  330 + 550 * (sgn_theta * theta - theta_max);//fast v�rde 0x0100//352 + 53 * 
	unsigned int speed_lift  =  300 + 400 * (sgn_theta * theta - theta_max);//fast v�rde 0x0100 //336 - 10 * //280
 
	//skickar hastighetsbegr�nsningar till servon.
	Send_Inner_P1_Velocity(speed_drive);
	Send_Middle_P1_Velocity(speed_lift);
	Send_Outer_P1_Velocity(speed_lift);
	Send_Inner_P2_Velocity(speed_drive);
	Send_Middle_P2_Velocity(speed_lift);
	Send_Outer_P2_Velocity(speed_lift);
}	

//justerar h�jd stegvis
void Adjust_Height(float l, float height, float height_step, float corner_pitch)
{
	//begr�nsa height h�r om f�r stor/liten
		for(uint8_t i = 0; i < m; i++)
		{
			last_height += height_step;
				
			triple_float kar_p1 = Tripod(l, 0, last_height, n); //kart koord f�r par 1
			triple_float kar_p2 = Tripod(l, 0, last_height, n + m); //kart koord f�r par 2
			
			Send_Legs_Kar(kar_p1, kar_p2, corner_pitch);
			_delay_ms(5); // =5 delay f�r kart
		}
		return;
}

//skickar ut kartesiska koordinater till ben
void Send_Legs_Kar(triple_float kar_p1, triple_float kar_p2, float corner_pitch)
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

//Roterar och skickar ut koordinater till ben ev: fixa s� den lyfter ett av (beror p� rotationsriktning) h�rnbenen extra
void Rotate_And_Send_Legs(triple_float kar_p1, triple_float kar_p2, float corner_pitch, int sgn_speed,float theta)
{			
	//koord f�r par 1 omvandlas till cyl
	triple_float cyl1 = Kar_To_Cyl_Leg1(kar_p1.a, kar_p1.b + corner_pitch, kar_p1.c);
	triple_float cyl4 = Kar_To_Cyl_Leg4(kar_p1.a, kar_p1.b, kar_p1.c);
	triple_float cyl5 = Kar_To_Cyl_Leg5(kar_p1.a, kar_p1.b - corner_pitch, kar_p1.c);
	
	//koord f�r par 2 omvandlas till cyl
	triple_float cyl3 = Kar_To_Cyl_Leg3(kar_p2.a, kar_p2.b, kar_p2.c);
	triple_float cyl2 = Kar_To_Cyl_Leg2(kar_p2.a, kar_p2.b + corner_pitch, kar_p2.c);
	triple_float cyl6 = Kar_To_Cyl_Leg6(kar_p2.a, kar_p2.b - corner_pitch, kar_p2.c);
	
	if(n <= m) //st�dfas par 1, svingfas par 2
	{
		//Stegvis rotation p� p1
		cyl1.b = cyl1.b + sgn_speed * (- theta/2 + n * theta/m);
		cyl4.b = cyl4.b + sgn_speed * (- theta/2 + n * theta/m);
		cyl5.b = cyl5.b + sgn_speed * (- theta/2 + n * theta/m);
		
			//Stegvis rotation p� p2
		cyl3.b = cyl3.b + sgn_speed * ( theta/2 - n * theta/m);
		cyl2.b = cyl2.b + sgn_speed * ( theta/2 - n * theta/m);
		cyl6.b = cyl6.b + sgn_speed * ( theta/2 - n * theta/m);
	}
	else //swingfas par1, st�dfas par 2
	{
			//Stegvis rotation p� p1
		cyl1.b = cyl1.b + sgn_speed * ( theta/2 - (n - m) * theta/m);
		cyl4.b = cyl4.b + sgn_speed * ( theta/2 - (n - m) * theta/m);
		cyl5.b = cyl5.b + sgn_speed * ( theta/2 - (n - m) * theta/m);
		//Stegvis rotation p� p2
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
	if(height < 8)
	{
		height = 8;
	}
	return height;
}

//Begr�nsar theta baserat p� speed "stegl�s" Returnerar max till�tna theta.
double_float Limit_Theta(float speed, int sgn_speed, float theta, int sgn_theta )
{
	speed = speed * sgn_speed;
	int speed_int = speed;
	float speed_dec = speed - speed_int;
	float thlimits[7] = {0.56,0.47,0.37,0.3,0.2,0.11,0};
	float thlin[7] = {-0.09,-0.1,-0.07,-0.1,-0.09,-0.11,0};
	
	//th_max ber�knas med linj�rsering mellan heltal av speed.
	float th_max = thlimits[speed_int] + speed_dec * thlin[speed_int];
	
	if( theta * sgn_theta > th_max )
	{
		theta = sgn_theta * th_max;
	}
	return create_double_float(theta, th_max);
}

//Generar tripod g�ng
triple_float Tripod(float x, float stroke, float height, uint8_t n)
{
	float lift = 2.5; //h�jd som ben lyfts i sving
	float y = 0;
	float z = - height;
	
	if(n > 2 * m) //ser till att det blir cykliskt
	{
		n -= 2 * m;
	}

	if(n <= m)//st�dfas
	{
		y = stroke/2 - stroke * n/m;
		z= - height;
	}
	else if( n <= m + 2)//benlyft i slut av st�dfas, k�rs 2 cykler f�r garantera liftoff
	{
		y = - stroke/2; 
		z = lift - height;
	}
	else if( n <= 3 * m/2) // pausar ben centrerat i svingfas 
	{
		y = 0;
		z = lift - height;
	}
	else if( n <=  2 * m - 4)//f�r ben fram�t mot startpos
	{
		y = stroke/2;
		z = lift - height;
	}
	else if( n <= 2 * m - 1)//s�nker ben n�ra mark 
	{
		y = stroke/2;
		z = 1 - height; 
	}
	else if(n == 2 * m )//s�nk ner ben i marken
	{
		y = stroke/2; 
		z = - height;
	}
	return create_triple_float(x,y,z);
}

// speed = hastighet fram/bak [-6,6], th = sv�ng[-0.57, 0.57], h = h�jd fr�n mark(11) H�JDJUSTERING UR DRIFT
//fixa mjuk �verg�ng f�r h och l(g�r globala) 
void Walk_Half_Cycle(float speed, float theta, float height) 
{
	//justeringar
	//height = 11;//tilf�llig steloperation
	float l = 13; //f�tters f�rskjuting fr�n kropp i x-led
	float corner_pitch = 8; //f�rskjutning av arbetsomr�de i y-led f�r h�rnben
	
	int sgn_speed = (speed >= 0) - (speed < 0) ;
	int sgn_theta = (theta >= 0) - (theta < 0) ;
	uint8_t walk_break = 1;	
	
	//filtrerade o begr�nsade v�rden kommer som globala last_...s�tter th_max och heigth step ocks�.
	height = LP_Filter_And_Limit_Input(speed, sgn_speed, theta, sgn_theta, height); 
	//s�tter parametrar till LP-filtrerade
	speed = last_speed;
	theta = last_theta;
	float height_step = (height - last_height)/m;
	
	if( speed == 0  && theta ==0 && height_step == 0) // g�r inget
	{
		return;
	}

	float stroke =  2.1 * speed; //stegl�ngd 2.2

	Adjust_Servo_Speed(theta, sgn_theta);
	
	if( speed == 0  && theta ==0 ) //justera h�jd utan att g�
	{
		Adjust_Height(l,height, height_step, corner_pitch);
		return;
	}
	
	//g�ngloop, utf�r en halv g�ngfas och stannar n�r st�djande ben �r i mitten av arbetsomr�de
	 while( walk_break || ( n != m/2 && n != 3 * m/2 ))
	{
		walk_break = 0;
		
		//stegvis h�jdjustering
		last_height += height_step;
		triple_float kar_p1 = Tripod(l, stroke, last_height, n); //kart koord f�r par 1
		triple_float kar_p2 = Tripod(l, stroke, last_height, n + m); //kart koord f�r par 2
		
		if(theta == 0) //beh�ver ej g� via cyl koord vid rak g�ng.
		{
			Send_Legs_Kar(kar_p1, kar_p2, corner_pitch);
			if(walk_break || ( n+1 != m/2 && n+1 != 3 * m/2 ))
			{
				_delay_ms(5); // =5 delay f�r kart skippar p� sista loopen, pga tids som mainloop tar TEST
			}
		}
		else //om sv�ng
		{
			Rotate_And_Send_Legs(kar_p1, kar_p2, corner_pitch, sgn_speed, theta);
				if(walk_break || ( n+1 != m/2 && n+1 != 3 * m/2 ))
			{
				_delay_ms(3); // =3 Delay f�r cyl skippar p� sista loopen, pga tids som mainloop tar TEST
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

//test, endast rak g�ng
//Crab people, Crab people, tastes like crab, talks like people.
//g�r ny copy fr�n walk_half_cycle.
void Walk_Half_Crab_Cycle(int speed, float height) 
{
		if( speed == 0  && (height - last_height) == 0) // g�r inget
		{
			return;
		}
		
	//l�gg in begr av max/min h, justera l efter h.
	
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
	float corner_pitch = 8; //f�rskjutning av arbetsomr�de i y-led f�r h�rnben
	float height_step = (height - last_height)/m; //delar upp h�jd�ndring i steg.
	
	triple_float kar_p1;
	triple_float kar_p2;
	
	if(sgn_speed * speed > m_speed ) //hastighetsbegr�nsing ): s�nker speed om f�r h�g, kan �ndras s� att speed 6 alltid = h�gsta realiserbara.
	{
		speed = m_speed * sgn_speed;
	}
	
	//justerar servospeed efter theta/speed INTE KLART
	unsigned int speed_drive = 352 - 53 * (m_speed - sgn_speed * speed); //fast v�rde 0x0100 
	unsigned int speed_lift  = 336 - 10 * (m_speed - sgn_speed * speed);//fast v�rde 0x0100 
	
	//justerar stegl�ngd
	float stroke =  2.2 * speed; 
	
	//skickar hastighetsbegr�nsningar till servon.
	Send_Inner_P1_Velocity(speed_drive);
	Send_Middle_P1_Velocity(speed_lift);
	Send_Outer_P1_Velocity(speed_lift);
	Send_Inner_P2_Velocity(speed_drive);
	Send_Middle_P2_Velocity(speed_lift);
	Send_Outer_P2_Velocity(speed_lift);
	
	//g�ngloop, utf�r en halv g�ngfas och stannar n�r st�djande ben �r i mitten av arbetsomr�de
	 while( walk_break || ( n != m/2 && n != 3 * m/2 ))
	{
		walk_break = 0;
		
		//stegvis h�jdjustering
		last_height += height_step;
		
		kar_p1 = Tripod(l, stroke, last_height, n); //kart koord f�r par 1
		kar_p2 = Tripod(l, stroke, last_height, n + m); //kart koord f�r par 2
		
		//par 1 f�r pos
		Send_Leg4_Kar(kar_p1.a - kar_p1.b, 0, kar_p1.c); 
		Send_Leg1_Kar(kar_p1.a + kar_p1.b, corner_pitch, kar_p1.c);
		Send_Leg5_Kar(kar_p1.a + kar_p1.b, - corner_pitch, kar_p1.c);
			
		//par 2 f�r pos
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
	
	last_height = height; //ny h�jd blir gammal
}
/*tankar xjustering
k�r liknande som height med new och old, andra v�rde i steg n�r benpar �r i svingas, b�da
par �r ju det under en loop :)
*/


// speed = hastighet fram/bak [-6,6], th = sv�ng[-0.57, 0.57], h = h�jd fr�n mark(11)
// l�ngsam version av Walk_Half_Hycle, utf�r samma r�relse, men l�ngsamt.

//OBS h�jdjustering kan ge felaktigt beteende! justera parsamt.
void Walk_Half_Safe_Cycle(int speed, float th, float height) 
{
	if( speed == 0  && th ==0 && (height - last_height) == 0) // g�r inget
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
	uint8_t m = 24; //delsekvenser per halv cykel (per st�dfas)
	uint8_t walk_break = 1;
	float corner_pitch = 8; //f�rskjutning av arbetsomr�de i y-led f�r h�rnben
	float height_step = (height - last_height)/m; //delar upp h�jd�ndring i steg.
	
	triple_float kar_p1;
	triple_float kar_p2;
	triple_float cyl1;
	triple_float cyl2;
	triple_float cyl3;
	triple_float cyl4;
	triple_float cyl5;
	triple_float cyl6;
	
	//test, begr�nsar maxvinkel efter speed ist f�r omv�nt
	float thlimits[7] = {0.57,0.48,0.38,0.3,0.2,0.11,0};
	float th_max = 0;	
	if( th * sgn_theta > (th_max =thlimits[speed * sgn_speed]))
	{
		th = sgn_theta * th_max;
	}
	
	/*
	if(th * sgn_theta > 0.57) // minskar theta om f�r stor
	{
		th=0.57 * sgn_theta;
	}
	  
	if(sgn_speed * speed > m_speed ) //hastighetsbegr�nsing ): s�nker speed om f�r h�g, kan �ndras s� att speed 6 alltid = h�gsta realiserbara.
	{
		speed = m_speed * sgn_speed;
	}
	*/

	//justerar servospeed efter theta/speed INTE KLART
	unsigned int speed_drive = 0x0100;//352- 53 * (m_speed - sgn_speed * speed);//fast v�rde 0x0100 
	unsigned int speed_lift  = 0x0100;//336 - 10 * (m_speed - sgn_speed * speed);//fast v�rde 0x0100 
	
	//justerar stegl�ngd
	float stroke =  2.2 * speed; 
	
	//skickar hastighetsbegr�nsningar till servon.
	Send_Inner_P1_Velocity(speed_drive);
	Send_Middle_P1_Velocity(speed_lift);
	Send_Outer_P1_Velocity(speed_lift);
	Send_Inner_P2_Velocity(speed_drive);
	Send_Middle_P2_Velocity(speed_lift);
	Send_Outer_P2_Velocity(speed_lift);
	
	
	 if( speed == 0  && th ==0 ) //justera h�jd utan att g�	
	 {
		for(uint8_t i = 0; i <= m; i++)
		{
			last_height += height_step;
					
			kar_p1 = Tripod(l, stroke, last_height, n); //kart koord f�r par 1
			kar_p2 = Tripod(l, stroke, last_height, n + m); //kart koord f�r par 2
					
			//par 1 f�r pos
			Send_Leg4_Kar(kar_p1.a, kar_p1.b, kar_p1.c);
			Send_Leg1_Kar(kar_p1.a, kar_p1.b + corner_pitch, kar_p1.c);
			Send_Leg5_Kar(kar_p1.a, kar_p1.b - corner_pitch, kar_p1.c);
					
			//par 2 f�r pos
			Send_Leg3_Kar(kar_p2.a, kar_p2.b, kar_p2.c);
			Send_Leg2_Kar(kar_p2.a, kar_p2.b + corner_pitch, kar_p2.c);
			Send_Leg6_Kar(kar_p2.a, kar_p2.b - corner_pitch, kar_p2.c);
			_delay_ms(50); // =5 delay f�r kart
		}
		return;
	}
			
	//g�ngloop, utf�r en halv g�ngfas och stannar n�r st�djande ben �r i mitten av arbetsomr�de
	 while( walk_break || ( n != m/2 && n != 3 * m/2 ))
	{
		walk_break = 0;
		
		//stegvis h�jdjustering
		last_height += height_step;
		
		kar_p1 = Tripod(l, stroke, last_height, n); //kart koord f�r par 1
		kar_p2 = Tripod(l, stroke, last_height, n + m); //kart koord f�r par 2
		
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
			_delay_ms(50); // =5 vore logiskt med olika delay f�r kart o cyl.
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
				cyl1.b = cyl1.b + sgn_speed * (- th/2 + n * th/m);
				cyl4.b = cyl4.b + sgn_speed * (- th/2 + n * th/m);
				cyl5.b = cyl5.b + sgn_speed * (- th/2 + n * th/m);
				
				//startl�ge f�r P2
				cyl3.b = cyl3.b - sgn_speed * th/2;
				cyl2.b = cyl2.b - sgn_speed * th/2;
				cyl6.b = cyl6.b - sgn_speed * th/2;
			}
			else //swingfas par1, st�dfas par 2
			{
				//rotation till startl�ge P1
				cyl1.b = cyl1.b - sgn_speed * th/2 ;
				cyl4.b = cyl4.b - sgn_speed * th/2 ;
				cyl5.b = cyl5.b - sgn_speed * th/2 ;
				
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
			_delay_ms(48); // =5 vore logiskt med olika delay f�r kart o cyl.
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
	last_height = height; //ny h�jd blir gamal
}
