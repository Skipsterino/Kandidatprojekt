/*
 * GANGSTILAR.c
 *
 * Created: 4/8/2016 3:03:52 PM
 *  Author: chrma018
 */ 

#include "gangstilar.h"

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
triple_float Tripod(float x, float s, float h,uint8_t speed, uint8_t m, uint8_t n)
{
	int lift = 2; //höjd som ben lyfts i sving
	float y = 0;
	float z = - h;
	
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

void Walk_Cycle(float l, float s,float h, uint8_t speed, float th, uint8_t m)
{
	if(speed > 6 )
	{
		speed = 6;
	}
	m=16;
	float corner_pitch = 8;
	
	triple_float kar_p1;
	triple_float kar_p2;
	
	triple_float cyl1;
	triple_float cyl2;
	triple_float cyl3;
	triple_float cyl4;
	triple_float cyl5;
	triple_float cyl6;
	unsigned int speed_theta = 0x0010 + 0x0010 * speed;
	unsigned int speed_lift = 0x0030 + 0x0030 * speed;
	s=2*speed;
	
	Send_Inner_P1_Velocity(speed_theta);
	Send_Middle_P1_Velocity(speed_lift);
	Send_Outer_P1_Velocity(speed_lift);
	Send_Inner_P2_Velocity(speed_theta);
	Send_Middle_P2_Velocity(speed_lift);
	Send_Outer_P2_Velocity(speed_theta);
	
	for (uint8_t n=1; n <= 2*m; n++)
	{
		
		kar_p1 = Tripod(l,s,h,speed, m,n); //kart koord för par 1
		kar_p2 = Tripod(l,s,h,speed, m,n + m); //kart koord för par 2
		
		
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
		_delay_ms(50); //dumpa?
	}
}







