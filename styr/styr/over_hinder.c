/*
 * over_hinder.c
 *
 * Created: 4/18/2016 1:55:26 PM
 *  Author: chrma018
 */ 

// Höjden på hinder kan ligga globalt...
#include "over_hinder.h"

unsigned int load;
float z;
void Walk_Up(void)
{
	
	//Denna kan man ju testa som probe start. 
	unsigned int contact_load = 0X04a0; //Mät fram denna
	z = 3;
	float probe_step = 0.4; 

	load = 0x000b;
	
	Send_Leg2_Kar(20, 0,0); //För att se att vi lyckats med load-test
	
	Send_Leg6_Kar(20, 0, 3); // upp med ben åt sidan 
	_delay_ms(1000);
	Send_Leg6_Kar(18, 4, 3); // fram med ben 
	_delay_ms(1000); 
	
	
	while(load < contact_load)
	{
		z= z - probe_step;
		_delay_ms(500);
		Send_Leg6_Kar(18, 4, z);
		load = Get_Servo_Load(3); // ID 3 är mittenservot på ben 1 
		//load = load && 0x3FF;
		_delay_ms(500);	
		if( z < -5) // Test om man missar ytan
		{
			z=3;
		}
	}
	
	Send_Leg2_Kar(20, 0,-10); //För att se att vi lyckats med load-test
	_delay_ms(3000);
	
}

void Walk_Up_Hard()
{
	float height = 15;
	float obstacle_height = 6.3;
	float x_ground = 11;
	float x_obstacle = 12;
	float corner_pitch = 4; 
	float step = 5.2;
	float lift = 2.5;
	uint8_t m = 20;
	
	unsigned int speed_inner = 0x0040;
	unsigned int speed_middle = 0x0040;
	unsigned int speed_outer = 0x0040;
	_delay_ms(1000);
	
	Configure_Servos_Angle_Limit('c'); // Ändra servobegränsningarna
	
	_delay_ms(1000);
	Send_Leg1_Kar(x_ground, 0+corner_pitch, -height);
	Send_Leg2_Kar(x_ground, 0+corner_pitch, -height);
	Send_Leg3_Kar(x_ground, 0, -height);
	Send_Leg4_Kar(x_ground, 0, -height);
	Send_Leg5_Kar(x_ground, 0-corner_pitch, -height);
	Send_Leg6_Kar(x_ground, 0-corner_pitch, -height);
	
	_delay_ms(8000);
	
	/////  PAR 1, BEN 1 UPP   /////
	//upp
	Send_Leg1_Kar_And_Velocity(x_obstacle, 0+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_ground, 0, -(height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_ground, 0-corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	//fram
	Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_ground, step, -(height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_ground, step-corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	//ner
	Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_ground, step, -(height), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_ground, step-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	

	for (uint8_t n = 0; n<=m; ++n)
	{
		Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step -(step*n/m)+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_ground, step -(step*n/m), -(height), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_ground, step -(step*n/m)-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
		
		Send_Leg2_Kar_And_Velocity(x_ground, -(step*n/m)+corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_ground, -(step*n/m), -(height), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, -(step*n/m)-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
		_delay_ms(100);
	}
	
	_delay_ms(3000);

	/////   PAR 2 BEN 2 UPP   /////
	//upp
	Send_Leg2_Kar_And_Velocity(x_obstacle, -step+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_ground, -step, -(height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_ground, -step-corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	//fram
	Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_ground, step, -(height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	//ner
	Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_ground, step, -(height), speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
		
	for (uint8_t n = 0; n<=m; ++n)
	{
		Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step -(step*n/m)+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_ground, step -(step*n/m), -(height), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, step -(step*n/m)-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
			
		Send_Leg1_Kar_And_Velocity(x_obstacle, step -(step*n/m)+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_ground, -(step*n/m), -(height), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_ground, -(step*n/m)-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
		_delay_ms(100);
	}
		
	_delay_ms(3000);
	
	/////   PAR 1, INGET BEN UPP   /////
	//upp
	Send_Leg1_Kar_And_Velocity(x_obstacle, 0+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_ground, -step, -(height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_ground, -step-corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	//fram
	Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_ground, step, -(height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_ground, step-corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	//ner
	Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_ground, step, -(height), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_ground, step-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	
	for (uint8_t n = 0; n<=m; ++n)
	{
		Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step -(step*n/m)+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_ground, step -(step*n/m), -(height), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_ground, step -(step*n/m)-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
		
		Send_Leg2_Kar_And_Velocity(x_obstacle, step-(step*n/m)+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_ground, -(step*n/m), -(height), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, -(step*n/m)-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
		_delay_ms(100);
	}
	
	_delay_ms(3000);

	/////   PAR 2, BEN 3 UPP   /////
	//upp
	Send_Leg2_Kar_And_Velocity(x_obstacle, 0+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_obstacle, -step, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_ground, -step-corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	//fram
	Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	//ner
	Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	
	for (uint8_t n = 0; n<=m; ++n)
	{
		Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step -(step*n/m)+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step -(step*n/m), -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, step -(step*n/m)-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
		
		Send_Leg1_Kar_And_Velocity(x_obstacle, step -(step*n/m)+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_ground, -(step*n/m), -(height), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_ground, -(step*n/m)-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
		_delay_ms(100);
	}
	
	_delay_ms(3000);

	/////   PAR 1, BEN 4 UPP   /////
	//upp
	Send_Leg1_Kar_And_Velocity(x_obstacle, 0+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_obstacle, -step, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_ground, -step-corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	//fram
	Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_obstacle, 2*step, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_ground, step-corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	//ner
	Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_obstacle, 2*step, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_ground, step-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);

	for (uint8_t n = 0; n<=m; ++n)
	{
		Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step -(step*n/m)+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_obstacle, 2*step -(step*n/m), -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_ground, step -(step*n/m)-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
	
		Send_Leg2_Kar_And_Velocity(x_obstacle, step-(step*n/m)+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_obstacle, step-(step*n/m), -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, -(step*n/m)-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
		_delay_ms(100);
	}

	_delay_ms(3000);
	
	/////   PAR 2, INGET BEN UPP   /////
	//upp
	Send_Leg2_Kar_And_Velocity(x_obstacle, 0+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_obstacle, 0, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_ground, -step-corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	//fram
	Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	//ner
	Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	
	for (uint8_t n = 0; n<=m; ++n)
	{
		Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step -(step*n/m)+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step -(step*n/m), -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, step -(step*n/m)-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
		
		Send_Leg1_Kar_And_Velocity(x_obstacle, step -(step*n/m)+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_obstacle, step-(step*n/m), -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_ground, -(step*n/m)-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
		_delay_ms(100);
	}
	
	_delay_ms(3000);

	/////   PAR 1, BEN 5 UPP   /////
	//upp
	Send_Leg1_Kar_And_Velocity(x_obstacle, 0+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_obstacle, 0, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_obstacle, -step-corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	//fram
	Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_obstacle, 2*step, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_obstacle, 2*step-corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	//ner
	Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_obstacle, 2*step, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_obstacle, 2*step-corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);

	for (uint8_t n = 0; n<=m; ++n)
	{
		Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step -(step*n/m)+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_obstacle, 2*step -(step*n/m), -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_obstacle, 2*step -(step*n/m)-corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	
		Send_Leg2_Kar_And_Velocity(x_obstacle, step-(step*n/m)+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_obstacle, step-(step*n/m), -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, -(step*n/m)-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
		_delay_ms(100);
	}

	_delay_ms(3000);


	/////   PAR 2, BEN 6 UPP   /////
	//upp
	Send_Leg2_Kar_And_Velocity(x_obstacle, 0+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_obstacle, 0, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_obstacle, -step-corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	//fram
	Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_obstacle, 2*step-corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	//ner
	Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_obstacle, 2*step-corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	
	for (uint8_t n = 0; n<=m; ++n)
	{
		Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step -(step*n/m)+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step -(step*n/m), -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_obstacle, 2*step -(step*n/m)-corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		
		Send_Leg1_Kar_And_Velocity(x_obstacle, step -(step*n/m)+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_obstacle, step-(step*n/m), -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_obstacle, step-(step*n/m)-corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		_delay_ms(100);
	}
	
	_delay_ms(3000);
	
	
	/////   TILL UTGÅNGSLÄGE   /////
	//upp
	Send_Leg1_Kar_And_Velocity(x_obstacle, 0+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_obstacle, 0, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_obstacle, 0-corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	Send_Leg1_Kar_And_Velocity(x_ground, 0+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_ground, 0, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_ground, 0-corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);


	for (uint8_t n = 0; n<=m; ++n)
	{
	
		Send_Leg2_Kar_And_Velocity(x_obstacle, step-(step*n/m)+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_obstacle, step-(step*n/m), -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_obstacle, step -(step*n/m)-corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		_delay_ms(100);
	}
	
	Send_Leg1_Kar_And_Velocity(x_ground, 0+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_ground, 0, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_ground, 0-corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	
	Send_Leg2_Kar_And_Velocity(x_obstacle, 0+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_obstacle, 0, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_obstacle, 0-corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	Send_Leg2_Kar_And_Velocity(x_ground, 0+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_ground, 0, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_ground, 0-corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	Send_Leg2_Kar_And_Velocity(x_ground, 0+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_ground, 0, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_ground, 0-corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	
	_delay_ms(3000);
	
	Configure_Servos_Angle_Limit('r'); //BEHÖVS VÄL FÖR ATT SERVOBEGRÄNSNINGARNA SKA ÅTERGÅ TILL RÄTTA VÄRDEN??????????????????????
}


void Walk_Down_Hard()
{
	float height = 15-6.3;
	float obstacle_height = -6.3;
	float x_ground = 12;
	float x_obstacle = 11;
	float corner_pitch = 4;
	float step = 5.2;
	float lift = 2.5;
	uint8_t m = 20;
	
	unsigned int speed_inner = 0x0040;
	unsigned int speed_middle = 0x0040;
	unsigned int speed_outer = 0x0040;
	_delay_ms(1000);
	
	Configure_Servos_Angle_Limit('c');

	_delay_ms(1000);
	Send_Leg1_Kar(x_ground, 0+corner_pitch, -height);
	Send_Leg2_Kar(x_ground, 0+corner_pitch, -height);
	Send_Leg3_Kar(x_ground, 0, -height);
	Send_Leg4_Kar(x_ground, 0, -height);
	Send_Leg5_Kar(x_ground, 0-corner_pitch, -height);
	Send_Leg6_Kar(x_ground, 0-corner_pitch, -height);
	
	_delay_ms(8000);
	
	/////  PAR 1, BEN 1 UPP   /////
	//upp
	Send_Leg1_Kar_And_Velocity(x_obstacle, 0+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_ground, 0, -(height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_ground, 0-corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	//fram
	Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_ground, step, -(height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_ground, step-corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	//ner
	Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_ground, step, -(height), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_ground, step-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	

	for (uint8_t n = 0; n<=m; ++n)
	{
		Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step -(step*n/m)+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_ground, step -(step*n/m), -(height), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_ground, step -(step*n/m)-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
		
		Send_Leg2_Kar_And_Velocity(x_ground, -(step*n/m)+corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_ground, -(step*n/m), -(height), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, -(step*n/m)-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
		_delay_ms(100);
	}
	
	_delay_ms(3000);

	/////   PAR 2 BEN 2 UPP   /////
	//upp
	Send_Leg2_Kar_And_Velocity(x_obstacle, -step+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_ground, -step, -(height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_ground, -step-corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	//fram
	Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_ground, step, -(height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	//ner
	Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_ground, step, -(height), speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	
	for (uint8_t n = 0; n<=m; ++n)
	{
		Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step -(step*n/m)+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_ground, step -(step*n/m), -(height), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, step -(step*n/m)-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
		
		Send_Leg1_Kar_And_Velocity(x_obstacle, step -(step*n/m)+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_ground, -(step*n/m), -(height), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_ground, -(step*n/m)-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
		_delay_ms(100);
	}
	
	_delay_ms(3000);
	
	/////   PAR 1, INGET BEN UPP   /////
	//upp
	Send_Leg1_Kar_And_Velocity(x_obstacle, 0+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_ground, -step, -(height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_ground, -step-corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	//fram
	Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_ground, step, -(height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_ground, step-corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	//ner
	Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_ground, step, -(height), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_ground, step-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	
	for (uint8_t n = 0; n<=m; ++n)
	{
		Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step -(step*n/m)+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_ground, step -(step*n/m), -(height), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_ground, step -(step*n/m)-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
		
		Send_Leg2_Kar_And_Velocity(x_obstacle, step-(step*n/m)+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_ground, -(step*n/m), -(height), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, -(step*n/m)-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
		_delay_ms(100);
	}
	
	_delay_ms(3000);

	/////   PAR 2, BEN 3 UPP   /////
	//upp
	Send_Leg2_Kar_And_Velocity(x_obstacle, 0+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_obstacle, -step, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_ground, -step-corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	//fram
	Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	//ner
	Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	
	for (uint8_t n = 0; n<=m; ++n)
	{
		Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step -(step*n/m)+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step -(step*n/m), -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, step -(step*n/m)-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
		
		Send_Leg1_Kar_And_Velocity(x_obstacle, step -(step*n/m)+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_ground, -(step*n/m), -(height), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_ground, -(step*n/m)-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
		_delay_ms(100);
	}
	
	_delay_ms(3000);

	/////   PAR 1, BEN 4 UPP   /////
	//upp
	Send_Leg1_Kar_And_Velocity(x_obstacle, 0+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_obstacle, -step, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_ground, -step-corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	//fram
	Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_obstacle, 2*step, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_ground, step-corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	//ner
	Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_obstacle, 2*step, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_ground, step-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);

	for (uint8_t n = 0; n<=m; ++n)
	{
		Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step -(step*n/m)+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_obstacle, 2*step -(step*n/m), -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_ground, step -(step*n/m)-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
		
		Send_Leg2_Kar_And_Velocity(x_obstacle, step-(step*n/m)+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_obstacle, step-(step*n/m), -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, -(step*n/m)-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
		_delay_ms(100);
	}

	_delay_ms(3000);
	
	/////   PAR 2, INGET BEN UPP   /////
	//upp
	Send_Leg2_Kar_And_Velocity(x_obstacle, 0+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_obstacle, 0, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_ground, -step-corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	//fram
	Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	//ner
	Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	
	for (uint8_t n = 0; n<=m; ++n)
	{
		Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step -(step*n/m)+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step -(step*n/m), -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, step -(step*n/m)-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
		
		Send_Leg1_Kar_And_Velocity(x_obstacle, step -(step*n/m)+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_obstacle, step-(step*n/m), -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_ground, -(step*n/m)-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
		_delay_ms(100);
	}
	
	_delay_ms(3000);

	/////   PAR 1, BEN 5 UPP   /////
	//upp
	Send_Leg1_Kar_And_Velocity(x_obstacle, 0+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_obstacle, 0, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_obstacle, -step-corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	//fram
	Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_obstacle, 2*step, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_obstacle, 2*step-corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	//ner
	Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_obstacle, 2*step, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_obstacle, 2*step-corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);

	for (uint8_t n = 0; n<=m; ++n)
	{
		Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step -(step*n/m)+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_obstacle, 2*step -(step*n/m), -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_obstacle, 2*step -(step*n/m)-corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		
		Send_Leg2_Kar_And_Velocity(x_obstacle, step-(step*n/m)+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_obstacle, step-(step*n/m), -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, -(step*n/m)-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
		_delay_ms(100);
	}

	_delay_ms(3000);


	/////   PAR 2, BEN 6 UPP   /////
	//upp
	Send_Leg2_Kar_And_Velocity(x_obstacle, 0+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_obstacle, 0, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_obstacle, -step-corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	//fram
	Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_obstacle, 2*step-corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	//ner
	Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_obstacle, 2*step-corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	
	for (uint8_t n = 0; n<=m; ++n)
	{
		Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step -(step*n/m)+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step -(step*n/m), -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_obstacle, 2*step -(step*n/m)-corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		
		Send_Leg1_Kar_And_Velocity(x_obstacle, step -(step*n/m)+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_obstacle, step-(step*n/m), -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_obstacle, step-(step*n/m)-corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		_delay_ms(100);
	}
	
	_delay_ms(3000);
	
	
	/////   TILL UTGÅNGSLÄGE   /////
	//upp
	Send_Leg1_Kar_And_Velocity(x_obstacle, 0+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_obstacle, 0, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_obstacle, 0-corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	Send_Leg1_Kar_And_Velocity(x_ground, 0+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_ground, 0, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_ground, 0-corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);


	for (uint8_t n = 0; n<=m; ++n)
	{
		
		Send_Leg2_Kar_And_Velocity(x_obstacle, step-(step*n/m)+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_obstacle, step-(step*n/m), -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_obstacle, step -(step*n/m)-corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		_delay_ms(100);
	}
	
	Send_Leg1_Kar_And_Velocity(x_ground, 0+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_ground, 0, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_ground, 0-corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	
	Send_Leg2_Kar_And_Velocity(x_obstacle, 0+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_obstacle, 0, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_obstacle, 0-corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	Send_Leg2_Kar_And_Velocity(x_ground, 0+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_ground, 0, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_ground, 0-corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	Send_Leg2_Kar_And_Velocity(x_ground, 0+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_ground, 0, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_ground, 0-corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	_delay_ms(3000);
	
	_delay_ms(3000);

	Configure_Servos_Angle_Limit('r'); //BEHÖVS VÄL FÖR ATT SERVOBEGRÄNSNINGARNA SKA ÅTERGÅ TILL RÄTTA VÄRDEN??????????????????????
}