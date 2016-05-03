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

float height;
float obstacle_height;
float x_ground;
float x_obstacle;
float corner_pitch;
float step;
float lift;
uint8_t m;

unsigned int speed_inner;
unsigned int speed_middle;
unsigned int speed_outer;

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
	height = 15;
	obstacle_height = 6.3;
	x_ground = 11;
	x_obstacle = 12;
	corner_pitch = 4;
	step = 5.2;
	lift = 2.5;
	m = 20;

	speed_inner = 0x0040;
	speed_middle = 0x0040;
	speed_outer = 0x0040;
	_delay_ms(1000);
	
	Configure_Servos_Angle_Limit('c'); // Ändra servobegränsningarna
	To_Climbing_Stance(); //Flytta benen till ett utgångsläge lämpligt för klättring
	
	First_Leg('u');
	Second_Leg('u');
	First_Body_Adjust(); //Flytta kroppen lite närmare hindret
	Third_Leg('u');
	Fourth_Leg('u');
	Second_Body_Adjust(); //Flytta kroppen lite närmare hindret
	Fifth_Leg('u');
	Sixth_Leg('u');
	
	To_Default_Stance();
	Configure_Servos_Angle_Limit('r'); //Återgå till normala begränsningar
}

void Walk_Down_Hard()
{
	height = 15-6.3;
	obstacle_height = -6.3;
	x_ground = 12;
	x_obstacle = 11;
	corner_pitch = 4;
	step = 5.2;
	lift = 2.5;
	m = 20;
	
	speed_inner = 0x0040;
	speed_middle = 0x0040;
	speed_outer = 0x0040;
	
	_delay_ms(1000);
	
	Configure_Servos_Angle_Limit('c');
	To_Climbing_Stance();
	
	First_Leg('d');
	Second_Leg('d');
	First_Body_Adjust(); //Flytta kroppen lite närmare hindret
	Third_Leg('d');
	Fourth_Leg('d');
	Second_Body_Adjust(); //Flytta kroppen lite närmare hindret
	Fifth_Leg('d');
	Sixth_Leg('d');
	
	To_Default_Stance();
	Configure_Servos_Angle_Limit('r'); //Återgå till normala begränsningar
	
}

void To_Climbing_Stance()
{
	_delay_ms(1000);
	Send_Leg1_Kar(x_ground, 0+corner_pitch, -height);
	Send_Leg2_Kar(x_ground, 0+corner_pitch, -height);
	Send_Leg3_Kar(x_ground, 0, -height);
	Send_Leg4_Kar(x_ground, 0, -height);
	Send_Leg5_Kar(x_ground, 0-corner_pitch, -height);
	Send_Leg6_Kar(x_ground, 0-corner_pitch, -height);
	
	_delay_ms(8000);
}

void To_Default_Stance()
{
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
}

void First_Leg(char direction)
{
	if(direction == 'u') //Klättra upp, dvs 'u'
	{
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
	}
	
	else //Klättra ner dvs 'd'
	{
		//upp
		Send_Leg1_Kar_And_Velocity(x_obstacle, 0+corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_ground, 0, -(height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_ground, 0-corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(3000);
		//fram
		Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_ground, step, -(height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_ground, step-corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(3000);
	}
	
	
	
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
}

void Second_Leg(char direction)
{
	/////   PAR 2 BEN 2 UPP   /////
	
	if(direction == 'u')//klättra upp dvs 'u'
	{
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
	}
	else
	{
		//upp
		Send_Leg2_Kar_And_Velocity(x_obstacle, -step+corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_ground, -step, -(height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, -step-corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(3000);
		//fram
		Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_ground, step, -(height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(3000);
	}
	
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
}

void First_Body_Adjust()
{
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
}

void Third_Leg(char direction)
{
	if(direction == 'u')
	{
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
	}
	else //Klättra ner dvs 'd'
	{
		//upp
		Send_Leg2_Kar_And_Velocity(x_obstacle, 0+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_obstacle, -step, -(height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, -step-corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(3000);
		//fram
		Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step, -(height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(3000);
	}
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
}

void Fourth_Leg(char direction)
{
	if(direction == 'u')
	{
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
	}
	
	else
	{
		//upp
		Send_Leg1_Kar_And_Velocity(x_obstacle, 0+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_obstacle, -step, -(height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_ground, -step-corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(3000);
		//fram
		Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_obstacle, 2*step, -(height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_ground, step-corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(3000);
	}

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
}

void Second_Body_Adjust()
{
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
}

void Fifth_Leg(char direction)
{
	if(direction == 'u')
	{
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
	}
	else
	{
		//upp
		Send_Leg1_Kar_And_Velocity(x_obstacle, 0+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_obstacle, 0, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_obstacle, -step-corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(3000);
		//fram
		Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_obstacle, 2*step, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_obstacle, 2*step-corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(3000);
	}
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
}

void Sixth_Leg(char direction)
{
	if(direction == 'u')
	{
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
	}
	else
	{
			//upp
			Send_Leg2_Kar_And_Velocity(x_obstacle, 0+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			Send_Leg3_Kar_And_Velocity(x_obstacle, 0, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			Send_Leg6_Kar_And_Velocity(x_obstacle, -step-corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
			_delay_ms(3000);
			//fram
			Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			Send_Leg6_Kar_And_Velocity(x_obstacle, 2*step-corner_pitch, -(height-lift), speed_inner, speed_middle, speed_outer);
			_delay_ms(3000);
	}

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
}