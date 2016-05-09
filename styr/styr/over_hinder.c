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

#define UP_DELAY 1000
#define FORWARD_DELAY 400
#define DOWN_DELAY 1000
#define BACK_DELAY 50

float height;
float obstacle_height;
float x_ground;
float x_obstacle;
float corner_pitch;
float step;
float last_step; 
float last_last_step; 
float lift;

float weight_adjust; 

unsigned int speed_inner;
unsigned int speed_middle;
unsigned int speed_outer;


void update_step(float new_step)
{
	last_last_step = last_step;
	last_step = step; 
	step = new_step; 
}

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
	height = 14;
	obstacle_height = 6.3;
	x_ground = 12;
	x_obstacle = 12;
	corner_pitch = 4;
	step = 5.2;
	last_step = step; 
	last_last_step = last_step;
	lift = 2.5;
	number_of_steps =20;

	speed_inner = 0x0060;
	speed_middle = 0x085;
	speed_outer = 0x00B0;
	_delay_ms(1000);
	
	Configure_Servos_Angle_Limit('c'); // Ändra servobegränsningarna
	To_Climbing_Stance(); //Flytta benen till ett utgångsläge lämpligt för klättring
	
	First_Leg('u');
	update_step(5.2);
	Second_Leg('u');
	update_step(5.2);
	First_Body_Adjust(); //Flytta kroppen lite närmare hindret
	update_step(5.2);
	Third_Leg('u');
	update_step(5.2);
	Fourth_Leg('u');
	update_step(5.2);
	Second_Body_Adjust(); //Flytta kroppen lite närmare hindret
	update_step(3);
	Fifth_Leg('u');
	update_step(3);
	Sixth_Leg('u');
	
	To_Default_Stance();
	Configure_Servos_Angle_Limit('r'); //Återgå till normala begränsningar
	Adjust_Stance_Climbed('u');
}

void Walk_Down_Hard()
{
	height = 14-6.3;
	obstacle_height = -6.3;
	x_ground = 12;
	x_obstacle = 12;
	corner_pitch = 4;
	step = 5.2;
	last_step = step;
	last_last_step = last_step; 
	lift = 2.5;
	
	number_of_steps =20;

	weight_adjust = 3;
	
	speed_inner = 0x0060;
	speed_middle = 0x0085;
	speed_outer = 0x00B0;
	
	_delay_ms(1000);
	
	Configure_Servos_Angle_Limit('c');
	To_Climbing_Stance();
	
	First_Leg('d');
	update_step(5.2);
	Second_Leg('d');
	update_step(5.2);
	First_Body_Adjust(); //Flytta kroppen lite närmare hindret
	update_step(5.2);
	Third_Leg('d');
	update_step(5.2);
	Fourth_Leg('d');
	update_step(5.2);
	Second_Body_Adjust(); //Flytta kroppen lite närmare hindret
	update_step(5.2);
	Fifth_Leg('d');
	update_step(5.2);
	Sixth_Leg('d');
	
	To_Default_Stance();
	Configure_Servos_Angle_Limit('r'); //Återgå till normala begränsningar
	Adjust_Stance_Climbed('d');
	
}

void To_Climbing_Stance()
{
	//mest bara för hastigheter 
	_delay_ms(300);
	Send_Leg1_Kar_And_Velocity(x_ground, 0+corner_pitch, -height, speed_inner, speed_middle, speed_outer);
	Send_Leg2_Kar_And_Velocity(x_ground, 0+corner_pitch, -height, speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_ground, 0, -height, speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_ground, 0, -height, speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_ground, 0-corner_pitch, -height, speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_ground, 0-corner_pitch, -height, speed_inner, speed_middle, speed_outer);
	
	_delay_ms(500);
	
	
	// Vikt förskjutning
	for (uint8_t n = 0; n<=number_of_steps; ++n)
	{
		Send_Leg1_Kar_And_Velocity(x_ground, 0 -(weight_adjust*n/number_of_steps)+corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_ground, 0 -(weight_adjust*n/number_of_steps), -(height), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_ground, 0 -(weight_adjust*n/number_of_steps)-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
		
		Send_Leg2_Kar_And_Velocity(x_ground, -(weight_adjust*n/number_of_steps)+corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_ground, -(weight_adjust*n/number_of_steps), -(height), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, -(weight_adjust*n/number_of_steps)-corner_pitch, -(height), speed_inner, speed_middle, speed_outer);
		_delay_ms(BACK_DELAY);
	}
	
}


// 
void First_Leg(char direction)
{
	if(direction == 'u') //Klättra upp, dvs 'u'
	{
		//upp
		Send_Leg1_Kar_And_Velocity(x_obstacle, 0+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_ground, 0-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_ground, 0-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(UP_DELAY);
		//fram
		Send_Leg1_Kar_And_Velocity(x_obstacle,step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_ground, step/2-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_ground, step/2-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(FORWARD_DELAY);
		Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_ground, step-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(FORWARD_DELAY);
	}
	
	else //Klättra ner dvs 'd'
	{
		//upp
		Send_Leg1_Kar_And_Velocity(x_obstacle, 0+corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_ground, 0-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_ground, 0-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(UP_DELAY);
		//fram
		Send_Leg1_Kar_And_Velocity(x_obstacle, step+corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_ground, step/2-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_ground, step/2-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(FORWARD_DELAY);
		Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_ground, step-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(FORWARD_DELAY);
	}
	
	
	
	//ner
	Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_ground, step-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
	_delay_ms(DOWN_DELAY);
	
	for (uint8_t n = 0; n<=number_of_steps; ++n)
	{
		Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step-(step*n/number_of_steps)+corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_ground, step -(step*n/number_of_steps)-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_ground, step -(step*n/number_of_steps)-corner_pitch-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
		
		Send_Leg2_Kar_And_Velocity(x_ground, -(step*n/number_of_steps)+corner_pitch-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_ground, -(step*n/number_of_steps)-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, -(step*n/number_of_steps)-corner_pitch-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
		_delay_ms(BACK_DELAY);
	}
	

}

void Second_Leg(char direction)
{
	/////   PAR 2 BEN 2 UPP   /////
	
	if(direction == 'u')//klättra upp dvs 'u'
	{
		//upp
		Send_Leg2_Kar_And_Velocity(x_obstacle, -last_step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_ground, -last_step-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, -last_step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(UP_DELAY);
		//fram
		Send_Leg2_Kar_And_Velocity(x_obstacle, step/2+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_ground, 0-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, 0-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(FORWARD_DELAY);
		Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_ground, step-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(FORWARD_DELAY);
	}
	else
	{
		//upp
		Send_Leg2_Kar_And_Velocity(x_obstacle, -last_step+corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_ground, -last_step-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, -last_step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(UP_DELAY);
		//fram
		Send_Leg2_Kar_And_Velocity(x_obstacle, step/2+corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_ground, 0-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, 0-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(FORWARD_DELAY);
		Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_ground, step-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(FORWARD_DELAY);
	}
	
	//ner
	Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_ground, step-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
	_delay_ms(DOWN_DELAY);
	
	for (uint8_t n = 0; n<=number_of_steps; ++n)
	{
		Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step -(step*n/number_of_steps)+corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_ground, step -(step*n/number_of_steps)-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, step -(step*n/number_of_steps)-corner_pitch-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
		
		Send_Leg1_Kar_And_Velocity(x_obstacle, last_step -(step*n/number_of_steps)+corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_ground, -(step*n/number_of_steps)-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_ground, -(step*n/number_of_steps)-corner_pitch-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
		_delay_ms(BACK_DELAY);
	}
	

}

void First_Body_Adjust()
{
	//upp
	Send_Leg1_Kar_And_Velocity(x_obstacle, last_last_step-last_step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_ground, -last_step-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_ground, -last_step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(UP_DELAY);
	//fram
	Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_ground, step-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(FORWARD_DELAY);
	//ner
	Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_ground, step-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
	_delay_ms(DOWN_DELAY);
	
	for (uint8_t n = 0; n<=number_of_steps; ++n)
	{
		Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step -(step*n/number_of_steps)+corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_ground, step -(step*n/number_of_steps)-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_ground, step -(step*n/number_of_steps)-corner_pitch-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
		
		Send_Leg2_Kar_And_Velocity(x_obstacle, last_step-(step*n/number_of_steps)+corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_ground, -(step*n/number_of_steps)-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, -(step*n/number_of_steps)-corner_pitch-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
		_delay_ms(BACK_DELAY);
	}
	
	
}

void Third_Leg(char direction)
{
	if(direction == 'u')
	{
		//upp
		Send_Leg2_Kar_And_Velocity(x_obstacle, last_last_step-last_step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_obstacle, -last_step-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, -last_step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(UP_DELAY);
		//fram
		Send_Leg2_Kar_And_Velocity(x_obstacle, step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_obstacle, step/2-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, 0-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(FORWARD_DELAY);
		Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(FORWARD_DELAY);
	}
	else //Klättra ner dvs 'd'
	{
		//upp
		Send_Leg2_Kar_And_Velocity(x_obstacle, last_last_step-last_step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_obstacle, -last_step-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, -last_step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(UP_DELAY);
		//fram
		Send_Leg2_Kar_And_Velocity(x_obstacle, step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_obstacle, step/2-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, 0-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(FORWARD_DELAY);
		Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step/2-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(FORWARD_DELAY);
	}
	//ner
	Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
	_delay_ms(DOWN_DELAY);
	
	for (uint8_t n = 0; n<=number_of_steps; ++n)
	{
		Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step -(step*n/number_of_steps)+corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step -(step*n/number_of_steps)-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, step -(step*n/number_of_steps)-corner_pitch-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
		
		Send_Leg1_Kar_And_Velocity(x_obstacle, last_step -(step*n/number_of_steps)+corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_ground, -(step*n/number_of_steps)-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_ground, -(step*n/number_of_steps)-corner_pitch-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
		_delay_ms(BACK_DELAY);
	}
	
}

void Fourth_Leg(char direction)
{
	if(direction == 'u')
	{
		//upp
		Send_Leg1_Kar_And_Velocity(x_obstacle, last_last_step-last_step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_obstacle, -last_step-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_ground, -last_step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(UP_DELAY);
		//fram
		Send_Leg1_Kar_And_Velocity(x_obstacle, step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_obstacle, step/2-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_ground, 0-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(FORWARD_DELAY);
		Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_obstacle, 2*step-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(FORWARD_DELAY);
	}
	
	else
	{
		//upp
		Send_Leg1_Kar_And_Velocity(x_obstacle, last_last_step-last_step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_obstacle, -last_step-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_ground, -last_step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(UP_DELAY);
		//fram
		Send_Leg1_Kar_And_Velocity(x_obstacle, step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_obstacle, step/2-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_ground, 0-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(FORWARD_DELAY);
		Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_obstacle, 2*step-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(FORWARD_DELAY);
	}

	//ner
	Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_obstacle, 2*step-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
	_delay_ms(DOWN_DELAY);

	for (uint8_t n = 0; n<=number_of_steps; ++n)
	{
		Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step -(step*n/number_of_steps)+corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_obstacle, 2*step -(step*n/number_of_steps)-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_ground, step -(step*n/number_of_steps)-corner_pitch-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
		
		Send_Leg2_Kar_And_Velocity(x_obstacle, last_step-(step*n/number_of_steps)+corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_obstacle, last_step-(step*n/number_of_steps)-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, -(step*n/number_of_steps)-corner_pitch-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
		_delay_ms(BACK_DELAY);
	}


}

void Second_Body_Adjust()
{
	/////   PAR 2, INGET BEN UPP   /////
	//upp
	Send_Leg2_Kar_And_Velocity(x_obstacle, last_last_step-last_step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_obstacle, last_last_step-last_step-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_ground, -last_step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(UP_DELAY);
	//fram
	Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(FORWARD_DELAY);
	//ner
	Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
	_delay_ms(DOWN_DELAY);
	
	for (uint8_t n = 0; n<=number_of_steps; ++n)
	{
		Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step -(step*n/number_of_steps)+corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step -(step*n/number_of_steps)-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, step -(step*n/number_of_steps)-corner_pitch-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
		
		Send_Leg1_Kar_And_Velocity(x_obstacle, last_step -(step*n/number_of_steps)+corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_obstacle, last_step-(step*n/number_of_steps)-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_ground, -(step*n/number_of_steps)-corner_pitch-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
		_delay_ms(BACK_DELAY);
	}
	

}

void Fifth_Leg(char direction)
{
	if(direction == 'u')
	{
		//upp
		Send_Leg1_Kar_And_Velocity(x_obstacle, last_last_step-last_step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_obstacle, last_last_step-last_step-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_obstacle, -last_step-corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(UP_DELAY);
		//fram
		Send_Leg1_Kar_And_Velocity(x_obstacle, step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_obstacle, step-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_obstacle, step/2-corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(FORWARD_DELAY);
		Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_obstacle, 2*step-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_obstacle, 2*step-corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(FORWARD_DELAY);
	}
	else
	{
		//upp
		Send_Leg1_Kar_And_Velocity(x_obstacle, last_last_step-last_step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_obstacle, last_last_step-last_step-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_obstacle, -last_step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(UP_DELAY);
		//fram
		Send_Leg1_Kar_And_Velocity(x_obstacle, step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_obstacle, step-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_obstacle, step/2-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(FORWARD_DELAY);
		Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_obstacle, 2*step-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_obstacle, 2*step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(FORWARD_DELAY);
	}
	//ner
	Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_obstacle, 2*step-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_obstacle, 2*step-corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	_delay_ms(DOWN_DELAY);

	for (uint8_t n = 0; n<=number_of_steps; ++n)
	{
		Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step -(step*n/number_of_steps)+corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_obstacle, 2*step -(step*n/number_of_steps)-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_obstacle, 2*step -(step*n/number_of_steps)-corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		
		Send_Leg2_Kar_And_Velocity(x_obstacle, last_step-(step*n/number_of_steps)+corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_obstacle, last_step-(step*n/number_of_steps)-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, -(step*n/number_of_steps)-corner_pitch-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
		_delay_ms(BACK_DELAY);
	}

}

void Sixth_Leg(char direction)
{
	if(direction == 'u')
	{
			//upp
			Send_Leg2_Kar_And_Velocity(x_obstacle, last_last_step-last_step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			Send_Leg3_Kar_And_Velocity(x_obstacle, last_last_step-last_step-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			Send_Leg6_Kar_And_Velocity(x_obstacle, -last_step-corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			_delay_ms(UP_DELAY);
			//fram
			Send_Leg2_Kar_And_Velocity(x_obstacle, step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			Send_Leg3_Kar_And_Velocity(x_obstacle, step-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			Send_Leg6_Kar_And_Velocity(x_obstacle, step/2-corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			_delay_ms(FORWARD_DELAY);
			Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			Send_Leg6_Kar_And_Velocity(x_obstacle, 2*step-corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			_delay_ms(FORWARD_DELAY);
		
	}
	else
	{
			//upp
			Send_Leg2_Kar_And_Velocity(x_obstacle, last_last_step-last_step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			Send_Leg3_Kar_And_Velocity(x_obstacle, last_last_step-last_step-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			Send_Leg6_Kar_And_Velocity(x_obstacle, -last_step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			_delay_ms(UP_DELAY);
			//fram
			Send_Leg2_Kar_And_Velocity(x_obstacle, step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			Send_Leg3_Kar_And_Velocity(x_obstacle, step-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			Send_Leg6_Kar_And_Velocity(x_obstacle, step/2-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			_delay_ms(FORWARD_DELAY);
			Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			Send_Leg6_Kar_And_Velocity(x_obstacle, 2*step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			_delay_ms(FORWARD_DELAY);
	}

	//ner
	Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_obstacle, 2*step-corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	_delay_ms(DOWN_DELAY);
	
	for (uint8_t n = 0; n<=number_of_steps; ++n)
	{
		Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step -(step*n/number_of_steps)+corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step -(step*n/number_of_steps)-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_obstacle, 2*step -(step*n/number_of_steps)-corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		
		Send_Leg1_Kar_And_Velocity(x_obstacle, last_step -(step*n/number_of_steps)+corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_obstacle, last_step-(step*n/number_of_steps)-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_obstacle, last_step-(step*n/number_of_steps)-corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		_delay_ms(BACK_DELAY);
	}
	
}

void To_Default_Stance()
{
	/////   TILL UTGÅNGSLÄGE   /////
	
	
	//upp
	Send_Leg1_Kar_And_Velocity(x_obstacle, last_last_step-last_step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_obstacle, last_last_step-last_step-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_obstacle, last_last_step-last_step-corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(500);

	for (uint8_t n = 0; n<=number_of_steps; ++n)
	{
		
		Send_Leg2_Kar_And_Velocity(x_obstacle, step-weight_adjust -((step-weight_adjust)*n/number_of_steps)+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_obstacle, step-weight_adjust -((step-weight_adjust)*n/number_of_steps), -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_obstacle, step-weight_adjust -((step-weight_adjust)*n/number_of_steps)-corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		
		Send_Leg1_Kar_And_Velocity(x_obstacle, last_last_step-last_step+corner_pitch-weight_adjust + ((-(last_last_step-last_step)+weight_adjust)*n/number_of_steps), -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_obstacle, last_last_step-last_step-weight_adjust +((-(last_last_step-last_step)+weight_adjust)*n/number_of_steps), -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_obstacle, last_last_step-last_step-corner_pitch-weight_adjust +((-(last_last_step-last_step)+weight_adjust)*n/number_of_steps), -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(BACK_DELAY);
		
	}
	
}