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

#define UP_DELAY 600
#define FORWARD_DELAY 600
#define DOWN_DELAY 700
#define BACK_DELAY 15

float height;
float obstacle_height;
float x_ground;
float x_obstacle;
float corner_pitch;
float step;
float last_step; 
float last_last_step; 
float lift;
uint8_t adjusted; 
float step_adjust;

float weight_adjust; 

unsigned int speed_inner;
unsigned int speed_middle;
unsigned int speed_outer;


uint8_t Servo_Load_Too_Small(unsigned char ID)
{
	unsigned int load = Get_Servo_Load(ID);
	if(((ID == 3) || (ID == 9) || (ID == 15)))
	{
		if(load > 0x0405)
		{
			return 0;
		}
		else
		{
			return 1;
		}
	}
	else // dvs ID = 4, 10, 16...
	{
		if((load > 0x0028) && (load < 0x0400))
		{
			return 0;
		}
		else
		{
			return 1;
		}
	}
}


void update_step(float new_step)
{
	last_last_step = last_step;
	last_step = step; 
	step = new_step; 

}


	
void Walk_Up()
{
	height = 14;
	obstacle_height = 6.3;
	x_ground = 12;
	x_obstacle = 12;
	corner_pitch = 4;
	step = 5;
	last_step = step; 
	last_last_step = last_step;
	lift = 3;
	number_of_steps =20;
	adjusted =0;
	weight_adjust = 3;

	speed_inner = 150;
	speed_middle = 300;
	speed_outer = 300;
	step_adjust = 5;
	
	
	Configure_Servos_Angle_Limit('c'); // Ändra servobegränsningarna
	To_Climbing_Stance(); //Flytta benen till ett utgångsläge lämpligt för klättring
	First_Leg('u');
	update_step(4);
	Second_Leg('u');
	update_step(4);
	First_Body_Adjust(); //Flytta kroppen lite närmare hindret
	update_step(6);
	Third_Leg('u');
	update_step(4);
	Fourth_Leg('u');
	update_step(6);
	Second_Body_Adjust(); //Flytta kroppen lite närmare hindret
	update_step(6.5);
	Fifth_Leg('u');
	update_step(4);
	Sixth_Leg('u');
	
	To_Default_Stance();
	Configure_Servos_Angle_Limit('r'); //Återgå till normala begränsningar
	Configure_Servos_Angle_Limit('r'); //Återgå till normala begränsningar bara vi failar med ett servo blir det tråkigt, så vi kör det väll två gånger 
	Adjust_Stance_Climbed('u');
}

void Walk_Down()
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
	step_adjust = 2; 
	
	number_of_steps =20;
	adjusted =0;
	weight_adjust = 3;
	
	speed_inner = 150;
	speed_middle = 300;
	speed_outer = 300;
	
	
	
	Configure_Servos_Angle_Limit('c');
	To_Climbing_Stance();
	
	First_Leg('d');
	update_step(5.2);
	Second_Leg('d');
	update_step(4);
	First_Body_Adjust(); //Flytta kroppen lite närmare hindret
	update_step(6);
	Third_Leg('d');
	update_step(5.2);
	Fourth_Leg('d');
	update_step(4);
	Second_Body_Adjust(); //Flytta kroppen lite närmare hindret
	update_step(6);
	Fifth_Leg('d');
	update_step(5.2);
	Sixth_Leg('d');
	
	To_Default_Stance();
	Configure_Servos_Angle_Limit('r'); //Återgå till normala begränsningar 
	Configure_Servos_Angle_Limit('r'); //Återgå till normala begränsningar bara vi failar med ett servo blir det tråkigt, så vi kör det väll två gånger 
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
	
	
	// Tyngdpunktsförskjutning
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
		Send_Leg1_Kar_And_Velocity(x_obstacle, step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
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
	
	if(Servo_Load_Too_Small(10))
	{
		
		if(direction == 'u') //Klättra upp, dvs 'u'
		{
			
			//upp gamla
			Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			Send_Leg4_Kar_And_Velocity(x_ground, step-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			Send_Leg5_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			_delay_ms(FORWARD_DELAY);
			//uppdatera step
			step=step+1;
			//fram nya
			Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			Send_Leg4_Kar_And_Velocity(x_ground, step-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			Send_Leg5_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			_delay_ms(FORWARD_DELAY);
			//ner
			Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
			Send_Leg4_Kar_And_Velocity(x_ground, step-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
			Send_Leg5_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
			_delay_ms(DOWN_DELAY);		
		}
		
		else //Klättra ner dvs 'd'
		{
			////upp gamla
			//Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			//Send_Leg4_Kar_And_Velocity(x_ground, step-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			//Send_Leg5_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			//_delay_ms(FORWARD_DELAY);
			////uppdatera step
			//step = step + 1 
			//// fram nya 
			//Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			//Send_Leg4_Kar_And_Velocity(x_ground, step-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			//Send_Leg5_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			//_delay_ms(FORWARD_DELAY);
			////ner 
			//Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
			//Send_Leg4_Kar_And_Velocity(x_ground, step-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
			//Send_Leg5_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
			//_delay_ms(DOWN_DELAY);
		}
		
	}
	
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
	
	//if(Servo_Load_Too_Small(8))
	//{
		//
		//if(direction == 'u') //Klättra upp, dvs 'u'
		//{
			//
			////upp gamla
			//Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			//Send_Leg3_Kar_And_Velocity(x_ground, step-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			//Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			//_delay_ms(FORWARD_DELAY);
			////uppdatera step
			//step=step+1;
			////fram nya
			//Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			//Send_Leg3_Kar_And_Velocity(x_ground, step-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			//Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			//_delay_ms(FORWARD_DELAY);
			////ner
			//Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
			//Send_Leg3_Kar_And_Velocity(x_ground, step-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
			//Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
			//_delay_ms(DOWN_DELAY);
		//}
		//
		//else //Klättra ner dvs 'd'
		//{
			//////upp gamla
			////Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			////Send_Leg3_Kar_And_Velocity(x_ground, step-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			////Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			////_delay_ms(FORWARD_DELAY);
			//////uppdatera step
			////step = step + 1
			////// fram nya
			////Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			////Send_Leg3_Kar_And_Velocity(x_ground, step-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			////Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			////_delay_ms(FORWARD_DELAY);
			//////ner
			////Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
			////Send_Leg3_Kar_And_Velocity(x_ground, step-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
			////Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
			////_delay_ms(DOWN_DELAY);
		//}
		//
	//}
	
	
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
		Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(FORWARD_DELAY);
	}
	//ner
	Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
	Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
	_delay_ms(DOWN_DELAY);
	
	if(Servo_Load_Too_Small(16))
	{
		
		if(direction == 'u') //Klättra upp, dvs 'u'
		{
			
			//upp gamla
			Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			_delay_ms(FORWARD_DELAY);
			//uppdatera step
			step=step+1;
			//fram nya
			Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			_delay_ms(FORWARD_DELAY);
			//ner
			Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
			Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
			Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
			_delay_ms(DOWN_DELAY);
		}
		
		else //Klättra ner dvs 'd'
		{
			////upp gamla
			//Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			//Send_Leg3_Kar_And_Velocity(x_ground, step-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			//Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			//_delay_ms(FORWARD_DELAY);
			////uppdatera step
			//step = step + 1
			//// fram nya
			//Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			//Send_Leg3_Kar_And_Velocity(x_ground, step-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			//Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			//_delay_ms(FORWARD_DELAY);
			////ner
			//Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
			//Send_Leg3_Kar_And_Velocity(x_ground, step-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
			//Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
			//_delay_ms(DOWN_DELAY);
		}
		
	}
	
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

	if(Servo_Load_Too_Small(15))
	{
		
		if(direction == 'u') //Klättra upp, dvs 'u'
		{
			
			//upp gamla
			Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			Send_Leg4_Kar_And_Velocity(x_obstacle, 2*step-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			Send_Leg5_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			_delay_ms(FORWARD_DELAY);
			//uppdatera step
			step=step+1;
			//fram nya
			Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			Send_Leg4_Kar_And_Velocity(x_obstacle, 2*step-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			Send_Leg5_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			_delay_ms(FORWARD_DELAY);
			//ner
			Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
			Send_Leg4_Kar_And_Velocity(x_obstacle, 2*step-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
			Send_Leg5_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
			_delay_ms(DOWN_DELAY);
		}
		
		else //Klättra ner dvs 'd'
		{
			////upp gamla
			//Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			//Send_Leg3_Kar_And_Velocity(x_ground, step-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			//Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			//_delay_ms(FORWARD_DELAY);
			////uppdatera step
			//step = step + 1
			//// fram nya
			//Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			//Send_Leg3_Kar_And_Velocity(x_ground, step-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			//Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			//_delay_ms(FORWARD_DELAY);
			////ner
			//Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
			//Send_Leg3_Kar_And_Velocity(x_ground, step-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
			//Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
			//_delay_ms(DOWN_DELAY);
		}
		
	}

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

	if(Servo_Load_Too_Small(4))
	{
		
		if(direction == 'u') //Klättra upp, dvs 'u'
		{
			
			//upp gamla
			Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			Send_Leg4_Kar_And_Velocity(x_obstacle, 2*step-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			Send_Leg5_Kar_And_Velocity(x_obstacle, 2*step-corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			_delay_ms(FORWARD_DELAY);
			//uppdatera step
			step=step+1;
			//fram nya
			Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			Send_Leg4_Kar_And_Velocity(x_obstacle, 2*step-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			Send_Leg5_Kar_And_Velocity(x_obstacle, 2*step-corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			_delay_ms(FORWARD_DELAY);
			//ner
			Send_Leg1_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
			Send_Leg4_Kar_And_Velocity(x_obstacle, 2*step-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
			Send_Leg5_Kar_And_Velocity(x_obstacle, 2*step-corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
			_delay_ms(DOWN_DELAY);
		}
		
		else //Klättra ner dvs 'd'
		{
			////upp gamla
			//Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			//Send_Leg3_Kar_And_Velocity(x_ground, step-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			//Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			//_delay_ms(FORWARD_DELAY);
			////uppdatera step
			//step = step + 1
			//// fram nya
			//Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			//Send_Leg3_Kar_And_Velocity(x_ground, step-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			//Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			//_delay_ms(FORWARD_DELAY);
			////ner
			//Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
			//Send_Leg3_Kar_And_Velocity(x_ground, step-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
			//Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
			//_delay_ms(DOWN_DELAY);
		}
		
	}
	
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
	
	if(Servo_Load_Too_Small(3))
	{
		
		if(direction == 'u') //Klättra upp, dvs 'u'
		{
			
			//upp gamla
			Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			Send_Leg6_Kar_And_Velocity(x_obstacle, 2*step-corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			_delay_ms(FORWARD_DELAY);
			//uppdatera step
			step=step+1;
			//fram nya
			Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			Send_Leg6_Kar_And_Velocity(x_obstacle, 2*step-corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			_delay_ms(FORWARD_DELAY);
			//ner
			Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
			Send_Leg3_Kar_And_Velocity(x_obstacle, 2*step-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
			Send_Leg6_Kar_And_Velocity(x_obstacle, 2*step-corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
			_delay_ms(DOWN_DELAY);
		}
		
		else //Klättra ner dvs 'd'
		{
			////upp gamla
			//Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			//Send_Leg3_Kar_And_Velocity(x_ground, step-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			//Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			//_delay_ms(FORWARD_DELAY);
			////uppdatera step
			//step = step + 1
			//// fram nya
			//Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
			//Send_Leg3_Kar_And_Velocity(x_ground, step-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			//Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height-lift), speed_inner, speed_middle, speed_outer);
			//_delay_ms(FORWARD_DELAY);
			////ner
			//Send_Leg2_Kar_And_Velocity(x_obstacle, 2*step+corner_pitch-weight_adjust, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
			//Send_Leg3_Kar_And_Velocity(x_ground, step-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
			//Send_Leg6_Kar_And_Velocity(x_ground, step-corner_pitch-weight_adjust, -(height), speed_inner, speed_middle, speed_outer);
			//_delay_ms(DOWN_DELAY);
		}
		
	}
	
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
	Send_Leg1_Kar_And_Velocity(x_obstacle, last_step-step+corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg4_Kar_And_Velocity(x_obstacle, last_step-step-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	Send_Leg5_Kar_And_Velocity(x_obstacle, last_step-step-corner_pitch-weight_adjust, -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
	_delay_ms(500);

	for (uint8_t n = 0; n<=number_of_steps; ++n)
	{
		
		Send_Leg2_Kar_And_Velocity(x_obstacle, step-weight_adjust -((step-weight_adjust)*n/number_of_steps)+corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg3_Kar_And_Velocity(x_obstacle, step-weight_adjust -((step-weight_adjust)*n/number_of_steps), -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		Send_Leg6_Kar_And_Velocity(x_obstacle, step-weight_adjust -((step-weight_adjust)*n/number_of_steps)-corner_pitch, -(height-obstacle_height), speed_inner, speed_middle, speed_outer);
		
		Send_Leg1_Kar_And_Velocity(x_obstacle, last_step-step+corner_pitch-weight_adjust + ((-(last_step-step)+weight_adjust)*n/number_of_steps), -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg4_Kar_And_Velocity(x_obstacle, last_step-step-weight_adjust +((-(last_step-step)+weight_adjust)*n/number_of_steps), -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		Send_Leg5_Kar_And_Velocity(x_obstacle, last_step-step-corner_pitch-weight_adjust +((-(last_step-step)+weight_adjust)*n/number_of_steps), -(height-obstacle_height-lift), speed_inner, speed_middle, speed_outer);
		_delay_ms(BACK_DELAY);
		
	}
	
}