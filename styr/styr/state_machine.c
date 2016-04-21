/*
* state_machine.c
*
* Created: 4/8/2016 2:53:09 PM
*  Author: chrma018
*/

#include "state_machine.h"

void calculate_Yaw()
{
	switch (ROBOT_STATE)
	{

		case CORRIDOR:
		{
			//Yaw = (IR_Yaw_right + IR_Yaw_left)/2;
			Yaw = IR_Yaw_left;
			break;
		}
		
		case OUT_OF_CORRIDOR_RIGHT_WALL:
		case OUT_OF_TURN_LEFT:
		case OUT_OF_JUNCTION_C_LEFT:
		case JUNCTION_B_LEFT:
		{
			Yaw = IR_Yaw_right;
			break;
		}
		
		case OUT_OF_CORRIDOR_LEFT_WALL:
		case OUT_OF_TURN_RIGHT:
		case OUT_OF_JUNCTION_C_RIGHT:
		case JUNCTION_B_RIGHT:
		{
			Yaw = IR_Yaw_left;
			break;
		}
		
		case OUT_OF_JUNCTION_A_RIGHT:
		case OUT_OF_JUNCTION_A_LEFT:
		{
			Yaw = IMU_Yaw - IMU_Yaw_start;
			break;
		}
		
	}
}

void update_state()
{
	load_sensor_values();
	
	switch (ROBOT_STATE)
	{
		//
		case CORRIDOR:
		{
			
			if(IR_0 < 20)
			{
				ROBOT_STATE = STOP;
				break;
			}
			//if (IR_0 < DEAD_END_DISTANCE && IR_2 < CORRIDOR_SIDE_DISTANCE && IR_3 < CORRIDOR_SIDE_DISTANCE && IR_5 < CORRIDOR_SIDE_DISTANCE && IR_6 < CORRIDOR_SIDE_DISTANCE)
			//{
			//ROBOT_STATE = DEAD_END;
			//break;
			//}
			
			//
			else if (IR_2 > CORRIDOR_SIDE_DISTANCE && IR_3 < CORRIDOR_SIDE_DISTANCE && IR_5 < CORRIDOR_SIDE_DISTANCE && IR_6 < CORRIDOR_SIDE_DISTANCE
			|| IR_2 > CORRIDOR_SIDE_DISTANCE && IR_3 > CORRIDOR_SIDE_DISTANCE && IR_5 < CORRIDOR_SIDE_DISTANCE && IR_6 < CORRIDOR_SIDE_DISTANCE)
			{
				ROBOT_STATE = OUT_OF_CORRIDOR_LEFT_WALL;
				break;
			}
			
			else if (IR_2 < CORRIDOR_SIDE_DISTANCE && IR_3 < CORRIDOR_SIDE_DISTANCE && IR_5 < CORRIDOR_SIDE_DISTANCE && IR_6 > CORRIDOR_SIDE_DISTANCE
			|| IR_2 < CORRIDOR_SIDE_DISTANCE && IR_3 < CORRIDOR_SIDE_DISTANCE && IR_5 > CORRIDOR_SIDE_DISTANCE && IR_6 > CORRIDOR_SIDE_DISTANCE)
			{
				ROBOT_STATE = OUT_OF_CORRIDOR_RIGHT_WALL;
				break;
			}
			
			//else if (US < US_HIGH_OBSTACLE_DISTANCE && IR_0 > NO_WALL_DISTANCE)
			//{
			//ROBOT_STATE = INTO_HIGH_OBSTACLE;
			//break;
			//}
			//
			//else if (IR_1 < LOW_OBSTACLE_DISTANCE && IR_0 > NO_WALL_DISTANCE)
			//{
			//ROBOT_STATE = INTO_LOW_OBSTACLE;
			//break;
			//}
		}
		
		case STOP:
		{
			break;
		}
		
		//
		//case DEAD_END:
		//{
		//if (IMU_Yaw - IMU_Yaw_start >= FULL_ROTATION_ANGLE)
		//{
		//ROBOT_STATE = CORRIDOR;
		//start_Yaw_set = 0;
		//break;
		//}
		//break;
		//}
		//
		//
		case OUT_OF_CORRIDOR_LEFT_WALL:
		case OUT_OF_CORRIDOR_RIGHT_WALL:
		case OUT_OF_CORRIDOR:
		{
			if (IR_2 > CORRIDOR_SIDE_DISTANCE && IR_3 > CORRIDOR_SIDE_DISTANCE && IR_5 < CORRIDOR_SIDE_DISTANCE && IR_6 < CORRIDOR_SIDE_DISTANCE && IR_0 < SHORT_TURN_DISTANCE)
			{
				ROBOT_STATE = TURN_RIGHT;
				break;
			}
			
			else if (IR_2 < CORRIDOR_SIDE_DISTANCE && IR_3 < CORRIDOR_SIDE_DISTANCE && IR_5 > CORRIDOR_SIDE_DISTANCE && IR_6 > CORRIDOR_SIDE_DISTANCE && IR_0 < SHORT_TURN_DISTANCE)
			{
				ROBOT_STATE = TURN_LEFT;
				break;
			}
			//
			//else if (IR_2 > SIDE_DEAD_END_DISTANCE && IR_3 > SIDE_DEAD_END_DISTANCE && IR_5 < CORRIDOR_SIDE_DISTANCE && IR_6 < CORRIDOR_SIDE_DISTANCE && IR_0 < LONG_TURN_DISTANCE)
			//{
			//ROBOT_STATE = JUNCTION_A_RIGHT;
			//break;
			//}
			//
			//else if (IR_2 < CORRIDOR_SIDE_DISTANCE && IR_3 < CORRIDOR_SIDE_DISTANCE && IR_5 > SIDE_DEAD_END_DISTANCE && IR_6 > SIDE_DEAD_END_DISTANCE && IR_0 < LONG_TURN_DISTANCE)
			//{
			//ROBOT_STATE = JUNCTION_A_LEFT;
			//break;
			//}
			//
			//else if (IR_2 > CORRIDOR_SIDE_DISTANCE && IR_2 < SIDE_DEAD_END_DISTANCE && IR_3 > CORRIDOR_SIDE_DISTANCE && IR_3 < SIDE_DEAD_END_DISTANCE && IR_5 < CORRIDOR_SIDE_DISTANCE && IR_6 < CORRIDOR_SIDE_DISTANCE && IR_0 > FORWARD_DEAD_END_DISTANCE)
			//{
			//ROBOT_STATE = JUNCTION_B_RIGHT;
			//break;
			//}
			//
			//else if (IR_2 < CORRIDOR_SIDE_DISTANCE && IR_3 < CORRIDOR_SIDE_DISTANCE && IR_5 > CORRIDOR_SIDE_DISTANCE && IR_5 < SIDE_DEAD_END_DISTANCE && IR_6 > CORRIDOR_SIDE_DISTANCE && IR_6 < SIDE_DEAD_END_DISTANCE && IR_0 > FORWARD_DEAD_END_DISTANCE)
			//{
			//ROBOT_STATE = JUNCTION_B_LEFT;
			//break;
			//}
			//
			//else if (IR_2 > CORRIDOR_SIDE_DISTANCE && IR_2 < SIDE_DEAD_END_DISTANCE && IR_3 > CORRIDOR_SIDE_DISTANCE && IR_3 < SIDE_DEAD_END_DISTANCE && IR_5 > SIDE_DEAD_END_DISTANCE && IR_6 > SIDE_DEAD_END_DISTANCE && IR_0 < SHORT_TURN_DISTANCE)
			//{
			//ROBOT_STATE = JUNCTION_C_LEFT;
			//break;
			//}
			//
			//else if (IR_2 > SIDE_DEAD_END_DISTANCE && IR_3 > SIDE_DEAD_END_DISTANCE && IR_5 > CORRIDOR_SIDE_DISTANCE && IR_5 < SIDE_DEAD_END_DISTANCE && IR_6 > CORRIDOR_SIDE_DISTANCE && IR_6 < SIDE_DEAD_END_DISTANCE && IR_0 < SHORT_TURN_DISTANCE)
			//{
			//ROBOT_STATE = JUNCTION_C_RIGHT;
			//break;
			//}
			//
			//else if (IR_0 > END_OF_COURSE_DISTANCE && IR_2 > END_OF_COURSE_DISTANCE && IR_3 > END_OF_COURSE_DISTANCE && IR_5 > END_OF_COURSE_DISTANCE && IR_6 > END_OF_COURSE_DISTANCE)
			//{
			//ROBOT_STATE = END_OF_COURSE;
			//break;
			//}
		}
		
		//
		case TURN_RIGHT:
		{
			//if (IMU_Yaw - IMU_Yaw_start >= HALF_ROTATION_ANGLE)
			//{
			//ROBOT_STATE = OUT_OF_TURN_RIGHT;
			//start_Yaw_set = 0;
			//break;
			//}
			//break;
			
			if (rotation_count > 9)
			{
				ROBOT_STATE = OUT_OF_TURN_RIGHT;
				start_Yaw_set = 0;
				break;
			}
			break;
		}
		
		//
		case TURN_LEFT:
		{
			//if (IMU_Yaw - IMU_Yaw_start <= -HALF_ROTATION_ANGLE)
			//{
			//ROBOT_STATE = OUT_OF_TURN_LEFT;
			//start_Yaw_set = 0;
			//break;
			//}
			//break;
			
			if (rotation_count > 9)
			{
				ROBOT_STATE = OUT_OF_TURN_LEFT;
				start_Yaw_set = 0;
				break;
			}
			break;
		}
		//
		//
		case OUT_OF_TURN_RIGHT:
		{
			if (IR_2 < CORRIDOR_SIDE_DISTANCE && IR_3 < CORRIDOR_SIDE_DISTANCE && IR_5 < CORRIDOR_SIDE_DISTANCE && IR_6 < CORRIDOR_SIDE_DISTANCE)
			{
				ROBOT_STATE = CORRIDOR;
				break;
			}
			break;
		}
		
		//
		case OUT_OF_TURN_LEFT:
		{
			if (IR_2 < CORRIDOR_SIDE_DISTANCE && IR_3 < CORRIDOR_SIDE_DISTANCE && IR_5 < CORRIDOR_SIDE_DISTANCE && IR_6 < CORRIDOR_SIDE_DISTANCE)
			{
				ROBOT_STATE = CORRIDOR;
				break;
			}
			break;
		}
		//
		////
		//case JUNCTION_A_RIGHT:
		//{
		//if (IMU_Yaw - IMU_Yaw_start >= HALF_ROTATION_ANGLE)
		//{
		//ROBOT_STATE = OUT_OF_JUNCTION_A_RIGHT;
		//start_Yaw_set = 0;
		//break;
		//}
		//break;
		//}
		//
		////
		//case JUNCTION_A_LEFT:
		//{
		//if (IMU_Yaw - IMU_Yaw_start <= -HALF_ROTATION_ANGLE)
		//{
		//ROBOT_STATE = OUT_OF_JUNCTION_A_RIGHT;
		//start_Yaw_set = 0;
		//break;
		//}
		//break;
		//}
		//
		////
		//case OUT_OF_JUNCTION_A_RIGHT:
		//{
		//if (IR_2 < CORRIDOR_SIDE_DISTANCE && IR_3 < CORRIDOR_SIDE_DISTANCE && IR_5 < CORRIDOR_SIDE_DISTANCE && IR_6 < CORRIDOR_SIDE_DISTANCE)
		//{
		//ROBOT_STATE = CORRIDOR;
		//start_Yaw_set = 0;
		//break;
		//}
		//break;
		//}
		//
		////
		//case OUT_OF_JUNCTION_A_LEFT:
		//{
		//if (IR_2 < CORRIDOR_SIDE_DISTANCE && IR_3 < CORRIDOR_SIDE_DISTANCE && IR_5 < CORRIDOR_SIDE_DISTANCE && IR_6 < CORRIDOR_SIDE_DISTANCE)
		//{
		//ROBOT_STATE = CORRIDOR;
		//start_Yaw_set = 0;
		//break;
		//}
		//break;
		//}
		//
		////
		//case JUNCTION_B_RIGHT:
		//{
		//if (IR_2 < CORRIDOR_SIDE_DISTANCE && IR_3 < CORRIDOR_SIDE_DISTANCE && IR_5 < CORRIDOR_SIDE_DISTANCE && IR_6 < CORRIDOR_SIDE_DISTANCE)
		//{
		//ROBOT_STATE = CORRIDOR;
		//break;
		//}
		//break;
		//}
		//
		////
		//case JUNCTION_B_LEFT:
		//{
		//if (IR_2 < CORRIDOR_SIDE_DISTANCE && IR_3 < CORRIDOR_SIDE_DISTANCE && IR_5 < CORRIDOR_SIDE_DISTANCE && IR_6 < CORRIDOR_SIDE_DISTANCE)
		//{
		//ROBOT_STATE = CORRIDOR;
		//break;
		//}
		//break;
		//}
		//
		////
		//case JUNCTION_C_RIGHT:
		//{
		//if (IMU_Yaw - IMU_Yaw_start >= HALF_ROTATION_ANGLE)
		//{
		//ROBOT_STATE = OUT_OF_JUNCTION_C_RIGHT;
		//start_Yaw_set = 0;
		//break;
		//}
		//break;
		//}
		//
		////
		//case JUNCTION_C_LEFT:
		//{
		//if (IMU_Yaw - IMU_Yaw_start <= -HALF_ROTATION_ANGLE)
		//{
		//ROBOT_STATE = OUT_OF_JUNCTION_C_LEFT;
		//start_Yaw_set = 0;
		//break;
		//}
		//break;
		//}
		//
		////
		//case OUT_OF_JUNCTION_C_RIGHT:
		//{
		//if (IR_2 < CORRIDOR_SIDE_DISTANCE && IR_3 < CORRIDOR_SIDE_DISTANCE && IR_5 < CORRIDOR_SIDE_DISTANCE && IR_6 < CORRIDOR_SIDE_DISTANCE)
		//{
		//ROBOT_STATE = CORRIDOR;
		//break;
		//}
		//break;
		//}
		//
		////
		//case OUT_OF_JUNCTION_C_LEFT:
		//{
		//if (IR_2 < CORRIDOR_SIDE_DISTANCE && IR_3 < CORRIDOR_SIDE_DISTANCE && IR_5 < CORRIDOR_SIDE_DISTANCE && IR_6 < CORRIDOR_SIDE_DISTANCE)
		//{
		//ROBOT_STATE = CORRIDOR;
		//break;
		//}
		//break;
		//}
		//
		////
		//case END_OF_COURSE:
		//{
		//break;
		//}
		//
		////
		////case INTO_HIGH_OBSTACLE:
		////{
		////if (ride_height == min_height)
		////{
		////ROBOT_STATE = CRAWLING_UNDER_HIGH_OBSTACLE;
		////break;
		////}
		////break;
		////}
		////
		//////
		////case CRAWLING_UNDER_HIGH_OBSTACLE:
		////{
		////if (IR_4 < IR_HIGH_OBSTACLE_DISTANCE)
		////{
		////ROBOT_STATE = CRAWLING_OUT_OF_HIGH_OBSTACLE;
		////break;
		////}
		////break;
		////}
		////
		//////
		////case CRAWLING_OUT_OF_HIGH_OBSTACLE:
		////{
		////if (IR_4 > IR_HIGH_OBSTACLE_DISTANCE)
		////{
		////ROBOT_STATE = OUT_OF_HIGH_OBSTACLE;
		////break;
		////}
		////break;
		////}
		////
		//////
		////case OUT_OF_HIGH_OBSTACLE:
		////{
		////if (ride_height = normal_height)
		////{
		////ROBOT_STATE = CORRIDOR;
		////break;
		////}
		////break;
		////}
		////
		//////
		////case INTO_LOW_OBSTACLE:
		////{
		////if (IR_1 < START_CLIMBING_UP_DISTANCE)
		////{
		////ROBOT_STATE = CLIMB_UP;
		////break;
		////}
		////break;
		////}
		////
		//////
		////case CLIMB_UP:
		////{
		////if (climbed_up)
		////{
		////ROBOT_STATE = LOW_OBSTACLE;
		////break;
		////}
		////break;
		////}
		////
		//////
		////case LOW_OBSTACLE:
		////{
		////if (IR_1 > START_CLIMBING_DOWN_DISTANCE)
		////{
		////ROBOT_STATE = OUT_OF_LOW_OBSTACLE;
		////break;
		////}
		////break;
		////}
		////
		//////
		////case OUT_OF_LOW_OBSTACLE:
		////{
		////if (down_from_low_obst)
		////{
		////ROBOT_STATE = CORRIDOR;
		////break;
		////}
		////break;
		////}

	}
	
	fromSen[14] = ROBOT_STATE; 
}

void run_state(float height_value)
{
	calculate_Yaw();
	
	switch (ROBOT_STATE)
	{
		
		case CORRIDOR:
		{
			float Yaw_rad = (Yaw/180)*3.1415926;
			float alpha_value = update_alpha(Yaw_rad, IR_3, IR_6);
			Walk_Half_Cycle(2, alpha_value, height_value);
			break;
		}
		
		case OUT_OF_CORRIDOR_RIGHT_WALL:
		case OUT_OF_CORRIDOR_LEFT_WALL:
		{
			float Yaw_rad = (Yaw/180)*3.1415926;
			float alpha_value = update_alpha(Yaw_rad, IR_3, IR_6);
			Walk_Half_Cycle(1, alpha_value, height_value);
			break;
		}
		
		case STOP:
		{
			break;
		}

		case TURN_RIGHT:
		{
			if(!start_Yaw_set)
			{
				IMU_Yaw_start = IMU_Yaw;
				start_Yaw_set = 1;
			}
			
			else
			{
				Walk_Half_Cycle(0, 0.2, height_value);
				++rotation_count;
			}
			break;
		}
		//
		case OUT_OF_TURN_LEFT:
		case OUT_OF_TURN_RIGHT:
		{
			float Yaw_rad = (Yaw/180)*3.1415926;
			float alpha_value = update_alpha(Yaw_rad, IR_3, IR_6);
			Walk_Half_Cycle(2, alpha_value, height_value);
			break;
		}
		
		
		//
		case TURN_LEFT:
		{
			if(!start_Yaw_set)
			{
				IMU_Yaw_start = IMU_Yaw;
				start_Yaw_set = 1;
			}
			
			else
			{
				Walk_Half_Cycle(0, -0.2, height_value);
				++rotation_count;	
			}
			break;
		}
		
		////
		//case JUNCTION_A_RIGHT:
		//{
		//if(!start_Yaw_set)
		//{
		//IMU_Yaw_start = IMU_Yaw;
		//start_Yaw_set = 1;
		//}
		//
		//else
		//{
		//
		//}
		//break;
		//}
		//
		////
		//case JUNCTION_A_LEFT:
		//{
		//if(!start_Yaw_set)
		//{
		//IMU_Yaw_start = IMU_Yaw;
		//start_Yaw_set = 1;
		//}
		//
		//else
		//{
		//
		//}
		//break;
		//}
		//
		////
		//case JUNCTION_C_RIGHT:
		//{
		//if(!start_Yaw_set)
		//{
		//IMU_Yaw_start = IMU_Yaw;
		//start_Yaw_set = 1;
		//}
		//
		//else
		//{
		//
		//}
		//break;
		//}
		//
		////
		//case JUNCTION_C_LEFT:
		//{
		//if(!start_Yaw_set)
		//{
		//IMU_Yaw_start = IMU_Yaw;
		//start_Yaw_set = 1;
		//}
		//
		//else
		//{
		//
		//}
		//break;
		//}
		//
		////
		//case DEAD_END:
		//{
		//if(!start_Yaw_set)
		//{
		//IMU_Yaw_start = IMU_Yaw;
		//start_Yaw_set = 1;
		//}
		//
		//else
		//{
		//
		//}
		//break;
		//}
		//
		////
		//case OUT_OF_JUNCTION_A_RIGHT:
		//{
		//if(!start_Yaw_set)
		//{
		//IMU_Yaw_start = IMU_Yaw;
		//start_Yaw_set = 1;
		//}
		//
		//else
		//{
		//
		//}
		//break;
		//}
		//
		////
		//case OUT_OF_JUNCTION_A_LEFT:
		//{
		//if(!start_Yaw_set)
		//{
		//IMU_Yaw_start = IMU_Yaw;
		//start_Yaw_set = 1;
		//}
		//
		//else
		//{
		//
		//}
		//break;
		//}
		//
	}
}

load_sensor_values()
{
	int16_t IMU_Yaw_16;
	int8_t IR_Yaw_left_int;
	int8_t IR_Yaw_right_int;
	
	IR_0 = (float)fromSen[0];
	IR_1 = (float)fromSen[1];
	IR_2 = (float)fromSen[2];
	IR_3 = (float)fromSen[3];
	IR_4 = (float)fromSen[4];
	IR_5 = (float)fromSen[5];
	IR_6 = (float)fromSen[6];
	US = (float)fromSen[7];
	IR_Yaw_left_int = fromSen[8];
	IR_Yaw_right_int = fromSen[9];
	IR_Yaw_left = (float)IR_Yaw_left_int;
	IR_Yaw_right = (float)IR_Yaw_right_int;
	IMU_Yaw_16 = fromSen[10];
	IMU_Yaw_16 += (fromSen[11]<<8);
	IMU_Yaw = (float)IMU_Yaw_16;
	Pitch = (float)fromSen[12];
	Roll = (float)fromSen[13];
}
