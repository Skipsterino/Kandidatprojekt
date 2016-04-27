/*
* state_machine.c
*
* Created: 4/8/2016 2:53:09 PM
*  Author: fregu856
*/

#include "state_machine.h"

void calculate_Yaw()
{
	switch (ROBOT_STATE)
	{
		//case INTO_HIGH_OBSTACLE:
		//case CRAWLING_UNDER_HIGH_OBSTACLE:
		//case INTO_LOW_OBSTACLE:
		//case LOW_OBSTACLE:
		case CORRIDOR:
		{
			Yaw = (IR_Yaw_right + IR_Yaw_left)/2;
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
		
		case OUT_OF_CORRIDOR_NO_WALL:
		case OUT_OF_JUNCTION_A_RIGHT:
		case OUT_OF_JUNCTION_A_LEFT:
		{
			if (first_state_cycle == true)
			{
				IMU_Yaw_start = IMU_Yaw;
				first_state_cycle = false;
			}
			
			Yaw = IMU_Yaw - IMU_Yaw_start; //Funkar?
			break;
		}
		
		default:
			Yaw = 0; //Om inget annat är sagt görs ingen reglering
			break;
	}

	Yaw_rad = (Yaw/180)*3.1415926; //Konvertera till radianer!
}

void calculate_p_part()
{
	switch (ROBOT_STATE)
	{
		//case INTO_HIGH_OBSTACLE:
		//case CRAWLING_UNDER_HIGH_OBSTACLE:
		//case INTO_LOW_OBSTACLE:
		//case LOW_OBSTACLE:
		case CORRIDOR:
		{
			p_part = IR_3 - IR_6;
			break;
		}
		
		case OUT_OF_CORRIDOR_RIGHT_WALL:
		case OUT_OF_TURN_LEFT:
		case OUT_OF_JUNCTION_C_LEFT:
		case JUNCTION_B_LEFT:
		{
			p_part = ((IR_2 + IR_3)/2 + CENTRE_OFFSET)*cos(Yaw_rad) - CORRIDOR_WIDTH/2;
			break;
		}
		
		case OUT_OF_CORRIDOR_LEFT_WALL:
		case OUT_OF_TURN_RIGHT:
		case OUT_OF_JUNCTION_C_RIGHT:
		case JUNCTION_B_RIGHT:
		{
			p_part = CORRIDOR_WIDTH/2 - ((IR_5 + IR_6)/2 + CENTRE_OFFSET)*cos(Yaw_rad);
			break;
		}
		
		case OUT_OF_CORRIDOR_NO_WALL:
		case OUT_OF_JUNCTION_A_RIGHT:
		case OUT_OF_JUNCTION_A_LEFT:
		{
			p_part = 0;
			break;
		}
		
		default:
			p_part = 0; //Om inget annat är sagt görs ingen reglering
			break;
	}
}

void update_state()
{
	load_sensor_values();
	
	if(IR_0 < 10)
	{
		ROBOT_STATE = STOP;
	}
	
	switch (ROBOT_STATE)
	{
		case CORRIDOR:
		{
			
			if ((IR_0 < DEAD_END_DISTANCE) && (IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE))
			{
				ROBOT_STATE = DEAD_END;
			}
			
			else if (((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE))
				  || ((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE)))
			{
				ROBOT_STATE = OUT_OF_CORRIDOR_LEFT_WALL;
				//first_state_cycle = true; BORDE VÄL INTE BEHÖVAS HÄR???
			}
			
			else if (((IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE))
				  || ((IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE)))
			{
				ROBOT_STATE = OUT_OF_CORRIDOR_RIGHT_WALL;
				//first_state_cycle = true; BORDE VÄL INTE BEHÖVAS HÄR HELLER???
			}
			
			else if (((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE))
				  || ((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE))
				  || ((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE))
				  || ((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE)))
			{
				ROBOT_STATE = OUT_OF_CORRIDOR_NO_WALL;
				first_state_cycle = true; //Behövs eftersom den används vid bestämning av Yaw i detta tillstånd.
			}
			
			//else if ((US < US_HIGH_OBSTACLE_DISTANCE) && (IR_0 > NO_WALL_DISTANCE))
			//{
			//ROBOT_STATE = INTO_HIGH_OBSTACLE;
			//break;
			//}
			//
			//else if ((IR_1 < LOW_OBSTACLE_DISTANCE) && (IR_0 > NO_WALL_DISTANCE))
			//{
			//ROBOT_STATE = INTO_LOW_OBSTACLE;
			//break;
			//}
			
			break;
		}
		
		case END_OF_COURSE:
		case STOP:
		{
			break;
		}
		
		case OUT_OF_CORRIDOR_LEFT_WALL:
		case OUT_OF_CORRIDOR_RIGHT_WALL:
		{
			if ((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE) && (IR_0 < SHORT_TURN_DISTANCE))
			{
				ROBOT_STATE = TURN_RIGHT;
			}
			
			else if ((IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE) && (IR_0 < SHORT_TURN_DISTANCE))
			{
				ROBOT_STATE = TURN_LEFT;
			}
			
			else if (((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE))
				  || ((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE))
				  || ((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE))
				  || ((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE)))
			{
				ROBOT_STATE = OUT_OF_CORRIDOR_NO_WALL;
				first_state_cycle = true;
			}
			
			else if ((IR_2 > SIDE_DEAD_END_DISTANCE) && (IR_3 > SIDE_DEAD_END_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) &&
					 (IR_6 < CORRIDOR_SIDE_DISTANCE) && (IR_0 < LONG_TURN_DISTANCE) && (IR_0 > JUNCTION_A_FORWARD_DISTANCE))
			{
				ROBOT_STATE = JUNCTION_A_RIGHT;
			}
			
			else if ((IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 > SIDE_DEAD_END_DISTANCE) &&
					 (IR_6 > SIDE_DEAD_END_DISTANCE) && (IR_0 < LONG_TURN_DISTANCE) && (IR_0 > JUNCTION_A_FORWARD_DISTANCE))
			{
				ROBOT_STATE = JUNCTION_A_LEFT;
			}

			else if ((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_2 < SIDE_DEAD_END_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_3 < SIDE_DEAD_END_DISTANCE) &&
					 (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE) && (IR_0 > FORWARD_DEAD_END_DISTANCE))
			{
				ROBOT_STATE = JUNCTION_B_RIGHT;
			}
			
			else if ((IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_5 < SIDE_DEAD_END_DISTANCE) &&
					 (IR_6 > CORRIDOR_SIDE_DISTANCE) && (IR_6 < SIDE_DEAD_END_DISTANCE) && (IR_0 > FORWARD_DEAD_END_DISTANCE))
			{
				ROBOT_STATE = JUNCTION_B_LEFT;
			}
			
			break;
		}
		
		case OUT_OF_CORRIDOR_NO_WALL:
		{
			if ((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_2 < SIDE_DEAD_END_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_3 < SIDE_DEAD_END_DISTANCE) &&
				(IR_5 > SIDE_DEAD_END_DISTANCE) && (IR_6 > SIDE_DEAD_END_DISTANCE) && (IR_0 < SHORT_TURN_DISTANCE))
			{
				ROBOT_STATE = JUNCTION_C_LEFT;
			}
			
			else if ((IR_2 > SIDE_DEAD_END_DISTANCE) && (IR_3 > SIDE_DEAD_END_DISTANCE) && (IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_5 < SIDE_DEAD_END_DISTANCE) &&
					 (IR_6 > CORRIDOR_SIDE_DISTANCE) && (IR_6 < SIDE_DEAD_END_DISTANCE) && (IR_0 < SHORT_TURN_DISTANCE))
			{
				ROBOT_STATE = JUNCTION_C_RIGHT;
			}
			
			else if ((IR_0 > END_OF_COURSE_DISTANCE) && (IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE))
			{
				ROBOT_STATE = END_OF_COURSE;
			}
			
			break;
		}
		
		case TURN_RIGHT:
		{
			if (((IMU_Yaw - IMU_Yaw_start) >= HALF_ROTATION_ANGLE) && (rotation_count > 4)) //Kan ta bort rotation_count sen?
			{
				ROBOT_STATE = OUT_OF_TURN_RIGHT;
				start_Yaw_set = false;
				rotation_count = 0;
			}
			
			//if (rotation_count > 9)
			//{
			//ROBOT_STATE = OUT_OF_TURN_RIGHT;
			//start_Yaw_set = false;
			//rotation_count = 0;
			//}
			
			break;
		}
		
		case TURN_LEFT:
		{
			if (((IMU_Yaw - IMU_Yaw_start) <= (-1)*HALF_ROTATION_ANGLE) && (rotation_count > 4)) //Kan ta bort rotation_count sen?
			{
				ROBOT_STATE = OUT_OF_TURN_LEFT;
				start_Yaw_set = false;
				rotation_count = 0;
			}
			
			//if (rotation_count > 9)
			//{
			//ROBOT_STATE = OUT_OF_TURN_LEFT;
			//start_Yaw_set = false;
			//rotation_count = 0;
			//}
			
			break;
		}

		case JUNCTION_A_RIGHT: //FIXA IMU
		{
			//if (IMU_Yaw - IMU_Yaw_start >= HALF_ROTATION_ANGLE)
			//{
			//ROBOT_STATE = OUT_OF_JUNCTION_A_RIGHT;
			//start_Yaw_set = false;
			//}
			
			if (rotation_count > 9)
			{
				ROBOT_STATE = OUT_OF_JUNCTION_A_RIGHT;
				first_state_cycle = true;
				start_Yaw_set = false;
				rotation_count = 0;
			}
			
			break;
		}
		
		case JUNCTION_A_LEFT: //FIXA IMU
		{
			//if (IMU_Yaw - IMU_Yaw_start <= -HALF_ROTATION_ANGLE)
			//{
			//ROBOT_STATE = OUT_OF_JUNCTION_A_LERFT;
			//start_Yaw_set = false;
			//break;
			//}
			
			if (rotation_count > 9)
			{
				ROBOT_STATE = OUT_OF_JUNCTION_A_LEFT;
				first_state_cycle = true;
				start_Yaw_set = false;
				rotation_count = 0;
			}
			
			break;
		}
		
		case JUNCTION_C_RIGHT: //FIXA IMU
		{
			//if (IMU_Yaw - IMU_Yaw_start >= HALF_ROTATION_ANGLE)
			//{
			//ROBOT_STATE = OUT_OF_JUNCTION_A_RIGHT;
			//start_Yaw_set = false;
			//break;
			//}
			
			if (rotation_count > 9)
			{
				ROBOT_STATE = OUT_OF_JUNCTION_C_RIGHT;
				start_Yaw_set = false;
				rotation_count = 0;
			}
			
			break;
		}
		
		case JUNCTION_C_LEFT: //FIXA IMU
		{
			//if (IMU_Yaw - IMU_Yaw_start >= HALF_ROTATION_ANGLE)
			//{
			//ROBOT_STATE = OUT_OF_JUNCTION_A_RIGHT;
			//start_Yaw_set = false;
			//break;
			//}
			
			if (rotation_count > 9)
			{
				ROBOT_STATE = OUT_OF_JUNCTION_C_LEFT;
				start_Yaw_set = false;
				rotation_count = 0;
			}
			
			break;
		}
		
		case DEAD_END: //FIXA IMU
		{
			//if (IMU_Yaw - IMU_Yaw_start >= HALF_ROTATION_ANGLE)
			//{
			//ROBOT_STATE = OUT_OF_JUNCTION_A_RIGHT;
			//start_Yaw_set = false;
			//break;
			//}
			
			if (rotation_count > 17)
			{
				ROBOT_STATE = CORRIDOR;
				start_Yaw_set = false;
				rotation_count = 0;
			}
			
			break;
		}
		
		case JUNCTION_B_LEFT:
		case JUNCTION_B_RIGHT:
		case OUT_OF_JUNCTION_C_LEFT:
		case OUT_OF_JUNCTION_C_RIGHT:
		case OUT_OF_JUNCTION_A_LEFT:
		case OUT_OF_JUNCTION_A_RIGHT:
		case OUT_OF_TURN_LEFT:
		case OUT_OF_TURN_RIGHT:
		{
			if ((IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE))
			{
				ROBOT_STATE = CORRIDOR;
			}
			
			break;
		}
		
		//case INTO_HIGH_OBSTACLE:
		//{
			//if (IR_4 < IR_HIGH_OBSTACLE_DISTANCE)
			//{
				//ROBOT_STATE = CRAWLING_UNDER_HIGH_OBSTACLE;
			//}
			//break;
		//}
		
		
		////
		//case CRAWLING_UNDER_HIGH_OBSTACLE:
		//{
			//if (IR_4 > IR_HIGH_OBSTACLE_DISTANCE)
			//{
				//ROBOT_STATE = CORRIDOR; //BORDE VARA LUGNT ATT HÖJA HÄR! Görs ju av CORRIDOR, kan skippa tillståndet OUT_OF_HIGH_OBSTACLE?
			//}
			//break;
		//}
	
		////
		//case OUT_OF_HIGH_OBSTACLE: //BORDE INTE BEHÖVAS ty CORRIDOR genomför höjningen!
		//{	
		//}
		
		////
		//case INTO_LOW_OBSTACLE:
		//{
			//if (IR_1 < START_CLIMBING_UP_DISTANCE)
			//{
				//ROBOT_STATE = CLIMB_UP;
			//}
			//break;
		//}
		
		////
		//case CLIMB_UP:
		//{
			//if (climbed_up)
			//{
				//ROBOT_STATE = LOW_OBSTACLE;
			//}
			//break;
		//}
		//
		////
		//case LOW_OBSTACLE:
		//{
			//if (IR_1 > START_CLIMBING_DOWN_DISTANCE)
			//{
				//ROBOT_STATE = OUT_OF_LOW_OBSTACLE;
			//}
			//break;
		//}
		//
		////
		//case OUT_OF_LOW_OBSTACLE:
		//{
			//if (climbed_down)
			//{
				//ROBOT_STATE = CORRIDOR;
			//}
			//break;
		//}
		
		default:
			break;
	}
}

void run_state(float height)
{
	calculate_Yaw();
	calculate_p_part();
	alpha = Kp*p_part + Kd*Yaw_rad;
	
	fromSen[14] = ROBOT_STATE; // Tillstånd till bussen
	
	switch (ROBOT_STATE)
	{
		case OUT_OF_JUNCTION_C_RIGHT:
		case OUT_OF_JUNCTION_C_LEFT:
		case JUNCTION_B_LEFT:
		case JUNCTION_B_RIGHT:
		case OUT_OF_TURN_LEFT:
		case OUT_OF_TURN_RIGHT:
		case CORRIDOR:
		{
			Walk_Half_Cycle(4, alpha, height);
			break;
		}
		
		case OUT_OF_JUNCTION_A_RIGHT:
		case OUT_OF_JUNCTION_A_LEFT:
		case OUT_OF_CORRIDOR_NO_WALL:
		case OUT_OF_CORRIDOR_RIGHT_WALL:
		case OUT_OF_CORRIDOR_LEFT_WALL:
		{
			//if (first_state_cycle == true)
			//{
				//Walk_Half_Cycle(2, 0, height); BORDE VÄL KUNNA SKIPPA if-SATSEN EFTERSOM alpha NU BERÄKNAS MED IMU???
				//first_state_cycle = false;
			//}
			//
			//else
			//{
			Walk_Half_Cycle(2, alpha, height);
			//}
			
			break;
		}
		
		case END_OF_COURSE:
		case STOP:
		{
			break;
		}
		
		case JUNCTION_C_RIGHT:
		case JUNCTION_A_RIGHT:
		case TURN_RIGHT:
		{
			if(start_Yaw_set == false) //Har startvärde INTE satts?
			{
				IMU_Yaw_start = IMU_Yaw;
				start_Yaw_set = true;
			}
			
			else
			{
				Walk_Half_Cycle(0, 0.2, height);
				++rotation_count;
			}
			
			break;
		}
		
		case DEAD_END:
		case JUNCTION_C_LEFT:
		case JUNCTION_A_LEFT:
		case TURN_LEFT:
		{
			if(start_Yaw_set == false)
			{
				IMU_Yaw_start = IMU_Yaw;
				start_Yaw_set = true;
			}
			
			else
			{
				Walk_Half_Cycle(0, -0.2, height);
				++rotation_count;
			}
			
			break;
		}
		
		//case INTO_HIGH_OBSTACLE:
		//{
			//Walk_Half_Cycle(1, alpha, height/2); //Testa fram lagom höjd som roboten ska sänkas till
		//}
		
		//case CRAWLING_UNDER_HIGH_OBSTACLE:
		//{
			//Walk_Half_Cycle(2, alpha, height/2); //Samma höjd som i INTO_HIGH_OBSTACLE
		//}
		
		//case INTO_LOW_OBSTACLE:
		//{
			//Walk_Half_Cycle(1, alpha, 1.5*height); //Testa fram lagom höjd som roboten ska höjas till
		//}
		
		//case CLIMB_UP:
		//{
			////Hårdkodad sekvens av steg för att klättra upp! När hela sekvensen är utförd -> sätt climbed_up till TRUE!
		//}
		
		//case LOW_OBSTACLE:
		//{
			//Walk_Half_Cycle(1, alpha, height/2); //Troligtvis ganska låg höjd här eftersom roboten inte ska luta vid gång över hindret
		//}
		
		//case OUT_OF_LOW_OBSTACLE:
		//{
			////Hårdkodad sekvens av steg för att klättra ned! När hela sekvensen är utförd -> sätt climbed_down till TRUE!
		//}
		
		default:
			break;
	}
}

void load_sensor_values()
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