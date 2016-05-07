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
		case INTO_HIGH_OBSTACLE:
		case CRAWLING_UNDER_HIGH_OBSTACLE:
		case PREPARE_CLIMBING_DOWN:
		case PREPARE_CLIMBING_UP:
		case LOW_OBSTACLE:
		{
			Yaw = (IR_Yaw_right + IR_Yaw_left)/2;
			break;
		}

		case CORRIDOR:
		{
			if ((fabs(fabs(IR_Yaw_right) - fabs(IR_Yaw_left)) > 10)
			|| ((IR_2 > 50) && (IR_3 < 40) && (IR_5 < 40) && (IR_6 < 40))
			|| ((IR_6 > 50) && (IR_2 < 40) && (IR_3 < 40) && (IR_5 < 40))
			|| ((IR_3 > 50) && (IR_2 < 40) && (IR_5 < 40) && (IR_6 < 40))
			|| ((IR_5 > 50) && (IR_2 < 40) && (IR_3 < 40) && (IR_6 < 40)))
			{
				trust_sensors = false;		// Om det är stor skillnad står roboten förmodligen PRECIS i gränsen till en korsning, bör då ej lita på sensorerna
			}
			else
			{
				Yaw = (IR_Yaw_right + IR_Yaw_left)/2;
			}
			
			break;
		}
		
		case JUNCTION_B_LEFT:
		case OUT_OF_CORRIDOR_RIGHT_WALL:
		case OUT_OF_JUNCTION_RIGHT_WALL:
		case RIGHT_WALL:
		{
			Yaw = IR_Yaw_right;
			break;
		}
		
		case JUNCTION_E_RIGHT:
		case JUNCTION_B_RIGHT:
		case OUT_OF_CORRIDOR_LEFT_WALL:
		case OUT_OF_JUNCTION_LEFT_WALL:
		case LEFT_WALL:
		{
			Yaw = IR_Yaw_left;
			break;
		}
		
		default:
		Yaw = 0;		//Om inget annat är sagt görs ingen reglering
		break;
	}

	Yaw_rad = (Yaw/180)*3.1415926;		//Konvertera till radianer!
}

void calculate_p_part()
{
	switch (ROBOT_STATE)
	{
		case INTO_HIGH_OBSTACLE:
		case CRAWLING_UNDER_HIGH_OBSTACLE:
		case PREPARE_CLIMBING_UP:
		case PREPARE_CLIMBING_DOWN:
		case LOW_OBSTACLE:
		case CORRIDOR:
		{
			p_part = IR_3 - IR_6;
			break;
		}
		
		case JUNCTION_B_LEFT:
		case OUT_OF_CORRIDOR_RIGHT_WALL:
		case OUT_OF_JUNCTION_RIGHT_WALL:
		case RIGHT_WALL:
		{
			p_part = ((IR_2 + IR_3)/2 + CENTRE_OFFSET)*cos(Yaw_rad) - CORRIDOR_WIDTH/2;
			break;
		}
		
		case JUNCTION_E_RIGHT:
		case JUNCTION_B_RIGHT:
		case OUT_OF_CORRIDOR_LEFT_WALL:
		case OUT_OF_JUNCTION_LEFT_WALL:
		case LEFT_WALL:
		{
			p_part = CORRIDOR_WIDTH/2 - ((IR_5 + IR_6)/2 + CENTRE_OFFSET)*cos(Yaw_rad);
			break;
		}
		
		case INTO_CORRIDOR_NO_WALL:
		{
			p_part = (IR_2 - IR_6);
			break;
		}
		
		default:
		p_part = 0;		//Om inget annat är sagt görs ingen reglering
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
				cycle_count = 0;
				ROBOT_STATE = OUT_OF_CORRIDOR_LEFT_WALL;
			}
			
			else if (((IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE))
			|| ((IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE)))
			{
				cycle_count = 0;
				ROBOT_STATE = OUT_OF_CORRIDOR_RIGHT_WALL;
			}
			
			else if (((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE))
			|| ((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE))
			|| ((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE))
			|| ((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE)))
			{
				ROBOT_STATE = OUT_OF_CORRIDOR_NO_WALL;
			}
			
			else if ((US < US_HIGH_OBSTACLE_DISTANCE) && (IR_0 > NO_WALL_DISTANCE) && (IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE))
			{
				ROBOT_STATE = INTO_HIGH_OBSTACLE;
			}
			
			else if ((IR_1 < PREPARE_CLIMBING_UP_DISTANCE) && (IR_0 > NO_WALL_DISTANCE) && (IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE))
			{
				cycle_count = 0;
				ROBOT_STATE = PREPARE_CLIMBING_UP;
			}
			
			break;
		}
		
		case END_OF_COURSE:
		case STOP:
		{
			break;
		}
		
		case OUT_OF_CORRIDOR_LEFT_WALL:
		{
			if ((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE))
			{
				ROBOT_STATE = LEFT_WALL;
				cycle_count = 0;
			}
			
			else if (((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE))
			|| ((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE))
			|| ((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE))
			|| ((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE)))
			{
				ROBOT_STATE = OUT_OF_CORRIDOR_NO_WALL;
			}
			
			break;
		}
		
		case OUT_OF_CORRIDOR_RIGHT_WALL:
		{
			if ((IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE) && (IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE))
			{
				ROBOT_STATE = RIGHT_WALL;
				cycle_count = 0;
			}
			
			else if (((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE))
			|| ((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE))
			|| ((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE))
			|| ((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE)))
			{
				ROBOT_STATE = OUT_OF_CORRIDOR_NO_WALL;
			}
			
			break;
		}
		
		case LEFT_WALL:
		{
			if (cycle_count > 2)
			{
				ROBOT_STATE = DETERMINE_JUNCTION_LEFT_WALL;
			}
			
			break;
		}
		
		case RIGHT_WALL:
		{
			if (cycle_count > 2)
			{
				ROBOT_STATE = DETERMINE_JUNCTION_RIGHT_WALL;
			}
			
			break;
		}
		
		case DETERMINE_JUNCTION_LEFT_WALL:
		{
			if ((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE) && (IR_0 < 70))
			{
				ROBOT_STATE = TURN_RIGHT;
			}
			
			else if ((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) &&
			(IR_6 < CORRIDOR_SIDE_DISTANCE) && (IR_0 > 70) && (IR_0 < 150))
			{
				ROBOT_STATE = JUNCTION_A_RIGHT;
			}

			else if ((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_2 < SIDE_DEAD_END_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_3 < SIDE_DEAD_END_DISTANCE) &&
			(IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE) && (IR_0 > 150))
			{
				ROBOT_STATE = JUNCTION_B_RIGHT;
			}
			
			else if (((IR_2 > SIDE_DEAD_END_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE) && (IR_0 > 150))
			||((IR_3 > SIDE_DEAD_END_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE) && (IR_0 > 150)))
			{
				ROBOT_STATE = JUNCTION_E_RIGHT;
			}
			
			break;
			
		}
		
		case DETERMINE_JUNCTION_RIGHT_WALL:
		{
			if ((IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE) && (IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_0 < 70))
			{
				ROBOT_STATE = TURN_LEFT;
			}
			
			else if ((IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE) && (IR_2 < CORRIDOR_SIDE_DISTANCE) &&
			(IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_0 > 70) && (IR_0 < 150))
			{
				ROBOT_STATE = JUNCTION_A_LEFT;
			}
			
			else if ((IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_5 < SIDE_DEAD_END_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE) && (IR_6 < SIDE_DEAD_END_DISTANCE) &&
			(IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_0 > 150))
			{
				ROBOT_STATE = JUNCTION_B_LEFT;
			}
			
			else if (((IR_5 > SIDE_DEAD_END_DISTANCE) && (IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_0 > 150))
			||((IR_6 > SIDE_DEAD_END_DISTANCE) && (IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_0 > 150)))
			{
				ROBOT_STATE = JUNCTION_E_LEFT;
			}
			
			break;
		}
		
		case OUT_OF_CORRIDOR_NO_WALL:
		{
			if (((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE))
			|| ((IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE)))
			{
				ROBOT_STATE = NO_WALL;
				cycle_count = 0;
			}
			
			break;
		}
		
		case NO_WALL:
		{
			if (cycle_count > 2)
			{
				ROBOT_STATE = DETERMINE_JUNCTION_NO_WALL;
			}
			
			break;
		}
		
		case DETERMINE_JUNCTION_NO_WALL:
		{
			if (((IR_2 < SIDE_DEAD_END_DISTANCE) && (IR_3 < SIDE_DEAD_END_DISTANCE) && (IR_5 > SIDE_DEAD_END_DISTANCE) && (IR_0 < 70))
			|| ((IR_2 < SIDE_DEAD_END_DISTANCE) && (IR_3 < SIDE_DEAD_END_DISTANCE) && (IR_6 > SIDE_DEAD_END_DISTANCE) && (IR_0 < 70)))
			{
				ROBOT_STATE = JUNCTION_C_LEFT;
			}
			
			else if (((IR_5 < SIDE_DEAD_END_DISTANCE) && (IR_6 < SIDE_DEAD_END_DISTANCE) && (IR_2 > SIDE_DEAD_END_DISTANCE) && (IR_0 < 70))
			|| ((IR_5 < SIDE_DEAD_END_DISTANCE) && (IR_6 < SIDE_DEAD_END_DISTANCE) && (IR_3 > SIDE_DEAD_END_DISTANCE) && (IR_0 < 70)))
			{
				ROBOT_STATE = JUNCTION_C_RIGHT;
			}
			
			else if (((IR_2 < SIDE_DEAD_END_DISTANCE) && (IR_3 < SIDE_DEAD_END_DISTANCE) && (IR_5 > SIDE_DEAD_END_DISTANCE) && (IR_0 > 70) && (IR_0 < 150))
			|| ((IR_2 < SIDE_DEAD_END_DISTANCE) && (IR_3 < SIDE_DEAD_END_DISTANCE) && (IR_6 > SIDE_DEAD_END_DISTANCE) && (IR_0 > 70) && (IR_0 < 150)))
			{
				ROBOT_STATE = JUNCTION_D_LEFT;
			}
			
			else if (((IR_5 < SIDE_DEAD_END_DISTANCE) && (IR_6 < SIDE_DEAD_END_DISTANCE) && (IR_2 > SIDE_DEAD_END_DISTANCE) && (IR_0 > 70) && (IR_0 < 150))
			|| ((IR_5 < SIDE_DEAD_END_DISTANCE) && (IR_6 < SIDE_DEAD_END_DISTANCE) && (IR_3 > SIDE_DEAD_END_DISTANCE) && (IR_0 > 70) && (IR_0 < 150)))
			{
				ROBOT_STATE = JUNCTION_D_RIGHT;
			}
			
			else if ((IR_2 < SIDE_DEAD_END_DISTANCE) && (IR_3 < SIDE_DEAD_END_DISTANCE) && (IR_5 < SIDE_DEAD_END_DISTANCE) && (IR_6 < SIDE_DEAD_END_DISTANCE) && (IR_0 > 150))
			{
				ROBOT_STATE = JUNCTION_D_STRAIGHT;
			}
			
			else if (((IR_2 < SIDE_DEAD_END_DISTANCE) && (IR_3 < SIDE_DEAD_END_DISTANCE) && (IR_5 > SIDE_DEAD_END_DISTANCE) && (IR_0 > 150))
			|| ((IR_2 < SIDE_DEAD_END_DISTANCE) && (IR_3 < SIDE_DEAD_END_DISTANCE) && (IR_6 > SIDE_DEAD_END_DISTANCE) && (IR_0 > 150)))
			{
				ROBOT_STATE = JUNCTION_H_LEFT;
				cycle_count = 0;
			}
			
			else if (((IR_5 < SIDE_DEAD_END_DISTANCE) && (IR_6 < SIDE_DEAD_END_DISTANCE) && (IR_2 > SIDE_DEAD_END_DISTANCE) && (IR_0 > 150))
			|| ((IR_5 < SIDE_DEAD_END_DISTANCE) && (IR_6 < SIDE_DEAD_END_DISTANCE) && (IR_3 > SIDE_DEAD_END_DISTANCE) && (IR_0 > 150)))
			{
				ROBOT_STATE = JUNCTION_H_RIGHT;
			}
			
			//else if ((IR_0 > END_OF_COURSE_DISTANCE) && (IR_2 > SIDE_DEAD_END_DISTANCE) && (IR_3 > SIDE_DEAD_END_DISTANCE) && (IR_5 > SIDE_DEAD_END_DISTANCE) && (IR_6 > SIDE_DEAD_END_DISTANCE))
			//{
				//ROBOT_STATE = END_OF_COURSE;
			//}
			
			else if (((IR_2 > SIDE_DEAD_END_DISTANCE) && (IR_6 > SIDE_DEAD_END_DISTANCE) && (IR_0 < 70))
			|| ((IR_2 > SIDE_DEAD_END_DISTANCE) && (IR_5 > SIDE_DEAD_END_DISTANCE) && (IR_0 < 70))
			|| ((IR_3 > SIDE_DEAD_END_DISTANCE) && (IR_6 > SIDE_DEAD_END_DISTANCE) && (IR_0 < 70))
			|| ((IR_3 > SIDE_DEAD_END_DISTANCE) && (IR_5 > SIDE_DEAD_END_DISTANCE) && (IR_0 < 70)))
			{
				ROBOT_STATE = JUNCTION_F;
			}
			
			else if (((IR_2 > SIDE_DEAD_END_DISTANCE) && (IR_6 > SIDE_DEAD_END_DISTANCE) && (IR_0 > 70) && (IR_0 < 150))
			|| ((IR_2 > SIDE_DEAD_END_DISTANCE) && (IR_5 > SIDE_DEAD_END_DISTANCE) && (IR_0 > 70) && (IR_0 < 150))
			|| ((IR_3 > SIDE_DEAD_END_DISTANCE) && (IR_6 > SIDE_DEAD_END_DISTANCE) && (IR_0 > 70) && (IR_0 < 150))
			|| ((IR_3 > SIDE_DEAD_END_DISTANCE) && (IR_5 > SIDE_DEAD_END_DISTANCE) && (IR_0 > 70) && (IR_0 < 150)))
			{
				ROBOT_STATE = JUNCTION_G;
			}
			
			else if (((IR_2 > SIDE_DEAD_END_DISTANCE) && (IR_6 > SIDE_DEAD_END_DISTANCE) && (IR_0 > 150))
			|| ((IR_2 > SIDE_DEAD_END_DISTANCE) && (IR_5 > SIDE_DEAD_END_DISTANCE) && (IR_0 > 150))
			|| ((IR_3 > SIDE_DEAD_END_DISTANCE) && (IR_6 > SIDE_DEAD_END_DISTANCE) && (IR_0 > 150))
			|| ((IR_3 > SIDE_DEAD_END_DISTANCE) && (IR_5 > SIDE_DEAD_END_DISTANCE) && (IR_0 > 150)))
			{
				ROBOT_STATE = JUNCTION_I_OR_END;
				cycle_count = 0;
			}
			
			break;
			
		}
		
		case JUNCTION_A_LEFT:
		case JUNCTION_A_RIGHT:
		case JUNCTION_E_LEFT:
		case JUNCTION_I:
		{
			if ((IR_0 > SIDE_DEAD_END_DISTANCE-30) && (IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE))
			{
				ROBOT_STATE = OUT_OF_JUNCTION_NO_WALL;
			}
			
			break;
		}
		
		case TURN_RIGHT:
		case JUNCTION_C_RIGHT:
		{
			if ((IR_0 > SIDE_DEAD_END_DISTANCE-30) && (IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE))
			{
				ROBOT_STATE = OUT_OF_JUNCTION_LEFT_WALL;
			}
			
			break;
		}
		
		case JUNCTION_F:
		case TURN_LEFT:
		case JUNCTION_C_LEFT:
		{
			if ((IR_0 > SIDE_DEAD_END_DISTANCE-30) && (IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE) && (IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE))
			{
				ROBOT_STATE = OUT_OF_JUNCTION_RIGHT_WALL;
			}
			
			break;
		}
		
		case JUNCTION_D_RIGHT:
		{
			if ((IR_0 > SIDE_DEAD_END_DISTANCE-30) && (IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_5 < SIDE_DEAD_END_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE) && (IR_6 < SIDE_DEAD_END_DISTANCE))
			{
				ROBOT_STATE = OUT_OF_JUNCTION_NO_WALL;
			}
			
			break;
		}
		
		case JUNCTION_G:
		case JUNCTION_D_LEFT:
		{
			if ((IR_0 > SIDE_DEAD_END_DISTANCE-30) && (IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE) && (IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_2 < SIDE_DEAD_END_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_3 < SIDE_DEAD_END_DISTANCE))
			{
				ROBOT_STATE = OUT_OF_JUNCTION_NO_WALL;
			}
			
			break;
		}
		
		case JUNCTION_H_LEFT:
		{
			if ((IR_0 > SIDE_DEAD_END_DISTANCE-30) && (IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE) 
				&& (((IR_2 > SIDE_DEAD_END_DISTANCE) && (IR_5 > SIDE_DEAD_END_DISTANCE)) || ((IR_2 > SIDE_DEAD_END_DISTANCE) && (IR_6 > SIDE_DEAD_END_DISTANCE)) || ((IR_3 > SIDE_DEAD_END_DISTANCE) && (IR_5 > SIDE_DEAD_END_DISTANCE)) || ((IR_3 > SIDE_DEAD_END_DISTANCE) && (IR_6 > SIDE_DEAD_END_DISTANCE))))
			{
				ROBOT_STATE = OUT_OF_JUNCTION_NO_WALL;
			}
			
			break;
		}
		
		case JUNCTION_I_OR_END:
		{
			if (cycle_count > 4)
			{
				if ((IR_0 > 100) && (IR_2 > 100) && (IR_3 > 100))
				{
					ROBOT_STATE = END_OF_COURSE;
				}
				
				else
				{
					ROBOT_STATE = JUNCTION_I;
				}
			}
			
			break;
		}
		
		case DEAD_END:
		{
			if ((IR_0 > SIDE_DEAD_END_DISTANCE-20) && (IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE))
			{
				ROBOT_STATE = CORRIDOR;
			}
			
			break;
		}
		
		case JUNCTION_E_RIGHT:
		case JUNCTION_B_LEFT:
		case JUNCTION_B_RIGHT:
		case INTO_CORRIDOR_NO_WALL:
		case OUT_OF_JUNCTION_RIGHT_WALL:
		case OUT_OF_JUNCTION_LEFT_WALL:
		{
			if ((IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE))
			{
				ROBOT_STATE = CORRIDOR;
			}
			
			break;
		}
		
		case JUNCTION_H_RIGHT:
		case JUNCTION_D_STRAIGHT:
		case OUT_OF_JUNCTION_NO_WALL:
		{
			if (((IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE))
			|| ((IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE))
			|| ((IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE)))
			{
				ROBOT_STATE = INTO_CORRIDOR_NO_WALL;
				trust_sensors = false;
			}
			
			else if ((IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE))
			{
				ROBOT_STATE = CORRIDOR;
			}
			
			break;
		}
		
		case INTO_HIGH_OBSTACLE:
		{
			if (IR_4 < IR_HIGH_OBSTACLE_DISTANCE)
			{
				ROBOT_STATE = CRAWLING_UNDER_HIGH_OBSTACLE;
			}
			break;
		}
		
		case CRAWLING_UNDER_HIGH_OBSTACLE:
		{
			if (IR_4 > IR_HIGH_OBSTACLE_DISTANCE)
			{
				height = 11;
				ROBOT_STATE = CORRIDOR;
			}
			break;
		}
		
		case PREPARE_CLIMBING_UP:
		{
			if (cycle_count > 6) //Testa fram lagom värde så avstånd till hinder blir lagom för klättring
			{
				ROBOT_STATE = CLIMBING_UP;
			}
			break;
		}
		
		case CLIMBING_UP:
		{
			if (on_top_of_obstacle)
			{
				ROBOT_STATE = LOW_OBSTACLE;
			}
			break;
		}
		
		
		case LOW_OBSTACLE:
		{
			if (IR_1 > PREPARE_CLIMBING_DOWN_DISTANCE)
			{
				cycle_count = 0;
				ROBOT_STATE = PREPARE_CLIMBING_DOWN;
			}
			break;
		}
		
		case PREPARE_CLIMBING_DOWN:
		{
			if (cycle_count > 5) //Testa fram lagom värde så avstånd till hinder blir lagom för klättring
			{
				ROBOT_STATE = CLIMBING_DOWN;
			}
			break;
		}
		
		case CLIMBING_DOWN:
		{
			if (!on_top_of_obstacle)
			{
				height = 11;
				ROBOT_STATE = CORRIDOR;
			}
			break;
		}
		
		default:
		break;
	}
}

void run_state()
{
	calculate_Yaw();
	calculate_p_part();
	alpha = Kp*p_part + Kd*Yaw_rad;
	
	fromSen[14] = ROBOT_STATE; // Tillstånd till bussen
	
	switch (ROBOT_STATE)
	{
		case JUNCTION_E_RIGHT:
		case INTO_CORRIDOR_NO_WALL:
		case JUNCTION_B_LEFT:
		case JUNCTION_B_RIGHT:
		case OUT_OF_JUNCTION_LEFT_WALL:
		case OUT_OF_JUNCTION_RIGHT_WALL:
		case CORRIDOR:
		{
			if(trust_sensors)
			{
				Walk_Half_Cycle(4, alpha, height);
			}
			
			else
			{
				Walk_Half_Cycle(4, 0, height);
				trust_sensors = true;			// Default så litar vi på sensorerna
			}
			
			break;
		}

		case OUT_OF_CORRIDOR_RIGHT_WALL:
		case OUT_OF_CORRIDOR_LEFT_WALL:
		{
			if (cycle_count <= 1)
			{
				Walk_Half_Cycle(3, 0, height);
				++cycle_count;
			}
			
			else
			{
				Walk_Half_Cycle(3, alpha, height);
			}
			
			break;
		}
		
		case OUT_OF_CORRIDOR_NO_WALL:
		{
			Walk_Half_Cycle(3, 0, height);
			break;
		}
		
		case NO_WALL:
		{
			Walk_Half_Cycle(4, 0, 0.8*height);
			++cycle_count;
			break;
		}
		
		case RIGHT_WALL:
		case LEFT_WALL:
		{
			Walk_Half_Cycle(4, alpha, 0.8*height);
			++cycle_count;
			break;
		}
		
		case JUNCTION_H_RIGHT:
		case JUNCTION_D_STRAIGHT:
		case OUT_OF_JUNCTION_NO_WALL:
		{
			Walk_Half_Cycle(4, 0, height);
			break;
		}
		
		case DETERMINE_JUNCTION_RIGHT_WALL:
		case DETERMINE_JUNCTION_LEFT_WALL:
		case DETERMINE_JUNCTION_NO_WALL:
		case END_OF_COURSE:
		case STOP:
		{
			break;
		}
		
		case JUNCTION_C_RIGHT:
		case JUNCTION_A_RIGHT:
		case TURN_RIGHT:
		case JUNCTION_D_RIGHT:
		{
			Walk_Half_Cycle(0, 0.2, height);
			break;
		}
		
		case JUNCTION_I:
		case JUNCTION_G:
		case JUNCTION_F:
		case JUNCTION_E_LEFT:
		case DEAD_END:
		case JUNCTION_C_LEFT:
		case JUNCTION_A_LEFT:
		case TURN_LEFT:
		case JUNCTION_D_LEFT:
		case JUNCTION_H_LEFT:
		{
			Walk_Half_Cycle(0, -0.2, height);
			break;
		}
		
		case JUNCTION_I_OR_END:
		{
			Walk_Half_Cycle(0, -0.2, height);
			++cycle_count;
			break;
		}
		
		case INTO_HIGH_OBSTACLE:
		{
			Walk_Half_Cycle(2, alpha, height/2);
			break;
		}
		
		case CRAWLING_UNDER_HIGH_OBSTACLE:
		{
			Walk_Half_Cycle(4, alpha, height/2); //Samma höjd som i INTO_HIGH_OBSTACLE
			break;
		}
		
		case PREPARE_CLIMBING_UP:
		{
			Walk_Half_Cycle(1.3, alpha, 14); //Testa fram lagom höjd och speed
			++cycle_count;
			break;
		}
		
		case CLIMBING_UP:
		{
			Walk_Up_Hard(); //Hårdkodad sekvens för att klättra upp
			on_top_of_obstacle = true;
			break;
		}
		
		case LOW_OBSTACLE:
		{
			Walk_Half_Cycle(1.95, alpha, 7.7);
			break;
		}
		
		case PREPARE_CLIMBING_DOWN:
		{
			Walk_Half_Cycle(2.2, alpha, 7.7); //Samma som i LOW_OBSTACLE
			++cycle_count;
			break;
		}
		
		case CLIMBING_DOWN:
		{
			Walk_Down_Hard();
			on_top_of_obstacle = false;
			break;
		}
		
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