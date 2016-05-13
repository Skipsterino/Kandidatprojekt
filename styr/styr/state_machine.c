/*
* state_machine.c
*
* Created: 4/8/2016 2:53:09 PM
*  Author: fregu856
*/

#include "state_machine.h"

#define HALF_ROTATION_ANGLE 45			/**< Rotation angle for a 90 degree turn. */
#define FULL_ROTATION_ANGLE 90			/**< Rotation angle for a 180 degree turn. */
#define CORRIDOR_SIDE_DISTANCE 65		/**< Distance for determining whether corridor or not. */
#define SIDE_DEAD_END_DISTANCE 150		/**< Distance for determining whether dead end to right or left in junction. */
#define END_OF_COURSE_DISTANCE 100		/**< Distance that IR_0, 2, 3, 5, 6 should be larger than at the end of the course. */
#define SHORT_TURN_DISTANCE 38			/**< Distance to wall for rotating in turns and C junctions. */
#define JUNCTION_C_TURN_DISTANCE 45
#define FORWARD_DEAD_END_DISTANCE 180	/**< Distance for determining whether dead end straight ahead in junction or not. */
#define JUNCTION_A_FORWARD_DISTANCE 80	/**< Distance for determining whether A junction or not (junction or turn?). */
#define PREPARE_CLIMBING_UP_DISTANCE 30	/**< Distance for preparing climbing onto low obstacle (if IR_1 is less than this). */
#define PREPARE_CLIMBING_DOWN_DISTANCE 40	/**< Distance for preparing climbing down from low obstacle (if IR_1 is greater than this). */
#define US_HIGH_OBSTACLE_DISTANCE 40	/**< Distance for determining whether high obstacle or not (ultrasound sensor). */
#define IR_HIGH_OBSTACLE_DISTANCE 60	/**< Distance for determining whether high obstacle or not (IR sensor). */
#define NO_WALL_DISTANCE 120				/**< Distance that IR_0 should be larger than near an obstacle (to tell obstacles and walls apart). */
#define DEAD_END_DISTANCE 48			/**< Distance to wall for turning in a dead end (which we shouldn't have entered). */

#define CENTRE_OFFSET 8 /**< Horizontal distance from center of robot to its legs. */
#define CORRIDOR_WIDTH 80 /**< Width of the labyrinth's corridors. */

#define STANDARD_HEIGHT 11 /**< Standard height for use in corridors etc. */
#define HIGH_OBSTACLE_HEIGHT 7 /**< Height for use in high obstacles. */
#define LOW_OBSTACLE_HIGH_HEIGHT 14 /**< Height for use in preparing for low obstacles. */
#define LOW_OBSTACLE_LOW_HEIGHT 7.7 /**< Height for use on top of low obstacles. */

int previous_state;

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
		
		case CHECK_IF_HIGH_OBSTACLE:
		case CHECK_IF_LOW_OBSTACLE:
		case SLOW_CORRIDOR:
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
		case SLOW_CORRIDOR:
		case INTO_HIGH_OBSTACLE:
		case CRAWLING_UNDER_HIGH_OBSTACLE:
		case CORRIDOR:
		{
			p_part = IR_3 - IR_6;
			break;
		}
		
		case CENTER_CRAB_UP:
		case CENTER_CRAB_DOWN:
		{
			p_part = (IR_3 - IR_6);
			break;
		}
		
		case JUNCTION_B_LEFT:
		case OUT_OF_CORRIDOR_RIGHT_WALL:
		case OUT_OF_JUNCTION_RIGHT_WALL:
		case RIGHT_WALL:
		{
			p_part = ((IR_2 + IR_3)/2 + CENTRE_OFFSET + 1)*cos(Yaw_rad) - CORRIDOR_WIDTH/2;
			break;
		}
		
		case JUNCTION_E_RIGHT:
		case JUNCTION_B_RIGHT:
		case OUT_OF_CORRIDOR_LEFT_WALL:
		case OUT_OF_JUNCTION_LEFT_WALL:
		case LEFT_WALL:
		{
			p_part = CORRIDOR_WIDTH/2 - ((IR_5 + IR_6)/2 + CENTRE_OFFSET - 1)*cos(Yaw_rad);
			break;
		}
		
		case INTO_CORRIDOR_NO_WALL:
		{
			p_part = (IR_2 - IR_6)*0.75;
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
	
	switch (ROBOT_STATE)
	{
		case CORRIDOR:
		{
			
			if ((IR_0 < DEAD_END_DISTANCE) && (IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE))
			{
				ROBOT_STATE = DEAD_END;
				cycle_count = 0;
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
				cycle_count = 0;
				ROBOT_STATE = CHECK_IF_HIGH_OBSTACLE;
			}
			
			else if ((IR_1 < PREPARE_CLIMBING_UP_DISTANCE) && (IR_0 > NO_WALL_DISTANCE) && (IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE))
			{
				cycle_count = 0;
				previous_state = CORRIDOR;
				ROBOT_STATE = CHECK_IF_LOW_OBSTACLE;
			}
			
			break;
		}
		
		case SLOW_CORRIDOR:
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
				cycle_count = 0;
				ROBOT_STATE = CHECK_IF_HIGH_OBSTACLE;
			}
			
			else if ((IR_1 < PREPARE_CLIMBING_UP_DISTANCE) && (IR_0 > NO_WALL_DISTANCE) && (IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE))
			{
				cycle_count = 0;
				previous_state = SLOW_CORRIDOR;
				ROBOT_STATE = CHECK_IF_LOW_OBSTACLE;
			}
			
			else if (cycle_count > 13)
			{
				ROBOT_STATE = CORRIDOR;
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
			if (((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE))
			|| ((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE))
			|| ((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE))
			|| ((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE)))
			{
				ROBOT_STATE = OUT_OF_CORRIDOR_NO_WALL;
			}
			
			else if (((cycle_count > 4) && (IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE))
			|| ((cycle_count > 4) &&  (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE)))
			{
				ROBOT_STATE = LEFT_WALL;
				cycle_count = 0;
			}
			
			break;
		}
		
		case OUT_OF_CORRIDOR_RIGHT_WALL:
		{
			if (((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE))
			|| ((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE))
			|| ((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE))
			|| ((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE)))
			{
				ROBOT_STATE = OUT_OF_CORRIDOR_NO_WALL;
			}
			
			else if (((cycle_count > 4) && (IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE) && (IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE))
			|| ((cycle_count > 4) && (IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE)))
			{
				ROBOT_STATE = RIGHT_WALL;
				cycle_count = 0;
			}
			
			break;
		}
		
		case LEFT_WALL:
		{
			if (cycle_count > 3)
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
			if ((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE)
			&& (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE)
			&& (IR_0 < 70) && ((IR_2 > SIDE_DEAD_END_DISTANCE) || (IR_3 > SIDE_DEAD_END_DISTANCE)))
			{
				ROBOT_STATE = TURN_RIGHT;
				cycle_count = 0;
			}
			
			else if ((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE) && (IR_0 > 70) && (IR_0 < 150) && ((IR_2 > SIDE_DEAD_END_DISTANCE) || (IR_3 > SIDE_DEAD_END_DISTANCE)))
			{
				ROBOT_STATE = JUNCTION_A_RIGHT;
				cycle_count = 0;
			}

			else if ((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_2 < SIDE_DEAD_END_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_3 < SIDE_DEAD_END_DISTANCE) &&
			(IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE) && (IR_0 > 140))
			{
				ROBOT_STATE = JUNCTION_B_RIGHT;
			}
			
			else if (((IR_2 > SIDE_DEAD_END_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE) && (IR_0 > 150))
			||((IR_3 > SIDE_DEAD_END_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE) && (IR_0 > 150)))
			{
				ROBOT_STATE = JUNCTION_E_RIGHT;
			}
			
			else if ((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_2 < SIDE_DEAD_END_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_3 < SIDE_DEAD_END_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE) && (IR_0 < 70))
			{
				ROBOT_STATE = DEAD_END_A_RIGHT;
				cycle_count = 0;
			}
			
			else if ((IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_2 < SIDE_DEAD_END_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_3 < SIDE_DEAD_END_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE) && (IR_0 > 70) && (IR_0 < 140))
			{
				ROBOT_STATE = DEAD_END_B_RIGHT;
				cycle_count = 0;
			}
			
			break;
			
		}
		
		case DETERMINE_JUNCTION_RIGHT_WALL:
		{
			if ((IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE)
			&& (IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE)
			&& (IR_0 < 70) && ((IR_5 > SIDE_DEAD_END_DISTANCE) || (IR_6 > SIDE_DEAD_END_DISTANCE)))
			{
				ROBOT_STATE = TURN_LEFT;
				cycle_count = 0;
			}
			
			else if ((IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE) && (IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_0 > 70) && (IR_0 < 150) && ((IR_5 > SIDE_DEAD_END_DISTANCE) || (IR_6 > SIDE_DEAD_END_DISTANCE)))
			{
				ROBOT_STATE = JUNCTION_A_LEFT;
				cycle_count = 0;
			}
			
			else if ((IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_5 < SIDE_DEAD_END_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE) && (IR_6 < SIDE_DEAD_END_DISTANCE) &&
			(IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_0 > 140))
			{
				ROBOT_STATE = JUNCTION_B_LEFT;
			}
			
			else if (((IR_5 > SIDE_DEAD_END_DISTANCE) && (IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_0 > 150))
			||((IR_6 > SIDE_DEAD_END_DISTANCE) && (IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_0 > 150)))
			{
				ROBOT_STATE = JUNCTION_E_LEFT;
				cycle_count = 0;
			}
			
			else if ((IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_5 < SIDE_DEAD_END_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE) && (IR_6 < SIDE_DEAD_END_DISTANCE) && (IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_0 < 70))
			{
				ROBOT_STATE = DEAD_END_A_LEFT;
				cycle_count = 0;
			}
			
			else if ((IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_5 < SIDE_DEAD_END_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE) && (IR_6 < SIDE_DEAD_END_DISTANCE) && (IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_0 > 70) && (IR_0 < 140))
			{
				ROBOT_STATE = DEAD_END_B_LEFT;
				cycle_count = 0;
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
			if (cycle_count > 3)
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
				cycle_count = 0;
			}
			
			else if (((IR_5 < SIDE_DEAD_END_DISTANCE) && (IR_6 < SIDE_DEAD_END_DISTANCE) && (IR_2 > SIDE_DEAD_END_DISTANCE) && (IR_0 < 70))
			|| ((IR_5 < SIDE_DEAD_END_DISTANCE) && (IR_6 < SIDE_DEAD_END_DISTANCE) && (IR_3 > SIDE_DEAD_END_DISTANCE) && (IR_0 < 70)))
			{
				ROBOT_STATE = JUNCTION_C_RIGHT;
				cycle_count = 0;
			}
			
			else if (((IR_2 < SIDE_DEAD_END_DISTANCE) && (IR_3 < SIDE_DEAD_END_DISTANCE) && (IR_5 > SIDE_DEAD_END_DISTANCE) && (IR_0 > 70) && (IR_0 < 150))
			|| ((IR_2 < SIDE_DEAD_END_DISTANCE) && (IR_3 < SIDE_DEAD_END_DISTANCE) && (IR_6 > SIDE_DEAD_END_DISTANCE) && (IR_0 > 70) && (IR_0 < 150)))
			{
				ROBOT_STATE = JUNCTION_D_LEFT;
				cycle_count = 0;
			}
			
			else if (((IR_5 < SIDE_DEAD_END_DISTANCE) && (IR_6 < SIDE_DEAD_END_DISTANCE) && (IR_2 > SIDE_DEAD_END_DISTANCE) && (IR_0 > 70) && (IR_0 < 150))
			|| ((IR_5 < SIDE_DEAD_END_DISTANCE) && (IR_6 < SIDE_DEAD_END_DISTANCE) && (IR_3 > SIDE_DEAD_END_DISTANCE) && (IR_0 > 70) && (IR_0 < 150)))
			{
				ROBOT_STATE = JUNCTION_D_RIGHT;
				cycle_count = 0;
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
			
			else if (((IR_2 > SIDE_DEAD_END_DISTANCE) && (IR_6 > SIDE_DEAD_END_DISTANCE) && (IR_0 < 70))
			|| ((IR_2 > SIDE_DEAD_END_DISTANCE) && (IR_5 > SIDE_DEAD_END_DISTANCE) && (IR_0 < 70))
			|| ((IR_3 > SIDE_DEAD_END_DISTANCE) && (IR_6 > SIDE_DEAD_END_DISTANCE) && (IR_0 < 70))
			|| ((IR_3 > SIDE_DEAD_END_DISTANCE) && (IR_5 > SIDE_DEAD_END_DISTANCE) && (IR_0 < 70)))
			{
				ROBOT_STATE = JUNCTION_F;
				cycle_count = 0;
			}
			
			else if (((IR_2 > SIDE_DEAD_END_DISTANCE) && (IR_6 > SIDE_DEAD_END_DISTANCE) && (IR_0 > 70) && (IR_0 < 150))
			|| ((IR_2 > SIDE_DEAD_END_DISTANCE) && (IR_5 > SIDE_DEAD_END_DISTANCE) && (IR_0 > 70) && (IR_0 < 150))
			|| ((IR_3 > SIDE_DEAD_END_DISTANCE) && (IR_6 > SIDE_DEAD_END_DISTANCE) && (IR_0 > 70) && (IR_0 < 150))
			|| ((IR_3 > SIDE_DEAD_END_DISTANCE) && (IR_5 > SIDE_DEAD_END_DISTANCE) && (IR_0 > 70) && (IR_0 < 150)))
			{
				ROBOT_STATE = JUNCTION_G;
				cycle_count = 0;
			}
			
			else if (((IR_2 > SIDE_DEAD_END_DISTANCE) && (IR_6 > SIDE_DEAD_END_DISTANCE) && (IR_0 > 150))
			|| ((IR_2 > SIDE_DEAD_END_DISTANCE) && (IR_5 > SIDE_DEAD_END_DISTANCE) && (IR_0 > 150))
			|| ((IR_3 > SIDE_DEAD_END_DISTANCE) && (IR_6 > SIDE_DEAD_END_DISTANCE) && (IR_0 > 150))
			|| ((IR_3 > SIDE_DEAD_END_DISTANCE) && (IR_5 > SIDE_DEAD_END_DISTANCE) && (IR_0 > 150)))
			{
				ROBOT_STATE = JUNCTION_I_OR_END;
				cycle_count = 0;
			}
			
			else if ((IR_2 < SIDE_DEAD_END_DISTANCE) && (IR_3 < SIDE_DEAD_END_DISTANCE) && (IR_5 < SIDE_DEAD_END_DISTANCE) && (IR_6 < SIDE_DEAD_END_DISTANCE) && (IR_0 < 70))
			{
				ROBOT_STATE = DEAD_END_C;
				cycle_count = 0;
			}
			
			else if ((IR_2 < SIDE_DEAD_END_DISTANCE) && (IR_3 < SIDE_DEAD_END_DISTANCE) && (IR_5 < SIDE_DEAD_END_DISTANCE) && (IR_6 < SIDE_DEAD_END_DISTANCE) && (IR_0 > 70) && (IR_0 < 150))
			{
				ROBOT_STATE = DEAD_END_D;
				cycle_count = 0;
			}
			
			break;
			
		}
		
		case DEAD_END_D:
		case DEAD_END_C:
		case JUNCTION_A_RIGHT:
		{
			if ((IR_0 > SIDE_DEAD_END_DISTANCE-40) && (IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE))
			{
				Walk_Half_Cycle(0, -0.15, STANDARD_HEIGHT);
				ROBOT_STATE = OUT_OF_JUNCTION_NO_WALL;
			}
			
			break;
		}
		
		case JUNCTION_A_LEFT:
		{
			if ((IR_0 > SIDE_DEAD_END_DISTANCE-40) && (IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE))
			{
				Walk_Half_Cycle(0, 0.15, STANDARD_HEIGHT);
				ROBOT_STATE = OUT_OF_JUNCTION_NO_WALL;
			}
			
			break;
		}
		
		case JUNCTION_E_LEFT:
		case JUNCTION_I:
		{
			if ((IR_0 > SIDE_DEAD_END_DISTANCE-40) && (IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE))
			{
				Walk_Half_Cycle(0, 0.15, STANDARD_HEIGHT);
				ROBOT_STATE = OUT_OF_JUNCTION_NO_WALL;
			}
			
			break;
		}
		
		case DEAD_END_B_LEFT:
		case DEAD_END_A_LEFT:
		case TURN_RIGHT:
		case JUNCTION_C_RIGHT:
		{
			if ((IR_0 > SIDE_DEAD_END_DISTANCE-60) && (IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE))
			{
				Walk_Half_Cycle(0, -0.15, STANDARD_HEIGHT);
				ROBOT_STATE = OUT_OF_JUNCTION_LEFT_WALL;
			}
			
			break;
		}
		
		case DEAD_END_B_RIGHT:
		case DEAD_END_A_RIGHT:
		case JUNCTION_F:
		case TURN_LEFT:
		case JUNCTION_C_LEFT:
		{
			if ((IR_0 > SIDE_DEAD_END_DISTANCE-60) && (IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE) && (IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE))
			{
				Walk_Half_Cycle(0, 0.15, STANDARD_HEIGHT);
				ROBOT_STATE = OUT_OF_JUNCTION_RIGHT_WALL;
			}
			
			break;
		}
		
		case JUNCTION_D_RIGHT:
		{
			if ((IR_0 > SIDE_DEAD_END_DISTANCE-40) && (IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_5 < SIDE_DEAD_END_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE) && (IR_6 < SIDE_DEAD_END_DISTANCE))
			{
				Walk_Half_Cycle(0, -0.15, STANDARD_HEIGHT);
				ROBOT_STATE = OUT_OF_JUNCTION_NO_WALL;
			}
			
			break;
		}
		
		case JUNCTION_G:
		case JUNCTION_D_LEFT:
		{
			if ((IR_0 > SIDE_DEAD_END_DISTANCE-40) && (IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE) && (IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_2 < SIDE_DEAD_END_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_3 < SIDE_DEAD_END_DISTANCE))
			{
				Walk_Half_Cycle(0, 0.15, STANDARD_HEIGHT);
				ROBOT_STATE = OUT_OF_JUNCTION_NO_WALL;
			}
			
			break;
		}
		
		case JUNCTION_H_LEFT:
		{
			if ((IR_0 > SIDE_DEAD_END_DISTANCE-60) && (IR_2 > CORRIDOR_SIDE_DISTANCE) && (IR_3 > CORRIDOR_SIDE_DISTANCE) && (IR_5 > CORRIDOR_SIDE_DISTANCE) && (IR_6 > CORRIDOR_SIDE_DISTANCE)
			&& (((IR_2 > SIDE_DEAD_END_DISTANCE) && (IR_5 > SIDE_DEAD_END_DISTANCE)) || ((IR_2 > SIDE_DEAD_END_DISTANCE) && (IR_6 > SIDE_DEAD_END_DISTANCE)) || ((IR_3 > SIDE_DEAD_END_DISTANCE) && (IR_5 > SIDE_DEAD_END_DISTANCE)) || ((IR_3 > SIDE_DEAD_END_DISTANCE) && (IR_6 > SIDE_DEAD_END_DISTANCE))))
			{
				Walk_Half_Cycle(0, 0.15, STANDARD_HEIGHT);
				ROBOT_STATE = OUT_OF_JUNCTION_NO_WALL;
			}
			
			break;
		}
		
		case JUNCTION_I_OR_END:
		{
			if (cycle_count > 3)
			{
				if ((IR_0 > 100) && (IR_2 > 100) && (IR_3 > 100))
				{
					cycle_count = 0;
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
				Walk_Half_Cycle(0, -0.18, STANDARD_HEIGHT);
				ROBOT_STATE = CORRIDOR;
			}
			
			break;
		}
		
		case JUNCTION_E_RIGHT:
		case JUNCTION_B_LEFT:
		case JUNCTION_B_RIGHT:
		case OUT_OF_JUNCTION_RIGHT_WALL:
		case OUT_OF_JUNCTION_LEFT_WALL:
		{
			if ((IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE))
			{
				ROBOT_STATE = CORRIDOR;
			}
			
			break;
		}
		
		case INTO_CORRIDOR_NO_WALL:
		{
			if ((IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE))
			{
				previous_alpha = 0;
				ROBOT_STATE = SLOW_CORRIDOR;
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
				cycle_count = 0;
			}
			
			else if ((IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE))
			{
				ROBOT_STATE = SLOW_CORRIDOR;
			}
			
			break;
		}
		
		case CHECK_IF_HIGH_OBSTACLE:
		{
			if (cycle_count > 5)
			{
				if ((US < US_HIGH_OBSTACLE_DISTANCE) && (IR_0 > NO_WALL_DISTANCE))// && (IR_2 < CORRIDOR_SIDE_DISTANCE) && (IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE))
				{
					ROBOT_STATE = INTO_HIGH_OBSTACLE;
					Walk_Half_Cycle(0, 0, HIGH_OBSTACLE_HEIGHT);
				}
				else
				{
					ROBOT_STATE = CORRIDOR; //Har inte gått framåt så måste vara i korridor här
				}
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
				ROBOT_STATE = CORRIDOR;
			}
			break;
		}
		
		case CHECK_IF_LOW_OBSTACLE:
		{
			if (cycle_count > 5)
			{
				if ((IR_1 < PREPARE_CLIMBING_UP_DISTANCE) && (IR_0 > NO_WALL_DISTANCE) && (IR_2 < CORRIDOR_SIDE_DISTANCE) &&
				(IR_3 < CORRIDOR_SIDE_DISTANCE) && (IR_5 < CORRIDOR_SIDE_DISTANCE) && (IR_6 < CORRIDOR_SIDE_DISTANCE))
				{
					cycle_count = 0;
					ROBOT_STATE = CENTER_CRAB_UP;
				}
				else
				{
					ROBOT_STATE = CORRIDOR; //Har inte gått framåt så måste vara i korridor här
				}
			}
			break;
		}
		
		case CENTER_CRAB_UP:
		{
			if (cycle_count > 5)
			{
				cycle_count = 0;
				previous_alpha = 0;
				ROBOT_STATE = PREPARE_CLIMBING_UP;
			}
			break;
		}
		
		case PREPARE_CLIMBING_UP:
		{
			if ((previous_state == CORRIDOR) && (cycle_count > 4)) //Testa fram lagom värde så avstånd till hinder blir lagom för klättring
			{
				ROBOT_STATE = CLIMBING_UP;
			}

			if ((previous_state == SLOW_CORRIDOR) && (cycle_count > 7)) //Testa fram lagom värde så avstånd till hinder blir lagom för klättring
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
				ROBOT_STATE = CENTER_CRAB_DOWN;
			}
			break;
		}
		
		case CENTER_CRAB_DOWN:
		{
			if (cycle_count > 5)
			{
				cycle_count = 0;
				previous_alpha = 0;
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
				ROBOT_STATE = SLOW_CORRIDOR;
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
	
	if (!((ROBOT_STATE == CENTER_CRAB_UP) || (ROBOT_STATE == CENTER_CRAB_DOWN)))	// LP-filtrera såvida vi inte ska köra crab_walk
	{
		float alpha_diff = alpha - previous_alpha;
		
		if(alpha_diff > 0.29)
		{
			alpha = previous_alpha + 0.29;
		}
		else if(alpha_diff < -0.29)
		{
			alpha = previous_alpha - 0.29;
		}
		
		previous_alpha = alpha;
	}
		
	fromSen[14] = ROBOT_STATE; // Tillstånd till bussen
	
	switch (ROBOT_STATE)
	{
		case JUNCTION_E_RIGHT:
		case JUNCTION_B_LEFT:
		case JUNCTION_B_RIGHT:
		case CORRIDOR:
		{
			if(trust_sensors)
			{
				Walk_Half_Cycle(4.5, alpha, STANDARD_HEIGHT);
			}
			
			else
			{
				Walk_Half_Cycle(4.5, 0, STANDARD_HEIGHT);
				trust_sensors = true;			// Default så litar vi på sensorerna
			}
			
			break;
		}
		
		case OUT_OF_JUNCTION_LEFT_WALL:
		case OUT_OF_JUNCTION_RIGHT_WALL:
		{
			Walk_Half_Cycle(4.5, alpha, STANDARD_HEIGHT);
			break;
		}
		
		case SLOW_CORRIDOR:
		{
			if(trust_sensors)
			{
				Walk_Half_Cycle(2.5, alpha, STANDARD_HEIGHT);
				++cycle_count;
			}
			
			else
			{
				Walk_Half_Cycle(2.5, 0, STANDARD_HEIGHT);
				trust_sensors = true;			// Default så litar vi på sensorerna
				++cycle_count;
			}
			
			break;
		}
		
		case INTO_CORRIDOR_NO_WALL:
		{
			if (cycle_count < 2)
			{
				Walk_Half_Cycle(2.5, 0, STANDARD_HEIGHT);
				++cycle_count;
			}
			
			else
			{
				Walk_Half_Cycle(2.5, alpha, STANDARD_HEIGHT);
			}
			
			break;
		}

		case OUT_OF_CORRIDOR_RIGHT_WALL:
		case OUT_OF_CORRIDOR_LEFT_WALL:
		{
			if (cycle_count <= 4)
			{
				Walk_Half_Cycle(2.5, 0, STANDARD_HEIGHT);
				++cycle_count;
			}
			
			else
			{
				Walk_Half_Cycle(2.5, alpha, STANDARD_HEIGHT);
			}
			
			break;
		}
		
		case OUT_OF_CORRIDOR_NO_WALL:
		{
			Walk_Half_Cycle(2.5, 0, STANDARD_HEIGHT);
			break;
		}
		
		case NO_WALL:
		{
			Walk_Half_Cycle(2.5, 0, STANDARD_HEIGHT);
			++cycle_count;
			break;
		}
		
		case RIGHT_WALL:
		case LEFT_WALL:
		{
			Walk_Half_Cycle(2.5, alpha, STANDARD_HEIGHT);
			++cycle_count;
			break;
		}
		
		case JUNCTION_H_RIGHT:
		case JUNCTION_D_STRAIGHT:
		case OUT_OF_JUNCTION_NO_WALL:
		{
			Walk_Half_Cycle(2.5, 0, STANDARD_HEIGHT);
			break;
		}
		
		case DETERMINE_JUNCTION_RIGHT_WALL:
		case DETERMINE_JUNCTION_LEFT_WALL:
		case DETERMINE_JUNCTION_NO_WALL:
		case STOP:
		{
			break;
		}
		
		case END_OF_COURSE:
		{
			if (cycle_count == 0)
			{
				Victory_Dance();
				++cycle_count;
			}
			
			else
			{
				
			}
			break;
			
		}
		
		case DEAD_END_D:
		case DEAD_END_C:
		case DEAD_END:
		case DEAD_END_B_LEFT:
		case DEAD_END_A_LEFT:
		case JUNCTION_C_RIGHT:
		case JUNCTION_A_RIGHT:
		case TURN_RIGHT:
		case JUNCTION_D_RIGHT:
		{
			if (cycle_count == 0)
			{
				Walk_Half_Cycle(0, 0.2, STANDARD_HEIGHT);
				++cycle_count;
				break;
			}
			
			Walk_Half_Cycle(0, 0.3, STANDARD_HEIGHT);
			++cycle_count;
			break;
		}
		
		case DEAD_END_B_RIGHT:
		case DEAD_END_A_RIGHT:
		case JUNCTION_I:
		case JUNCTION_G:
		case JUNCTION_F:
		case JUNCTION_E_LEFT:
		case JUNCTION_C_LEFT:
		case JUNCTION_A_LEFT:
		case TURN_LEFT:
		case JUNCTION_D_LEFT:
		case JUNCTION_H_LEFT:
		{
			if (cycle_count == 0)
			{
				Walk_Half_Cycle(0, -0.2, STANDARD_HEIGHT);
				++cycle_count;
				break;
			}
			
			Walk_Half_Cycle(0, -0.3, STANDARD_HEIGHT);
			++cycle_count;
			break;
		}
		
		case JUNCTION_I_OR_END:
		{
			if (cycle_count == 0)
			{
				Walk_Half_Cycle(0, -0.2, STANDARD_HEIGHT);
				++cycle_count;
				break;
			}
			
			Walk_Half_Cycle(0, -0.3, STANDARD_HEIGHT);
			++cycle_count;
			break;
			
		}
		
		case INTO_HIGH_OBSTACLE:
		{
			Walk_Half_Cycle(4.5, alpha, HIGH_OBSTACLE_HEIGHT);
			break;
		}
		
		case CRAWLING_UNDER_HIGH_OBSTACLE:
		{
			Walk_Half_Cycle(4.5, alpha, HIGH_OBSTACLE_HEIGHT); //Samma höjd som i INTO_HIGH_OBSTACLE
			break;
		}
		
		case CHECK_IF_HIGH_OBSTACLE:
		case CHECK_IF_LOW_OBSTACLE:
		{
			Walk_Half_Cycle(0, alpha, STANDARD_HEIGHT);
			++cycle_count;
			break;
		}
		
		case PREPARE_CLIMBING_UP:
		{
			Walk_Half_Cycle(1.5, alpha, LOW_OBSTACLE_HIGH_HEIGHT); //Testa fram lagom höjd och speed
			++cycle_count;
			break;
		}
		
		case CLIMBING_UP:
		{
			Walk_Up(); //Hårdkodad sekvens för att klättra upp
			on_top_of_obstacle = true;
			break;
		}
		
		case LOW_OBSTACLE:
		{
			Walk_Half_Cycle(1.5, alpha, LOW_OBSTACLE_LOW_HEIGHT);
			break;
		}
		
		case CENTER_CRAB_UP:
		case CENTER_CRAB_DOWN:
		{
			if (alpha > 2)
			{
				alpha = 2;
			}
			
			Walk_Half_Crab_Cycle(alpha*50);
			++cycle_count;
			break;
		}
		
		case PREPARE_CLIMBING_DOWN:
		{
			Walk_Half_Cycle(1.65, alpha, LOW_OBSTACLE_LOW_HEIGHT); //Samma som i LOW_OBSTACLE
			++cycle_count;
			break;
		}
		
		case CLIMBING_DOWN:
		{
			Walk_Down();
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