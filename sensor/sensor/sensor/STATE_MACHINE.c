enum STATES {
	CORRIDOR, OUT_OF_CORRIDOR,
	INTO_HIGH_OBSTACLE, INTO_LOW_OBSTACLE, INTO_TURN_RIGHT, INTO_TURN_LEFT, INTO_JUNCTION_A_RIGHT, INTO_JUNCTION_A_LEFT,
	TURN_RIGHT, TURN_LEFT, JUNCTION_A_LEFT, JUNCTION_A_RIGHT, JUNCTION_B_RIGHT, JUNCTION_B_LEFT, JUNCTION_C_RIGHT, JUNCTION_C_LEFT, DEAD_END, MID_DEAD_END,
	HIGH_OBSTACLE, LOW_OBSTACLE, CRAWLING_OUT_OF_HIGH_OBSTACLE, CRAWLING_UNDER_HIGH_OBSTACLE, CLIMB_UP,
	OUT_OF_TURN_RIGHT, OUT_OF_TURN_LEFT, OUT_OF_JUNCTION_A_RIGHT, OUT_OF_JUNCTION_A_LEFT, OUT_OF_JUNCTION_C_RIGHT, OUT_OF_JUNCTION_C_LEFT, OUT_OF_HIGH_OBSTACLE, OUT_OF_LOW_OBSTACLE,
	END_OF_COURSE
};

#define HALF_ROTATION_ANGLE 85
#define FULL_ROTATION_ANGLE 175
#define CORRIDOR_SIDE_DISTANCE 75
#define SIDE_DEAD_END_DISTANCE 150
#define END_OF_COURSE_DISTANCE 80
#define SHORT_TURN_DISTANCE 30
#define LONG_TURN_DISTANCE 110
#define FORWARD_DEAD_END_DISTANCE 180
#define LOW_OBSTACLE_DISTANCE 40
#define START_CLIMBING_UP_DISTANCE 20
#define START_CLIMBING_DOWN_DISTANCE 50
#define US_HIGH_OBSTACLE_DISTANCE 40
#define IR_HIGH_OBSTACLE_DISTANCE 50
#define NO_WALL_DISTANCE 50
#define DEAD_END_DISTANCE 40				// Avståndet vid vilket vi vänder om vi skulle komma in i en återvändsgränd (vilket ej ska hända)


int8_t IMU_Yaw_start;
int start_Yaw_set;		// 0 = har ej satt ett startvärde

STATES ROBOT_STATE = CORRIDOR;

void calculate_Yaw()
{
	switch (ROBOT_STATE)
	{

		case CORRIDOR:
		{
			Yaw = (IR_Yaw_right + IR_Yaw_left)/2;
			break;
		}
		
		case OUT_OF_TURN_LEFT:
		case OUT_OF_JUNCTION_C_LEFT:
		case JUNCTION_B_LEFT:
		{
			Yaw = IR_Yaw_right;
			break;
		}
		
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
			Yaw = IMU_Yaw - start_Yaw_value;
			break;
		}
		
	}
}

void update_state()
{
	switch (ROBOT_STATE)
	{

		//
		case CORRIDOR:
		{
			if (IR_0 < DEAD_END_DISTANCE && IR_2 < CORRIDOR_SIDE_DISTANCE && IR_3 < CORRIDOR_SIDE_DISTANCE && IR_5 < CORRIDOR_SIDE_DISTANCE && IR_6 < CORRIDOR_SIDE_DISTANCE)
			{
				ROBOT_STATE = DEAD_END;
				break;
			}

			else if (IR_2 > CORRIDOR_SIDE_DISTANCE && IR_3 < CORRIDOR_SIDE_DISTANCE && IR_5 < CORRIDOR_SIDE_DISTANCE && IR_6 < CORRIDOR_SIDE_DISTANCE
			|| IR_2 < CORRIDOR_SIDE_DISTANCE && IR_3 < CORRIDOR_SIDE_DISTANCE && IR_5 < CORRIDOR_SIDE_DISTANCE && IR_6 > CORRIDOR_SIDE_DISTANCE
			|| IR_2 > CORRIDOR_SIDE_DISTANCE && IR_3 < CORRIDOR_SIDE_DISTANCE && IR_5 < CORRIDOR_SIDE_DISTANCE && IR_6 > CORRIDOR_SIDE_DISTANCE)
			{
				ROBOT_STATE = OUT_OF_CORRIDOR;
				break;
			}

			else if (US < US_HIGH_OBSTACLE_DISTANCE && IR_0 > NO_WALL_DISTANCE)
			{
				ROBOT_STATE = INTO_HIGH_OBSTACLE;
				break;
			}
			
			else if (IR_1 < LOW_OBSTACLE_DISTANCE && IR_0 > NO_WALL_DISTANCE)
			{
				ROBOT_STATE = INTO_LOW_OBSTACLE;
				break;
			}
		}
		
		//
		case DEAD_END:
		{
			if (IMU_Yaw - start_Yaw_value >= FULL_ROTATION_ANGLE)
			{
				ROBOT_STATE = CORRIDOR;
				start_Yaw_set = 0;
				break;
			}
			break;
		}
		
		//
		case OUT_OF_CORRIDOR:
		{
			if (IR_2 > CORRIDOR_SIDE_DISTANCE && IR_3 > CORRIDOR_SIDE_DISTANCE && IR_5 < CORRIDOR_SIDE_DISTANCE && IR_6 < CORRIDOR_SIDE_DISTANCE && IR0 < SHORT_TURN_DISTANCE)
			{
				ROBOT_STATE = TURN_RIGHT;
				break;
			}

			else if (IR_2 < CORRIDOR_SIDE_DISTANCE && IR_3 < CORRIDOR_SIDE_DISTANCE && IR_5 > CORRIDOR_SIDE_DISTANCE && IR_6 > CORRIDOR_SIDE_DISTANCE && IR0 < SHORT_TURN_DISTANCE)
			{
				ROBOT_STATE = TURN_LEFT;
				break;
			}

			else if (IR_2 > SIDE_DEAD_END_DISTANCE && IR_3 > SIDE_DEAD_END_DISTANCE && IR_5 < CORRIDOR_SIDE_DISTANCE && IR_6 < CORRIDOR_SIDE_DISTANCE && IR0 < LONG_TURN_DISTANCE)
			{
				ROBOT_STATE = JUNCTION_A_RIGHT;
				break;
			}

			else if (IR_2 < CORRIDOR_SIDE_DISTANCE && IR_3 < CORRIDOR_SIDE_DISTANCE && IR_5 > SIDE_DEAD_END_DISTANCE && IR_6 > SIDE_DEAD_END_DISTANCE && IR0 < LONG_TURN_DISTANCE)
			{
				ROBOT_STATE = JUNCTION_A_LEFT;
				break;
			}

			else if (IR_2 > CORRIDOR_SIDE_DISTANCE && IR_2 < SIDE_DEAD_END_DISTANCE && IR_3 > CORRIDOR_SIDE_DISTANCE && IR_3 < SIDE_DEAD_END_DISTANCE && IR_5 < CORRIDOR_SIDE_DISTANCE && IR_6 < CORRIDOR_SIDE_DISTANCE && IR0 > FORWARD_DEAD_END_DISTANCE)
			{
				ROBOT_STATE = JUNCTION_B_RIGHT;
				break;
			}

			else if (IR_2 < CORRIDOR_SIDE_DISTANCE && IR_3 < CORRIDOR_SIDE_DISTANCE && IR_5 > CORRIDOR_SIDE_DISTANCE && IR_5 < SIDE_DEAD_END_DISTANCE IR_6 > CORRIDOR_SIDE_DISTANCE && IR_6 < SIDE_DEAD_END_DISTANCE && IR0 > FORWARD_DEAD_END_DISTANCE)
			{
				ROBOT_STATE = JUNCTION_B_LEFT;
				break;
			}

			else if (IR_2 > CORRIDOR_SIDE_DISTANCE && IR_2 < SIDE_DEAD_END_DISTANCE && IR_3 > CORRIDOR_SIDE_DISTANCE && IR_3 < SIDE_DEAD_END_DISTANCE && IR_5 > SIDE_DEAD_END_DISTANCE && IR_6 > SIDE_DEAD_END_DISTANCE && IR0 < SHORT_TURN_DISTANCE)
			{
				ROBOT_STATE = JUNCTION_C_LEFT;
				break;
			}

			else if (IR_2 > SIDE_DEAD_END_DISTANCE && IR_3 > SIDE_DEAD_END_DISTANCE && IR_5 > CORRIDOR_SIDE_DISTANCE && IR_5 < SIDE_DEAD_END_DISTANCE IR_6 > CORRIDOR_SIDE_DISTANCE && IR_6 < SIDE_DEAD_END_DISTANCE && IR0 < SHORT_TURN_DISTANCE)
			{
				ROBOT_STATE = JUNCTION_C_RIGHT;
				break;
			}

			else if (IR_0 > END_OF_COURSE_DISTANCE && IR_2 > END_OF_COURSE_DISTANCE && IR_3 > END_OF_COURSE_DISTANCE && IR_5 > END_OF_COURSE_DISTANCE && IR_6 > END_OF_COURSE_DISTANCE)
			{
				ROBOT_STATE = END_OF_COURSE;
				break;
			}
		}
		
		//
		case TURN_RIGHT:
		{
			if (IMU_Yaw - start_Yaw_value >= HALF_ROTATION_ANGLE)
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
			if (IMU_Yaw - start_Yaw_value <= -HALF_ROTATION_ANGLE)
			{
				ROBOT_STATE = OUT_OF_TURN_LEFT;
				start_Yaw_set = 0;
				break;
			}
			break;
		}
		
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
		case JUNCTION_A_RIGHT:
		{
			if (IMU_Yaw - start_Yaw_value >= HALF_ROTATION_ANGLE)
			{
				ROBOT_STATE = OUT_OF_JUNCTION_A_RIGHT;
				start_Yaw_set = 0;
				break;
			}
			break;
		}
		
		//
		case JUNCTION_A_LEFT:
		{
			if (IMU_Yaw - start_Yaw_value <= -HALF_ROTATION_ANGLE)
			{
				ROBOT_STATE = OUT_OF_JUNCTION_A_RIGHT;
				start_Yaw_set = 0;
				break;
			}
			break;
		}
		
		//
		case OUT_OF_JUNCTION_A_RIGHT:
		{
			if (IR_2 < CORRIDOR_SIDE_DISTANCE && IR_3 < CORRIDOR_SIDE_DISTANCE && IR_5 < CORRIDOR_SIDE_DISTANCE && IR_6 < CORRIDOR_SIDE_DISTANCE)
			{
				ROBOT_STATE = CORRIDOR;
				start_Yaw_set = 0;
				break;
			}
			break;
		}

		//
		case OUT_OF_JUNCTION_A_LEFT:
		{
			if (IR_2 < CORRIDOR_SIDE_DISTANCE && IR_3 < CORRIDOR_SIDE_DISTANCE && IR_5 < CORRIDOR_SIDE_DISTANCE && IR_6 < CORRIDOR_SIDE_DISTANCE)
			{
				ROBOT_STATE = CORRIDOR;
				start_Yaw_set = 0;
				break;
			}
			break;
		}
		
		//
		case JUNCTION_B_RIGHT:
		{
			if (IR_2 < CORRIDOR_SIDE_DISTANCE && IR_3 < CORRIDOR_SIDE_DISTANCE && IR_5 < CORRIDOR_SIDE_DISTANCE && IR_6 < CORRIDOR_SIDE_DISTANCE)
			{
				ROBOT_STATE = CORRIDOR;
				break;
			}
			break;
		}

		//
		case JUNCTION_B_LEFT:
		{
			if (IR_2 < CORRIDOR_SIDE_DISTANCE && IR_3 < CORRIDOR_SIDE_DISTANCE && IR_5 < CORRIDOR_SIDE_DISTANCE && IR_6 < CORRIDOR_SIDE_DISTANCE)
			{
				ROBOT_STATE = CORRIDOR;
				break;
			}
			break;
		}
		
		//
		case JUNCTION_C_RIGHT:
		{
			if (IMU_Yaw - start_Yaw_value >= HALF_ROTATION_ANGLE)
			{
				ROBOT_STATE = OUT_OF_JUNCTION_C_RIGHT;
				start_Yaw_set = 0;
				break;
			}
			break;
		}

		//
		case JUNCTION_C_LEFT:
		{
			if (IMU_Yaw - start_Yaw_value <= -HALF_ROTATION_ANGLE)
			{
				ROBOT_STATE = OUT_OF_JUNCTION_C_LEFT;
				start_Yaw_set = 0;
				break;
			}
			break;
		}
		
		//
		case OUT_OF_JUNCTION_C_RIGHT:
		{
			if (IR_2 < CORRIDOR_SIDE_DISTANCE && IR_3 < CORRIDOR_SIDE_DISTANCE && IR_5 < CORRIDOR_SIDE_DISTANCE && IR_6 < CORRIDOR_SIDE_DISTANCE)
			{
				ROBOT_STATE = CORRIDOR;
				break;
			}
			break;
		}

		//
		case OUT_OF_JUNCTION_C_LEFT:
		{
			if (IR_2 < CORRIDOR_SIDE_DISTANCE && IR_3 < CORRIDOR_SIDE_DISTANCE && IR_5 < CORRIDOR_SIDE_DISTANCE && IR_6 < CORRIDOR_SIDE_DISTANCE)
			{
				ROBOT_STATE = CORRIDOR;
				break;
			}
			break;
		}
		
		//
		case END_OF_COURSE:
		{
			break;
		}

		//
		case INTO_HIGH_OBSTACLE:
		{
			if (ride_height = min_height)
			{
				ROBOT_STATE = CRAWLING_UNDER_HIGH_OBSTACLE;
				break;
			}
			break;
		}
		
		//
		case CRAWLING_UNDER_HIGH_OBSTACLE:
		{
			if (IR_4 < IR_HIGH_OBSTACLE_DISTANCE)
			{
				ROBOT_STATE = CRAWLING_OUT_OF_HIGH_OBSTACLE;
				break;
			}
			break;
		}
		
		//
		case CRAWLING_OUT_OF_HIGH_OBSTACLE:
		{
			if (IR_4 > IR_HIGH_OBSTACLE_DISTANCE)
			{
				ROBOT_STATE = OUT_OF_HIGH_OBSTACLE;
				break;
			}
			break;
		}
		
		//
		case OUT_OF_HIGH_OBSTACLE:
		{
			if (ride_height = normal_height)
			{
				ROBOT_STATE = CORRIDOR;
				break;
			}
			break;
		}
		
		//
		case INTO_LOW_OBSTACLE:
		{
			if (IR_1 < START_CLIMBING_UP_DISTANCE)
			{
				ROBOT_STATE = CLIMB_UP;
				break;
			}
			break;
		}
		
		//
		case CLIMB_UP:
		{
			if (climbed_up = true)
			{
				ROBOT_STATE = LOW_OBSTACLE;
				break;
			}
			break;
		}
		
		//
		case LOW_OBSTACLE:
		{
			if (IR_1 > START_CLIMBING_DOWN_DISTANCE)
			{
				ROBOT_STATE = OUT_OF_LOW_OBSTACLE;
				break;
			}
			break;
		}
		
		//
		case OUT_OF_LOW_OBSTACLE:
		{
			if (down_from_low_obst = true)
			{
				ROBOT_STATE = CORRIDOR;
				break;
			}
			break;
		}

	}
}

void run_state()
{
	switch (ROBOT_STATE)
	{

		case TURN_RIGHT:
		{
			if(!start_Yaw_set)
			{
				IMU_Yaw_start = IMU_Yaw;
				start_Yaw_set = 1;
			}
			
			else
			{
				
			}
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
				
			}
			break;
		}
		
		//
		case JUNCTION_A_RIGHT:
		{
			if(!start_Yaw_set)
			{
				IMU_Yaw_start = IMU_Yaw;
				start_Yaw_set = 1;
			}
			
			else
			{
				
			}
			break;
		}
		
		//
		case JUNCTION_A_LEFT:
		{
			if(!start_Yaw_set)
			{
				IMU_Yaw_start = IMU_Yaw;
				start_Yaw_set = 1;
			}
			
			else
			{
				
			}
			break;
		}
		
		//
		case JUNCTION_C_RIGHT:
		{
			if(!start_Yaw_set)
			{
				IMU_Yaw_start = IMU_Yaw;
				start_Yaw_set = 1;
			}
			
			else
			{
				
			}
			break;
		}
		
		//
		case JUNCTION_C_LEFT:
		{
			if(!start_Yaw_set)
			{
				IMU_Yaw_start = IMU_Yaw;
				start_Yaw_set = 1;
			}
			
			else
			{
				
			}
			break;
		}
		
		//
		case DEAD_END:
		{
			if(!start_Yaw_set)
			{
				IMU_Yaw_start = IMU_Yaw;
				start_Yaw_set = 1;
			}
			
			else
			{
				
			}
			break;
		}
		
		//
		case OUT_OF_JUNCTION_A_RIGHT:
		{
			if(!start_Yaw_set)
			{
				IMU_Yaw_start = IMU_Yaw;
				start_Yaw_set = 1;
			}
			
			else
			{
				
			}
			break;
		}
		
		//
		case OUT_OF_JUNCTION_A_LEFT:
		{
			if(!start_Yaw_set)
			{
				IMU_Yaw_start = IMU_Yaw;
				start_Yaw_set = 1;
			}
			
			else
			{
				
			}
			break;
		}
		
	}
}
