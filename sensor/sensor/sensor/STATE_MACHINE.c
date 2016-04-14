enum STATES {
	CORRIDOR, OUT_OF_CORRIDOR,
	INTO_HIGH_OBSTACLE, INTO_LOW_OBSTACLE, INTO_TURN_RIGHT, INTO_TURN_LEFT, INTO_JUNCTION_A_RIGHT, INTO_JUNCTION_A_LEFT,
	TURN_RIGHT, TURN_LEFT, JUNCTION_A_LEFT, JUNCTION_A_RIGHT, JUNCTION_B_RIGHT, JUNCTION_B_LEFT, JUNCTION_C_RIGHT, JUNCTION_C_LEFT, DEAD_END, MID_DEAD_END,
	HIGH_OBSTACLE, LOW_OBSTACLE, CRAWLING_OUT_OF_HIGH_OBSTACLE, CRAWLING_UNDER_HIGH_OBSTACLE,
	OUT_OF_TURN_RIGHT, OUT_OF_TURN_LEFT, OUT_OF_JUNCTION_A_RIGHT, OUT_OF_JUNCTION_A_LEFT, OUT_OF_JUNCTION_C_RIGHT, OUT_OF_JUNCTION_C_LEFT, OUT_OF_HIGH_OBSTACLE, OUT_OF_LOW_OBSTACLE,
	END_OF_COURSE
};

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
			IMU_Yaw - start_Yaw_value
			break;
		}
		
	}
}

void update_state()
{
	switch (ROBOT_STATE)
	{

		case CORRIDOR:
		{
			if (IR_0 < 20 && IR_2 < 60 && IR_3 < 60 && IR_5 < 60 && IR_6 < 60)
			{
				ROBOT_STATE = DEAD_END;
				break;
			}

			else if (IR_2 > 60 && IR_3 < 60 && IR_5 < 60 && IR_6 < 60 || IR_2 < 60 && IR_3 < 60 && IR_5 < 60 && IR_6 > 60 || IR_2 > 60 && IR_3 < 60 && IR_5 < 60 && IR_6 > 60)
			{
				ROBOT_STATE = OUT_OF_CORRIDOR;
				break;
			}

			else if (US < 40 && IR_0 > 50)
			{
				ROBOT_STATE = INTO_HIGH_OBSTACLE;
				break;
			}
			
			else if (IR_1 < kort && IR_0 > lång)
			{
				ROBOT_STATE = INTO_LOW_OBSTACLE;
				break;
			}
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
		case INTO_LOW_OBSTACLE:
		{
			if (climbed_up = true)
			{
				ROBOT_STATE = LOW_OBSTACLE;
				break;
			}
			break;
		}

		//
		case CRAWLING_UNDER_HIGH_OBSTACLE:
		{
			if (IR_4 < 30)
			{
				ROBOT_STATE = CRAWLING_OUT_OF_HIGH_OBSTACLE;
				break;
			}
			break;
		}

		//
		case CRAWLING_OUT_OF_HIGH_OBSTACLE:
		{
			if (IR_4 > 50)
			{
				ROBOT_STATE = OUT_OF_HIGH_OBSTACLE;
				break;
			}
			break;
		}
		
		//
		case LOW_OBSTACLE:
		{
			if (IR_1 > lång)
			{
				ROBOT_STATE = OUT_OF_LOW_OBSTACLE;
				break;
			}
			break;
		}

		//
		case TURN_RIGHT:
		{
			if (IMU_Yaw - start_Yaw_value >= 85)
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
			if (IMU_Yaw - start_Yaw_value <= -85)
			{
				ROBOT_STATE = OUT_OF_TURN_LEFT;
				start_Yaw_set = 0;
				break;
			}
			break;
		}

		//
		case JUNCTION_A_RIGHT:
		{
			if (IMU_Yaw - start_Yaw_value >= 85)
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
			if (IMU_Yaw - start_Yaw_value <= -85)
			{
				ROBOT_STATE = OUT_OF_JUNCTION_A_RIGHT;
				start_Yaw_set = 0;
				break;
			}
			break;
		}

		//
		case JUNCTION_B_RIGHT:
		{
			if (IR_2 < 60 && IR_3 < 60 && IR_5 < 60 && IR_6 < 60)
			{
				ROBOT_STATE = CORRIDOR;
				break;
			}
			break;
		}

		//
		case JUNCTION_B_LEFT:
		{
			if (IR_2 < 60 && IR_3 < 60 && IR_5 < 60 && IR_6 < 60)
			{
				ROBOT_STATE = CORRIDOR;
				break;
			}
			break;
		}

		//
		case JUNCTION_C_RIGHT:
		{
			if (IMU_Yaw - start_Yaw_value >= 85)
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
			if (IMU_Yaw - start_Yaw_value <= -85)
			{
				ROBOT_STATE = OUT_OF_JUNCTION_C_LEFT;
				start_Yaw_set = 0;
				break;
			}
			break;
		}

		//
		case OUT_OF_TURN_RIGHT:
		{
			if (IR_2 < 60 && IR_3 < 60 && IR_5 < 60 && IR_6 < 60)
			{
				ROBOT_STATE = CORRIDOR;
				break;
			}
			break;
		}

		//
		case OUT_OF_TURN_LEFT:
		{
			if (IR_2 < 60 && IR_3 < 60 && IR_5 < 60 && IR_6 < 60)
			{
				ROBOT_STATE = CORRIDOR;
				break;
			}
			break;
		}

		//
		case OUT_OF_JUNCTION_A_RIGHT:
		{
			if (IR_2 < 60 && IR_3 < 60 && IR_5 < 60 && IR_6 < 60)
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
			if (IR_2 < 60 && IR_3 < 60 && IR_5 < 60 && IR_6 < 60)
			{
				ROBOT_STATE = CORRIDOR;
				start_Yaw_set = 0;
				break;
			}
			break;
		}

		//
		case OUT_OF_JUNCTION_C_RIGHT:
		{
			if (IR_2 < 60 && IR_3 < 60 && IR_5 < 60 && IR_6 < 60)
			{
				ROBOT_STATE = CORRIDOR;
				break;
			}
			break;
		}

		//
		case OUT_OF_JUNCTION_C_LEFT:
		{
			if (IR_2 < 60 && IR_3 < 60 && IR_5 < 60 && IR_6 < 60)
			{
				ROBOT_STATE = CORRIDOR;
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
		case OUT_OF_LOW_OBSTACLE:
		{
			if (down_from_low_obst = true)
			{
				ROBOT_STATE = CORRIDOR;
				break;
			}
			break;
		}

		//
		case OUT_OF_CORRIDOR:
		{
			if (IR_2 > 60 && IR_3 > 60 && IR_5 < 60 && IR_6 < 60 && IR0 < 20)
			{
				ROBOT_STATE = TURN_RIGHT;
				break;
			}

			else if (IR_2 < 60 && IR_3 < 60 && IR_5 > 60 && IR_6 > 60 && IR0 < 20)
			{
				ROBOT_STATE = TURN_LEFT;
				break;
			}

			else if (IR_2 > 150 && IR_3 > 150 && IR_5 < 60 && IR_6 < 60 && IR0 < 110)
			{
				ROBOT_STATE = JUNCTION_A_RIGHT;
				break;
			}

			else if (IR_2 < 60 && IR_3 < 60 && IR_5 > 150 && IR_6 > 150 && IR0 < 110)
			{
				ROBOT_STATE = JUNCTION_A_LEFT;
				break;
			}

			else if (IR_2 > 60 && IR_2 < 120 IR_3 > 60 && IR_3 < 120 && IR_5 < 60 && IR_6 < 60 && IR0 > 150)
			{
				ROBOT_STATE = JUNCTION_B_RIGHT;
				break;
			}

			else if (IR_2 < 60 && IR_3 < 60 && IR_5 > 60 && IR_5 < 120 IR_6 > 60 && IR_6 < 120 && IR0 > 150)
			{
				ROBOT_STATE = JUNCTION_B_LEFT;
				break;
			}

			else if (IR_2 > 60 && IR_2 < 120 IR_3 > 60 && IR_3 < 120 && IR_5 > 150 && IR_6 > 150 && IR0 < 20)
			{
				ROBOT_STATE = JUNCTION_C_LEFT;
				break;
			}

			else if (IR_2 > 150 && IR_3 > 150 && IR_5 > 60 && IR_5 < 120 IR_6 > 60 && IR_6 < 120 && IR0 < 20)
			{
				ROBOT_STATE = JUNCTION_C_RIGHT;
				break;
			}

			else if (IR_0 > 100 && IR_2 > 100 && IR_3 > 100 && IR_5 > 100 && IR_6 > 100)
			{
				ROBOT_STATE = END_OF_COURSE;
				break;
			}
		}

		//
		case END_OF_COURSE:
		{
			break;
		}

		//
		case DEAD_END:
		{
			if (IMU_Yaw - start_Yaw_value <= -85)
			{
				ROBOT_STATE = MID_DEAD_END;
				start_Yaw_set = 0;
				break;
			}
			break;
		}
		
		//
		case MID_DEAD_END:
		{
			if (IMU_Yaw - start_Yaw_value <= -85)
			{
				ROBOT_STATE = CORRIDOR;
				start_Yaw_set = 0;
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
		case MID_DEAD_END:
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
