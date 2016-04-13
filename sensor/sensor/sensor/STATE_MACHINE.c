enum STATES {CORRIDOR,
			 INTO_HIGH_OBSTACLE, INTO_LOW_OBSTACLE, INTO_TURN_RIGHT, INTO_TURN_LEFT, INTO_JUNCTION_A_RIGHT, INTO_JUNCTION_A_LEFT,
			 TURN_RIGHT, TURN_LEFT, JUNCTION_A_LEFT, JUNCTION_A_RIGHT, JUNCTION_B_RIGHT, JUNCTION_B_LEFT, JUNCTION_C_RIGHT, JUNCTION_C_LEFT, DEAD_END,
			 HIGH_OBSTACLE,  LOW_OBSTACLE,
			 OUT_OF_TURN_RIGHT, OUT_OF_TURN_LEFT, OUT_OF_JUNCTION_A_RIGHT, OUT_OF_JUNCTION_A_LEFT,
			 END_OF_COURSE};

STATES ROBOT_STATE = CORRIDOR;

void updateState()
{
	switch (ROBOT_STATE)
	{
		case CORRIDOR:
		{
			if (IR_2 > 60 && IR_3 < 60 && IR_5 < 60 && IR_6 < 60 && IR_0 < 100)
			{
				ROBOT_STATE = INTO_TURN_RIGHT;
				break;
			}
			else if (IR_2 < 60 && IR_3 < 60 && IR_5 < 60 && IR_6 > 60 && IR_0 < 100)
			{
				ROBOT_STATE = INTO_TURN_LEFT;
				break;
			}
			else if (IR_2 > MAX && IR_3 < 60 && IR_5 < 60 && IR_6 < 60 && IR_0 > 100)
			{
				ROBOT_STATE = INTO_JUNCTION_A_RIGHT;
				break;
			}
			else if (IR_2 < 60 && IR_3 < 60 && IR_5 < 60 && IR_6 > MAX && IR_0 > 100)
			{
				ROBOT_STATE = INTO_JUNCTION_A_LEFT;
				break;
			}
			
			
			else if (IR_0 < 60 && IR_1 < 60 && UL < 60) //Korta värden
			{
				ROBOT_STATE = DEAD_END;
				break;
			}
			else if (IR_0 > 100 && IR_2 > 100 && IR_3 > 100 && IR_5 > 100 && IR_6 > 100)
			{
				ROBOT_STATE = END_OF_COURSE;
				break;
			}
		}
		
		case INTO_TURN_RIGHT:
		{
			if (IR_0 < 60)
			{
				ROBOT_STATE = TURN_RIGHT;
				break;
			}
		}
		
		case INTO_TURN_LEFT:
		{
			if (IR_0 < 60)
			{
				ROBOT_STATE = TURN_LEFT;
				break;
			}
		}
		
		case INTO_JUNCTION_A_RIGHT:
		{
			if (IR_2 > 60 && IR_3 > 60 && IR_0 < 110)
			{
				ROBOT_STATE = JUNCTION_A_RIGHT;
				break;
			}
		}
		
		case INTO_JUNCTION_A_LEFT:
		{
			if (IR_5 > 60 && IR_6 > 60 && IR_0 < 110)
			{
				ROBOT_STATE = JUNCTION_A_LEFT;
				break;
			}
		}

		case TURN_RIGHT:
		{
			if (YAW > 85)
			{
				ROBOT_STATE = OUT_OF_TURN_RIGHT;
				break;
			}
		}
		
		case TURN_LEFT:
		{
			if (YAW < -85)
			{
				ROBOT_STATE = OUT_OF_TURN_LEFT;
				break;
			}
		}
		
		case JUNCTION_A_RIGHT:
		{
			if (YAW >= 85)
			{
				ROBOT_STATE = OUT_OF_JUNCTION_A_RIGHT;
				break;
			}
		}				
		
		case OUT_OF_TURN_RIGHT:
		{
			if (IR_2 < 60 && IR_3 < 60 && IR_5 < 60 && IR_6 < 60)
			{
				ROBOT_STATE = CORRIDOR;
				break;
			}
		}
		
		case OUT_OF_TURN_LEFT:
		{
			if (IR_2 < 60 && IR_3 < 60 && IR_5 < 60 && IR_6 < 60)
			{
				ROBOT_STATE = CORRIDOR;
				break;
			}
		}
		
		case OUT_OF_JUNCTION_A_RIGHT:
		{
			if (IR_2 < 60 && IR_3 < 60 && IR_5 < 60 && IR_6 < 60)
			{
				ROBOT_STATE = CORRIDOR;
				break;
			}
		}
		
		case OUT_OF_JUNCTION_A_LEFT:
		{
			if (IR_2 < 60 && IR_3 < 60 && IR_5 < 60 && IR_6 < 60)
			{
				ROBOT_STATE = CORRIDOR;
				break;
			}
		}

		case DEAD_END:
		{
			if (YAW >= 180)
			{
				ROBOT_STATE = CORRIDOR;
				break;
			}
			
		}
	}
}