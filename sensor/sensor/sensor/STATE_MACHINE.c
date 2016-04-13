enum STATES {
	CORRIDOR, OUT_OF_CORRIDOR,
	INTO_HIGH_OBSTACLE, INTO_LOW_OBSTACLE, INTO_TURN_RIGHT, INTO_TURN_LEFT, INTO_JUNCTION_A_RIGHT, INTO_JUNCTION_A_LEFT,
	TURN_RIGHT, TURN_LEFT, JUNCTION_A_LEFT, JUNCTION_A_RIGHT, JUNCTION_B_RIGHT, JUNCTION_B_LEFT, JUNCTION_C_RIGHT, JUNCTION_C_LEFT, DEAD_END,
	HIGH_OBSTACLE, LOW_OBSTACLE,
	OUT_OF_TURN_RIGHT, OUT_OF_TURN_LEFT, OUT_OF_JUNCTION_A_RIGHT, OUT_OF_JUNCTION_A_LEFT, OUT_OF_JUNCTION_C_RIGHT, OUT_OF_JUNCTION_C_LEFT,
	END_OF_COURSE
};

STATES ROBOT_STATE = CORRIDOR;

void updateState()
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
	}

	//
	case INTO_HIGH_OBSTACLE:
	{
		if (ride_height = min_height)
		{
			ROBOT_STATE = LOW_CRAWL;
			break;
		}
	}

	//
	case LOW_CRAWL:
	{
		if (IR_4 < 30)
		{
			ROBOT_STATE = BELOW_HIGH_OBSTACLE;
			break;
		}
	}

	//
	case BELOW_HIGH_OBSTACLE:
	{
		if (IR_4 > 50)
		{
			ROBOT_STATE = OUT_OF_HIGH_OBSTACLE;
			break;
		}
	}

	//
	case OUT_OF_HIGH_OBSTACLE:
	{
		if (ride_height = normal_height)
		{
			ROBOT_STATE = CORRIDOR;
			break;
		}
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
	case TURN_RIGHT:
	{
		if (Yaw > 85)
		{
			ROBOT_STATE = OUT_OF_TURN_RIGHT;
			break;
		}
	}

	//
	case TURN_LEFT:
	{
		if (Yaw < -85)
		{
			ROBOT_STATE = OUT_OF_TURN_LEFT;
			break;
		}
	}

	//
	case JUNCTION_A_RIGHT:
	{
		if (Yaw >= 85)
		{
			ROBOT_STATE = OUT_OF_JUNCTION_A_RIGHT;
			break;
		}
	}

	case JUNCTION_A_LEFT:
	{
		if (Yaw <= -85)
		{
			ROBOT_STATE = OUT_OF_JUNCTION_A_RIGHT;
			break;
		}
	}

	//
	case JUNCTION_B_RIGHT:
	{
		if (IR_2 < 60 && IR_3 < 60 && IR_5 < 60 && IR_6 < 60)
		{
			ROBOT_STATE = CORRIDOR;
			break;
		}
	}

	//
	case JUNCTION_B_LEFT:
	{
		if (IR_2 < 60 && IR_3 < 60 && IR_5 < 60 && IR_6 < 60)
		{
			ROBOT_STATE = CORRIDOR;
			break;
		}
	}

	//
	case JUNCTION_C_RIGHT:
	{
		if (Yaw >= 85)
		{
			ROBOT_STATE = OUT_OF_JUNCTION_C_RIGHT;
			break;
		}
	}

	//
	case JUNCTION_C_LEFT:
	{
		if (Yaw <= -85)
		{
			ROBOT_STATE = OUT_OF_JUNCTION_C_LEFT;
			break;
		}
	}

	//
	case OUT_OF_TURN_RIGHT:
	{
		if (IR_2 < 60 && IR_3 < 60 && IR_5 < 60 && IR_6 < 60)
		{
			ROBOT_STATE = CORRIDOR;
			break;
		}
	}

	//
	case OUT_OF_TURN_LEFT:
	{
		if (IR_2 < 60 && IR_3 < 60 && IR_5 < 60 && IR_6 < 60)
		{
			ROBOT_STATE = CORRIDOR;
			break;
		}
	}

	//
	case OUT_OF_JUNCTION_A_RIGHT:
	{
		if (IR_2 < 60 && IR_3 < 60 && IR_5 < 60 && IR_6 < 60)
		{
			ROBOT_STATE = CORRIDOR;
			break;
		}
	}

	//
	case OUT_OF_JUNCTION_A_LEFT:
	{
		if (IR_2 < 60 && IR_3 < 60 && IR_5 < 60 && IR_6 < 60)
		{
			ROBOT_STATE = CORRIDOR;
			break;
		}
	}

	//
	case OUT_OF_JUNCTION_C_RIGHT:
	{
		if (IR_2 < 60 && IR_3 < 60 && IR_5 < 60 && IR_6 < 60)
		{
			ROBOT_STATE = CORRIDOR;
			break;
		}
	}

	//
	case OUT_OF_JUNCTION_C_LEFT:
	{
		if (IR_2 < 60 && IR_3 < 60 && IR_5 < 60 && IR_6 < 60)
		{
			ROBOT_STATE = CORRIDOR;
			break;
		}
	}

	//
	case END_OF_COURSE:
	{

	}

	//
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
