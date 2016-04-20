/*
 * state_machine.h
 *
 * Created: 4/8/2016 2:53:17 PM
 *  Author: chrma018
 */ 


#ifndef STATE_MACHINE_H_
#define STATE_MACHINE_H_

#ifndef F_CPU
#define F_CPU 16000000UL		// 16 MHz
#endif

#include "SPI.h"
#include "reglering.h"
#include "gangstilar.h"

typedef enum {
	CORRIDOR, OUT_OF_CORRIDOR,
	INTO_HIGH_OBSTACLE, INTO_LOW_OBSTACLE, INTO_TURN_RIGHT, INTO_TURN_LEFT, INTO_JUNCTION_A_RIGHT, INTO_JUNCTION_A_LEFT,
	TURN_RIGHT, TURN_LEFT, JUNCTION_A_LEFT, JUNCTION_A_RIGHT, JUNCTION_B_RIGHT, JUNCTION_B_LEFT, JUNCTION_C_RIGHT, JUNCTION_C_LEFT, DEAD_END, MID_DEAD_END,
	HIGH_OBSTACLE, LOW_OBSTACLE, CRAWLING_OUT_OF_HIGH_OBSTACLE, CRAWLING_UNDER_HIGH_OBSTACLE, CLIMB_UP,
	OUT_OF_TURN_RIGHT, OUT_OF_TURN_LEFT, OUT_OF_JUNCTION_A_RIGHT, OUT_OF_JUNCTION_A_LEFT, OUT_OF_JUNCTION_C_RIGHT, OUT_OF_JUNCTION_C_LEFT, OUT_OF_HIGH_OBSTACLE, OUT_OF_LOW_OBSTACLE,
	END_OF_COURSE, STOP
} STATES;

#define HALF_ROTATION_ANGLE 85
#define FULL_ROTATION_ANGLE 175
#define CORRIDOR_SIDE_DISTANCE 50
#define SIDE_DEAD_END_DISTANCE 150
#define END_OF_COURSE_DISTANCE 80
#define SHORT_TURN_DISTANCE 35
#define LONG_TURN_DISTANCE 110
#define FORWARD_DEAD_END_DISTANCE 180
#define LOW_OBSTACLE_DISTANCE 40
#define START_CLIMBING_UP_DISTANCE 20
#define START_CLIMBING_DOWN_DISTANCE 50
#define US_HIGH_OBSTACLE_DISTANCE 40
#define IR_HIGH_OBSTACLE_DISTANCE 50
#define NO_WALL_DISTANCE 50
#define DEAD_END_DISTANCE 40				// Avståndet vid vilket vi vänder om vi skulle komma in i en återvändsgränd (vilket ej ska hända)


float IMU_Yaw_start;
int start_Yaw_set;		// 0 = har ej satt ett startvärde
int rotation_count;

float Yaw;
float IR_0, IR_1, IR_2, IR_3, IR_4, IR_5, IR_6, US, IR_Yaw_left, IR_Yaw_right, IMU_Yaw, Yaw, Pitch, Roll;

STATES ROBOT_STATE;

void update_state();
void run_state(float height_value);
void load_sensor_values();
void calculate_Yaw();

#endif /* STATE_MACHINE_H_ */