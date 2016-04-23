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
#include "gangstilar.h"

typedef enum {
	CORRIDOR = 1, 
	OUT_OF_CORRIDOR_NO_WALL = 2, 
	OUT_OF_CORRIDOR_LEFT_WALL = 3, 
	OUT_OF_CORRIDOR_RIGHT_WALL = 4,
	
	INTO_HIGH_OBSTACLE = 5, 
	INTO_LOW_OBSTACLE = 6, 
	INTO_TURN_RIGHT = 7, 
	INTO_TURN_LEFT = 8, 
	INTO_JUNCTION_A_RIGHT = 9, 
	INTO_JUNCTION_A_LEFT = 10,
	
	TURN_RIGHT = 11, 
	TURN_LEFT = 12, 
	JUNCTION_A_LEFT = 13, 
	JUNCTION_A_RIGHT = 14, 
	JUNCTION_B_RIGHT = 15, 
	JUNCTION_B_LEFT = 16, 
	JUNCTION_C_RIGHT = 17, 
	JUNCTION_C_LEFT = 18, 
	
	DEAD_END = 19,
	MID_DEAD_END = 20,
	
	HIGH_OBSTACLE = 21, 
	LOW_OBSTACLE = 22, 
	CRAWLING_OUT_OF_HIGH_OBSTACLE = 23, 
	CRAWLING_UNDER_HIGH_OBSTACLE = 24, 
	CLIMB_UP = 25,
	
	OUT_OF_TURN_RIGHT = 26, 
	OUT_OF_TURN_LEFT = 27, 
	OUT_OF_JUNCTION_A_RIGHT = 28, 
	OUT_OF_JUNCTION_A_LEFT = 29, 
	OUT_OF_JUNCTION_C_RIGHT = 30, 
	OUT_OF_JUNCTION_C_LEFT = 31, 
	OUT_OF_HIGH_OBSTACLE = 32, 
	OUT_OF_LOW_OBSTACLE = 33,
	
	END_OF_COURSE = 34, 
	STOP = 35,
	INTO_JUNCTION_C_RIGHT = 36,
	INTO_JUNCTION_C_LEFT = 37,
	INTO_CORRIDOR_NO_WALL = 38,
} STATES;

#define HALF_ROTATION_ANGLE 85			// Vinkeln vi ska rotera till vid en 90-graders rotation
#define FULL_ROTATION_ANGLE 175			// Vinkeln vi ska rotera till vid en 180-graders rotation
#define CORRIDOR_SIDE_DISTANCE 60		// Gränsavstånd för om vi är i korridor eller inte (om sidosensorerna visar mindre än detta avstånd är vi korridor, annars inte)
#define SIDE_DEAD_END_DISTANCE 150		// Gränsavstånd för en återvändsgränd till höger eller vänster i en korsning (fri väg till höger/vänster eller inte)
#define END_OF_COURSE_DISTANCE 100		// Avståndet som IR_0, 2, 3, 5, 6 ska vara större än vid banslutet
#define SHORT_TURN_DISTANCE 40			// Avståndet vid vilket vi ska rotera vid svängar och korsning C.
#define LONG_TURN_DISTANCE 110			// Avståndet vid vilket vi ska rotera vid korsning A
#define FORWARD_DEAD_END_DISTANCE 180		// Gränsavstånd för en återvändsgränd frammåt i en korsning (fri väg framåt eller inte)
#define LOW_OBSTACLE_DISTANCE 40		// Avståndet på IR_1 vid vilken lågt hinder har upptäckts och vi ska börja gå försiktigt
#define START_CLIMBING_UP_DISTANCE 20		// Avståndet på IR_1 vid vilken vi ska börja klättra upp på ett lågt hinder
#define START_CLIMBING_DOWN_DISTANCE 50		// Avståndet på IR_1 vid vilken vi ska börja gå ner ifrån ett lågt hinder
#define US_HIGH_OBSTACLE_DISTANCE 40		// Gränsavståndet för ultraljudssensorn (högt hinder eller inte)
#define IR_HIGH_OBSTACLE_DISTANCE 50		// Gränsavståndet för sensorn riktad uppåt (högt hinder i vägen eller inte)	
#define NO_WALL_DISTANCE 50			// Avståndet som IR_0 ska vara större än vid ett hinder för att säkerställa att det inte bara är en vägg
#define DEAD_END_DISTANCE 40			// Avståndet vid vilket vi vänder om vi skulle komma in i en återvändsgränd (vilket ej ska hända)

#define CENTER_OFFSET 8
#define CORRIDOR_WIDTH 80

float IMU_Yaw_start;
int start_Yaw_set;		// 0 = har ej satt ett startvärde
int rotation_count;

float Yaw, Yaw_rad, p_part, Kp, Kd, alpha;
float IR_0, IR_1, IR_2, IR_3, IR_4, IR_5, IR_6, US, IR_Yaw_left, IR_Yaw_right, IMU_Yaw, Yaw, Pitch, Roll;

STATES ROBOT_STATE;

void update_state();
void run_state(float height_value);
void load_sensor_values();
void calculate_Yaw();
void calculate_p_part();

#endif /* STATE_MACHINE_H_ */
