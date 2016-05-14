/**
* @file state_machine.h
* @author Fredrik, Erik
* @date 25 apr 2016
* @brief State machine for the robot's autonomous mode
*
* Contains the robot's state machine, which handles updating and execution of the robot's
* state in its autonomous mode.
*/

#ifndef STATE_MACHINE_H_
#define STATE_MACHINE_H_

#ifndef F_CPU
#define F_CPU 16000000UL		// 16 MHz
#endif

#include "SPI.h"
#include "gangstilar.h"
#include "over_hinder.h"
#include <stdbool.h>

/**
* Enum that contains the different possible states that the robot can be in.
*/
typedef enum {
	STOP = 1,
	DEAD_END = 2,
	CORRIDOR = 3,
	
	OUT_OF_CORRIDOR_RIGHT_WALL = 4,
	OUT_OF_CORRIDOR_LEFT_WALL = 5,
	OUT_OF_CORRIDOR_NO_WALL = 6,
	
	LEFT_WALL = 7,
	RIGHT_WALL = 8,
	NO_WALL = 9,
	
	DETERMINE_JUNCTION_RIGHT_WALL = 10,
	DETERMINE_JUNCTION_LEFT_WALL = 11,

	DETERMINE_JUNCTION_NO_WALL = 12,

	OUT_OF_JUNCTION_RIGHT_WALL = 13,	
	OUT_OF_JUNCTION_LEFT_WALL = 14,
	OUT_OF_JUNCTION_NO_WALL = 15,
	
	INTO_CORRIDOR_NO_WALL = 16,
	
	INTO_HIGH_OBSTACLE = 17,
	CRAWLING_UNDER_HIGH_OBSTACLE = 18,
	
	PREPARE_CLIMBING_UP = 19,
	CLIMBING_UP = 20,
	LOW_OBSTACLE = 21,
	PREPARE_CLIMBING_DOWN = 22,
	CLIMBING_DOWN = 23,
	
	TURN_RIGHT = 24,
	TURN_LEFT = 25,
	JUNCTION_A_RIGHT = 26,
	JUNCTION_A_LEFT = 27,
	JUNCTION_B_RIGHT = 28,
	JUNCTION_B_LEFT = 29,
	JUNCTION_C_RIGHT = 30,
	JUNCTION_C_LEFT = 31,
	JUNCTION_D_RIGHT = 32,
	JUNCTION_D_LEFT = 33,
	JUNCTION_D_STRAIGHT = 34,
	JUNCTION_E_RIGHT = 35,
	JUNCTION_E_LEFT = 36,
	JUNCTION_F = 37,
	JUNCTION_G = 38,
	JUNCTION_H_RIGHT = 39,
	JUNCTION_H_LEFT = 40,
	JUNCTION_I = 41,
	
	JUNCTION_I_OR_END = 42,
	END_OF_COURSE = 43,
	
	SLOW_CORRIDOR = 44,			// TEST_STATE_1
	DEAD_END_A_RIGHT = 45,		// TEST_STATE_2	
	DEAD_END_A_LEFT = 46,		// TEST_STATE_3	
	DEAD_END_B_RIGHT = 47,		// TEST_STATE_4
	DEAD_END_B_LEFT = 48,		// TEST_STATE_5
	DEAD_END_C = 49,			// TEST_STATE_6
	DEAD_END_D = 50,			// TEST_STATE_7
	CHECK_IF_LOW_OBSTACLE = 51,
	CHECK_IF_HIGH_OBSTACLE = 52,
	CENTER_CRAB = 53,
	CENTER_NORMAL = 54,
} STATES;

bool on_top_of_obstacle; /**< true -> the robot has climbed up on top of a low obstacle. */
bool trust_sensors; /**< false -> for some reason, we don't trust the current sensor data. */
uint8_t rotation_count; /**< Counter that keeps track of rotation in a turn or junction. */
uint8_t cycle_count; /**< Counter that keeps track of the number of cycles. Is used in some, but not all, states. */

float Kp, Kd;
float Yaw, Yaw_rad, p_part, alpha, previous_alpha; /**< Angles and parameters for motion control. */
float IR_0, IR_1, IR_2, IR_3, IR_4, IR_5, IR_6, US, IR_Yaw_left, IR_Yaw_right, IMU_Yaw, Yaw, Pitch, Roll; /**< Sensor values. */

STATES ROBOT_STATE; /**< Represents the robot's current state. */

/**
* @brief Updates the robot's current state
*
* Updates robot state based on current state and sensor values.
*/
void update_state();

/**
* @brief Executes the robot's current state
*
* Executes robot state based on current state and sensor values.
*/
void run_state();

/**
* @brief Loads sensor values
*
* Loads sensor values into corresponding variables.
*/
void load_sensor_values();

/**
* @brief Calculates current Yaw angle
*
* Calculates current Yaw angle (for control) in different ways
* depending on current state.
*/
void calculate_Yaw();

/**
* @brief Calculates current P part
*
* Calculates current P part (for control) in different ways
* depending on current state.
*/
void calculate_p_part();

#endif /* STATE_MACHINE_H_ */

