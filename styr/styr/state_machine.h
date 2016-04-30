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
#include <stdbool.h>

/**
* Enum that contains the different possible states that the robot can be in.
*/
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
	JUNCTION_A_RIGHT = 13, 
	JUNCTION_A_LEFT = 14, 
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
	OUT_OF_JUNCTION_A = 39,
} STATES;

#define HALF_ROTATION_ANGLE 45			/**< Rotation angle for a 90 degree turn. */
#define FULL_ROTATION_ANGLE 90			/**< Rotation angle for a 180 degree turn. */
#define CORRIDOR_SIDE_DISTANCE 60		/**< Distance for determining whether corridor or not. */
#define SIDE_DEAD_END_DISTANCE 150		/**< Distance for determining whether dead end to right or left in junction. */
#define END_OF_COURSE_DISTANCE 80		/**< Distance that IR_0, 2, 3, 5, 6 should be larger than at the end of the course. */
#define SHORT_TURN_DISTANCE 40			/**< Distance to wall for rotating in turns and C junctions. */
#define LONG_TURN_DISTANCE 113			/**< Distance to wall for rotating in A junctions. */
#define JUNCTION_C_TURN_DISTANCE 45
#define FORWARD_DEAD_END_DISTANCE 180	/**< Distance for determining whether dead end straight ahead in junction or not. */
#define JUNCTION_A_FORWARD_DISTANCE 80	/**< Distance for determining whether A junction or not (junction or turn?). */
#define LOW_OBSTACLE_DISTANCE 40		/**< Distance for determining whether low obstacle or not. If IR_1 is less than this, slow down. */
#define START_CLIMBING_UP_DISTANCE 20	/**< Distance for beginning climbing onto low obstacle (if IR_1 is less than this). */
#define START_CLIMBING_DOWN_DISTANCE 50	/**< Distance for beginning climbing down from low obstacle (if IR_1 is greater than this). */
#define US_HIGH_OBSTACLE_DISTANCE 40	/**< Distance for determining whether high obstacle or not (ultrasound sensor). */
#define IR_HIGH_OBSTACLE_DISTANCE 40	/**< Distance for determining whether high obstacle or not (IR sensor). */	
#define NO_WALL_DISTANCE 60				/**< Distance that IR_0 should be larger than near an obstacle (to tell obstacles and walls apart). */
#define DEAD_END_DISTANCE 40			/**< Distance to wall for turning in a dead end (which we shouldn't even enter). */

#define CENTRE_OFFSET 8 /**< Horizontal distance from centre of robot to its legs. */
#define CORRIDOR_WIDTH 80 /**< Width of the labyrinth's corridors. */

float IMU_Yaw_start; /**< The IMU's start Yaw angle when entering turns and junctions. */ 
bool climbed_up; /**< true -> the robot has climbed up on top of a low obstacle. */
bool climbed_down; /**< true -> the robot has climbed down from a low obstacle. */
bool trust_sensors; /**< false -> for some reason, we don't trust the current sensor data. */
uint8_t rotation_count; /**< Counter that keeps track of rotation in a turn or junction. */
uint8_t cycle_count;

float Yaw, Yaw_rad, p_part, Kp, Kd, alpha; /**< Angles and parameters for motion control. */
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
* @param height Indicates the robot's distance to ground.
*/
void run_state(float height);

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

