/**
* @file sensor.h
* @version 1.0
* @author Fredrik, Jonas
* @date 26 apr 2016
* @brief The code governing the behaviour of the sensor module
*
* The code that governs the sensor module. Handles the collection of all the sensor
* data.
*/

#ifndef SENSOR_H_
#define SENSOR_H_

#define F_CPU 16000000UL	/**< Define the frequency for the CPU as 16 MHz */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <math.h>
#include <util/delay.h>
#include <compat/twi.h>

#define x 0				/**< Define the x as zero for index in matrices */
#define y 1				/**< Define the y as one for index in matrices */
#define z 2				/**< Define the z as two for index in matrices */

volatile uint8_t SPI_done = 0;		/**< Flag for if there's been a SPI transmission recently */
volatile uint8_t SPI_overflow = 0;	/**< Help variable for the timer since last SPI transmission */

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const int delay_time = 50;			/**< A delay for testing to simulate activity on the SPI-bus */

const double IR_sensor_distance_right = 15.5;		/**< Distance between the two IR-sensors on the right side [cm] */
const double IR_sensor_distance_left = 15.5;		/**< Distance between the two IR-sensors on the left side [cm] */
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

volatile double IR_latest_reading[7];	/**< The most recent readings from all the IR-sensors */
double IR_reading[7][5];				/**< 2D-array with the 5 latest readings for the 7 IR-sensors */
double IR_ADC[7];						/**< Average value for the last 5 readings for the 7 IR-sensors */
double IR_distance[7];					/**< Distance measured by the 7 IR-sensors */
float IR_Yaw_right, IR_Yaw_left;		/**< Yaw angle calculated from the IR-measurements on each side */

/**
* Struct that contains the tables that describe the conversion from sensor readings to distances
*/
typedef struct
{
	double ADC_data;
	double distance;
} ADC_distance_pair;

volatile double US_latest_reading;				/**< The most recent reading from the US sensor */
double US_reading[5];
double US_distance;						/**< Distance calculated from the US sensor */

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t buffer0_IR0 = 0xf0;				/**< Buffer containing distance from IR_0 to be put on the bus */
uint8_t buffer1_IR1 = 0xf1;				/**< Buffer containing distance from IR_1 to be put on the bus */
uint8_t buffer2_IR2 = 0xf2;				/**< Buffer containing distance from IR_2 to be put on the bus */
uint8_t buffer3_IR3 = 0xf3;				/**< Buffer containing distance from IR_3 to be put on the bus */
uint8_t buffer4_IR4 = 0xf4;				/**< Buffer containing distance from IR_4 to be put on the bus */
uint8_t buffer5_IR5 = 0xf5;				/**< Buffer containing distance from IR_5 to be put on the bus */
uint8_t buffer6_IR6 = 0xf6;				/**< Buffer containing distance from IR_6 to be put on the bus */
uint8_t buffer7_US = 0xf7;				/**< Buffer containing distance from US to be put on the bus */
int8_t buffer8_IR_Yaw_left = 0xf8;		/**< Buffer containing Yaw angle calculated with IR sensors on the left side to be put on the bus */
int8_t buffer9_IR_Yaw_right = 0xf9;		/**< Buffer containing Yaw angle calculated with IR sensors on the right side to be put on the bus */
int8_t buffer10_IMU_Yaw_Low = 0xfa;		/**< Buffer containing low bits of the Yaw angle from the IMU */
int8_t buffer11_IMU_Yaw_High = 0xfb;	/**< Buffer containing high bits of the Yaw angle from the IMU */
int8_t buffer12_Pitch = 0xfc;			/**< Buffer containing Pitch angle from the IMU */
int8_t buffer13_Roll = 0xfd;			/**< Buffer containing Roll angle from the IMU */

volatile int byte_to_send = 0;			/**< Counter on which byte to send next */

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define QUAT_W 0	/**< Define index 0 as quaternion w */
#define QUAT_X 1	/**< Define index 1 as quaternion x */
#define QUAT_Y 2	/**< Define index 2 as quaternion y */
#define QUAT_Z 3	/**< Define index 3 as quaternion z */

#define MPU_HZ 12	/**< Frequency with which the IMU generates interrupts (DMP always running at 200 Hz internally) */
#define USE_DMP 1	/**< Use the internal DMP */

volatile unsigned char IMU_data_ready = 0;							/**< Flag stating if there's been a interrupt by the IMU */

const signed char _orientation[9] = {1, 0, 0,  0, 1, 0,  0, 0, 1};	/**< Rotation matrix describing the mounting of the IMU */

float quaternion[4];					/**< Quaternions describing the orientation of the IMU */
float gravity[3];						/**< The gravity vectors used to remove the effect of the gravity on the measurements from the IMU */
float IMU_Yaw, IMU_Pitch, IMU_Roll;		/**< The orientation of the IMU in Euler angles */

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double counter = 0;				// XXXXX Endast för sensor-kalibrering
double sum = 0;					// XXXXX Endast för sensor-kalibrering
int initial_counter = 0;		// XXXXX Endast för sensor-kalibrering
double result;					// XXXXX Endast för sensor-kalibrering

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ADC_distance_pair IR0_table[] =
{
	{57.2, 220},
	{89.2, 150},
	{96.0, 140},
	{104.0, 130},
	{113.6, 120},
	{123.9, 110},
	{136.6, 100},
	{151.0, 90},
	{168.1, 80},
	{194.9, 70},
	{227.7, 60},
	{273.5, 50},
	{305.5, 45},
	{341.7, 40},
	{392.3, 35},
	{445.2, 30},
	{505.4, 25},
	{566.3, 20},
	{598.7, 15}
}; /**< Table describing relation between sensor value to distance for IR sensor 0 */
ADC_distance_pair IR1_table[] =
{
	{39.3, 150},
	{83.9, 80},
	{93.0, 70},
	{106.1, 60},
	{127.3, 50},
	{135.7, 45},
	{151.9, 40},
	{168.9, 35},
	{193.7, 30},
	{231.3, 25},
	{284.3, 20},
	{383.8, 15},
	{576.1, 10}
};	/**< Table describing relation between sensor value to distance for IR sensor 0 */
ADC_distance_pair IR2_table[] =
{
	{42.4, 240},
	{72.2, 150},
	{76.2, 140},
	{84.5, 130},
	{92.9, 120},
	{105.3, 110},
	{116.9, 100},
	{133.3, 90},
	{153.0, 80},
	{171.6, 70},
	{206.4, 60},
	{252.9, 50},
	{283.4, 45},
	{321.3, 40},
	{370.6, 35},
	{421.9, 30},
	{483.5, 25},
	{531.0, 20},
	{561.3, 15}
};	/**< Table describing relation between sensor value to distance for IR sensor 0 */
ADC_distance_pair IR3_table[] =
{
	{45.3, 240},
	{74.4, 150},
	{80.6, 140},
	{88.8, 130},
	{97.1, 120},
	{107.3, 110},
	{122.2, 100},
	{136.8, 90},
	{157.1, 80},
	{181.5, 70},
	{213.9, 60},
	{258.3, 50},
	{289.8, 45},
	{330.5, 40},
	{378.8, 35},
	{438.6, 30},
	{499.4, 25},
	{550.7, 20},
	{574.7, 15}
};	/**< Table describing relation between sensor value to distance for IR sensor 0 */
ADC_distance_pair IR4_table[] =
{
	{36.6, 150},
	{84.5, 80},
	{96.8, 70},
	{109.4, 60},
	{124.6, 50},
	{138.2, 45},
	{150.6, 40},
	{172.9, 35},
	{196.2, 30},
	{234.3, 25},
	{285.7, 20},
	{390.1, 15},
	{570.8, 10},
};	/**< Table describing relation between sensor value to distance for IR sensor 0 */
ADC_distance_pair IR5_table[] =
{
	{47.7, 240},
	{75.7, 150},
	{81.2, 140},
	{89.5, 130},
	{99.5, 120},
	{108.2, 110},
	{121.0, 100},
	{137.6, 90},
	{156.5, 80},
	{179.1, 70},
	{213.3, 60},
	{257.7, 50},
	{290.3, 45},
	{325.9, 40},
	{375.9, 35},
	{428.7, 30},
	{496.8, 25},
	{548.2, 20},
	{562.6, 15}
};	/**< Table describing relation between sensor value to distance for IR sensor 0 */
ADC_distance_pair IR6_table[] =
{
	{43.0, 240},
	{69.4, 150},
	{75.9, 140},
	{82.7, 130},
	{92.5, 120},
	{101.8, 110},
	{114.6, 100},
	{132.0, 90},
	{148.5, 80},
	{170.3, 70},
	{205.2, 60},
	{252.6, 50},
	{284.5, 45},
	{325.5, 40},
	{373.1, 35},
	{430.7, 30},
	{490.3, 25},
	{544.7, 20},
	{576.6, 15}
};	/**< Table describing relation between sensor value to distance for IR sensor 0 */

///////////////////////////////////////////////////////////////////////////////////////
///									FUNKTIONER										///
///////////////////////////////////////////////////////////////////////////////////////

/**
* @brief Initiates the A/D-converter
*
* Initiates the A/D-converter and enables noice cancelling mode.
*/
void init_ADC();

/**
* @brief Initiates the ultra sound sensor
*
* Initiates the ultra sound sensor and allows interrupts from it.
*/
void init_US();

/**
* @brief Initiates the SPI-bus
*
* Initiates the SPI-bus and start a timer to keep the bus in sync.
*/
void init_SPI();

/**
* @brief Initiates a timer
*
* Initiates a timer to give us a timestamp.
*/
void init_timer();

/**
* @brief Initiates the I2C-bus
*
* Initiates the I2C-bus which enables communication with the IMU.
*/
void init_I2C();

/**
* @brief Initiates the IMU
*
* Initiates the IMU by waking it up from sleep mode and programming the DMP.
*/
void init_IMU();

/**
* @brief Runs a self-test on the IMU
*
* Runs a self-test on the IMU calibrating the gyros and accelerometers.
*/
void run_self_test();

/**
* @brief Initiates the interrupt from the IMU
*
* Enables a interrupt from the IMU. Interrupt comes when data is available.
*/
void init_IMU_interrupt();

/**
* @brief Samples data from the IR sensors
*
* Samples from all the IR-sensors, saves them in a buffer with the five latest measurements for each sensor and takes an average.
*/
void ADC_IR();

/**
* @brief Gathers data from the IMU
*
* Gathers data from the IMU and calculates roll, yaw and pitch angle from returned quaternions.
*/
void read_IMU();

/**
* @brief Forces a sound pulse from the US
*
* Sends a trigger pulse for 12µs. Starts a timer that waits for a answer from the US-sensor.
*/
void send_ping();

/**
* @brief Converts the readings from the IR sensors to distances
*
* Converts the IR-sensors values from the A/D-converter trough linear interpolation and the tables.
*/
void ADC_to_distance();

/**
* @brief Looks up equivalent distance to a ADC value in the table
*
* Uses the table to convert the values from the ADC to distances using the tables and linear interpolation.
*/
double lookup_distance(ADC_distance_pair* ADC_dist_table, double ADC_value, int table_size);

/**
* @brief Converts time for the US to distance
*
* Calculates a distance for the US sensor using the time-of-flight for the sound.
*/
void time_to_distance();

/**
* @brief Calculate Yaw angle from the values from the IR sensors
*
* Uses the distances from the IR sensors to calculate a Yaw angle relative to the corridor using trigonometry.
*/
void calculate_Yaw();

/**
* @brief Saves all the data in a buffer
*
* Converts and compiles all the data in a buffer, ready to be sent.
*/
void save_to_buffer();

/**
* @brief Normalizes a quaternion
*
* Normalizes a quaternion
*/
void NormalizeQuaternion(float *quat);

/**
* @brief Restricts an angle to range for int_8t
*
* Restricts an angle to be between -128 and 127 deg to avoid overflow when converting to int_8t
*/
float restrict_angle(float angle);

void kalibrering();		// XXXXX Endast för att kunna kalibrera sensorer!

#endif /* SENSOR_H_ */
