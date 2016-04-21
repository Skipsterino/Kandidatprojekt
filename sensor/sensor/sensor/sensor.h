/*
 * sensor.h
 *
 * Created: 4/12/2016 8:59:49 AM
 *  Author: joneh084
 */ 


#ifndef SENSOR_H_
#define SENSOR_H_

#define F_CPU 16000000UL		// 16 MHz

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <math.h>
#include <util/delay.h>
#include <compat/twi.h>

#define x 0
#define y 1
#define z 2

volatile uint8_t SPI_done = 0;
volatile uint8_t SPI_overflow = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const int delay_time = 50;							// tid i millisekunder (50 ger alltså ungefär 20Hz, lagom för US-sensorn)

const double IR_sensor_distance_right = 16.5;		// Avståndet mellan högra sidosensorerna (cm)
const double IR_sensor_distance_left = 16.5;		// Avståndet mellan vänstra sidosensorerna (cm)

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

volatile double IR_latest_reading[7];
double IR_reading[7][5];							// 2D-array med de 5 senaste avläsningarna för de 7 sensorerna
double IR_ADC[7], IR_distance[7];
float IR_Yaw_right, IR_Yaw_left;					

typedef struct
{
	double ADC_data;
	double distance;
} ADC_distance_pair;

volatile double US_reading;									// (US = Ultra Sound)
double US_distance;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t buffer0_IR0 = 0xf0;
uint8_t buffer1_IR1 = 0xf1;
uint8_t buffer2_IR2 = 0xf2;
uint8_t buffer3_IR3 = 0xf3;
uint8_t buffer4_IR4 = 0xf4;
uint8_t buffer5_IR5 = 0xf5;
uint8_t buffer6_IR6 = 0xf6;
uint8_t buffer7_US = 0xf7;		// Unsigned 8-bitars int, 0 - 255
int8_t buffer8_IR_Yaw_left = 0xf8;
int8_t buffer9_IR_Yaw_right = 0xf9;
int8_t buffer10_IMU_Yaw_Low = 0xfa;
int8_t buffer11_IMU_Yaw_High = 0xfb;
int8_t buffer12_Pitch = 0xfc;
int8_t buffer13_Roll =0xfd;		// Signed 8-bitars int

volatile int byte_to_send = 0;			// Vilken byte i bufferten som skall skickas härnäst

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define QUAT_W 0
#define QUAT_X 1
#define QUAT_Y 2
#define QUAT_Z 3

#define MPU_HZ 15		//frekvensen som IMU:n genererar avbrott med (internt kör DMP:n alltid med 200 Hz)
#define USE_DMP 1

volatile unsigned char IMU_data_ready = 0;

const signed char _orientation[9] = {1, 0, 0,  0, 1, 0,  0, 0, 1};

float quaternion[4];
int16_t temp_reading;
double IMU_temperature;
float gravity[3];
float IMU_Yaw, IMU_Pitch, IMU_Roll;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double counter = 0;				// XXXXX Endast för sensor-kalibrering
double sum = 0;					// XXXXX Endast för sensor-kalibrering
int initial_counter = 0;		// XXXXX Endast för sensor-kalibrering
double result;					// XXXXX Endast för sensor-kalibrering

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ADC_distance_pair IR0_table[] =
{
	{48.0, 220},
	{72.3, 150},
	{80.0, 140},
	{88.1, 130},
	{97.8, 120},
	{110.2, 110},
	{123.6, 100},
	{140.0, 90},
	{157.4, 80},
	{183.4, 70},
	{217.2, 60},
	{263.2, 50},
	{294.6, 45},
	{334.2, 40},
	{382.9, 35},
	{437.4, 30},
	{501.9, 25},
	{554.3, 20},
	{572.5, 15}
};
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
};
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
};
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
};
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
};
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
};
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
};

///////////////////////////////////////////////////////////////////////////////////////
///									FUNKTIONER										///
///////////////////////////////////////////////////////////////////////////////////////

//initiering
void init_ADC();
void init_US();
void init_SPI();
void init_timer();
void init_I2C();
void init_IMU();
void run_self_test();
void init_IMU_interrupt();

void ADC_IR();
void read_IMU();
void send_ping();
void ADC_to_distance();
double lookup_distance(ADC_distance_pair* ADC_dist_table, double ADC_value, int table_size);
void time_to_distance();
void calculate_Yaw();
void save_to_buffer();

//Hjälpfunktioner
void NormalizeQuaternion(float *quat);
float restrict_angle(float angle);

void kalibrering();		// XXXXX Endast för att kunna kalibrera sensorer!

#endif /* SENSOR_H_ */
