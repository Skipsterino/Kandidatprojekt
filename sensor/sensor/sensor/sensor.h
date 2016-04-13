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

uint8_t SPI_start = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const int delay_time = 50;							// tid i millisekunder (50 ger alltså ungefär 20Hz, lagom för US-sensorn)

const double IR_sensor_distance_right = 14.5;		// Avståndet mellan högra sidosensorerna (cm)
const double IR_sensor_distance_left = 14.5;		// Avståndet mellan vänstra sidosensorerna (cm)

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double IR_latest_reading[7];
double IR_reading[7][5];							// 2D-array med de 5 senaste avläsningarna för de 7 sensorerna
double IR_ADC[7], IR_distance[7];
double IR_Yaw, Yaw_right, Yaw_left;					// XXXXX Yaw_right och Yaw_left skall göras lokala så småningom
typedef struct
{
	double ADC_data;
	double distance;
} ADC_distance_pair;

double US_reading;									// (US = Ultra Sound)
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
int8_t buffer8_IR_Yaw = 0xf8;
int8_t buffer9_IMU_Yaw = 0xf9;
int8_t buffer10_Pitch = 0xfa;
int8_t buffer11_Roll = 0xfb;		// Signed 8-bitars int

int byte_to_send = 0;			// Vilken byte i bufferten som skall skickas härnäst

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define QUAT_W 0
#define QUAT_X 1
#define QUAT_Y 2
#define QUAT_Z 3

#define MPU_HZ 15		//frekvensen som IMU:n genererar avbrott med (internt kör DMP:n alltid med 200 Hz)
#define USE_DMP 1

volatile unsigned char _dataReady = 0;

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
	{99.6, 140},
	{102.2, 130},
	{107.7, 120},
	{117.7, 110},
	{130.2, 100},
	{142.9, 90},
	{162.7, 80},
	{184.6, 70},
	{217.2, 60},
	{260.0, 50},
	{291.9, 45},
	{328.2, 40},
	{374.0, 35},
	{430.6, 30},
	{493.6, 25},
	{548.2, 20},
	{570.1, 15}
};
ADC_distance_pair IR1_table[] =
{
	{137.9, 55},
	{142.9, 50},
	{150.7, 45},
	{163.6, 40},
	{179.8, 35},
	{204.3, 30},
	{237.7, 25},
	{289.5, 20},
	{378.6, 15},
	{554.5, 10},
	{598.5, 5}
};
ADC_distance_pair IR2_table[] =
{
	{97.0, 130},
	{106.2, 120},
	{115.7, 110},
	{124.8, 100},
	{137.2, 90},
	{154.2, 80},
	{173.3, 70},
	{204.2, 60},
	{248.0, 50},
	{276.3, 45},
	{316.0, 40},
	{359.7, 35},
	{416.5, 30},
	{468.6, 25},
	{524.1, 20},
	{557.3, 15}
};
ADC_distance_pair IR3_table[] =
{
	{16, 140},
	{33.9, 130},
	{65.7, 120},
	{99.6, 110},
	{122.4, 100},
	{145.2, 90},
	{166.8, 80},
	{193.6, 70},
	{225.2, 60},
	{275.9, 50},
	{309.5, 45},
	{351.9, 40},
	{403.8, 35},
	{466.5, 30},
	{533.6, 25},
	{588.6, 20},
	{615.7, 15}
};
ADC_distance_pair IR4_table[] =
{
	{139.2, 55},
	{141.4, 50},
	{149.3, 45},
	{161.6, 40},
	{175.9, 35},
	{199.2, 30},
	{231.8, 25},
	{284.4, 20},
	{368.9, 15},
	{548.4, 10},
	{621.4, 5}
};
ADC_distance_pair IR5_table[] =
{
	{101.2, 130},
	{108.5, 120},
	{118.9, 110},
	{127.5, 100},
	{142.5, 90},
	{159.4, 80},
	{179.0, 70},
	{208.5, 60},
	{252.5, 50},
	{283.5, 45},
	{318.8, 40},
	{368.5, 35},
	{420.5, 30},
	{484.9, 25},
	{538.8, 20},
	{562.2, 15}
};
ADC_distance_pair IR6_table[] =
{
	{96.6, 130},
	{100.6, 120},
	{108.7, 110},
	{120.2, 100},
	{133.0, 90},
	{150.5, 80},
	{171.2, 70},
	{202.8, 60},
	{249.4, 50},
	{278.2, 45},
	{317.9, 40},
	{364.4, 35},
	{420.9, 30},
	{482.0, 25},
	{536.3, 20},
	{571.7, 15}
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
void init_IMU_interrupt();

void ADC_IR();
void read_IMU();
void send_ping();
void ADC_to_distance();
double lookup_distance(ADC_distance_pair* ADC_dist_table, double ADC_value, int table_size);
void time_to_distance();
void calculate_Yaw();
void save_to_buffer();

//Matte för IMU
void NormalizeQuaternion(float *quat);

void kalibrering();		// XXXXX Endast för att kunna kalibrera sensorer!

#endif /* SENSOR_H_ */
