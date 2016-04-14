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

uint8_t SPI_done = 0;
uint8_t SPI_overflow = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const int delay_time = 50;							// tid i millisekunder (50 ger alltså ungefär 20Hz, lagom för US-sensorn)

const double IR_sensor_distance_right = 14.5;		// Avståndet mellan högra sidosensorerna (cm)
const double IR_sensor_distance_left = 14.5;		// Avståndet mellan vänstra sidosensorerna (cm)

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double IR_latest_reading[7];
double IR_reading[7][5];							// 2D-array med de 5 senaste avläsningarna för de 7 sensorerna
double IR_ADC[7], IR_distance[7];
float IR_Yaw_right, IR_Yaw_left;					

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
int8_t buffer8_IR_Yaw_left = 0xf8;
int8_t buffer9_IR_Yaw_right = 0xf9;
int8_t buffer10_IMU_Yaw = 0xfa;
int8_t buffer11_Pitch = 0xfb;
int8_t buffer12_Roll =0xfc;		// Signed 8-bitars int

int byte_to_send = 0;			// Vilken byte i bufferten som skall skickas härnäst

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
	{115.2, 120},
	{119.0, 110},
	{128.7, 100},
	{143.8, 90},
	{161.0, 80},
	{183.3, 70},
	{216.5, 60},
	{266.0, 50},
	{297.7, 45},
	{335.7, 40},
	{385.8, 35},
	{440.7, 30},
	{502.1, 25},
	{554,5, 20},
	{568.0, 15}
};
ADC_distance_pair IR1_table[] =
{
	{135.5, 55},
	{139.7, 50},
	{144.4, 45},
	{159.9, 40},
	{172.8, 35},
	{197.2, 30},
	{236.2, 25},
	{283.2, 20},
	{379.3, 15},
	{560.4, 10},
	{562.7, 5}
};
ADC_distance_pair IR2_table[] =
{
	{112.1, 120},
	{116.2, 110},
	{124.5, 100},
	{137.0, 90},
	{153.7, 80},
	{176.1, 70},
	{208.0, 60},
	{253.0, 50},
	{286.0, 45},
	{323.1, 40},
	{371.3, 35},
	{422.2, 30},
	{484.2, 25},
	{536.1, 20},
	{558.1, 15}
};
ADC_distance_pair IR3_table[] =
{
	{114.2, 120},
	{120.2 110},
	{128.7, 100},
	{140.9, 90},
	{158.5, 80},
	{182.1, 70},
	{214.0, 60},
	{260.0, 50},
	{291.7, 45},
	{331.1, 40},
	{379.6, 35},
	{439.2, 30},
	{502.3, 25},
	{555.1, 20},
	{569.0, 15}
};
ADC_distance_pair IR4_table[] =
{
	{139.6, 55},
	{143.0, 50},
	{147.8, 45},
	{161.0, 40},
	{177.1, 35},
	{198.2, 30},
	{236.2, 25},
	{291.0, 20},
	{383.7, 15},
	{567.0, 10},
	{581.2, 5}
};
ADC_distance_pair IR5_table[] =
{
	{112.3, 120},
	{116.9, 110},
	{128.9, 100},
	{141.4, 90},
	{158.2, 80},
	{180.2, 70},
	{213.0, 60},
	{258.1, 50},
	{287.2, 45},
	{327.3, 40},
	{376.4, 35},
	{431.1, 30},
	{497.0, 25},
	{548.5, 20},
	{559.4, 15}
};
ADC_distance_pair IR6_table[] =
{
	{105.9, 120},
	{109.7, 110},
	{119.2, 100},
	{135.2, 90},
	{152.0, 80},
	{171.7, 70},
	{203.6, 60},
	{252.3, 50},
	{284.4, 45},
	{324.7, 40},
	{372.5, 35},
	{428.1, 30},
	{492.7, 25},
	{545.4, 20},
	{573.3, 15}
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
