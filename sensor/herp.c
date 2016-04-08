/*
* Projekt_test.c
*
* Created: 3/30/2016 8:38:40 AM
*  Author: fregu856
*/

// IR0 = IR fram - rakt
// IR1 = IR fram - ned�t
// IR2 = IR fram - h�ger
// IR3 = IR bak - h�ger
// IR4 = IR bak - upp�t
// IR5 = IR bak - v�nster
// IR6 = IR fram - v�nster

#define F_CPU 16000000UL		// 16 MHz

//#define I2Cstatus_START 0x08
//
//#define I2Cstatus_MT_SLA_ACK 0x18		// Master Transmitter Mode
//#define I2Cstatus_MT_SLA_NACK 0x20
//#define I2Cstatus_MT_DATA_ACK 0x28
//#define I2Cstatus_MT_DATA_NACK 0x30
//
//#define I2Cstatus_MR_SLA_ACK 0x40		// Master Receiver Mode
//#define I2Cstatus_MR_SLA_NACK 0x48
//#define I2Cstatus_MR_DATA_ACK 0x50
//#define I2Cstatus_MR_DATA_NACK 0x58



#define x 0
#define y 1
#define z 2

#define GYROSCOPE_SENSITIVITY 131

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <math.h>
#include <util/delay.h>
#include <compat/twi.h>
#include "I2C.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

double Vref = 5;		// XXXXX Kommer f�rmdligen tas bort
const int delay_time = 20;		// tid i millisekunder (50 ger allts� ungef�r 20Hz, lagom f�r US-sensorn)

double IR_reading[7][5];		// 2D-array med de 5 senaste avl�sningarna f�r de 7 sensorerna
double IR_ADC[7], IR_distance[7];

int16_t acc_reading[3], gyro_reading[3], gyro_offset[3];

uint8_t testdata[3] = {1, 2, 3};
uint8_t testdata2[3];

double acc_Yaw, acc_Pitch, acc_Roll, gyro_Yaw, gyro_Pitch, gyro_Roll;
int16_t temp_reading;
double IMU_temperature;
double gyro_angular_rate[3];	// XXXX SKA G�RAS LOKAL
int first_reading =1;			// Om det �r f�rsta m�tningen ska gyro-vinklarna "nollst�llas" till acc_vinklarna
#define dt 0.02347				// Sampeltidsperiod

double US_reading;		// (US = Ultra Sound)
double US_distance;

double IR_Yaw, Yaw_right, Yaw_left;		// XXXXX Yaw_right och Yaw_left skall g�ras lokala s� sm�ningom
double IMU_Yaw, IMU_Pitch, IMU_Roll;
double Yaw, Pitch, Roll;

double IR_sensor_distance_right = 14.5;		// Avst�ndet mellan h�gra sidosensorerna (cm)
double IR_sensor_distance_left = 14.5;		// Avst�ndet mellan v�nstra sidosensorerna (cm)

uint8_t buffer0_IR0, buffer1_IR1, buffer2_IR2, buffer3_IR3, buffer4_IR4, buffer5_IR5, buffer6_IR6, buffer7_US;		// Unsigned 8-bitars int, 0 - 255
int8_t buffer8_Yaw, buffer9_Pitch, buffer10_Roll;		// Signed 8-bitars int

int byte_to_send = 0;		// Vilken byte i bufferten som skall skickas h�rn�st

double counter = 0;		// XXXXX Endast f�r sensor-kalibrering
double sum = 0;		// XXXXX Endast f�r sensor-kalibrering
int initial_counter = 0;		// XXXXX Endast f�r sensor-kalibrering
double result;		// XXXXX Endast f�r sensor-kalibrering

#define QUAT_W 0
#define QUAT_X 1
#define QUAT_Y 2
#define QUAT_Z 3

#define VEC_X 0
#define VEC_Y 1
#define VEC_Z 2

#define PIN_D0 0

#define MPU_HZ 20
#define USE_DMP 1

float quaternion[4];

volatile unsigned char _dataReady = 0;

char _orientation[9] = { 1, 0, 0,
	0, 1, 0,
0, 0, 1 };


uint8_t I2C_reading;

typedef struct
{
	double ADC_data;
	double distance;
} table;
table IR0_table[] =
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
table IR1_table[] =
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
table IR2_table[] =
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
table IR3_table[] =
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
table IR4_table[] =
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
table IR5_table[] =
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
table IR6_table[] =
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

void ADC_IR();
double lookup_distance(table* ADC_dist_table, double ADC_value, int table_size);
void ADC_to_distance();
void read_IMU();
void time_to_distance();
void calculate_Yaw();
void save_to_buffer();
void send_ping();
void kalibrering();		// XXXXX Endast f�r att kunna kalibrera sensorer!
void calibrate_gyro();
void init_I2C();
void I2C_IMU_write(uint8_t reg_adr, uint8_t data);
void I2C_IMU_read();
void InitMPU();
static void DataReadyCallback();
unsigned short Row2Scale(const char *row);
unsigned short Matrix2Scalar(const char *mtx);
void NormalizeQuaternion(float *quat);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////


int main(void)
{
	ADMUX = 0x60;						//Skicka in 0110 0000 p� ADMUX f�r att v�lja ADC0 som inkanal till A/D (bit 4-0), s�tta ADLAR (bit 5) (=> de 8 mest sig. bitarna av resultatet ges i ADCH) och v�lja "AVCC with external capacitor at AREF pin" som Voltage reference select. (bit 7-6)
	ADCSRA = 0x8F;						//Skicka in 1000 1111 p� ADCSRA f�r att enable:a ADC (bit 7), ADC Complete-avbrott (bit 3) samt dela klockfrekvensen med 128 (=> A/D klockas 125 kHz)
	SMCR |= 0<<SM2 | 0<<SM1 | 1<<SM0;	// Sleep Mode Select till "ADC Noise Reduction" (S� att "sleep_cpu()" ger just detta vilol�ge)
	
	DDRD = 0x02;						// Skicka in 0000 0010 p� DDRD f�r att s�tta PD1 till utg�ng, PD6 & PD2 till ing�ng
	
	EICRA |= 1<<ISC01 | 1<<ISC00;		// Externt avbrott INT0 p� rising edge
	
	TIMSK1 |= 1<<ICIE1;					// Enable:a "Input Capture Interrupt" (F�r US)
	TCCR1B &= ~(1<<ICES1);				// Input Capture triggar p� negativ (fallande) flank
	
	SPSR |= 1<<SPIE;					// Enable avbrott SPI
	DDRB = 0x40;						// Skicka in 0100 0000 p� DDRB f�r att s�tta MISO till utg�ng, resten ing�ng. (SPI)
	SPCR = (1<<SPE);					// Enable:a SPI

	TCCR0B |= 1<<CS01 | 1<<CS00;		//S�tt preskalern till 64
	TIMSK0 |= 1<<TOIE0;					//Till�t overflow-avbrott p� timer0
	
	//init_I2C();
	
	InitMPU();
	
	sei();								// Enable avbrott �ht (bit 7 p� SREG s�tts till 1)
	_dataReady = 0;
	_delay_ms(200);						// Ge IMU tid att starta
	
	
		
	//calibrate_gyro;
	
	//i2c_write(0x75, 0x0D, 3, testdata);
	//i2c_read(0x75, 0x0D, 3, testdata2);

	/////////////////////////////////////////////////////

	while (1)
	{
		
		//ADC_IR();						// (X) Sampla IR-sensorerna
		//read_IMU();						// H�mta data fr�n IMU
//
		//send_ping();					// (X) Starta en US-m�tning
//
		//ADC_to_distance();				// (X) Konvertera sp�nning till avst�nd (IR-sensorerna)
		//time_to_distance();				// (X) Konvertera tid till sp�nning (US-sensorn)
		//calculate_Yaw();				// (\) R�kna ut Yaw-vinkeln (XXXXX Endast grundfunktionalitet)
//
		//save_to_buffer();				// (\) Spara undan i buffert (XXXXX Endast prelimin�rt. Vilka datatyper? Hur casta till dessa optimalt? Maska bort avbrott?)
//
		//_delay_ms(delay_time);			// (X) Vila


		//kalibrering();					// XXXXX Endast f�r att kunna kalibrera sensorer!
		
		if (_dataReady)
		{
			short gyro[3];
			short accel[3];
			unsigned long timestamp;
			unsigned char more;
			int ret;

			if (USE_DMP == 1)
			{
				long quat[4];
				
				short sensors;

				if (0 == (ret = dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more)))
				{
					if (!more)
					{
						_dataReady = 0;
					}
					
					quaternion[QUAT_W] = (float)quat[QUAT_W];
					quaternion[QUAT_X] = (float)quat[QUAT_X];
					quaternion[QUAT_Y] = (float)quat[QUAT_Y];
					quaternion[QUAT_Z] = (float)quat[QUAT_Z];

					NormalizeQuaternion(quaternion);

					//Serial.print(quaternion[QUAT_W]); Serial.print(";");
					//Serial.print(quaternion[QUAT_X]); Serial.print(";");
					//Serial.print(quaternion[QUAT_Y]); Serial.print(";");
					//Serial.print(quaternion[QUAT_Z]);Serial.print("\t");
//
					//Serial.print(gyro[VEC_X]);Serial.print(";");
					//Serial.print(gyro[VEC_Y]);Serial.print(";");
					//Serial.print(gyro[VEC_Z]);Serial.print("\t");
//
					//Serial.print(accel[VEC_X]);Serial.print(";");
					//Serial.print(accel[VEC_Y]);Serial.print(";");
					//Serial.println(accel[VEC_Z]);
				}
				else
				{
					//Serial.print("Error: ");Serial.println(ret);
				}

			}
			else
			{
				unsigned char sensors;

				if (0 == (ret = mpu_read_fifo(gyro, accel, &timestamp, &sensors, &more)))
				{
					if (!more)
					{
						_dataReady = 0;
					}

					//Serial.print(gyro[VEC_X]);Serial.print(";");
					//Serial.print(gyro[VEC_Y]);Serial.print(";");
					//Serial.print(gyro[VEC_Z]);Serial.print("\t");
//
					//Serial.print(accel[VEC_X]);Serial.print(";");
					//Serial.print(accel[VEC_Y]);Serial.print(";");
					//Serial.println(accel[VEC_Z]);
				}
				else
				{
					//Serial.print("Error: ");Serial.println(ret);
				}
			}
		}
	}
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void init_I2C()
{
	PORTC = (1<<PC0) | (1<<PC1);		// G�r I2C-portarna till utg�ngar

	TWSR &= ~((1<<TWPS1) | (1<<TWPS0));	// S�tt presecalerbitarna till 0 (ger prescalerv�rde = 1)
	TWBR = 0x20;						// Skicka in 0x20 p� TWBR f�r att tillsammans med Prescalerbitarna TWPS1-TWPS0 s�tta I2C-frekvensen till ungef�r 100 kHz
	I2C_IMU_write(0x6B,0x00);			// V�cker IMU ur vilol�ge
}


void I2C_IMU_write(uint8_t reg_adr, uint8_t data)
{
	I2C_start();
	
	TWDR = SLA_W;								// Ladda in IMU:ns adress + indikera att skrivning ska ske
	TWCR = (1<<TWINT) | (1<<TWEN);// | (1<<TWIE);				// Skicka

	while (!(TWCR & (1<<TWINT)));				// V�nta p� att det har skickats

	if ((TWSR & 0xF8) !=I2Cstatus_MT_SLA_ACK)	// Kolla s� att status �r r�tt (att de som vi ville skulle h�nda faktiskt h�nde)
	error();
	
	TWDR = reg_adr;								// Ladda in adressen f�r IMU-registret vi ska l�sa ifr�n
	TWCR = (1<<TWINT) | (1<<TWEN);				// Skicka

	while (!(TWCR & (1<<TWINT)));				// V�nta p� att det har skickats

	if ((TWSR & 0xF8) !=I2Cstatus_MT_DATA_ACK)	// Kolla s� att status �r r�tt
	error();
	
	TWDR = data;								// Ta IMU ur vila och v�lj intern 8 MHz som klocka
	TWCR = (1<<TWINT) | (1<<TWEN);				// Skicka

	while (!(TWCR & (1<<TWINT)));				// V�nta p� att det har skickats

	if ((TWSR & 0xF8) !=I2Cstatus_MT_DATA_ACK)	// Kolla s� att status �r r�tt
	error();

	I2C_stop();
}

void I2C_IMU_read()
{
	I2C_start();

	TWDR = SLA_W;								// Ladda in IMU:ns adress + indikera att skrivning ska ske
	TWCR = (1<<TWINT) | (1<<TWEN);// | (1<<TWIE);				// Skicka

	while (!(TWCR & (1<<TWINT)));				// V�nta p� att det har skickats

	if ((TWSR & 0xF8) !=I2Cstatus_MT_SLA_ACK)	// Kolla s� att status �r r�tt (att de som vi ville skulle h�nda faktiskt h�nde)
	error();

	TWDR = 0x3B;								// Ladda in adressen f�r IMU-registret vi ska l�sa ifr�n
	TWCR = (1<<TWINT) | (1<<TWEN);				// Skicka

	while (!(TWCR & (1<<TWINT)));				// V�nta p� att det har skickats

	if ((TWSR & 0xF8) !=I2Cstatus_MT_DATA_ACK)	// Kolla s� att status �r r�tt
	error();

	I2C_start();

	TWDR = SLA_R;								// Ladda in IMU:ns adress + indikera att l�sning ska ske
	TWCR = (1<<TWINT) | (1<<TWEN);// | (1<<TWIE);				// Skicka

	while (!(TWCR & (1<<TWINT)));				// V�nta p� att det har skickats

	if ((TWSR & 0xF8) !=I2Cstatus_MR_SLA_ACK)	// Kolla s� att status �r r�tt (att de som vi ville skulle h�nda faktiskt h�nde)
	error();
	
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);				// Ta emot data och skicka ACK (mottagningen klar)

	while (!(TWCR & (1<<TWINT)));					// V�nta p� att det har skickats
	
	acc_reading[x] = (TWDR<<8);
	
	if ((TWSR & 0xF8) !=I2Cstatus_MR_DATA_ACK)	// Kolla s� att status �r r�tt (att de som vi ville skulle h�nda faktiskt h�nde)
	error();
	
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);				// Ta emot data och skicka ACK (mottagningen klar)

	while (!(TWCR & (1<<TWINT)));					// V�nta p� att det har skickats
	
	acc_reading[x] += TWDR;
	
	if ((TWSR & 0xF8) !=I2Cstatus_MR_DATA_ACK)	// Kolla s� att status �r r�tt (att de som vi ville skulle h�nda faktiskt h�nde)
	error();
	
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);				// Ta emot data och skicka ACK (mottagningen klar)

	while (!(TWCR & (1<<TWINT)));					// V�nta p� att det har skickats
	
	acc_reading[y] = (TWDR<<8);
	
	if ((TWSR & 0xF8) !=I2Cstatus_MR_DATA_ACK)	// Kolla s� att status �r r�tt (att de som vi ville skulle h�nda faktiskt h�nde)
	error();
	
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);				// Ta emot data och skicka ACK (mottagningen klar)

	while (!(TWCR & (1<<TWINT)));					// V�nta p� att det har skickats
	
	acc_reading[y] += TWDR;
	
	if ((TWSR & 0xF8) !=I2Cstatus_MR_DATA_ACK)	// Kolla s� att status �r r�tt (att de som vi ville skulle h�nda faktiskt h�nde)
	error();
	
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);				// Ta emot data och skicka ACK (mottagningen klar)

	while (!(TWCR & (1<<TWINT)));					// V�nta p� att det har skickats
	
	acc_reading[z] = (TWDR<<8);
	
	if ((TWSR & 0xF8) !=I2Cstatus_MR_DATA_ACK)	// Kolla s� att status �r r�tt (att de som vi ville skulle h�nda faktiskt h�nde)
	error();
	
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);				// Ta emot data och skicka ACK (mottagningen klar)

	while (!(TWCR & (1<<TWINT)));					// V�nta p� att det har skickats
	
	acc_reading[z] += TWDR;
	
	if ((TWSR & 0xF8) !=I2Cstatus_MR_DATA_ACK)	// Kolla s� att status �r r�tt (att de som vi ville skulle h�nda faktiskt h�nde)
	error();
	
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);				// Ta emot data och skicka ACK (mottagningen klar)

	while (!(TWCR & (1<<TWINT)));					// V�nta p� att det har skickats
	
	temp_reading = (TWDR<<8);
	
	if ((TWSR & 0xF8) !=I2Cstatus_MR_DATA_ACK)	// Kolla s� att status �r r�tt (att de som vi ville skulle h�nda faktiskt h�nde)
	error();
	
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);				// Ta emot data och skicka ACK (mottagningen klar)

	while (!(TWCR & (1<<TWINT)));					// V�nta p� att det har skickats
	
	temp_reading += TWDR;
	
	if ((TWSR & 0xF8) !=I2Cstatus_MR_DATA_ACK)	// Kolla s� att status �r r�tt (att de som vi ville skulle h�nda faktiskt h�nde)
	error();
	
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);				// Ta emot data och skicka ACK (mottagningen klar)

	while (!(TWCR & (1<<TWINT)));					// V�nta p� att det har skickats
	
	gyro_reading[x] = (TWDR<<8);
	
	if ((TWSR & 0xF8) !=I2Cstatus_MR_DATA_ACK)	// Kolla s� att status �r r�tt (att de som vi ville skulle h�nda faktiskt h�nde)
	error();
	
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);				// Ta emot data och skicka ACK (mottagningen klar)

	while (!(TWCR & (1<<TWINT)));					// V�nta p� att det har skickats
	
	gyro_reading[x] += TWDR;
	
	if ((TWSR & 0xF8) !=I2Cstatus_MR_DATA_ACK)	// Kolla s� att status �r r�tt (att de som vi ville skulle h�nda faktiskt h�nde)
	error();
	
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);				// Ta emot data och skicka ACK (mottagningen klar)

	while (!(TWCR & (1<<TWINT)));					// V�nta p� att det har skickats
	
	gyro_reading[y] = (TWDR<<8);
	
	if ((TWSR & 0xF8) !=I2Cstatus_MR_DATA_ACK)	// Kolla s� att status �r r�tt (att de som vi ville skulle h�nda faktiskt h�nde)
	error();
	
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);				// Ta emot data och skicka ACK (mottagningen klar)

	while (!(TWCR & (1<<TWINT)));					// V�nta p� att det har skickats
	
	gyro_reading[y] += TWDR;
	
	if ((TWSR & 0xF8) !=I2Cstatus_MR_DATA_ACK)	// Kolla s� att status �r r�tt (att de som vi ville skulle h�nda faktiskt h�nde)
	error();
	
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);				// Ta emot data och skicka ACK (mottagningen klar)

	while (!(TWCR & (1<<TWINT)));					// V�nta p� att det har skickats
	
	gyro_reading[z] = (TWDR<<8);
	
	if ((TWSR & 0xF8) !=I2Cstatus_MR_DATA_ACK)	// Kolla s� att status �r r�tt (att de som vi ville skulle h�nda faktiskt h�nde)
	error();
	
	TWCR = (1<<TWINT) | (1<<TWEN);				// Ta emot data och skicka NACK (mottagningen klar)

	while (!(TWCR & (1<<TWINT)));				// V�nta p� att det har skickats
	
	gyro_reading[z] += TWDR;
	
	if ((TWSR & 0xF8) !=I2Cstatus_MR_DATA_NACK)	// Kolla s� att status �r r�tt (att de som vi ville skulle h�nda faktiskt h�nde)
	error();
	
	I2C_stop();
}


ISR(INT0_vect)
{
	DataReadyCallback();
}

ISR(TIMER1_CAPT_vect)		// Input Capture (US-sensorn)
{
	TCCR1B &= 0<<CS12 | 0<<CS10;		// Stanna timer

	US_reading = ICR1L;							// L�s in timer-v�rdets l�ga byte...
	US_reading = US_reading + (ICR1H<<8);		// ...och addera timer-v�rdets h�ga byte.
}

ISR(ADC_vect)		// ADC Conversion Complete
{
	int sensor_ID = 0;

	switch (ADMUX)
	{
		case 0x60:
		sensor_ID = 0;
		break;
		case 0x61:
		sensor_ID = 1;
		break;
		case 0x62:
		sensor_ID = 2;
		break;
		case 0x63:
		sensor_ID = 3;
		break;
		case 0x64:
		sensor_ID = 4;
		break;
		case 0x65:
		sensor_ID = 5;
		break;
		case 0x66:
		sensor_ID = 6;
		break;
	}

	for (int i = 5; i>=2; --i)		// Skifta ut den �ldsta avl�sningen
	{
		IR_reading[sensor_ID][i] = IR_reading[sensor_ID][i-1];
	}

	IR_reading[sensor_ID][1] = ADCH<<2;		// L�s in ADC:ns 8 h�gsta (av 10) bitar, skiftade tv� steg upp�t
	IR_ADC[sensor_ID] = (IR_reading[sensor_ID][1] + IR_reading[sensor_ID][2] + IR_reading[sensor_ID][3] + IR_reading[sensor_ID][4] + IR_reading[sensor_ID][5])/5;

	++ADMUX;
}

ISR(SPI_STC_vect)		// Avbrottsvektor f�r data-s�ndning (kan beh�va ut�kas)
{
	switch(byte_to_send)
	{
		case 0:
		{
			SPDR = buffer0_IR0;
			++byte_to_send;
			break;
		}
		case 1:
		{
			SPDR = buffer1_IR1;
			++byte_to_send;
			break;
		}
		case 2:
		{
			SPDR = buffer2_IR2;
			++byte_to_send;
			break;
		}
		case 3:
		{
			SPDR = buffer3_IR3;
			++byte_to_send;
			break;
		}
		case 4:
		{
			SPDR = buffer4_IR4;
			++byte_to_send;
			break;
		}
		case 5:
		{
			SPDR = buffer5_IR5;
			++byte_to_send;
			break;
		}
		case 6:
		{
			SPDR = buffer6_IR6;
			++byte_to_send;
			break;
		}
		case 7:
		{
			SPDR = buffer7_US;
			++byte_to_send;
			break;
		}
		case 8:
		{
			SPDR = buffer8_Yaw;
			++byte_to_send;
			break;
		}
		case 9:
		{
			SPDR = buffer9_Pitch;
			++byte_to_send;
			break;
		}
		case 10:
		{
			SPDR = buffer10_Roll;
			byte_to_send = 0;
			break;
		}
	}
}

void ADC_IR()
{
	while(ADMUX < 0x67)
	{
		sleep_enable();
		sleep_cpu();			// Processorn g�r ner i "ADC Noise Reduction"-vila och startar direkt en ADC
		sleep_disable();		// (Vilofunktionen enable:as endast h�r f�r att "programmeraren ska ha full kontroll �ver n�r processorn sover")
	}

	ADMUX = 0x60;				// �terst�ll ADC:n till ADC0 som inkanal
}

void ADC_to_distance()
{
	IR_distance[0] = lookup_distance(IR0_table, IR_ADC[0], 17);
	IR_distance[1] = lookup_distance(IR1_table, IR_ADC[1], 11);
	IR_distance[2] = lookup_distance(IR2_table, IR_ADC[2], 16);
	IR_distance[3] = lookup_distance(IR3_table, IR_ADC[3], 17);
	IR_distance[4] = lookup_distance(IR4_table, IR_ADC[4], 11);
	IR_distance[5] = lookup_distance(IR5_table, IR_ADC[5], 16);
	IR_distance[6] = lookup_distance(IR6_table, IR_ADC[6], 16);
}

double lookup_distance(table* ADC_dist_table, double ADC_value, int table_size)
{
	if(ADC_value <= ADC_dist_table[0].ADC_data)
	return ADC_dist_table[0].distance;

	if(ADC_value >= ADC_dist_table[table_size-1].ADC_data)
	return ADC_dist_table[table_size-1].distance;

	for(int i = 0; i < table_size-1; i++)
	{
		if (ADC_dist_table[i].ADC_data <= ADC_value && ADC_dist_table[i+1].ADC_data >= ADC_value )
		{
			double diff_ADC = ADC_value - ADC_dist_table[i].ADC_data;
			double step_length = ADC_dist_table[i+1].ADC_data - ADC_dist_table[i].ADC_data;

			return ADC_dist_table[i].distance + (ADC_dist_table[i+1].distance - ADC_dist_table[i].distance)*(diff_ADC/step_length);
		}
	}
}

void send_ping()
{
	PORTD &= 0<<PORTD1;
	_delay_us(2);
	PORTD |= 1<<PORTD1;
	_delay_us(12);					// Skickar ut trigger-puls p� minst 10 mikrosekunder
	PORTD &= 0<<PORTD1;				// Ping triggas d� trigger-pulsen g�r l�g

	TCNT1 = 0;						// Nollst�ll timer
	TCCR1B |= 1<<CS12 | 1<<CS10;	// Starta timer
}

void read_IMU()
{
	double acc_Roll_rad, acc_Pitch_rad;
	//double gyro_angular_rate[3];
	
	I2C_IMU_read();
	IMU_temperature = ((double)temp_reading)/340.0f + 36.53f;
	
	acc_Roll_rad = atan( ((double)acc_reading[y])/(sqrt( pow(((double)acc_reading[x]), 2) + pow(((double)acc_reading[z]), 2) ) ));
	acc_Roll = (acc_Roll_rad/3.141592)*180;
	
	acc_Pitch_rad = atan( ((double)acc_reading[x])/(sqrt( pow(((double)acc_reading[y]), 2) + pow(((double)acc_reading[z]), 2) ) ));
	acc_Pitch = (acc_Pitch_rad/3.141592)*180;
	
	if(first_reading)
	{
		gyro_Pitch = acc_Pitch;
		gyro_Roll = acc_Roll;
		gyro_Yaw = 0;
		
		first_reading = 0;
	}
	
	else
	{
		gyro_angular_rate[x] = (((double)gyro_reading[x]) - ((double)gyro_offset[x]))/GYROSCOPE_SENSITIVITY;
		gyro_angular_rate[y] = (((double)gyro_reading[y]) - ((double)gyro_offset[y]))/GYROSCOPE_SENSITIVITY;
		gyro_angular_rate[z] = (((double)gyro_reading[z]) - ((double)gyro_offset[z]))/GYROSCOPE_SENSITIVITY;
		
		gyro_Yaw += gyro_angular_rate[z]*dt;
		gyro_Pitch -= gyro_angular_rate[y]*dt;
		gyro_Roll += gyro_angular_rate[x]*dt;
		
		IMU_Pitch = 0.95*gyro_Pitch + 0.05*acc_Pitch;
		IMU_Roll = 0.95*gyro_Roll + 0.05*acc_Roll;
	}
}

void time_to_distance()
{
	US_distance =  US_reading*0.0642857143;		// G�r om till tid i millisekunder
	US_distance = US_distance - 0.75;			// Subrtrahera offset mellan triggerpuls och ekopuls
	US_distance = 34.33*US_distance/2;			// G�r om till cm

	if(US_distance > 250)
	US_distance = 250;							// S�tt max-avst�nd (s� att ej f�r �verslag d� det g�rs om till 8 bitar)
}

// XXXXX Endast grundfunktionalitet, kommer beh�va ut�kas f�r att uppt�cka t.ex. n�r bara ena sidan �r tillf�rlitlig.
void calculate_Yaw()
{
	double l_delta_right = IR_distance[2] - IR_distance[3];
	double l_delta_left = IR_distance[5] - IR_distance[6];

	Yaw_right = (atan(l_delta_right/IR_sensor_distance_right)/3.14)*180;		// Yaw-ber�kning med de h�gra sidosensorerna
	Yaw_left = (atan(l_delta_left/IR_sensor_distance_left)/3.14)*180;			// Yaw-ber�kning med de v�nstra sidosensorerna

	IR_Yaw = (Yaw_right + Yaw_left)/2;
}

void save_to_buffer()
{
	//cli();					// XXXXX Beh�vs v�l inte maska bort avbrott? Vad kan h�nda?

	buffer0_IR0 = IR_distance[0];
	buffer1_IR1 = IR_distance[1];
	buffer2_IR2 = IR_distance[2];
	buffer3_IR3 = IR_distance[3];
	buffer4_IR4 = IR_distance[4];
	buffer5_IR5 = IR_distance[5];
	buffer6_IR6 = IR_distance[6];

	buffer7_US = US_distance;

	buffer8_Yaw = IR_Yaw;
	buffer9_Pitch = IMU_Pitch;
	buffer10_Roll = IMU_Roll;

	//sei();					// XXXXX Beh�vs v�l inte maska bort avbrott? Vad kan h�nda?
}

void kalibrering()				// XXXXX Endast f�r att kunna kalibrera sensorer!
{
	if(initial_counter < 10)
	{
		++initial_counter;
	}
	else
	{
		sum = sum + IR_ADC[3];
		++counter;
		result = sum/counter;
	}
}

void calibrate_gyro()
{
	for (int i=0; i<100; ++i)
	{
		I2C_IMU_read();
		
		gyro_offset[x] += gyro_reading[x];
		gyro_offset[y] += gyro_reading[y];
		gyro_offset[z] += gyro_reading[z];
		
		_delay_us(100);
	}
	
	gyro_offset[x] = gyro_offset[x]/100;
	gyro_offset[y] = gyro_offset[y]/100;
	gyro_offset[z] = gyro_offset[z]/100;

}

void InitMPU()
{
	int ret;

	struct int_param_s int_param;
	int_param.cb = DataReadyCallback;
	int_param.pin = PIN_D0;

	mpu_init(&int_param);
	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	mpu_set_sample_rate(MPU_HZ);
	dmp_load_motion_driver_firmware();
	dmp_set_orientation(Matrix2Scalar(_orientation));
	dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL | DMP_FEATURE_TAP);
	dmp_set_fifo_rate(MPU_HZ);
	mpu_set_dmp_state(USE_DMP);

}

unsigned short Matrix2Scalar(const char *mtx)
{
	unsigned short scalar;

	scalar = Row2Scale(mtx);
	scalar |= Row2Scale(mtx + 3) << 3;
	scalar |= Row2Scale(mtx + 6) << 6;

	return scalar;
}

unsigned short Row2Scale(const char *row)
{
	unsigned short b;

	if (row[0] > 0)
	b = 0;
	else if (row[0] < 0)
	b = 4;
	else if (row[1] > 0)
	b = 1;
	else if (row[1] < 0)
	b = 5;
	else if (row[2] > 0)
	b = 2;
	else if (row[2] < 0)
	b = 6;
	else
	b = 7;

	return b;
}
static void DataReadyCallback()
{
	_dataReady = 1;
}

void NormalizeQuaternion(float *quat)
{
	float length = sqrt(quat[QUAT_W] * quat[QUAT_W] + quat[QUAT_X] * quat[QUAT_X] +
	quat[QUAT_Y] * quat[QUAT_Y] + quat[QUAT_Z] * quat[QUAT_Z]);

	if (length == 0)
	return;

	quat[QUAT_W] /= length;
	quat[QUAT_X] /= length;
	quat[QUAT_Y] /= length;
	quat[QUAT_Z] /= length;
}