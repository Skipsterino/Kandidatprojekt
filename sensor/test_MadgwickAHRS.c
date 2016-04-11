/*
* Projekt_test.c
*
* Created: 3/30/2016 8:38:40 AM
*  Author: fregu856
*/

// IR0 = IR fram - rakt
// IR1 = IR fram - nedåt
// IR2 = IR fram - höger
// IR3 = IR bak - höger
// IR4 = IR bak - uppåt
// IR5 = IR bak - vänster
// IR6 = IR fram - vänster

#define F_CPU 16000000UL		// 16 MHz

#define I2Cstatus_START 0x08

#define I2Cstatus_MT_SLA_ACK 0x18		// Master Transmitter Mode
#define I2Cstatus_MT_SLA_NACK 0x20
#define I2Cstatus_MT_DATA_ACK 0x28
#define I2Cstatus_MT_DATA_NACK 0x30

#define I2Cstatus_MR_SLA_ACK 0x40		// Master Receiver Mode
#define I2Cstatus_MR_SLA_NACK 0x48
#define I2Cstatus_MR_DATA_ACK 0x50
#define I2Cstatus_MR_DATA_NACK 0x58

#define SLA_W 0xD0
#define SLA_R 0xD1

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
#include "MadgwickAHRS.h"

double Vref = 5;		// XXXXX Kommer förmdligen tas bort
const int delay_time = 5;		// tid i millisekunder (50 ger alltså ungefär 20Hz, lagom för US-sensorn)

double IR_reading[7][5];		// 2D-array med de 5 senaste avläsningarna för de 7 sensorerna
double IR_ADC[7], IR_distance[7];

int16_t acc_reading[3], gyro_reading[3], gyro_offset[3];
double acc_Yaw, acc_Pitch, acc_Roll, gyro_Yaw, gyro_Pitch, gyro_Roll;
int16_t temp_reading;
double IMU_temperature;
float gyro_angular_rate[3];	// XXXX SKA GÖRAS LOKAL
int first_reading =1;			// Om det är första mätningen ska gyro-vinklarna "nollställas" till acc_vinklarna
#define dt 0.02347				// Sampeltidsperiod



double US_reading;		// (US = Ultra Sound)
double US_distance;

float gravity[3];

double IR_Yaw, Yaw_right, Yaw_left;		// XXXXX Yaw_right och Yaw_left skall göras lokala så småningom
float IMU_Yaw, IMU_Pitch, IMU_Roll;
double Yaw, Pitch, Roll;

double IR_sensor_distance_right = 14.5;		// Avståndet mellan högra sidosensorerna (cm)
double IR_sensor_distance_left = 14.5;		// Avståndet mellan vänstra sidosensorerna (cm)

uint8_t buffer0_IR0, buffer1_IR1, buffer2_IR2, buffer3_IR3, buffer4_IR4, buffer5_IR5, buffer6_IR6, buffer7_US;		// Unsigned 8-bitars int, 0 - 255
int8_t buffer8_Yaw, buffer9_Pitch, buffer10_Roll;		// Signed 8-bitars int

int byte_to_send = 0;		// Vilken byte i bufferten som skall skickas härnäst

double counter = 0;		// XXXXX Endast för sensor-kalibrering
double sum = 0;		// XXXXX Endast för sensor-kalibrering
int initial_counter = 0;		// XXXXX Endast för sensor-kalibrering
double result;		// XXXXX Endast för sensor-kalibrering

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
void kalibrering();		// XXXXX Endast för att kunna kalibrera sensorer!
void error();
void init_I2C();
void I2C_start();
void I2C_stop();
void I2C_IMU_write(uint8_t reg_address, uint8_t data);
void I2C_IMU_read();
void calibrate_gyro();


///////////////////////////////////////////////////////////////////////////////////////////////////////////////


int main(void)
{
	ADMUX = 0x60;						//Skicka in 0110 0000 på ADMUX för att välja ADC0 som inkanal till A/D (bit 4-0), sätta ADLAR (bit 5) (=> de 8 mest sig. bitarna av resultatet ges i ADCH) och välja "AVCC with external capacitor at AREF pin" som Voltage reference select. (bit 7-6)
	ADCSRA = 0x8F;						//Skicka in 1000 1111 på ADCSRA för att enable:a ADC (bit 7), ADC Complete-avbrott (bit 3) samt dela klockfrekvensen med 128 (=> A/D klockas 125 kHz)
	SMCR |= 0<<SM2 | 0<<SM1 | 1<<SM0;	// Sleep Mode Select till "ADC Noise Reduction" (Så att "sleep_cpu()" ger just detta viloläge)
	
	DDRD = 0x02;						// Skicka in 0000 0010 på DDRD för att sätta PD1 till utgång, PD6 till ingång
	
	TIMSK1 |= 1<<ICIE1;					// Enable:a "Input Capture Interrupt" (För US)
	TCCR1B &= ~(1<<ICES1);				// Input Capture triggar på negativ (fallande) flank
	
	SPSR |= 1<<SPIE;					// Enable avbrott SPI
	DDRB = 0x40;						// Skicka in 0100 0000 på DDRB för att sätta MISO till utgång, resten ingång. (SPI)
	SPCR = (1<<SPE);					// Enable:a SPI

	init_I2C();
	
	sei();								// Enable avbrott öht (bit 7 på SREG sätts till 1)
	
	_delay_ms(200);						// Ge IMU tid att starta
	
	calibrate_gyro();
	

	/////////////////////////////////////////////////////

	while (1)
	{
		
		//ADC_IR();						// (X) Sampla IR-sensorerna
		read_IMU();						// Hämta data från IMU

		//send_ping();					// (X) Starta en US-mätning

		//ADC_to_distance();				// (X) Konvertera spänning till avstånd (IR-sensorerna)
		//time_to_distance();				// (X) Konvertera tid till spänning (US-sensorn)
		//calculate_Yaw();				// (\) Räkna ut Yaw-vinkeln (XXXXX Endast grundfunktionalitet)

		//save_to_buffer();				// (\) Spara undan i buffert (XXXXX Endast preliminärt. Vilka datatyper? Hur casta till dessa optimalt? Maska bort avbrott?)

		_delay_ms(delay_time);			// (X) Vila


		//kalibrering();					// XXXXX Endast för att kunna kalibrera sensorer!
	}
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void init_I2C()
{
	PORTC = (1<<PC0) | (1<<PC1);		// Gör I2C-portarna till utgångar

	TWSR &= ~((1<<TWPS1) | (1<<TWPS0));	// Sätt presecalerbitarna till 0 (ger prescalervärde = 1)
	TWBR = 0x20;						// Skicka in 0x20 på TWBR för att tillsammans med Prescalerbitarna TWPS1-TWPS0 sätta I2C-frekvensen till ungefär 100 kHz
	I2C_IMU_write(0x6B,0x00);			// Väcker IMU ur viloläge
}

void I2C_start()
{
	TWCR = 0;
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);// | (1<<TWIE);	// Start

	while (!(TWCR & (1<<TWINT)));				// Vänta på att det har skickats

	if ((TWSR & 0xF8) != I2Cstatus_START)		// Kolla så status = Start
	error();
}

void I2C_stop()
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);			// STOP
}

void I2C_IMU_write(uint8_t reg_adr, uint8_t data)
{
	I2C_start();
	
	TWDR = SLA_W;								// Ladda in IMU:ns adress + indikera att skrivning ska ske
	TWCR = (1<<TWINT) | (1<<TWEN);// | (1<<TWIE);				// Skicka

	while (!(TWCR & (1<<TWINT)));				// Vänta på att det har skickats

	if ((TWSR & 0xF8) !=I2Cstatus_MT_SLA_ACK)	// Kolla så att status är rätt (att de som vi ville skulle hända faktiskt hände)
	error();
	
	TWDR = reg_adr;								// Ladda in adressen för IMU-registret vi ska läsa ifrån
	TWCR = (1<<TWINT) | (1<<TWEN);				// Skicka

	while (!(TWCR & (1<<TWINT)));				// Vänta på att det har skickats

	if ((TWSR & 0xF8) !=I2Cstatus_MT_DATA_ACK)	// Kolla så att status är rätt
	error();
	
	TWDR = data;								// Ta IMU ur vila och välj intern 8 MHz som klocka
	TWCR = (1<<TWINT) | (1<<TWEN);				// Skicka

	while (!(TWCR & (1<<TWINT)));				// Vänta på att det har skickats

	if ((TWSR & 0xF8) !=I2Cstatus_MT_DATA_ACK)	// Kolla så att status är rätt
	error();

	I2C_stop();
}

void I2C_IMU_read()
{
	I2C_start();

	TWDR = SLA_W;								// Ladda in IMU:ns adress + indikera att skrivning ska ske
	TWCR = (1<<TWINT) | (1<<TWEN);// | (1<<TWIE);				// Skicka

	while (!(TWCR & (1<<TWINT)));				// Vänta på att det har skickats

	if ((TWSR & 0xF8) !=I2Cstatus_MT_SLA_ACK)	// Kolla så att status är rätt (att de som vi ville skulle hända faktiskt hände)
	error();

	TWDR = 0x3B;								// Ladda in adressen för IMU-registret vi ska läsa ifrån
	TWCR = (1<<TWINT) | (1<<TWEN);				// Skicka

	while (!(TWCR & (1<<TWINT)));				// Vänta på att det har skickats

	if ((TWSR & 0xF8) !=I2Cstatus_MT_DATA_ACK)	// Kolla så att status är rätt
	error();

	I2C_start();

	TWDR = SLA_R;								// Ladda in IMU:ns adress + indikera att läsning ska ske
	TWCR = (1<<TWINT) | (1<<TWEN);// | (1<<TWIE);				// Skicka

	while (!(TWCR & (1<<TWINT)));				// Vänta på att det har skickats

	if ((TWSR & 0xF8) !=I2Cstatus_MR_SLA_ACK)	// Kolla så att status är rätt (att de som vi ville skulle hända faktiskt hände)
	error();
	
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);				// Ta emot data och skicka ACK (mottagningen klar)

	while (!(TWCR & (1<<TWINT)));					// Vänta på att det har skickats
	
	acc_reading[x] = (TWDR<<8);
	
	if ((TWSR & 0xF8) !=I2Cstatus_MR_DATA_ACK)	// Kolla så att status är rätt (att de som vi ville skulle hända faktiskt hände)
	error();
	
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);				// Ta emot data och skicka ACK (mottagningen klar)

	while (!(TWCR & (1<<TWINT)));					// Vänta på att det har skickats
	
	acc_reading[x] += TWDR;
	
	if ((TWSR & 0xF8) !=I2Cstatus_MR_DATA_ACK)	// Kolla så att status är rätt (att de som vi ville skulle hända faktiskt hände)
	error();
	
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);				// Ta emot data och skicka ACK (mottagningen klar)

	while (!(TWCR & (1<<TWINT)));					// Vänta på att det har skickats
	
	acc_reading[y] = (TWDR<<8);
	
	if ((TWSR & 0xF8) !=I2Cstatus_MR_DATA_ACK)	// Kolla så att status är rätt (att de som vi ville skulle hända faktiskt hände)
	error();
	
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);				// Ta emot data och skicka ACK (mottagningen klar)

	while (!(TWCR & (1<<TWINT)));					// Vänta på att det har skickats
	
	acc_reading[y] += TWDR;
	
	if ((TWSR & 0xF8) !=I2Cstatus_MR_DATA_ACK)	// Kolla så att status är rätt (att de som vi ville skulle hända faktiskt hände)
	error();
	
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);				// Ta emot data och skicka ACK (mottagningen klar)

	while (!(TWCR & (1<<TWINT)));					// Vänta på att det har skickats
	
	acc_reading[z] = (TWDR<<8);
	
	if ((TWSR & 0xF8) !=I2Cstatus_MR_DATA_ACK)	// Kolla så att status är rätt (att de som vi ville skulle hända faktiskt hände)
	error();
	
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);				// Ta emot data och skicka ACK (mottagningen klar)

	while (!(TWCR & (1<<TWINT)));					// Vänta på att det har skickats
	
	acc_reading[z] += TWDR;
	
	if ((TWSR & 0xF8) !=I2Cstatus_MR_DATA_ACK)	// Kolla så att status är rätt (att de som vi ville skulle hända faktiskt hände)
	error();
	
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);				// Ta emot data och skicka ACK (mottagningen klar)

	while (!(TWCR & (1<<TWINT)));					// Vänta på att det har skickats
	
	temp_reading = (TWDR<<8);
	
	if ((TWSR & 0xF8) !=I2Cstatus_MR_DATA_ACK)	// Kolla så att status är rätt (att de som vi ville skulle hända faktiskt hände)
	error();
	
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);				// Ta emot data och skicka ACK (mottagningen klar)

	while (!(TWCR & (1<<TWINT)));					// Vänta på att det har skickats
	
	temp_reading += TWDR;
	
	if ((TWSR & 0xF8) !=I2Cstatus_MR_DATA_ACK)	// Kolla så att status är rätt (att de som vi ville skulle hända faktiskt hände)
	error();
	
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);				// Ta emot data och skicka ACK (mottagningen klar)

	while (!(TWCR & (1<<TWINT)));					// Vänta på att det har skickats
	
	gyro_reading[x] = (TWDR<<8);
	
	if ((TWSR & 0xF8) !=I2Cstatus_MR_DATA_ACK)	// Kolla så att status är rätt (att de som vi ville skulle hända faktiskt hände)
	error();
	
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);				// Ta emot data och skicka ACK (mottagningen klar)

	while (!(TWCR & (1<<TWINT)));					// Vänta på att det har skickats
	
	gyro_reading[x] += TWDR;
	
	if ((TWSR & 0xF8) !=I2Cstatus_MR_DATA_ACK)	// Kolla så att status är rätt (att de som vi ville skulle hända faktiskt hände)
	error();
	
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);				// Ta emot data och skicka ACK (mottagningen klar)

	while (!(TWCR & (1<<TWINT)));					// Vänta på att det har skickats
	
	gyro_reading[y] = (TWDR<<8);
	
	if ((TWSR & 0xF8) !=I2Cstatus_MR_DATA_ACK)	// Kolla så att status är rätt (att de som vi ville skulle hända faktiskt hände)
	error();
	
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);				// Ta emot data och skicka ACK (mottagningen klar)

	while (!(TWCR & (1<<TWINT)));					// Vänta på att det har skickats
	
	gyro_reading[y] += TWDR;
	
	if ((TWSR & 0xF8) !=I2Cstatus_MR_DATA_ACK)	// Kolla så att status är rätt (att de som vi ville skulle hända faktiskt hände)
	error();
	
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);				// Ta emot data och skicka ACK (mottagningen klar)

	while (!(TWCR & (1<<TWINT)));					// Vänta på att det har skickats
	
	gyro_reading[z] = (TWDR<<8);
	
	if ((TWSR & 0xF8) !=I2Cstatus_MR_DATA_ACK)	// Kolla så att status är rätt (att de som vi ville skulle hända faktiskt hände)
	error();
	
	TWCR = (1<<TWINT) | (1<<TWEN);				// Ta emot data och skicka NACK (mottagningen klar)

	while (!(TWCR & (1<<TWINT)));				// Vänta på att det har skickats
	
	gyro_reading[z] += TWDR;
	
	if ((TWSR & 0xF8) !=I2Cstatus_MR_DATA_NACK)	// Kolla så att status är rätt (att de som vi ville skulle hända faktiskt hände)
	error();
	
	I2C_stop();
}

void error()			// XXXXX SKA ÄNDRAS!
{
	I2C_reading = 111;
}

ISR(TIMER1_CAPT_vect)		// Input Capture (US-sensorn)
{
	TCCR1B &= 0<<CS12 | 0<<CS10;		// Stanna timer

	US_reading = ICR1L;							// Läs in timer-värdets låga byte...
	US_reading = US_reading + (ICR1H<<8);		// ...och addera timer-värdets höga byte.
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

	for (int i = 5; i>=2; --i)		// Skifta ut den äldsta avläsningen
	{
		IR_reading[sensor_ID][i] = IR_reading[sensor_ID][i-1];
	}

	IR_reading[sensor_ID][1] = ADCH<<2;		// Läs in ADC:ns 8 högsta (av 10) bitar, skiftade två steg uppåt
	IR_ADC[sensor_ID] = (IR_reading[sensor_ID][1] + IR_reading[sensor_ID][2] + IR_reading[sensor_ID][3] + IR_reading[sensor_ID][4] + IR_reading[sensor_ID][5])/5;

	++ADMUX;
}

ISR(SPI_STC_vect)		// Avbrottsvektor för data-sändning (kan behöva utökas)
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
		sleep_cpu();			// Processorn går ner i "ADC Noise Reduction"-vila och startar direkt en ADC
		sleep_disable();		// (Vilofunktionen enable:as endast här för att "programmeraren ska ha full kontroll över när processorn sover")
	}

	ADMUX = 0x60;				// Återställ ADC:n till ADC0 som inkanal
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
	_delay_us(12);					// Skickar ut trigger-puls på minst 10 mikrosekunder
	PORTD &= 0<<PORTD1;				// Ping triggas då trigger-pulsen går låg

	TCNT1 = 0;						// Nollställ timer
	TCCR1B |= 1<<CS12 | 1<<CS10;	// Starta timer
}

void read_IMU()
{
	//double acc_Roll_rad, acc_Pitch_rad;
	////double gyro_angular_rate[3];
	//
	//I2C_IMU_read();
	//IMU_temperature = ((double)temp_reading)/340.0f + 36.53f;
	//
	//acc_Roll_rad = atan( ((double)acc_reading[y])/(sqrt( pow(((double)acc_reading[x]), 2) + pow(((double)acc_reading[z]), 2) ) ));
	//acc_Roll = (acc_Roll_rad/3.141592)*180;
	//
	//acc_Pitch_rad = atan( ((double)acc_reading[x])/(sqrt( pow(((double)acc_reading[y]), 2) + pow(((double)acc_reading[z]), 2) ) ));
	//acc_Pitch = (acc_Pitch_rad/3.141592)*180;
	//
	//if(first_reading)
	//{
		//gyro_Pitch = acc_Pitch;
		//gyro_Roll = acc_Roll;
		//gyro_Yaw = 0;
		//
		//first_reading = 0;
	//}
	//
	//else
	//{
		//gyro_angular_rate[x] = (((double)gyro_reading[x]) - ((double)gyro_offset[x]))/GYROSCOPE_SENSITIVITY;
		//gyro_angular_rate[y] = (((double)gyro_reading[y]) - ((double)gyro_offset[y]))/GYROSCOPE_SENSITIVITY;
		//gyro_angular_rate[z] = (((double)gyro_reading[z]) - ((double)gyro_offset[z]))/GYROSCOPE_SENSITIVITY;
		//
		//gyro_Yaw += gyro_angular_rate[z]*dt;
		//gyro_Pitch -= gyro_angular_rate[y]*dt;
		//gyro_Roll += gyro_angular_rate[x]*dt;
		//
		//IMU_Pitch = 0.95*gyro_Pitch + 0.05*acc_Pitch;
		//IMU_Roll = 0.95*gyro_Roll + 0.05*acc_Roll;
	//}
	
	I2C_IMU_read();
	
	gyro_angular_rate[x] = ((((float)gyro_reading[x]) - ((float)gyro_offset[x]))/GYROSCOPE_SENSITIVITY)*(3.14/180); // rad per sek
	gyro_angular_rate[y] = ((((float)gyro_reading[y]) - ((float)gyro_offset[y]))/GYROSCOPE_SENSITIVITY)*(3.14/180);
	gyro_angular_rate[z] = ((((float)gyro_reading[z]) - ((float)gyro_offset[z]))/GYROSCOPE_SENSITIVITY)*(3.14/180);
	
	MadgwickAHRSupdateIMU(gyro_angular_rate[x], gyro_angular_rate[y], gyro_angular_rate[z], ((float)acc_reading[x]), ((float)acc_reading[y]), ((float)acc_reading[z]));
	
	IMU_Yaw = (atan2(2*q1*q2 - 2*q0*q3, 2*pow(q0,2) + 2*pow(q1,2) -1)/3.14)*180;
	IMU_Pitch = -(asin(2*q1*q3 + 2*q0*q2)/3.14)*180;
	IMU_Roll = (atan2(2*q2*q3 - 2*q0*q1, 2*pow(q0,2) + 2*pow(q3,2) - 1)/3.14)*180;
	
	//gravity[x] = 2*(q1*q3 - q0*q2);
	//gravity[y] = 2*(q0*q1 + q2*q3);
	//gravity[z] = q0*q0 - q1*q1 - q2*q2 + q3*q3;
	//
	//IMU_Yaw = (atan2(2*q1*q2 - 2*q0*q3, 2*q0*q0 + 2*q1*q1 - 1)/3.14)*180;
	//IMU_Pitch = (atan(gravity[x] / sqrt(gravity[y]*gravity[y] + gravity[z]*gravity[z]))/3.14)*180;
	//IMU_Roll = (atan(gravity[y] / sqrt(gravity[x]*gravity[x] + gravity[z]*gravity[z]))/3.14)*180;

	
}

void time_to_distance()
{
	US_distance =  US_reading*0.0642857143;		// Gör om till tid i millisekunder
	US_distance = US_distance - 0.75;			// Subrtrahera offset mellan triggerpuls och ekopuls
	US_distance = 34.33*US_distance/2;			// Gör om till cm

	if(US_distance > 250)
	US_distance = 250;							// Sätt max-avstånd (så att ej får överslag då det görs om till 8 bitar)
}

// XXXXX Endast grundfunktionalitet, kommer behöva utökas för att upptäcka t.ex. när bara ena sidan är tillförlitlig.
void calculate_Yaw()
{
	double l_delta_right = IR_distance[2] - IR_distance[3];
	double l_delta_left = IR_distance[5] - IR_distance[6];

	Yaw_right = (atan(l_delta_right/IR_sensor_distance_right)/3.14)*180;		// Yaw-beräkning med de högra sidosensorerna
	Yaw_left = (atan(l_delta_left/IR_sensor_distance_left)/3.14)*180;			// Yaw-beräkning med de vänstra sidosensorerna

	IR_Yaw = (Yaw_right + Yaw_left)/2;
}

void save_to_buffer()
{
	//cli();					// XXXXX Behövs väl inte maska bort avbrott? Vad kan hända?

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

	//sei();					// XXXXX Behövs väl inte maska bort avbrott? Vad kan hända?
}

void kalibrering()				// XXXXX Endast för att kunna kalibrera sensorer!
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