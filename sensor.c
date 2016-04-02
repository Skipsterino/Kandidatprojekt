/*
 * Projekt_test.c
 *
 * Created: 3/30/2016 8:38:40 AM
 *  Author: fregu856
 */ 

#define F_CPU 16000000UL  // 16 MHz

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <math.h>
#include <util/delay.h>

// IR0 = IR rakt fram
// IR1 = IR fram - nedåt
// IR2 = IR fram - höger
// IR3 = IR bak - höger
// IR4 = IR bak - uppåt
// IR5 = IR bak - vänster
// IR6 = IR fram - vänster

double Vref = 5;
const int delay_time = 50; // tid i millisekunder (50 ger alltså ungefär 20Hz, lagom för US-sensorn)

double IR0_reading1, IR0_reading2, IR0_reading3, IR0_reading4, IR0_reading5;
double IR1_reading1, IR1_reading2, IR1_reading3, IR1_reading4, IR1_reading5;
double IR2_reading1, IR2_reading2, IR2_reading3, IR2_reading4, IR2_reading5;
double IR3_reading1, IR3_reading2, IR3_reading3, IR3_reading4, IR3_reading5;
double IR4_reading1, IR4_reading2, IR4_reading3, IR4_reading4, IR4_reading5;
double IR5_reading1, IR5_reading2, IR5_reading3, IR5_reading4, IR5_reading5;
double IR6_reading1, IR6_reading2, IR6_reading3, IR6_reading4, IR6_reading5;

double IR0_ADC, IR1_ADC, IR2_ADC, IR3_ADC, IR4_ADC, IR5_ADC, IR6_ADC;
double IR0_voltage, IR1_voltage, IR2_voltage, IR3_voltage, IR4_voltage, IR5_voltage, IR6_voltage;	// XXXXX Kommer förmodligen tas bort
double IR0_distance, IR1_distance, IR2_distance, IR3_distance, IR4_distance, IR5_distance, IR6_distance;

double US_reading;	// (US = Ultra Sound)
double US_distance;

double Yaw, Pitch, Roll, Yaw_right, Yaw_left; // XXXXX Yaw_right och Yaw_left skall göras lokala så småningom

double counter = 0;	// XXXXX Endast för sensor-kalibrering
double sum = 0;		// XXXXX Endast för sensor-kalibrering

double IR_sensor_distance_right = 14.5;	// Avståndet mellan högra sidosensorerna (cm)
double IR_sensor_distance_left = 14.5;	// Avståndet mellan vänstra sidosensorerna (cm)

uint8_t buffer0_IR0, buffer1_IR1, buffer2_IR2, buffer3_IR3, buffer4_IR4, buffer5_IR5, buffer6_IR6, buffer7_US;
int8_t buffer8_Yaw, buffer9_Pitch, buffer10_Roll;

int byte_to_send = 0;

typedef struct
{
	double voltage;			// XXXXX Kommer förmodligen bytas mot ADC-värdet
	double distance;
} table;

table IR0_table[] =			// XXXXX Med ADC-värdet isället för spänning
{
	{119, 150},
	{120, 140},
	{121, 130},
	{122, 120},
	{123.0, 110},
	{131.3, 100},
	{145.0, 90},
	{160.7, 80},
	{180.0, 70},
	{215.2, 60},
	{259.8, 50},
	{325.6, 40},
	{426.6, 30},
	{543.7, 20}
};

table IR1_table[] =
{
	{0.4, 150},
	{0.45, 140},
	{0.5, 130},
	{0.55, 120},
	{0.6, 110},
	{0.65, 100},
	{0.7, 90},
	{0.8, 80},
	{0.9, 70},
	{1.05, 60},
	{1.25, 50},
	{1.55, 40},
	{2, 30},
	{2.5, 20}
};

table IR2_table[] =
{
	{0.4, 150},
	{0.45, 140},
	{0.5, 130},
	{0.55, 120},
	{0.6, 110},
	{0.65, 100},
	{0.7, 90},
	{0.8, 80},
	{0.9, 70},
	{1.05, 60},
	{1.25, 50},
	{1.55, 40},
	{2, 30},
	{2.5, 20}
};

table IR3_table[] =
{
	{0.4, 150},
	{0.45, 140},
	{0.5, 130},
	{0.55, 120},
	{0.6, 110},
	{0.65, 100},
	{0.7, 90},
	{0.8, 80},
	{0.9, 70},
	{1.05, 60},
	{1.25, 50},
	{1.55, 40},
	{2, 30},
	{2.5, 20}
};

table IR4_table[] =
{
	{0.4, 150},
	{0.45, 140},
	{0.5, 130},
	{0.55, 120},
	{0.6, 110},
	{0.65, 100},
	{0.7, 90},
	{0.8, 80},
	{0.9, 70},
	{1.05, 60},
	{1.25, 50},
	{1.55, 40},
	{2, 30},
	{2.5, 20}
};

table IR5_table[] =
{
	{0.4, 150},
	{0.45, 140},
	{0.5, 130},
	{0.55, 120},
	{0.6, 110},
	{0.65, 100},
	{0.7, 90},
	{0.8, 80},
	{0.9, 70},
	{1.05, 60},
	{1.25, 50},
	{1.55, 40},
	{2, 30},
	{2.5, 20}
};

table IR6_table[] =
{
	{0.4, 150},
	{0.45, 140},
	{0.5, 130},
	{0.55, 120},
	{0.6, 110},
	{0.65, 100},
	{0.7, 90},
	{0.8, 80},
	{0.9, 70},
	{1.05, 60},
	{1.25, 50},
	{1.55, 40},
	{2, 30},
	{2.5, 20}
};

void ADC_IR();
void ADC_to_voltage();				// XXXXX Kommer förmodligen tas bort (onödig)
double lookup_voltage(table* volt_dist_table, double voltage, int table_size);	// (lookup_ADC)
void voltage_to_distance();			// XXXXX (ADC_to_distance)
void read_IMU();
void time_to_distance();
void calculate_Yaw();
void save_to_buffer();
void send_ping();
void kalibrering();					// XXXXX Endast för att kunna kalibrera sensorer!


///////////////////////////////////////////////////////////////////////////////////////////////////////////////


int main(void)
{
	ADMUX = 0x60;						//Skicka in 0110 0000 på ADMUX för att välja ADC0 som inkanal till A/D (bit 4-0), sätta ADLAR (bit 5) (=> de 8 mest sig. bitarna av resultatet ges i ADCH) och välja "AVCC with external capacitor at AREF pin" som Voltage reference select. (bit 7-6) 
	ADCSRA = 0x8F;						//Skicka in 1000 1111 på ADCSRA för att enable:a ADC (bit 7), ADC Complete-avbrott (bit 3) samt dela klockfrekvensen med 128 (=> A/D klockas 125 kHz)
	SMCR |= 0<<SM2 | 0<<SM1 | 1<<SM0;	// Sleep Mode Select till "ADC Noise Reduction" (Så att "sleep_cpu()" ger just detta viloläge)
	DDRD = 0x02;						// Skicka in 0000 0010 på DDRD för att sätta PD1 till utgång, PD6 till ingång
	TIMSK1 |= 1<<ICIE1;					// Enable:a "Input Capture Interrupt" (För US)
	TCCR1B &= 0<<ICES1;					// Input Capture triggar på negativ (fallande) flank
	sei();								// Enable avbrott öht (bit 7 på SREG sätts till 1)
	
	while (1)
	{
		ADC_IR();						// (X) Sampla IR-sensorerna
		read_IMU();						// Hämta data från IMU
		
		send_ping();					// (X) Starta en US-mätning 
		
		voltage_to_distance();			// (X) Konvertera spänning till avstånd (IR-sensorerna)
		time_to_distance();				// (X) Konvertera tid till spänning (US-sensorn)
		calculate_Yaw();				// (\) Räkna ut Yaw-vinkeln (XXXXX Endast grundfunktionalitet)
		
		save_to_buffer();				// (\) Spara undan i buffert (XXXXX Endast högst preliminärt. Vilka datatyper? Hur casta till dessa? Maska bort avbrott?)
		
		_delay_ms(delay_time);			// (X) Vila
		
		
		kalibrering();					// XXXXX Endast för att kunna kalibrera sensorer!
	}
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////


ISR(TIMER1_CAPT_vect)		// Input Capture (US-sensorn)
{
	TCCR1B &= 0<<CS12 | 0<<CS10;			// Stanna timer
	
	US_reading = ICR1L;						// Läs in timer-värdets låga byte...
	US_reading = US_reading + (ICR1H<<8);	// ...och addera timer-värdets höga byte.
}

ISR(ADC_vect)	// ADC Conversion Complete
{
	if(ADMUX == 0x60)
	{
		IR0_reading5 = IR0_reading4;
		IR0_reading4 = IR0_reading3;
		IR0_reading3 = IR0_reading2;
		IR0_reading2 = IR0_reading1;
		IR0_reading1 = ADCH << 2;				// Läs in ADC:ns 8 högsta (av 10) bitar, skiftade två steg uppåt
		IR0_ADC = (IR0_reading1 + IR0_reading2 + IR0_reading3 + IR0_reading4 + IR0_reading5)/5;
	}
	
	if(ADMUX == 0x61)
	{
		IR1_reading5 = IR1_reading4;
		IR1_reading4 = IR1_reading3;
		IR1_reading3 = IR1_reading2;
		IR1_reading2 = IR1_reading1;
		IR1_reading1 = ADCH << 2;
		IR1_ADC = (IR1_reading1 + IR1_reading2 + IR1_reading3 + IR1_reading4 + IR1_reading5)/5;
	}
	if(ADMUX == 0x62)
	{
		IR2_reading5 = IR2_reading4;
		IR2_reading4 = IR2_reading3;
		IR2_reading3 = IR2_reading2;
		IR2_reading2 = IR2_reading1;
		IR2_reading1 = ADCH << 2;
		IR2_ADC = (IR2_reading1 + IR2_reading2 + IR2_reading3 + IR2_reading4 + IR2_reading5)/5;
	}
	if(ADMUX == 0x63)
	{
		IR3_reading5 = IR3_reading4;
		IR3_reading4 = IR3_reading3;
		IR3_reading3 = IR3_reading2;
		IR3_reading2 = IR3_reading1;
		IR3_reading1 = ADCH << 2;
		IR3_ADC = (IR3_reading1 + IR3_reading2 + IR3_reading3 + IR3_reading4 + IR3_reading5)/5;
	}
	if(ADMUX == 0x64)
	{
		IR4_reading5 = IR4_reading4;
		IR4_reading4 = IR4_reading3;
		IR4_reading3 = IR4_reading2;
		IR4_reading2 = IR4_reading1;
		IR4_reading1 = ADCH << 2;
		IR4_ADC = (IR4_reading1 + IR4_reading2 + IR4_reading3 + IR4_reading4 + IR4_reading5)/5;
	}
	if(ADMUX == 0x65)
	{
		IR5_reading5 = IR5_reading4;
		IR5_reading4 = IR5_reading3;
		IR5_reading3 = IR5_reading2;
		IR5_reading2 = IR5_reading1;
		IR5_reading1 = ADCH << 2;
		IR5_ADC = (IR5_reading1 + IR5_reading2 + IR5_reading3 + IR5_reading4 + IR5_reading5)/5;
	}
	if(ADMUX == 0x66)
	{
		IR6_reading5 = IR6_reading4;
		IR6_reading4 = IR6_reading3;
		IR6_reading3 = IR6_reading2;
		IR6_reading2 = IR6_reading1;
		IR6_reading1 = ADCH << 2;
		IR6_ADC = (IR6_reading1 + IR6_reading2 + IR6_reading3 + IR6_reading4 + IR6_reading5)/5;
	}
	
	++ADMUX;
}

//ISR(SPI_STC_vect) // Avbrottsvektor för data-sändning
//{
	//switch(byte_to_send)
	//{
		//case 0:
			//SPI_REGISTER_XXXXX = buffer0_IR0;
			//++byte_to_send;
			//break;
		//case 1:
			//SPI_REGISTER_XXXXX = buffer1_IR1;
			//++byte_to_send;
			//break;
		//case 2:
			//SPI_REGISTER_XXXXX = buffer2_IR2;
			//++byte_to_send;
			//break;
		//case 3:
			//SPI_REGISTER_XXXXX = buffer3_IR3;
			//++byte_to_send;
			//break;
		//case 4:
			//SPI_REGISTER_XXXXX = buffer4_IR4;
			//++byte_to_send;
			//break;
		//case 5:
			//SPI_REGISTER_XXXXX = buffer5_IR5;
			//++byte_to_send;
			//break;
		//case 6:
			//SPI_REGISTER_XXXXX = buffer6_IR6;
			//++byte_to_send;
			//break;
		//case 7:
			//SPI_REGISTER_XXXXX = buffer7_US;
			//++byte_to_send;
			//break;
		//case 8:
			//SPI_REGISTER_XXXXX = buffer8_Yaw;
			//++byte_to_send;
			//break;
		//case 9:
			//SPI_REGISTER_XXXXX = buffer9_Pitch;
			//++byte_to_send;
			//break;
		//case 10:
			//SPI_REGISTER_XXXXX = buffer10_Roll;
			//byte_to_send = 0;
			//break;
	//}
//}

void ADC_IR()
{
	while(ADMUX < 0x67)
	{
		sleep_enable();
		sleep_cpu();				// Processorn går ner i "ADC Noise Reduction"-vila och startar direkt en ADC
		sleep_disable();			// (Vilofunktionen enable:as endast här för att "programmeraren ska ha full kontroll över när processorn sover")
	}
	
	ADMUX = 0x60;					// Återställ ADC:n till ADC0 som inkanal
	ADC_to_voltage();				// XXXXX Kommer förmodligen tas bort
}

void ADC_to_voltage()				// XXXXX Kommer förmodligen tas bort
{
	IR0_voltage = (Vref*IR0_ADC)/1024;
	IR1_voltage = (Vref*IR1_ADC)/1024;
	IR2_voltage = (Vref*IR2_ADC)/1024;
	IR3_voltage = (Vref*IR3_ADC)/1024;
	IR4_voltage = (Vref*IR4_ADC)/1024;
	IR5_voltage = (Vref*IR5_ADC)/1024;
	IR6_voltage = (Vref*IR6_ADC)/1024;
} 

void voltage_to_distance()
{
	IR0_distance = lookup_voltage(IR0_table, IR0_ADC, 14);
	IR1_distance = lookup_voltage(IR1_table, IR1_voltage, 14);
	IR2_distance = lookup_voltage(IR2_table, IR2_voltage, 14);
	IR3_distance = lookup_voltage(IR3_table, IR3_voltage, 14);
	IR4_distance = lookup_voltage(IR4_table, IR4_voltage, 14);
	IR5_distance = lookup_voltage(IR5_table, IR5_voltage, 14);
	IR6_distance = lookup_voltage(IR6_table, IR6_voltage, 14);
}

double lookup_voltage(table* volt_dist_table, double voltage, int table_size)
{
	if(voltage <= volt_dist_table[0].voltage)
		return volt_dist_table[0].distance;
	
	if(voltage >= volt_dist_table[table_size-1].voltage)
		return volt_dist_table[table_size-1].distance;
	
	int i;

	for(i = 0; i < table_size-1; i++)
	{
		if (volt_dist_table[i].voltage <= voltage && volt_dist_table[i+1].voltage >= voltage )
		{
			double diff_volt = voltage - volt_dist_table[i].voltage;
			double step_length = volt_dist_table[i+1].voltage - volt_dist_table[i].voltage;

			return volt_dist_table[i].distance + (volt_dist_table[i+1].distance - volt_dist_table[i].distance)*(diff_volt/step_length);
		}
	}
}

void send_ping()
{
	PORTD &= 0<<PORTD1;
	_delay_us(2);
	PORTD |= 1<<PORTD1;
	_delay_us(12);
	PORTD &= 0<<PORTD1;					// Trigga ping
	
	TCNT1 = 0;							// Nollställ timer
	TCCR1B |= 1<<CS12 | 1<<CS10;		// Starta timer
}

void read_IMU()
{
	
}

void time_to_distance()
{
	US_distance =  US_reading*0.0642857143;		// Gör om till tid i millisekunder
	US_distance = US_distance - 0.75;			// Subrtrahera offset mellan triggerpuls och ekopuls
	US_distance = 34.33*US_distance/2;			// Gör om till cm
}

void calculate_Yaw()	// XXXXX Endast grundfunktionalitet, kommer behöva utökas för att upptäcka t.ex. när bara ena sidan är tillförlitlig.
{
	double l_delta_right = IR2_distance - IR3_distance;
	double l_delta_left = IR5_distance - IR6_distance;
	
	Yaw_right = (atan(l_delta_right/IR_sensor_distance_right)/3.14)*180;	// Yaw-beräkning med de högra sidosensorerna
	Yaw_left = (atan(l_delta_left/IR_sensor_distance_left)/3.14)*180;		// Yaw-beräkning med de vänstra sidosensorerna
	
	Yaw = (Yaw_right + Yaw_left)/2;										
}

void save_to_buffer()
{
	//cli();					// XXXXX Behövs väl inte maska bort avbrott? Vad kan hända?
	
	buffer0_IR0 = IR0_distance;
	buffer1_IR1 = IR1_distance;
	buffer2_IR2 = IR2_distance;
	buffer3_IR3 = IR3_distance;
	buffer4_IR4 = IR4_distance;
	buffer5_IR5 = IR5_distance;
	buffer6_IR6 = IR6_distance;
	
	buffer7_US = US_distance;
	
	buffer8_Yaw = Yaw;
	buffer9_Pitch = Pitch;
	buffer10_Roll = Roll;
	
	//sei();					// XXXXX Behövs väl inte maska bort avbrott? Vad kan hända?
}	

void kalibrering()		// XXXXX Endast för att kunna kalibrera sensorer!
{
	sum = sum + IR0_ADC;	
	++counter;				
}
