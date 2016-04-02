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

// IR0 = IR fram - rakt
// IR1 = IR fram - nedåt
// IR2 = IR fram - höger
// IR3 = IR bak - höger
// IR4 = IR bak - uppåt
// IR5 = IR bak - vänster
// IR6 = IR fram - vänster

double Vref = 5;
const int delay_time = 100; // tid i millisekunder

double IR_reading[7][5];

double IR_ADC[7], IR_voltage[7], IR_distance[7];

double US_reading;	// (US = Ultra Sound)
double US_distance;

double Yaw, Pitch, Roll, Yaw_right, Yaw_left; // XXXXX Yaw_right och Yaw_left skall göras lokala så småningom

double counter = 0;	// XXXXX Endast för sensor-kalibrering
double sum = 0;		// XXXXX Endast för sensor-kalibrering

double IR_sensor_distance_right = 14.5;	// Avståndet mellan högra sidosensorerna (cm)
double IR_sensor_distance_left = 14.5;	// Avståndet mellan vänstra sidosensorerna (cm)

uint8_t buffer0_IR0, buffer1_IR1, buffer2_IR2, buffer3_IR3, buffer4_IR4, buffer5_IR5, buffer6_IR6, buffer7_US;	// Unsigned 8-bitars int, 0 - 255
int8_t buffer8_Yaw, buffer9_Pitch, buffer10_Roll;	// Signed 8-bitars int

int byte_to_send = 0;

typedef struct
{
	double voltage;
	double distance;
} table;

table IR0_table[] =
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
void ADC_to_voltage();
double lookup_voltage(table* volt_dist_table, double voltage, int table_size);
void voltage_to_distance();
void read_IMU();
void time_to_distance();
void calculate_Yaw();
void save_to_buffer();
void send_ping();
void kalibrering();			// XXXXX Endast för att kunna kalibrera sensorer!


///////////////////////////////////////////////////////////////////////////////////////////////////////////////


int main(void)
{
	ADMUX = 0x60;						//Skicka in 0110 0000 på ADMUX för att välja ADC0 som inkanal till A/D (bit 4-0), sätta ADLAR (bit 5) (=> de 8 mest sig. bitarna av resultatet ges i ADCH) och välja "AVCC with external capacitor at AREF pin" som Voltage reference select. (bit 7-6) 
	ADCSRA = 0x8F;						//Skicka in 1000 1111 på ADCSRA för att enable:a ADC (bit 7), ADC Complete-avbrott (bit 3) samt dela klockfrekvensen med 128 (=> A/D klockas 125 kHz)
	SMCR |= 0<<SM2 | 0<<SM1 | 1<<SM0;	// Sleep Mode Select till "ADC Noise Reduction" (Så att "sleep_cpu()" ger just detta viloläge)
	DDRD = 0x02;						// Skicka in 0000 0010 på DDRD för att sätta PD1 till utgång, PD6 till ingång
	TIMSK1 |= 1<<ICIE1;					// Enable:a "Input Capture Interrupt" (För US)
	TCCR1B &= 0<<ICES1;					// Input Capture tiggar på negativ (fallande) flank
	sei();								// Enable avbrott öht (bit 7 på SREG sätts till 1)
	
	while (1)
	{
		ADC_IR();						// (X) Sampla IR-sensorerna
		read_IMU();						// Hämta data från IMU
		
		send_ping();
		
		voltage_to_distance();			// (X) Konvertera spänning till avstånd (IR-sensorerna)
		time_to_distance();				// Konvertera tid till spänning (US-sensorn)
		calculate_Yaw();				// Räkna ut Yaw-vinkeln
		
		save_to_buffer();				// Spara undan i buffert (stäng av avbrott)
		
		_delay_ms(delay_time);			// (X) Vila
		
		kalibrering();					// XXXXX Endast för att kunna kalibrera sensorer!
	}
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////

ISR(TIMER1_CAPT_vect)		// Input Capture (US-sensorn)
{
	TCCR1B &= 0<<CS12 | 0<<CS10;			// Stanna timer
	
	US_reading = ICR1L;							// Läs in timer-värdets låga byte...
	US_reading = US_reading + (ICR1H<<8);		// ...och addera timer-värdets höga byte.
}

ISR(ADC_vect)	// ADC Conversion Complete
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
	
	for (int i = 5; i>=2; --i)
	{
		IR_reading[sensor_ID][i]=IR_reading[sensor_ID][i-1];
	}
	IR_reading[sensor_ID][1] = ADCH << 2;				// Läs in ADC:ns 8 högsta (av 10) bitar, skiftade två steg uppåt
	IR_ADC[sensor_ID] = (IR_reading[sensor_ID][1] + IR_reading[sensor_ID][2] + IR_reading[sensor_ID][3] + IR_reading[sensor_ID][4] + IR_reading[sensor_ID][5])/5;

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
	ADC_to_voltage();	
}

void ADC_to_voltage()
{
	IR_voltage[0] = (Vref*IR_ADC[0])/1024;
	IR_voltage[1] = (Vref*IR_ADC[1])/1024;
	IR_voltage[2] = (Vref*IR_ADC[2])/1024;
	IR_voltage[3] = (Vref*IR_ADC[3])/1024;
	IR_voltage[4] = (Vref*IR_ADC[4])/1024;
	IR_voltage[5] = (Vref*IR_ADC[5])/1024;
	IR_voltage[6] = (Vref*IR_ADC[6])/1024;
} 

void voltage_to_distance()
{
	IR_distance[0] = lookup_voltage(IR0_table, IR_voltage[0], 14);
	IR_distance[1] = lookup_voltage(IR1_table, IR_voltage[1], 14);
	IR_distance[2] = lookup_voltage(IR2_table, IR_voltage[2], 14);
	IR_distance[3] = lookup_voltage(IR3_table, IR_voltage[3], 14);
	IR_distance[4] = lookup_voltage(IR4_table, IR_voltage[4], 14);
	IR_distance[5] = lookup_voltage(IR5_table, IR_voltage[5], 14);
	IR_distance[6] = lookup_voltage(IR6_table, IR_voltage[6], 14);
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
	PORTD &= 0<<PORTD1;				// Trigga ping
	
	TCNT1 = 0;						// Nollställ timer
	TCCR1B |= 1<<CS12 | 1<<CS10;	// Starta timer 
}

void read_IMU()
{
	
}

void time_to_distance()
{
	US_distance =  US_reading*0.0642857143;		// Gör om till tid i millisekunder
	US_distance = US_distance - 0.75;			// Subrtrahera offset mellan triggerpuls och ekopuls
	US_distance = 34.33*US_distance/2;			// Gör om till cm
	
	if(US_distance > 250)
	US_distance = 250;						// Sätt max-avstånd (så att ej får överslag då det görs om till 8 bitar)
}

void calculate_Yaw()	// XXXXX Endast grundfunktionalitet, kommer behöva utökas för att upptäcka t.ex. när bara ena sidan är tillförlitlig.
{
	double l_delta_right = IR_distance[2] - IR_distance[3];
	double l_delta_left = IR_distance[5] - IR_distance[6];
	
	Yaw_right = (atan(l_delta_right/IR_sensor_distance_right)/3.14)*180;	// Yaw-beräkning med de högra sidosensorerna
	Yaw_left = (atan(l_delta_left/IR_sensor_distance_left)/3.14)*180;		// Yaw-beräkning med de vänstra sidosensorerna
	
	Yaw = (Yaw_right + Yaw_left)/2;
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
	
	buffer8_Yaw = Yaw;
	buffer9_Pitch = Pitch;
	buffer10_Roll = Roll;
	
	//sei();					// XXXXX Behövs väl inte maska bort avbrott? Vad kan hända?
}

void kalibrering()		// XXXXX Endast för att kunna kalibrera sensorer!
{
	sum = sum + IR_ADC[0];
	++counter;
}
