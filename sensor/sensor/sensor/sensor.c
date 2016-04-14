/*
* sensor.c
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

#include "sensor.h"
#include "I2C.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "motion_driver_test.c"

int16_t testvariabel;

int main(void)
{
	init_ADC();
	init_US();
	init_SPI();
	init_timer();						// Initiera en timer för att hålla koll på förfluten tid.
	init_I2C();
	init_IMU();

	sei();								// Tillåt avbrott (bit 7 på SREG sätts till 1)

	while (1)
	{
		if (SPI_done)
		{

			ADC_IR();						// Sampla IR-sensorerna
			read_IMU();						// Hämta data från IMU
			
			send_ping();					// Starta en US-mätning
			
			ADC_to_distance();				// Konvertera ADC-värde till avstånd (IR-sensorerna)
			time_to_distance();				// Konvertera tid till avstånd (US-sensorn)
			calculate_Yaw();				// Räkna ut Yaw-vinkeln
			save_to_buffer();				// Spara undan i buffert
			
			SPI_done = 0;

			//kalibrering();				// XXXXX Endast för att kunna kalibrera sensorer!
			
		}
		
		_delay_ms(delay_time);			// Vila för att få lagom frekvens
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
* AVBROTTSVEKTOR INT0_vect
*
* BESKRIVNING
*
* Hanterar avbrott från IMU:n, nu finns ny data att läsa.
*
* INDATA
*
* -
*
* UTDATA
*
* -
*
* PÅVERKAR
*
* IMU_data_ready:		Anger om det finns ny data att hämta från IMU:n (1) eller ej (0)
*
*/

ISR(INT0_vect)
{
	IMU_data_ready = 1;
}

/*
* AVBROTTSVEKTOR TIMER1_CAPT_vect
*
* BESKRIVNING
*
* Hanterar avbrott för ultraljudssensorn. Stannar timern som startades när ljudpulsen sändes och läser av dess värde.
* Sparar undan värdet i US_reading.
*
* INDATA
*
* -
*
* UTDATA
*
* -
*
* PÅVERKAR
*
* US_reading:		Det senaste mätvärdet från US-sensorn (tid)
*
*/

ISR(TIMER1_CAPT_vect)
{
	TCCR1B &= 0<<CS12 | 0<<CS10;			// Stanna timer

	US_reading = ICR1L;						// Läs in timer-värdets låga byte...
	US_reading = US_reading + (ICR1H<<8);	// ...och addera timer-värdets höga byte.
}

/*
* AVBROTTSVEKTOR ADC_vect
*
* BESKRIVNING
*
* Hanterar avbrott från ADC när en konvertering blir klar. Läser in värdet till IR_latest_reading.
* Sätter att nästa IR-sensor ska samplas härnäst.
*
* INDATA
*
* -
*
* UTDATA
*
* -
*
* PÅVERKAR
*
* IR_latest_reading:		Den senaste avlästa värdet för respektive IR-sensor
*
*/

ISR(ADC_vect)		// ADC Conversion Complete
{
	IR_latest_reading[ADMUX - 0x60] = ADCH<<2;		// Läs in ADC:ns 8 högsta (av 10) bitar, skiftade två steg uppåt
	++ADMUX;
}

/*
* AVBROTTSVEKTOR TIMER2_OVF_vect
*
* BESKRIVNING
*
* Räknar antalet overflows som vi får på timern som rullar sedan senaste SPI-händelse.
* När SPI_overflow är stor vet vi att nästa byte som vi ska skicka är byte noll.
* Används för att återställa sync på bussen om vi skulle råka hamna ur sync.
*
* INDATA
*
* -
*
* UTDATA
*
* -
*
* PÅVERKAR
*
* SPI_overflow:		Anger hur många gånger vi har fått overflow på TIMER2
*
*/

ISR(TIMER2_OVF_vect)
{
	++SPI_overflow;
}

/*
* AVBROTTSVEKTOR SPI_STC_vect
*
* BESKRIVNING
*
* Avbrottsvektor för datasändning. En ny bit skiftas in i "sänd-registret"
*
* INDATA
*
* -
*
* UTDATA
*
* -
*
* PÅVERKAR
*
* byte_to_send:		Nästa byte som skall skickas
* SPI_done:			Anger om en hel överföring (16 bytes) är klar
*
*/

ISR(SPI_STC_vect)
{
	if(SPI_overflow >= 2)		// Är det länge sedan vi fick ett avbrott ska vi börja om på byte noll (ny sändning)
	{
		byte_to_send = 0;
	}
	
	SPI_overflow = 0;
	
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
			SPDR = buffer8_IR_Yaw_left;
			++byte_to_send;
			break;
		}
		case 9:
		{
			SPDR = buffer9_IR_Yaw_right;
			++byte_to_send;
			break;
		}
		case 10:
		{
			SPDR = buffer10_IMU_Yaw_Low;
			++byte_to_send;
			break;
		}
		case 11:
		{
			SPDR = buffer11_IMU_Yaw_High;
			++byte_to_send;
			break;
		}
		case 12:
		{
			SPDR = buffer12_Pitch;
			++byte_to_send;
			break;
		}
		case 13:
		{
			SPDR = buffer13_Roll;
			++byte_to_send;
			break;
		}
		case 14:
		{
			SPDR = 0xff;
			++byte_to_send;
			break;
		}
		case 15:
		{
			SPDR = 0xff;
			byte_to_send = 0;
			SPI_done = 1;		//Klart! Nu kan vi göra annat som genererar avbrott
			break;
		}
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
* FUNKTION init_ADC()
*
* BESKRIVNING
*
* Initierar ADC, se detaljer nednan.
*
* INDATA
*
* -
*
* UTDATA
*
* -
*
* PÅVERKAR
*
* -
*
*/

void init_ADC()
{
	ADMUX = 0x60;						// Skicka in 0110 0000 på ADMUX för att välja ADC0 som inkanal till A/D (bit 4-0),
	// sätta ADLAR (bit 5) (=> de 8 mest sig. bitarna av resultatet ges i ADCH) och välja
	// "AVCC with external capacitor at AREF pin" som Voltage reference select. (bit 7-6)
	ADCSRA = 0x8F;						// Skicka in 1000 1111 på ADCSRA för att enable:a ADC (bit 7), ADC Complete-avbrott
	// (bit 3) samt dela klockfrekvensen med 128 (=> A/D klockas 125 kHz)
	SMCR |= 0<<SM2 | 0<<SM1 | 1<<SM0;	// Sleep Mode Select till "ADC Noise Reduction" (Så att "sleep_cpu()" ger just detta viloläge)
}

/*
* FUNKTION init_US()
*
* BESKRIVNING
*
* Initierar avbrottssystem för ultraljudssensorn, se detljer nedan.
*
* INDATA
*
* -
*
* UTDATA
*
* -
*
* PÅVERKAR
*
* -
*
*/

void init_US()
{
	DDRD |= 1<<PORTD1;					// Sätt PD1 till utgång
	DDRD &= ~(1<<PORTD6);				// Sätt PD6(ICP1) till ingång
	TIMSK1 |= 1<<ICIE1;					// Enable:a "Input Capture Interrupt" (För US)
	TCCR1B &= ~(1<<ICES1);				// Input Capture triggar på negativ (fallande) flank
}

/*
* FUNKTION init_SPI()
*
* BESKRIVNING
*
* Initierar SPI bussen. Se detaljer nedan.
*
* INDATA
*
* -
*
* UTDATA
*
* -
*
* PÅVERKAR
*
* -
*
*/

void init_SPI()
{
	SPCR = 1<<SPIE;							// Enable avbrott SPI
	DDRB = 0x40;							// Skicka in 0100 0000 på DDRB för att sätta MISO till utgång, resten ingång. (SPI)
	SPCR |= (1<<SPE);						// Enable:a SPI
	SPDR = 0xff;							// Lägg in något nollskiljt i data registret, så att inte första sändningen missas.
	
	//Starta en timer för att hålla bussen i sync
	TCCR2B |= 1<<CS22 | 1<< CS21| 1<<CS20;	// Prescaler 1024
	TCNT2 = 0;								// Nollställ timern
	TIMSK2 |= 1<<TOIE2;						// Tillåt overflow-avbrott
}

/*
* FUNKTION init_timer()
*
* BESKRIVNING
*
* Initierar en timer för att hålla koll på förfluten tid sedan programstart.
*
* INDATA
*
* -
*
* UTDATA
*
* -
*
* PÅVERKAR
*
* -
*
*/

void init_timer()
{
	TCCR0B |= 1<<CS01 | 1<<CS00;		//Sätt preskalern till 64
	TIMSK0 |= 1<<TOIE0;					//Tillåt overflow-avbrott på timer0 för millis
}

/*
* FUNKTION init_I2C()
*
* BESKRIVNING
*
* Initierar I2C, se detaljer nedan.
*
* INDATA
*
* -
*
* UTDATA
*
* -
*
* PÅVERKAR
*
* -
*
*/

void init_I2C()
{
	PORTC = (1<<PC0) | (1<<PC1);		// Gör I2C-portarna till utgångar

	TWSR &= ~((1<<TWPS1) | (1<<TWPS0));	// Sätt presecalerbitarna till 0 (ger prescalervärde = 1)
	TWBR = 0x20;						// Skicka in 0x20 på TWBR för att tillsammans med Prescalerbitarna TWPS1-TWPS0 sätta I2C-frekvensen till ungefär 200 kHz
}

/*
* FUNKTION init_IMU()
*
* BESKRIVNING
*
* Initierar IMU:n, se detaljer nedan.
*
* INDATA
*
* -
*
* UTDATA
*
* -
*
*/

void init_IMU()
{
	init_IMU_interrupt();													// TIllåter avbrott från IMU:n
	mpu_init();																// Initierar IMU, väcker IMU:n från viloläge
	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);							// Sätter vilka sensorer vi ska använda
	mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);						// Väljer vilka sensordata som skickas till FIFO
	mpu_set_sample_rate(MPU_HZ);											// Väljer samplingsfrekvens
	dmp_load_motion_driver_firmware();										// Programmerar DMP (IMU:ns interna processor)
	dmp_set_orientation(inv_orientation_matrix_to_scalar(_orientation));	// Sätt i vilket läge som IMU:n är monterad.
	dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL | DMP_FEATURE_TAP); // Välj funktioner som vi vill ha
	dmp_set_fifo_rate(MPU_HZ);												// Väljer med vilken frekvens som data ska läggas ut i FIFO
	mpu_set_dmp_state(USE_DMP);												// Slår på DMP
	
	_delay_ms(200);															// Ge IMU tid att starta
	
	run_self_test();														// Kalibrera gyro och accelometer
	
	IMU_data_ready = 0;															// Från start finns ingen data klar
}

/*
* FUNKTION init_IMU_interrupt()
*
* BESKRIVNING
*
* Initierar så att IMU:n ger externt avbrott 0.
*
* INDATA
*
* -
*
* UTDATA
*
* -
*
* PÅVERKAR
*
* -
*
*/

void init_IMU_interrupt()
{
	DDRD &= ~(1<<PORTD2);				// Sätt PD2(INT0) till ingång
	EICRA |= 1<<ISC01 | 1<<ISC00;		// Externt avbrott INT0 på rising edge
	EIMSK |= (1 << INT0);				// Tillåter avbrott INT0
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
* FUNKTION ADC_IR()
*
* BESKRIVNING
*
* Samplar alla IR-sensorer, sparar undan i en buffert med de fem senaste mätningarna för respektive
* sensor och medelvärdesbildar.
*
* INDATA
*
* -
*
* UTDATA
*
* -
*
* PÅVERKAR
*
* IR_ADC innehåller medelvärdet över de fem senaste mätvärdena för varje sensor.
*
*/

void ADC_IR()
{
	while(ADMUX < 0x67)
	{
		sleep_enable();
		sleep_cpu();			// Processorn går ner i "ADC Noise Reduction"-vila och startar direkt en ADC
		sleep_disable();		// (Vilofunktionen enable:as endast här för att "programmeraren ska ha full kontroll över när processorn sover")
	}

	ADMUX = 0x60;				// Återställ ADC:n till ADC0 som inkanal
	
	for(int sensor_ID = 0; sensor_ID <=7; ++sensor_ID)
	{
		for (int i = 5; i>=2; --i)		// Skifta ut den äldsta avläsningen
		{
			IR_reading[sensor_ID][i] = IR_reading[sensor_ID][i-1];
		}
		
		IR_reading[sensor_ID][1] = IR_latest_reading[sensor_ID];	// Spara in den senaste.
		
		IR_ADC[sensor_ID] = (IR_reading[sensor_ID][1] + IR_reading[sensor_ID][2] + IR_reading[sensor_ID][3] + IR_reading[sensor_ID][4] + IR_reading[sensor_ID][5])/5;	// Medelvärdesbilda över de 5 senaste
	}
}

/*
* FUNKTION read_IMU()
*
* BESKRIVNING
*
* Hämtar data från IMU och räknar ut roll, pitch och yaw utifrån returnerade
* kvaternioner.
*
* INDATA
*
* -
*
* UTDATA
*
* -
*
* PÅVERKAR
*
* IMU_Yaw:			Den beräknade Yaw-vinkeln
* IMU_Pitch:		Den beräknade Pitch-vinkeln
* IMU_Roll:			Den beräknade Roll-vinkeln
*
*/

void read_IMU()
{
	if (IMU_data_ready)
	{
		short gyro[3];
		short accel[3];
		unsigned long timestamp;
		unsigned char more;

		long quat[4];

		short sensors;

		if (0 == dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more)) // Läser från IMU
		{
			if (!more) // Kolla så att vi får med all data
			{
				IMU_data_ready = 0;
			}

			quaternion[QUAT_W] = (float)quat[QUAT_W];
			quaternion[QUAT_X] = (float)quat[QUAT_X];
			quaternion[QUAT_Y] = (float)quat[QUAT_Y];
			quaternion[QUAT_Z] = (float)quat[QUAT_Z];

			NormalizeQuaternion(quaternion);
			
			float q0 = quaternion[QUAT_W];
			float q1 = quaternion[QUAT_X];
			float q2 = quaternion[QUAT_Y];
			float q3 = quaternion[QUAT_Z];
			
			/* Räkna ut gravitationsvektorn */
			gravity[x] = 2*(q1*q3 - q0*q2);
			gravity[y] = 2*(q0*q1 + q2*q3);
			gravity[z] = q0*q0 - q1*q1 - q2*q2 + q3*q3;
			
			IMU_Yaw = (atan2(2*q1*q2 - 2*q0*q3, 2*q0*q0 + 2*q1*q1 - 1)/3.14)*180 - 0.28/250 - 0.06/250;		// Beräkna yaw-vinkel i grader
			IMU_Pitch = (atan(gravity[x] / sqrt(gravity[y]*gravity[y] + gravity[z]*gravity[z]))/3.14)*180;	// Beräkna pitch-vinkel i grader
			IMU_Roll = (atan(gravity[y] / sqrt(gravity[x]*gravity[x] + gravity[z]*gravity[z]))/3.14)*180;	// Beräkna roll-vinkel i grader
			
			/* Säkerställ att vi inte får overflow i datatyperna */
			IMU_Yaw = restrict_angle(IMU_Yaw);
			IMU_Pitch = restrict_angle(IMU_Pitch);
			IMU_Roll = restrict_angle(IMU_Roll);
			
		}
		else
		{
			error();
		}
	}
}

/*
* FUNKTION send_ping()
*
* BESKRIVNING
*
* Skickar ut en trigger-puls på 12 µs. Startar en timer som väntar på svar från
* ultraljudssensor.
*
* INDATA
*
* -
*
* UTDATA
*
* -
*
* PÅVERKAR
*
* -
*
*/

void send_ping()
{
	PORTD&= ~(1<<PORTD1);
	_delay_us(2);
	PORTD |= 1<<PORTD1;
	_delay_us(12);					// Skickar ut trigger-puls på minst 10 mikrosekunder
	PORTD&= ~(1<<PORTD1);			// Ping triggas då trigger-pulsen går låg

	TCNT1 = 0;						// Nollställ timer
	TCCR1B |= 1<<CS12 | 1<<CS10;	// Starta timer
}

/*
* FUNKTION ADC_to_distance()
*
* BESKRIVNING
*
* Omvandlar IR-sensorernas ADC-värden till avstånd genom linjärinterpolation och tabellvärden.
*
* INDATA
*
* -
*
* UTDATA
*
* -
*
* PÅVERKAR
*
* IR_distance:		IR-sensorernas uppmätta avstånd
*
*/

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

/*
* FUNKTION lookup_distance(ADC_distance_pair* ADC_dist_table, double ADC_value, int table_size)
*
* BESKRIVNING
*
* Omvandlar IR-sensorernas ADC-värden till avstånd genom linjärinterpolation och tabellvärden.
*
* INDATA
*
* ADC_dist_table:		IR-sensorns tabell för hur ADC relaterar till avstånd
* ADC_value:			IR-sensorns ADC-värde
* table_size:			Antal poster i tabellen
*
* UTDATA
*
* double:				ADC_value:s motsvarande avstånd
*
* PÅVERKAR
*
* IMU_Yaw:			Den beräknade Yaw-vinkeln
* IMU_Pitch			Den beräknade Pitch-vinkeln
* IMU_Roll			Den beräknade Roll-vinkeln
*
*/

double lookup_distance(ADC_distance_pair* ADC_dist_table, double ADC_value, int table_size)
{
	if(ADC_value <= ADC_dist_table[0].ADC_data)					// Ge min-avståndet om ADC-värdet är mindre än minsta tabell-värdet
	return ADC_dist_table[0].distance;

	if(ADC_value >= ADC_dist_table[table_size-1].ADC_data)		// Ge max-avståndet om ADC-värdet är större än största tabell-värdet
	return ADC_dist_table[table_size-1].distance;

	for(int i = 0; i < table_size-1; i++)						// Linjärinterpolation
	{
		if (ADC_dist_table[i].ADC_data <= ADC_value && ADC_value <= ADC_dist_table[i+1].ADC_data) // Linjärinterpolera om ADC-värdet ligger mellan två tabell-värden
		{
			double diff_ADC = ADC_value - ADC_dist_table[i].ADC_data;
			double step_length = ADC_dist_table[i+1].ADC_data - ADC_dist_table[i].ADC_data;

			return ADC_dist_table[i].distance + (ADC_dist_table[i+1].distance - ADC_dist_table[i].distance)*(diff_ADC/step_length);
		}
	}
	return -1;
}

/*
* FUNKTION time_to_distance()
*
* BESKRIVNING
*
* Omvandlar ultraljudssensorns tid till avstånd
*
* INDATA
*
* -
*
* UTDATA
*
* -
*
* PÅVERKAR
*
* US_distance:		Avståndet till objekt fram för ultraljudssensorn
*
*/

void time_to_distance()
{
	US_distance =  US_reading*0.0642857143;		// Gör om till tid i millisekunder
	US_distance = US_distance - 0.75;			// Subrtrahera offset mellan triggerpuls och ekopuls
	US_distance = 34.33*US_distance/2;			// Gör om till cm

	if(US_distance > 250)
	US_distance = 250;							// Sätt max-avstånd (så att ej får överslag då det görs om till 8 bitar)
}

/*
* FUNKTION calculate_Yaw()
*
* BESKRIVNING
*
* Beräknar Yaw-vinkeln baserat på IR-sensorernas avstånd
*
* INDATA
*
* -
*
* UTDATA
*
* -
*
* PÅVERKAR
*
* IR_Yaw_right:		Yaw-vinkeln baserat på de högra IR-sensorernas avstånd
* IR_Yaw_left:		Yaw-vinkeln baserat på de vänstra IR-sensorernas avstånd
*
*/

void calculate_Yaw()
{
	double l_delta_right = IR_distance[2] - IR_distance[3];						// Notation enligt Förstudie: Sensorer
	double l_delta_left = IR_distance[5] - IR_distance[6];

	IR_Yaw_right = (atan(l_delta_right/IR_sensor_distance_right)/3.14)*180;		// Yaw-beräkning med de högra sidosensorerna
	IR_Yaw_left = (atan(l_delta_left/IR_sensor_distance_left)/3.14)*180;		// Yaw-beräkning med de vänstra sidosensorerna
	
	IR_Yaw_right = restrict_angle(IR_Yaw_right);												// Begränsa vinkeln så att overflow ej fås med int8_t
	IR_Yaw_left = restrict_angle(IR_Yaw_left);
}

/*
* FUNKTION save_to_buffer()
*
* BESKRIVNING
*
* Omvandlar och sparar undan sensordatan i en buffert, klar att skickas till STYR-enheten
*
* INDATA
*
* -
*
* UTDATA
*
* -
*
*/

void save_to_buffer()
{
	IMU_Yaw_16 = IMU_Yaw;						// Konvertera IMU_Yaw till int16_t
	
	buffer0_IR0 = IR_distance[0];
	buffer1_IR1 = IR_distance[1];
	buffer2_IR2 = IR_distance[2];
	buffer3_IR3 = IR_distance[3];
	buffer4_IR4 = IR_distance[4];
	buffer5_IR5 = IR_distance[5];
	buffer6_IR6 = IR_distance[6];

	buffer7_US = US_distance;

	buffer8_IR_Yaw_left = IR_Yaw_left;
	buffer9_IR_Yaw_right = IR_Yaw_right;
	
	buffer10_IMU_Yaw_Low = IMU_Yaw_16;			// Skicka låga byten av IMU_Yaw_16
	buffer11_IMU_Yaw_High = (IMU_Yaw_16>>8);	// Skicka höga byten av IMU_Yaw_16
	
	buffer12_Pitch = IMU_Pitch;
	buffer13_Roll = IMU_Roll;
}

/*
* FUNKTION restrict_angle(float angle)
*
* BESKRIVNING
*
* Begränsar vinkeln angle så att overflow ej fås då den konverteras till int8_t
*
* INDATA
*
* angle:		Vinkeln som ska begränsas
*
* UTDATA
*
* float:		Begränsad vinkel
*
* PÅVERKAR
*
* -
*
*/

float restrict_angle(float angle)
{
	if(angle > 127)
	angle = 127;							// Sätt max-vinkel (så att ej får överslag då det görs om till 8 bitar (signed))
	
	if(angle < -128)
	angle = -128;							// Sätt min-vinkel (så att ej får överslag då det görs om till 8 bitar (signed))
	
	return angle;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
* FUNKTION NormalizeQuaternion(float *quat)
*
* BESKRIVNING
*
* Normaliserar kvaternionen quat
*
* INDATA
*
* quat:		Kvaternionen som ska normaliseras
*
* UTDATA
*
* -
*
* PÅVERKAR
*
* quat:		Den nu normaliserade kvaternionen
*
*/

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

/*
* FUNKTION run_self_test()
*
* BESKRIVNING
*
* Testar och kalibrerar gyrot och accelerometrarna.
*
* INDATA
*
* -
*
* UTDATA
*
* -
*
* PÅVERKAR
*
* Register i IMU:n
*
*/

void run_self_test()
{
	long gyro_cal[3], accel_cal[3];
	
	mpu_run_self_test(gyro_cal, accel_cal);

	for(uint8_t i = 0; i<3; i++)
	{
		gyro_cal[i] = (long)(gyro_cal[i] * 32.8f); //konvertera to +-1000dps
		accel_cal[i] *= 2048.f; //konvertera to +-16G
		accel_cal[i] = accel_cal[i] >> 16;
		gyro_cal[i] = (long)(gyro_cal[i] >> 16);
	}
	mpu_set_gyro_bias_reg(gyro_cal);
	mpu_set_accel_bias_6050_reg(accel_cal);
}

/*
* SLUT PÅ FILEN sensor.c
*/

////////////////////////////////////////////////////////

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
