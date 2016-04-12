/*
* I2C.c
*
* Created: 4/8/2016 9:47:46 AM
*  Author: joneh084
*/

#ifndef _I2C_H_
#define _I2C_H_

#include "I2C.h"

#include <avr/io.h>
#include <avr/interrupt.h>

void I2C_start()
{
	TWCR = 0;
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);	// Start

	while (!(TWCR & (1<<TWINT)));					// Vänta på att det har skickats
	if ((TWSR & 0xF8) != I2Cstatus_START)			// Kolla så status = Start
	error();
}

void I2C_stop()
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);		// STOP
}

int i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data)
{
	I2C_start();
	
	TWDR = SLA_W;									// Ladda in IMU:ns adress + indikera att skrivning ska ske
	TWCR = (1<<TWINT) | (1<<TWEN);					// Skicka
	while (!(TWCR & (1<<TWINT)));					// Vänta på att det har skickats
	if ((TWSR & 0xF8) !=I2Cstatus_MT_SLA_ACK)		// Kolla så att status är rätt (att de som vi ville skulle hända faktiskt hände)
	error();
	
	TWDR = reg_addr;								// Ladda in adressen för IMU-registret vi ska skriva till
	TWCR = (1<<TWINT) | (1<<TWEN);					// Skicka
	while (!(TWCR & (1<<TWINT)));					// Vänta på att det har skickats
	if ((TWSR & 0xF8) !=I2Cstatus_MT_DATA_ACK)		// Kolla så att status är rätt
	error();
	
	for (uint8_t for_counter = 0; for_counter < length; for_counter++)
	{
		TWDR = data[for_counter];					// Lägg in den byte som ska skickas
		TWCR = (1<<TWINT) | (1<<TWEN);				// Skicka byte:n
		while (!(TWCR & (1<<TWINT)));				// Vänta på att det har skickats
		if ((TWSR & 0xF8) !=I2Cstatus_MT_DATA_ACK)	// Kolla så att status är rätt
		error();
	}
	
	I2C_stop();										// Avsluta överföring
	return 0;
}

int i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data)
{
	I2C_start();
	
	TWDR = SLA_W;									// Ladda in IMU:ns adress + indikera att skrivning ska ske
	TWCR = (1<<TWINT) | (1<<TWEN);					// Skicka
	while (!(TWCR & (1<<TWINT)));					// Vänta på att det har skickats
	if ((TWSR & 0xF8) !=I2Cstatus_MT_SLA_ACK)		// Kolla så att status är rätt (att de som vi ville skulle hända faktiskt hände)
	error();
	
	TWDR = reg_addr;								// Ladda in adressen för IMU-registret vi ska läsa ifrån
	TWCR = (1<<TWINT) | (1<<TWEN);					// Skicka
	while (!(TWCR & (1<<TWINT)));					// Vänta på att det har skickats
	if ((TWSR & 0xF8) !=I2Cstatus_MT_DATA_ACK)		// Kolla så att status är rätt
	error();
	
	I2C_start();
	
	TWDR = SLA_R;									// Ladda in IMU:ns adress + indikera att läsning ska ske
	TWCR = (1<<TWINT) | (1<<TWEN);					// Skicka
	while (!(TWCR & (1<<TWINT)));					// Vänta på att det har skickats
	if ((TWSR & 0xF8) !=I2Cstatus_MR_SLA_ACK)		// Kolla så att status är rätt (att de som vi ville skulle hända faktiskt hände)
	error();
	
	for (uint8_t for_counter = 0; for_counter < length; for_counter++)
	{
		if(for_counter == length -1)
		{
			TWCR = (1<<TWINT) | (1<<TWEN);					// Ta emot data och skicka ACK (ACK = mottagningen klar och vi ska fortsätta ta emot data)
			while (!(TWCR & (1<<TWINT)));					// Vänta på att det har skickats
			data[for_counter] = (TWDR);						// Spara undan den mottagna datan
			if ((TWSR & 0xF8) !=I2Cstatus_MR_DATA_NACK)		// Kolla så att status är rätt (att de som vi ville skulle hända faktiskt hände)
			error();
		}
		else
		{
			TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);		// Ta emot data och skicka NACK (NACK = mottagningen klar och vi ska ej ta emot mer)
			while (!(TWCR & (1<<TWINT)));					// Vänta på att det har skickats
			data[for_counter] = (TWDR);						// Spara undan den mottagna datan
			if ((TWSR & 0xF8) !=I2Cstatus_MR_DATA_ACK)		// Kolla så att status är rätt (att de som vi ville skulle hända faktiskt hände)
			error();
		}
	}
	
	I2C_stop();												// Avsluta överföring
	return 0;
}

int error()			// XXXXX SKA ÄNDRAS!
{
	return -1;
}

#endif  // _I2C_H_
