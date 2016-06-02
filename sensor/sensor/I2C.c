/**
* File: I2C.c
* Version: 1.0
* Last edited: 19 maj 2016
*/

#ifndef _I2C_H_
#define _I2C_H_

#include "I2C.h"

#include <avr/io.h>
#include <avr/interrupt.h>

void I2C_start()
{
	int8_t timer = 0;

	TWCR = 0;
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);	// Start

	// Vänta på att det har skickats
	while (!(TWCR & (1<<TWINT)))
	{
		_delay_ms(1);
		++timer;
		if (timer >= 20)
		{
			return -1;
		}
	}					
	timer = 0;

	if ((TWSR & 0xF8) != I2Cstatus_START)	// Kolla så status = Start
	error();
}

void I2C_stop()
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);		// STOP
}

int i2c_write(unsigned char slave_addr, unsigned char reg_addr, 
				unsigned char length, unsigned char const *data)
{
	I2C_start();
	int8_t timer = 0;

	// Ladda in IMU:ns adress + indikera att skrivning ska ske
	TWDR = SLA_W;	
	TWCR = (1<<TWINT) | (1<<TWEN);			// Skicka

	// Vänta på att det har skickats
	while (!(TWCR & (1<<TWINT)))
	{
		_delay_ms(1);
		++timer;
		if (timer >= 20)
		{
			return -1;
		}
	}					
	timer = 0;

	// Kolla så att status är rätt 
	// (att de som vi ville skulle hända faktiskt hände)
	if ((TWSR & 0xF8) !=I2Cstatus_MT_SLA_ACK)		
	error();

	// Ladda in adressen för IMU-registret vi ska skriva till
	TWDR = reg_addr;
	TWCR = (1<<TWINT) | (1<<TWEN);					// Skicka

	// Vänta på att det har skickats
	while (!(TWCR & (1<<TWINT)))
	{
		_delay_ms(1);
		++timer;
		if (timer >= 20)
		{
			return -1;
		}
	}					
	timer = 0;

	// Kolla så att status är rätt
	if ((TWSR & 0xF8) !=I2Cstatus_MT_DATA_ACK)		
	error();

	for (uint8_t for_counter = 0; for_counter < length; for_counter++)
	{
		TWDR = data[for_counter];			// Lägg in den byte som ska skickas
		TWCR = (1<<TWINT) | (1<<TWEN);		// Skicka byte:n

		// Vänta på att det har skickats
		while (!(TWCR & (1<<TWINT)))
		{
			_delay_ms(1);
			++timer;
			if (timer >= 20)
			{
				return -1;
			}
		}					
		timer = 0;

		// Kolla så att status är rätt
		if ((TWSR & 0xF8) !=I2Cstatus_MT_DATA_ACK)	
		error();
	}

	I2C_stop();			// Avsluta överföring
	return 0;
}

int i2c_read(unsigned char slave_addr, unsigned char reg_addr,
			 unsigned char length, unsigned char *data)
{
	int8_t timer = 0;

	I2C_start();

	// Ladda in IMU:ns adress + indikera att skrivning ska ske
	TWDR = SLA_W;									
	TWCR = (1<<TWINT) | (1<<TWEN);		// Skicka

	// Vänta på att det har skickats
	while (!(TWCR & (1<<TWINT)))
	{
		_delay_ms(1);
		++timer;
		if (timer >= 20)
		{
			return -1;
		}
	}					
	timer = 0;

	// Kolla så att status är rätt 
	// (att de som vi ville skulle hända faktiskt hände)
	if ((TWSR & 0xF8) !=I2Cstatus_MT_SLA_ACK)		
	error();

	// Ladda in adressen för IMU-registret vi ska läsa ifrån
	TWDR = reg_addr;								
	TWCR = (1<<TWINT) | (1<<TWEN);		// Skicka

	// Vänta på att det har skickats
	while (!(TWCR & (1<<TWINT)))
	{
		_delay_ms(1);
		++timer;
		if (timer >= 20)
		{
			return -1;
		}
	}					
	timer = 0;

	// Kolla så att status är rätt
	if ((TWSR & 0xF8) !=I2Cstatus_MT_DATA_ACK)		
	error();

	I2C_start();

	// Ladda in IMU:ns adress + indikera att läsning ska ske
	TWDR = SLA_R;									
	TWCR = (1<<TWINT) | (1<<TWEN);		// Skicka

	// Vänta på att det har skickats
	while (!(TWCR & (1<<TWINT)))
	{
		_delay_ms(1);
		++timer;
		if (timer >= 20)
		{
			return -1;
		}
	}					
	timer = 0;

	// Kolla så att status är rätt
	// (att de som vi ville skulle hända faktiskt hände)
	if ((TWSR & 0xF8) !=I2Cstatus_MR_SLA_ACK)		
	error();

	for (uint8_t for_counter = 0; for_counter < length; for_counter++)
	{
		if(for_counter == length -1)
		{
			// Ta emot data och skicka NACK
			// (NACK = mottagningen klar och vi ska ej ta emot mer)
			TWCR = (1<<TWINT) | (1<<TWEN);					

			while (!(TWCR & (1<<TWINT)))
			{
				_delay_ms(1);
				++timer;
				if (timer >= 20)
				{
					return -1;
				}
			}					// Vänta på att det har skickats
			timer = 0;

			data[for_counter] = (TWDR);		// Spara undan den mottagna datan

			// Kolla så att status är rätt
			// (att de som vi ville skulle hända faktiskt hände)
			if ((TWSR & 0xF8) !=I2Cstatus_MR_DATA_NACK)		
			error();
		}
		else
		{	
			// Ta emot data och skicka ACK
			// (ACK = mottagningen klar och vi ska fortsätta ta emot data)
			TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);		

			// Vänta på att det har skickats
			while (!(TWCR & (1<<TWINT)))
			{
				_delay_ms(1);
				++timer;
				if (timer >= 20)
				{
					return -1;
				}
			}					
			timer = 0;

			data[for_counter] = (TWDR);		// Spara undan den mottagna datan

			// Kolla så att status är rätt
			// (att de som vi ville skulle hända faktiskt hände)
			if ((TWSR & 0xF8) !=I2Cstatus_MR_DATA_ACK)		
			error();
		}
	}

	I2C_stop();		// Avsluta överföring
	return 0;
}

int error()			// XXXXX SKA ÄNDRAS!
{
	return -1;
}

#endif  // _I2C_H_
