/*
 * I2C.c
 *
 * Created: 4/8/2016 9:47:46 AM
 *  Author: joneh084
 */ 

#ifndef _I2C_H_
#define _I2C_H_

#include "I2C.h"
#include <util/atomic.h>
#include <avr/io.h>
#include <avr/interrupt.h>

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

int i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data)
{
	I2C_start();
	
	TWDR = SLA_W;							// Ladda in IMU:ns adress + indikera att skrivning ska ske
	TWCR = (1<<TWINT) | (1<<TWEN);// |(1<<TWIE);// Skicka
	while (!(TWCR & (1<<TWINT)));				// Vänta på att det har skickats
	if ((TWSR & 0xF8) !=I2Cstatus_MT_SLA_ACK)	// Kolla så att status är rätt (att de som vi ville skulle hända faktiskt hände)
	error();
	
	TWDR = reg_addr;							// Ladda in adressen för IMU-registret vi ska skriva till
	TWCR = (1<<TWINT) | (1<<TWEN);				// Skicka
	while (!(TWCR & (1<<TWINT)));				// Vänta på att det har skickats
	if ((TWSR & 0xF8) !=I2Cstatus_MT_DATA_ACK)	// Kolla så att status är rätt
	error();
	
	for (uint8_t for_counter = 0; for_counter < length; for_counter++)
	{
		TWDR = data[for_counter];					// Ta IMU ur vila och välj intern 8 MHz som klocka
		TWCR = (1<<TWINT) | (1<<TWEN);				// Skicka
		while (!(TWCR & (1<<TWINT)));				// Vänta på att det har skickats
		if ((TWSR & 0xF8) !=I2Cstatus_MT_DATA_ACK)	// Kolla så att status är rätt
		error();
	}
	
	I2C_stop();
	
	return 0;
}

int i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data)
{
	I2C_start();
	
	TWDR = SLA_W;								// Ladda in IMU:ns adress + indikera att skrivning ska ske
	TWCR = (1<<TWINT) | (1<<TWEN);// |(1<<TWIE);// Skicka
	while (!(TWCR & (1<<TWINT)));				// Vänta på att det har skickats
	if ((TWSR & 0xF8) !=I2Cstatus_MT_SLA_ACK)	// Kolla så att status är rätt (att de som vi ville skulle hända faktiskt hände)
	error();
		
	TWDR = reg_addr;							// Ladda in adressen för IMU-registret vi ska läsa ifrån
	TWCR = (1<<TWINT) | (1<<TWEN);				// Skicka
	while (!(TWCR & (1<<TWINT)));				// Vänta på att det har skickats
	if ((TWSR & 0xF8) !=I2Cstatus_MT_DATA_ACK)	// Kolla så att status är rätt
	error();
		
	I2C_start();
	
	TWDR = SLA_R;								// Ladda in IMU:ns adress + indikera att läsning ska ske
	TWCR = (1<<TWINT) | (1<<TWEN);// |(1<<TWIE);// Skicka
	while (!(TWCR & (1<<TWINT)));				// Vänta på att det har skickats
	if ((TWSR & 0xF8) !=I2Cstatus_MR_SLA_ACK)	// Kolla så att status är rätt (att de som vi ville skulle hända faktiskt hände)
	error();
	
	for (uint8_t for_counter = 0; for_counter < length; for_counter++)
	{
		TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);		// Ta emot data och skicka ACK (mottagningen klar)
		while (!(TWCR & (1<<TWINT)));					// Vänta på att det har skickats
		data[for_counter] = (TWDR);
		if ((TWSR & 0xF8) !=I2Cstatus_MR_DATA_ACK)		// Kolla så att status är rätt (att de som vi ville skulle hända faktiskt hände)
		error();
	}
	
	I2C_stop();
	
	return 0;
}

int error()			// XXXXX SKA ÄNDRAS!
{  
	return -1;
}


unsigned long millis()
{
	unsigned long m;
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		m = timer0_millis;
	}
	return m;
}

SIGNAL(TIMER0_OVF_vect)
{
	unsigned long m = timer0_millis;
	unsigned char f = timer0_fract;
	
	m += MILLIS_INC;
	f += FRACT_INC;
	if (f >= FRACT_MAX)
	{
		f -= FRACT_MAX;
		m += 1;
	}
	
	timer0_fract = f;
	timer0_millis = m;
	timer0_overflow_count++;
}

#endif  // _I2C_H_