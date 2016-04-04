/*
 * komm_modul.c
 *
 * Created: 2016-04-04 15:31:53
 * Author : Joakim
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>


#define F_CPU 14745600UL
#define BAUD_RATE 115200

ISR() //Avbrott från buss
{
	
}

ISR() //Avbrott från dator
{
	
}

void initLCD()
{
	
}

void initSPI()
{
	
}

void initBluetooth()
{
	uint16_t ubrr_val = 7;
	
	//Sätt baud rate prescaler
	UBRR0L = ubrr_val;
	UBRR0H = (ubrr_val>>8);
	
	//Interrupt-saker 
	UCSR0B = 0b11011000;
	
	//Async
	//No parity
	//1 stop bit
	//8 char size
	UCSR0C = 0b00000110;
}

int main(void)
{
	//Initiera LCD
	initLCD();
	
	
	//Initiera buss	
	initSPI();
	
	//Initiera BT
	initBluetooth();
	
	sei();
	//Sleep
	
    /* Replace with your application code */
    while (1) 
    {
    }
}

