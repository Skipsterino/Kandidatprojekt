/*
 * init.c
 *
 * Created: 4/8/2016 3:12:02 PM
 *  Author: erilj291
 */ 

#include "init.h"

void Init(void)
{
	DDRD = 1<<DDD2;
	DDRC = 0; //JTAG, alla väljs till ingångar
	DDRB = (1<<DDB3) | (1<<DDB4) | (1<<DDB5) | (0<<DDB6) | (1<<DDB7); //SPI, allt ut förutom PB6
	
	PORTD |= 1<<PORTD2; //Välj riktning "till servon" i tri-state
	
	UART_Init();
	SPI_init_master();
}

void UART_Init(void)
{
	//Se till att baud-inställningarna blir rätt
	UCSR0A &= ~(1<<U2X0);
	//Sätt baud-prescaler = 0 så att baud-rate blir 1 MBPS
	UBRR0H = 0;
	UBRR0L = 0;
	//Ställ in processorn som sändare och mottagare på bussen
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	//8 data, 1 stopbit
	UCSR0C = (0<<USBS0)|(3<<UCSZ00);
}