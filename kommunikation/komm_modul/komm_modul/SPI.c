/*
 * SPI.c
 *
 * Created: 2016-04-06 16:33:37
 *  Author: Joakim
 */ 

#include "SPI.h"

void SPI_init_master()
{
	//MOSI och SCK �r utg�ngar
	DDRB = 0b10100000;	
	
	//SPI Enable
	//SPI Master
	//SPI clock f/16
	//SPI interrupt enable
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0)|(1<<SPIE);
}

void SPI_init_slave()
{
	//MISO �r utg�ng
	DDRB = 0b01000000;
	
	//SPI Enable
	//SPI interrupt enable
	SPCR = (1<<SPE)| (1<<SPIE)
}

void SPI_transmit_master(unsigned char data)
{
	SPDR = data;
}

//SPI Serial transfer complete avbrottsvektor
ISR(SPI_STC_vect)
{
	
}