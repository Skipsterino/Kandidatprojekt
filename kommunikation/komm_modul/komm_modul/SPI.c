/*
 * SPI.c
 *
 * Created: 2016-04-06 16:33:37
 *  Author: Joakim
 */ 

#include "SPI.h"
#include "USART.h"

uint8_t SPIcounter;

void SPI_init_master()
{
	//MOSI och SCK är utgångar
	DDRB |= 0b10100000;	
	
	//SPI Enable
	//SPI Master
	//SPI clock f/16
	//SPI interrupt enable
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0)|(1<<SPIE);
	
	SPIcounter = 0;
}

void SPI_init_slave()
{
	//MISO är utgång
	DDRB |= 0b01000000;
	
	//SPI Enable
	//SPI interrupt enable
	SPCR = (1<<SPE)| (1<<SPIE);
	
	SPIcounter = 0;
}

void SPI_transmit_master(unsigned char data)
{
	SPIcounter = 0;
	SPDR = data;
}

//SPI Serial transfer complete avbrottsvektor
ISR(SPI_STC_vect)
{
	toBluetooth[SPIcounter] = SPDR;
	
	if(SPIcounter < 15){
		++SPIcounter;
		SPDR = toSPI[SPIcounter];		
	}
	else{
		SPIcounter = 0;
		USART_Transmit_Array(toBluetooth, 16);
	}
}