/*
* SPI.c
*
* Created: 2016-04-06 16:33:37
*  Author: Joakim
*/

#include "SPI_comm.h"
#include "USART.h"
#include "LCD_controller.h"

volatile uint8_t SPIcounter;

void SPI_init_slave()
{
	//MISO är utgång
	DDRB |= 0b01000000;
	
	//SPI Enable
	//SPI interrupt enable
	SPCR = (1<<SPE)| (1<<SPIE);
	
	SPIcounter = 0;
}

//SPI Serial transfer complete avbrottsvektor
ISR(SPI_STC_vect)
{
	//Om det har gått ett tag sedan senaste avbrottet nollställer vi räknaren.
	if(spiOverflow >= 2)
	{
		SPIcounter = 0;
	}
	spiOverflow = 0;
	
	//Läs in data
	toBluetooth[SPIcounter] = SPDR;
	
	//Fyll på med ny data
	if(SPIcounter < 15){
		++SPIcounter;
		SPDR = toSPI[SPIcounter];
	}
	else{
		//Om det var sista byten i överföringen vi fick
		SPIcounter = 0;
		SPDR = toSPI[0];
		USART_Transmit_Array(toBluetooth, 16);
		
	}
}
