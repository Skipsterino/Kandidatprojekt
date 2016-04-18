/*
* SPI.c
*
* Created: 2016-04-06 16:33:37
*  Author: Joakim
*/

#include "SPI.h"
#include "USART.h"
#include "LCD_controller.h"

uint8_t SPIcounter;

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
	if(spiOverflow >= 2)
	{
		SPIcounter = 0;
	}
	spiOverflow = 0;
	
	toBluetooth[SPIcounter] = SPDR;
	
	if(SPIcounter < 15){
		++SPIcounter;
		SPDR = toSPI[SPIcounter];
	}
	else{
		SPIcounter = 0;
		SPDR = toSPI[0];
		USART_Transmit_Array(toBluetooth, 16);
		
	}
}
