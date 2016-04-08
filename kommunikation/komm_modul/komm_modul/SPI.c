/*
 * SPI.c
 *
 * Created: 2016-04-06 16:33:37
 *  Author: Joakim
 */ 

#include "SPI.h"
#include "USART.h"

uint8_t SPIcounter;

///////////////////////////////////////////////////////////*
unsigned char fromSen[16];
unsigned char fromKom[16];
uint8_t SPIcounter;
uint8_t toSen;
uint8_t toKom;
// timer0 interruppt funktion
ISR(TIMER0_OVF_vect)
{
	SPI_sen_transmit_master();
}

void SPI_sen_transmit_master()
{
	SPIcounter = 0;
	//Sätter SS för styr->sen
	toSen = 1;
	toKom = 0;
	Set_SS_sen_kom(toSen, toKom);
	// skickar noll till Sen
	SPDR = 0;
}

ISR(SPI_STC_vect)
{
	if(toSen == 1){
		fromSen[SPIcounter] = SPDR
		
		if(SPIcounter < 15){
			SPDR = 0;
			++SPIcounter;
		}
		else{
			SPIcounter = 0;
			//sätter SS till styr->kom
			toSen = 0;
			toKom = 1;
			Set_SS_sen_kom(toSen, toKom);
			//Startar styr->kom buss
			SPDR = fromSen[SPIcounter];
		}
	}
	else if(toKom == 1){
		fromKom[SPIcounter] = SPDR
		
		if(SPIcounter < 15){
			SPDR = fromSen[SPIcounter + 1];
			++SPIcounter;
		}
		else{
			SPIcounter = 0;
			toSen = 0;
			toKom = 0;
			Set_SS_sen_kom(toSen, toKom);
		}
	}
}

void Set_SS_sen_kom(uint8_t toSen, uint8_t toKom)
{
	if(toSen == 1){
		PORTB &= ~(1 << 3);
		PORTB |= (1 << 4);
	}
	else if(toKom == 1){
		PORTB |= (1 << 3);
		PORTB &= ~(1 << 4);
	}
	else{
		PORTB |= (1 << 3);
		PORTB |= (1 << 4);
	}
}

void SPI_init_master()
{
	//MOSI, SCK , SS0 och SS1 är utgångar
	DDRB |= 0b10111000;	
	
	//SPI Enable
	//SPI Master
	//SPI clock f/16
	//SPI interrupt enable
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0)|(1<<SPIE);
	
	// Enable timer0 interruppt
	TIMSK = (1<<TOIE0);
	// set timer0 counter initial value to 0
	TCNT0=0x00;
	// start timer0 with /1024 prescaler
	TCCR0 = (1<<CS02) | (1<<CS00);
	
	SPIcounter = 0;
	uint8_t toSen = 0;
	uint8_t toKom = 0;
	
	Set_SS_sen_kom(toSen, toKom);
	
	// Activale global interruppt
	sei();
}

*//////////////////////////////////////////////////////////

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
