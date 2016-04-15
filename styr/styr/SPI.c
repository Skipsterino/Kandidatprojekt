/*
 * SPI.c
 *
 * Created: 4/8/2016 2:53:09 PM
 *  Author: chrma018
 */ 

#include "SPI.h"

uint8_t SPIcounter;
uint8_t toSen;
uint8_t toKom;
uint8_t overflow;
// timer0 interruppt funktion
ISR(TIMER0_OVF_vect)
{
	++overflow;
	
	if(overflow >= 5){
		SPI_sen_transmit_master();
		overflow = 0;
	}
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
		fromSen[SPIcounter] = SPDR;
		
		if(SPIcounter < 15){
			++SPIcounter;
			toSen = 0;
			toKom = 0;
			Set_SS_sen_kom(toSen, toKom);
			toSen = 1;
			toKom = 0;
			Set_SS_sen_kom(toSen, toKom);
			SPDR = 0;
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
		fromKom[SPIcounter] = SPDR;
		
		if(SPIcounter < 15){
			toSen = 0;
			toKom = 0;
			Set_SS_sen_kom(toSen, toKom);
			toSen = 0;
			toKom = 1;
			Set_SS_sen_kom(toSen, toKom);
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
	/*
	if(1){
		fromKom[SPIcounter] = SPDR;
		
		if(SPIcounter < 15){
			toSen = 0;
			toKom = 0;
			Set_SS_sen_kom(toSen, toKom);
			toSen = 1;
			toKom = 0;
			Set_SS_sen_kom(toSen, toKom);
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
	*/
}
void Set_SS_sen_kom(uint8_t toSen, uint8_t toKom)
{
	if(toKom == 1){
		PORTB &= ~(1 << SS_sen);
		PORTB |= (1 << SS_kom);
	}
	else if(toSen == 1){
		PORTB |= (1 << SS_sen);
		PORTB &= ~(1 << SS_kom);
	}
	else{
		PORTB |= (1 << SS_sen);
		PORTB |= (1 << SS_kom);
	}
}
void SPI_init_master()
{
	fromSen[0] = 1;
	fromSen[1] = 2;
	fromSen[2] = 3;
	fromSen[3] = 4;
	fromSen[4] = 5;
	fromSen[5] = 6;
	fromSen[6] = 7;
	fromSen[7] = 8;
	fromSen[8] = 9;
	fromSen[9] = 10;
	fromSen[10] = 11;
	fromSen[11] = 12;
	fromSen[12] = 13;
	fromSen[13] = 14;
	fromSen[14] = 15;
	fromSen[15] = 16;
	
	//MOSI, SCK , SS0 och SS1 är utgångar
	DDRB |= 0b10111000;	
	overflow = 0;
	//SPI Enable
	//SPI Master
	//SPI clock f/16
	//SPI interrupt enable
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0)|(1<<SPIE);
	
	// Enable timer0 interruppt
	TIMSK0 = (1<<TOIE0);
	// set timer0 counter initial value to 0
	TCNT0=0x00;
	// start timer0 with /1024 prescaler
	TCCR0B = (1<<CS02) | (1<<CS00);
	
	SPIcounter = 0;
	uint8_t toSen = 0;
	uint8_t toKom = 0;
	
	Set_SS_sen_kom(toSen, toKom);
}