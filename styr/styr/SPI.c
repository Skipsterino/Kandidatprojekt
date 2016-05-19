/*
 * SPI.c
 *
 * Created: 4/8/2016 2:53:09 PM
 *  Author: chrma018
 */ 

#include "SPI.h"

volatile uint8_t SPIcounter;
volatile uint8_t toSen;
volatile uint8_t toKom;
volatile uint8_t overflow;

// Avbrottsrutin för timer 0
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

uint8_t calculateChecksum()
{
	uint8_t checkSum = 0;
	
	for(uint8_t i = 0; i < 15; ++i)
	{
		checkSum += fromKom[i];
	}
	
	return (checkSum == fromKom[15]);
}

ISR(SPI_STC_vect)
{
	
	if(toSen == 1){
		
		//Rensa bort nollor på plats 14 från sensor
		//så tillståndsdatan inte skrivs över.
		if(SPIcounter != 14)
		{
			fromSen[SPIcounter] = SPDR;
		}
		else
		{
			unsigned char dummy = SPDR;
		}
		
		
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
			
			if(calculateChecksum())
			{
				memcpy(lastValidPacket, fromKom, sizeof(fromKom));
			}
		}
	}
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
	/*Nollställer fromKom & fromSen (tar bort ev skräp på minnesplatserna) 
	  så inget konstigt händer innan första avbrottet kommit.*/
	memset(fromKom, 0, sizeof(fromKom)); 
	memset(fromSen, 0, sizeof(fromSen));
	memset(lastValidPacket, 0, sizeof(lastValidPacket));
	
	//MOSI, SCK , SS0 och SS1 är utgångar
	DDRB |= 0b10111000;	
	overflow = 0;
	//SPI Enable
	//SPI Master
	//SPI clock f/16
	//SPI interrupt enable
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0)|(1<<SPIE);
	
	// Aktivera avbrott för timer 0
	TIMSK0 = (1<<TOIE0);
	// Sätt timer 0-räknarens initialvärde till 0
	TCNT0=0x00;
	// Starta timer 0 med /1024 "prescaler"
	TCCR0B = (1<<CS02) | (1<<CS00);
	
	SPIcounter = 0;
	uint8_t toSen = 0;
	uint8_t toKom = 0;
	
	Set_SS_sen_kom(toSen, toKom);
}