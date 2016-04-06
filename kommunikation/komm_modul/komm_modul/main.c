/*
 * komm_modul.c
 *
 * Created: 2016-04-04 15:31:53
 * Author : Joakim
 */ 

#define F_CPU 14745600UL

#include <avr/io.h>
#include <avr/interrupt.h>

#include "USART.h"

void initLCD()
{
	
}

void initSPI()
{
	
}

//Mainfunktion för komm-cpu
int main(void)
{
	cli();
	//Initiera LCD
	initLCD();
	
	//Initiera buss	
	initSPI();
	
	//Initiera BT
	initUSART();
	
	sei();
	
	//unsigned char packet[16] = { 66,111,111,116,105,110,103,46,46,46,32,32,32,32,32,32};
	
	//USART_Transmit_Array(packet, 16);
  
   while(1);
}