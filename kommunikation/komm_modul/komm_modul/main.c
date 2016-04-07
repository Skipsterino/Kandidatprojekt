/*
 * komm_modul.c
 *
 * Created: 2016-04-04 15:31:53
 * Author : Joakim
 */ 
#ifndef F_CPU
#define F_CPU 14745600UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>

#include "USART.h"
#include "SPI.h"
#include "LCD.h"

void initSPI()
{
	
}

//Mainfunktion för komm-cpu
int main(void)
{
	cli();
	//Initiera LCD
	LCD_init();
	
	//Initiera buss	
	initSPI();
	
	//Initiera BT
	initUSART();
	
	sei();
	
	const char* s1 = "Booting...";
	const char* s2 = "Hello World!";
	
	char packet1[16] = "HEJ123";
		
	char packet2[16];
	
	//strcpy(packet1, s1);
	strcpy(packet2, s2);
	
	LCD_print_string(packet1, "Kalle");
	
	//USART_Transmit_Array(packet, 16);
  
   while(1);
}