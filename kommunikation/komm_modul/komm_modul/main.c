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
#include "LCD_controller.h"

void initSPI()
{
	
}

//Mainfunktion för komm-cpu
int main(void)
{
	cli();
	//Initiera LCD
	LCD_init();
	
	LCD_controller_init();
	
	//Initiera buss	
	initSPI();
	
	//Initiera BT
	initUSART();
	
	sei();
	
	LCD_controller_put_line(0, "Hejsan");
	LCD_controller_put_line(1, "Hell0 World");
	LCD_controller_put_line(2, "HerpDerp");
	LCD_controller_put_line(3, "DerpHerp");
	LCD_controller_put_line(4, "Hopp");
	LCD_controller_put_line(5, "1234");
	LCD_controller_put_line(6, "6721");
	LCD_controller_put_line(7, "Hejsan");
	LCD_controller_put_line(8, "Hell0 World");
	LCD_controller_put_line(9, "HerpDerp");
	LCD_controller_put_line(10, "Hejsan");
	LCD_controller_put_line(11, "Hell0 World");
	LCD_controller_put_line(12, "HerpDerp");
	LCD_controller_put_line(13, "Hejsan");
	LCD_controller_put_line(14, "Hell0 World");
	LCD_controller_put_line(15, "HerpDerp");
	
   while(1);
}