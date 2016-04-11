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

//Mainfunktion för komm-cpu
int main(void)
{
	cli();
	//Initiera LCD
	LCD_init();
	
	LCD_controller_init();
	
	//Initiera buss	
	SPI_init_slave();
	
	//Initiera BT
	initUSART();
	
	sei();
	
	LCD_controller_put_line(0, "BT status");
	LCD_controller_put_line(1, "Disconnected");
	LCD_controller_put_line(2, "Control Mode");
	LCD_controller_put_line(3, "Manual");
	LCD_controller_put_line(4, "Sensor1");
	LCD_controller_put_line(5, "14cm");
	LCD_controller_put_line(6, "Sensor2");
	LCD_controller_put_line(7, "12cm");
	LCD_controller_put_line(8, "Angle error");
	LCD_controller_put_line(9, "4 deg");
	LCD_controller_put_line(10, "Mood");
	LCD_controller_put_line(11, "Happy");
	LCD_controller_put_line(12, "Decision");
	LCD_controller_put_line(13, "Go forward");
	LCD_controller_put_line(14, "Bytes sent");
	LCD_controller_put_line(15, "812");
	
	char array[16] = {111,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
		
	BTtimeout = 4;
	while(1)
    {
	   if(BTtimeout > 3)
	   {
		   LCD_controller_put_line(1,"Disconnected");
	   }
	    LCD_print_string(lines[currentLine], lines[currentLine+1]);
	    _delay_ms(500);
	    USART_Transmit_Array(array, 16);
		BTtimeout++;
	}
}