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
	LCD_controller_put_line(4, "kd");
	LCD_controller_put_line(5, "40");
	LCD_controller_put_line(6, "kp");
	LCD_controller_put_line(7, "68");
	LCD_controller_put_line(8, "Angle error");
	LCD_controller_put_line(9, "4 deg");
	LCD_controller_put_line(10, "Mood");
	LCD_controller_put_line(11, "Happy");
	LCD_controller_put_line(12, "Decision");
	LCD_controller_put_line(13, "Go forward");
	LCD_controller_put_line(14, "Bytes sent");
	LCD_controller_put_line(15, "812");
	
	BTtimeout = 4;
	while(1)
	{
		/*
		if(BTtimeout > 3)
		{
			LCD_controller_put_line(1,"Disconnected");
		}
		else
		{
			LCD_controller_put_line(1,"Connected");	
		}
		*/
		char digits[16];
		sprintf(digits, "%i", (int8_t)toBluetooth[8]);
		LCD_controller_put_line(1, digits);
		sprintf(digits, "%i", (int8_t)toBluetooth[9]);
		LCD_controller_put_line(2, digits);
		sprintf(digits, "%i", (int8_t)toBluetooth[8] + (int8_t)toBluetooth[9]);
		LCD_controller_put_line(3, digits);
		
		
		LCD_print_string(lines[currentLine], lines[currentLine+1], lines[currentLine + 2], lines[currentLine + 3]);
		
		_delay_ms(100);
		
		BTtimeout++;
	}
}