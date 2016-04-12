/*
* LCD_controller.c
*
* Created: 2016-04-08 08:49:38
*  Author: Joakim
*/

#include "LCD_controller.h"

void LCD_controller_init()
{
	//Clear lines
	for(int i = 0; i < 16; ++i)
	{
		for(int j = 0; j < 16; ++j)
		{
			lines[i][j] = '\0';
		}
	}
	currentLine = 0;
	overflow_counter = 0;
	spiOverflow = 0;
	
	TCCR0B |= (1<<CS02) | (1<<CS00);
	
	TCNT0 = 0;
	
	TIMSK0 |= (1 << TOIE0);
}

void LCD_controller_put_line(uint8_t line, char string[16])
{
	for(int i = 0; i < 16; ++i)
	{
		lines[line][i] = string[i];
	}
}

ISR(TIMER0_OVF_vect)
{
	spiOverflow++;	
	if(++overflow_counter > 100)
	{
		TCNT0 = 0;
		currentLine = (currentLine + 2) % 2;
		overflow_counter = 0;
	}
}