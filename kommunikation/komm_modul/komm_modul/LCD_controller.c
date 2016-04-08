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
	if(overflow_counter > 100)
	{
		TCNT0 = 0;
		if(currentLine == 15)
		{
			LCD_print_string(lines[15], lines[0]);
			currentLine = 0;
		}
		else
		{
			LCD_print_string(lines[currentLine], lines[currentLine+1]);
		}
		++currentLine;
		overflow_counter = 0;
	}
	overflow_counter++;
}