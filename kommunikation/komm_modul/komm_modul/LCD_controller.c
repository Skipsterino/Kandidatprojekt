/**
* File: LCD_controller.c
* Version: 1.0
* Last edited: 19 Maj 2016
*/

#include "LCD_controller.h"'

void LCD_controller_init()
{
	//Clear line buffer
	for(int i = 0; i < LCD_NUMBER_OF_LINES*CONTROLLER_NUMBER_OF_PAGES; ++i)
	{
		for(int j = 0; j < LCD_LINE_WIDTH; ++j)
		{
			lines[i][j] = '\0';
		}
	}
	
	currentLine = 0;
	overflow_counter = 0;
	spiOverflow = 0;
	
	//Init TIMER0 med prescaler 1024
	TCCR0B |= (1<<CS02) | (1<<CS00);
	
	//Nollställ räknaren
	TCNT0 = 0;
	
	//Aktivera avbrott vid overflow
	TIMSK0 |= (1 << TOIE0);
}

void LCD_controller_put_line(uint8_t line, char string[16])
{
	if(line >= LCD_NUMBER_OF_LINES*CONTROLLER_NUMBER_OF_PAGES)
	{
		//Error line utanför intervallet
		return;
	}
	for(int i = 0; i < LCD_LINE_WIDTH; ++i)
	{
		lines[line][i] = string[i];
	}
}

ISR(TIMER0_OVF_vect)
{
	++spiOverflow;	
	if(++overflow_counter > 150)
	{
		currentLine = (currentLine + LCD_NUMBER_OF_LINES) % 4/*(LCD_NUMBER_OF_LINES*CONTROLLER_NUMBER_OF_PAGES)*/;
		overflow_counter = 0;
	}
}
