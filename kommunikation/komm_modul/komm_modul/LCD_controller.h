/*
* LCD_controller.h
*
* Created: 2016-04-08 08:47:43
*  Author: Joakim
*/


#ifndef LCD_CONTROLLER_H_
#define LCD_CONTROLLER_H_

#include <avr/interrupt.h>
#include <avr/io.h>

#include "LCD.h"

#define CONTROLLER_NUMBER_OF_PAGES 4

char lines[CONTROLLER_NUMBER_OF_PAGES*LCD_NUMBER_OF_LINES][LCD_LINE_WIDTH];
uint8_t currentLine;
uint8_t overflow_counter;
uint8_t spiOverflow;

void LCD_controller_init();

void LCD_controller_put_line(uint8_t line, char string[LCD_LINE_WIDTH]);


#endif /* LCD_CONTROLLER_H_ */