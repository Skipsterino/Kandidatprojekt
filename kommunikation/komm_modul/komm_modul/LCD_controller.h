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

char lines[16][16];
uint8_t currentLine;
uint8_t overflow_counter;

void LCD_controller_init();

void LCD_controller_put_line(uint8_t line, char string[16]);


#endif /* LCD_CONTROLLER_H_ */