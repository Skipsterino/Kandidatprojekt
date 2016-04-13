/*
* LCD.h
*
* Created: 2016-04-07 10:40:57
*  Author: Joakim
*/



#ifndef LCD_H_
#define LCD_H_

#ifndef F_CPU
#define F_CPU 14745600UL
#endif

#include <avr/io.h>
#include <util/delay.h>

void LCD_clear();

void LCD_put_num_u(unsigned int num);

void LCD_putc(char c);

void LCD_print_string(char line1[16], char line2[16], char line3[], char line4[]);

int LCD_busy();

void LCD_send_command(unsigned char command);

void LCD_init();

#endif /* LCD_H_ */