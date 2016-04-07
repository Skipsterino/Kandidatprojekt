/*
 * LCD.c
 *
 * Created: 2016-04-07 10:43:31
 *  Author: Joakim
 */ 

#include "LCD.h"

#define RS 0
#define RW 1
#define E  2

void LCD_clear()
{
	LCD_send_command(0x01);
}

void LCD_putc(char c)
{
	while(LCD_busy())
	{
		_delay_ms(1);
	}
	PORTB |= (1<<RS);
	PORTA = c;
	PORTB |= (1<<E);
	_delay_us(50);
	PORTB &= ~(1<<E);
}

void LCD_print_string(char line1[16], char line2[16])
{
	LCD_clear();
	
	for(int i = 0; i < 16; ++i)
	{
		if(line1[i] == '\0')
		{
			break;
		}
		LCD_putc(line1[i]);
	}
	
	LCD_send_command(0b11000000);
	
	for(int i = 0; i < 16; ++i)
	{
		if(line2[i] == '\0')
		{
			break;
		}
		LCD_putc(line2[i]);
	}
}

int LCD_busy()
{
	DDRA = 0;
	PORTB |= (1<<RW);
	PORTB &= ~(1<<RS);
	PORTB |= (1<<E);
	
	_delay_us(2);
	
	char data = PINA;
	
	PORTB &= ~(1<<E);
	PORTB &= ~(1<<RW);
	DDRA = 0xFF;
	return (data & (1<<7));
}

void LCD_send_command(unsigned char command)
{
	while(LCD_busy())
	{
		_delay_ms(1);
	}
	
	PORTB &= ~(1<<RS);
	PORTA = command;
	PORTB |= (1<<E);
	
	if(command == 0b01 || command == 0b10)
	{
		_delay_us(1500);
	}
	else
	{
		_delay_us(50);
	}
	
	PORTB &= ~(1<<E);
}

void LCD_init()
{
	//Sätt alla a-pinnar till utgångar
	DDRA = 0xFF;
	
	//Sätt E, R/W, RS till utgångar
	DDRB |= 0x07;
	
	//Vänta ett tag
	_delay_ms(31);
	
	LCD_send_command(0b00111000);
	
	_delay_us(40);
	
	LCD_send_command(0b00001000);
	
	_delay_us(40);
	
	LCD_clear();
	
	_delay_us(1600);
	
	LCD_send_command(0b00000110);
	
	LCD_send_command(0b00001100);
}