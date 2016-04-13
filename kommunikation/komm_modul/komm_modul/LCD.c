/*
* LCD.c
*
* Created: 2016-04-07 10:43:31
*  Author: Joakim
*/

#include "LCD.h"


//Definitioner av pinnars position i PORTB-registret
#define RS 0
#define RW 1
#define E  2

//Definitioner av datariktningar
#define DATA_OUT 0xFF
#define DATA_IN 0x00
#define COMMAND_PINS 0x07

//Definitioner av adressen för första tecknet på varje rad
//i LCD:ns interna minne 
#define DDRAM_LINE_1 0x80
#define DDRAM_LINE_2 0xC0
#define DDRAM_LINE_3 0x90
#define DDRAM_LINE_4 0xD0

//Definitioner av möjliga kommandon 
#define COMMAND_CLEAR 0b00000001
#define COMMAND_8BIT_2LINE_8DOTS 0b00110000
#define COMMAND_8BIT_2LINE_11DOTS 0b00111000
#define COMMAND_DISPLAY_OFF 0b00001000
#define COMMAND_CURSOR_INC_NO_SHIFT 0b00000110 
#define COMMAND_DISPLAY_ON_CURSOR_MOVE 0b00001100

void LCD_clear()
{
	LCD_send_command(COMMAND_CLEAR);
}

void LCD_put_num_u(unsigned int num)
{
	char digits[16];
	sprintf(digits,"%d", num);
	
	for(int i = 0; i < 16; ++i)
	{
		if(digits[i] == '\0')
		{
			break;
		}
		LCD_putc(digits[i]);
	}
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

void LCD_print_string(char line1[16], char line2[16], char line3[], char line4[])
{
	//Clear display
	LCD_clear();
	
	for(int i = 0; i < 16; ++i)
	{
		if(line1[i] == '\0')
		{
			break;
		}
		LCD_putc(line1[i]);
	}
	
	LCD_send_command(DDRAM_LINE_2);
	
	for(int i = 0; i < 16; ++i)
	{
		if(line2[i] == '\0')
		{
			break;
		}
		LCD_putc(line2[i]);
	}
	
	LCD_send_command(DDRAM_LINE_3);
	
	for(int i = 0; i < 16; ++i)
	{
		if(line3[i] == '\0')
		{
			break;
		}
		LCD_putc(line3[i]);
	}
	
	LCD_send_command(DDRAM_LINE_4);
	
	for(int i = 0; i < 16; ++i)
	{
		if(line4[i] == '\0')
		{
			break;
		}
		LCD_putc(line4[i]);
	}
}

int LCD_busy()
{
	//Byt riktning på dataregister
	DDRA = DATA_IN;
	//Sätt RW till 1 för att läsa data
	PORTB |= (1<<RW);
	PORTB &= ~(1<<RS);
	PORTB |= (1<<E);
	
	_delay_us(2);
	
	//Data hämtas från dataportar
	char data = PINA;
	
	PORTB &= ~(1<<E);
	PORTB &= ~(1<<RW);
	//Återställ riktning
	DDRA = DATA_OUT;
	
	//Skicka tillbaka BF-flaggan(bit 7)
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
	
	//Clear och return home tar 1.53 ms
	//Alla andra tar 39 µs
	if(command == 0x01 || command == 0x02)
	{
		_delay_us(1600);
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
	DDRA = DATA_OUT;
	
	//Sätt E, R/W, RS till utgångar
	DDRB |= COMMAND_PINS;
	
	//Vänta ett tag
	_delay_ms(40);
	
	LCD_send_command(COMMAND_8BIT_2LINE_11DOTS);
	
	_delay_us(40);
	
	LCD_send_command(COMMAND_DISPLAY_OFF);
	
	_delay_us(40);
	
	LCD_clear();
	
	_delay_us(1600);
	
	LCD_send_command(COMMAND_CURSOR_INC_NO_SHIFT);
	
	_delay_us(40);
	
	LCD_send_command(COMMAND_DISPLAY_ON_CURSOR_MOVE);
}