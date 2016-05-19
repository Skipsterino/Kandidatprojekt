/**
* File: LCD.c
* Version: 1.0
* Last edited: 19 Maj 2016
*/

#include "LCD.h"

//Definitioner av pinnars position i PORTB-registret
#define RS 0
#define RW 1
#define E  2

//Definitioner av datariktningar
#define DATA_OUT		0xFF
#define DATA_IN			0x00
#define COMMAND_PINS	0x07

//Definitioner av adressen för första tecknet på varje rad
//i LCD:ns interna minne 
#define DDRAM_LINE_1	0x80
#define DDRAM_LINE_2	0xC0
#define DDRAM_LINE_3	0x90
#define DDRAM_LINE_4	0xD0

//Definitioner av några möjliga kommandon 
#define COMMAND_CLEAR					0b00000001
#define COMMAND_8BIT_2LINE_8DOTS		0b00110000
#define COMMAND_8BIT_2LINE_11DOTS		0b00111000
#define COMMAND_DISPLAY_OFF				0b00001000
#define COMMAND_CURSOR_INC_NO_SHIFT		0b00000110 
#define COMMAND_DISPLAY_ON_CURSOR_MOVE	0b00001100

void LCD_clear()
{
	LCD_send_command(COMMAND_CLEAR);
}

void LCD_put_num_u(unsigned int num)
{
	char digits[LCD_LINE_WIDTH];
	sprintf(digits,"%d", num);
	
	for(int i = 0; i < LCD_LINE_WIDTH; ++i)
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
	//Vänta tills LCDn är redo
	while(LCD_busy())
	{
		_delay_ms(1);
	}
	//Sätt RS till 1 och skicka in data på port A
	PORTB |= (1<<RS);
	PORTA = c;
	
	//Signalera att kommandot är redo
	PORTB |= (1<<E);
	_delay_us(50);
	PORTB &= ~(1<<E);
}

void LCD_print_line(char line[LCD_LINE_WIDTH])
{
	int i = 0;
	//Loopa igenom alla tecken
	for(i; i < LCD_LINE_WIDTH; ++i)
	{
		//Om vi hittar radslut, avbryt
		if(line[i] == '\0')
		{
			break;
		}
		LCD_putc(line[i]);
	}
	//Fyll i resten med tomma tecken
	for(i; i < LCD_LINE_WIDTH; ++i)
	{
		LCD_putc(' ');
	}
}

void LCD_print_string(char line1[LCD_LINE_WIDTH], char line2[LCD_LINE_WIDTH], char line3[LCD_LINE_WIDTH], char line4[LCD_LINE_WIDTH])
{	
	LCD_send_command(DDRAM_LINE_1);
	
	LCD_print_line(line1);
	
	LCD_send_command(DDRAM_LINE_2);
	
	LCD_print_line(line2);
	
	LCD_send_command(DDRAM_LINE_3);
	
	LCD_print_line(line3);
	
	LCD_send_command(DDRAM_LINE_4);
	
	LCD_print_line(line4);
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
	//Vi väntar med lite extra marginal
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

	_delay_us(40);
	
	//8 bit dataportar
	//2 raders display (faktiskt 4 st)
	//11 dot karaktärer
	LCD_send_command(COMMAND_8BIT_2LINE_11DOTS);
	
	LCD_send_command(COMMAND_DISPLAY_OFF);
	
	LCD_clear();
	
	//Cursor stegas uppåt
	//Ingen inskiftning
	LCD_send_command(COMMAND_CURSOR_INC_NO_SHIFT);
	
	//Automatisk förflyttning av cursor vid skrivning
	LCD_send_command(COMMAND_DISPLAY_ON_CURSOR_MOVE);
}
