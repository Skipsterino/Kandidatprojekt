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
#include "SPI_comm.h"
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
	
	//Sätt in tomma rader
	LCD_controller_put_line(0, "");
	LCD_controller_put_line(1, "");
	LCD_controller_put_line(2, "");
	LCD_controller_put_line(3, "");
	
	uint8_t lastStates[5];
	memset(lastStates,0,sizeof(lastStates));
	while(1)
	{
		char digits[16];
		//Skriv ut senaste states
		
		uint8_t currState = toBluetooth[14];
		uint8_t changedState = 0;
		
		if(currState != lastStates[0])
		{
			changedState = 1;
			for(int i = 3; i >= 0; --i)
			{
				lastStates[i+1] = lastStates[i];
			}
			lastStates[0] = currState;
			if(lastStates[4] != 0)
			{
				sprintf(digits,"%i,%i,%i,%i,%i", lastStates[0], lastStates[1], lastStates[2], lastStates[3], lastStates[4]);
			}
			else if(lastStates[3] != 0)
			{
				sprintf(digits,"%i,%i,%i,%i", lastStates[0], lastStates[1], lastStates[2], lastStates[3]);
			}
			else if(lastStates[2] != 0)
			{
				sprintf(digits,"%i,%i,%i", lastStates[0], lastStates[1], lastStates[2]);
			}
			else if(lastStates[1] != 0)
			{
				sprintf(digits,"%i,%i", lastStates[0], lastStates[1]);
			}
			else
			{
				sprintf(digits,"%i", lastStates[0]);
			}
			
			LCD_controller_put_line(0, digits);
		}
		
		//Skriv ut Kp och Kd
		sprintf(digits, "Kp:%i, Kd:%i", toSPI[5], toSPI[6]);
		LCD_controller_put_line(1, digits);
		
		//Skriv ut styrläge (går att göra snyggare)
		if(toSPI[0] & 0b00001000)
		{
			
			if(toSPI[4] == 0b00001111)
			{
				LCD_controller_put_line(2,"MODE=MANUAL");
			}
			else if(toSPI[4] == 0b00111100)
			{
				LCD_controller_put_line(2,"MODE=AUTO");
			}
			else if(toSPI[4] == 0b11110000)
			{
				LCD_controller_put_line(2,"MODE=RACE");
			}
			else
			{
				LCD_controller_put_line(2,"MODE=UNDEF");
			}
		}
		if(changedState)
		{
			LCD_controller_put_line(2,"MODE=AUTO");
		}
		
		//OBS tar lång tid
		LCD_print_string(lines[currentLine], lines[currentLine+1], lines[currentLine + 2], lines[currentLine + 3]);
		
		//Tar nog längre tid
		_delay_ms(100);
	}
}