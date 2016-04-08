/*
 * Styr.c
 *
 * Created: 4/5/2016 8:44:50 AM
 *  Author: erilj291
 */ 
#ifndef F_CPU
#define F_CPU 16000000UL		// 16 MHz
#endif
		
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <math.h>
#include <util/delay.h>

#include "init.h"
#include "structs.h"
#include "SPI.h"
#include "servo_UART.h"
#include "invers_kinematik.h"
	
int main(void)
{
	Init();
	USART_Init(BAUD_PRESCALER);
	SPI_init_master();
	//KÖR FÖLJANDE FUNKTIONER NÄR SERVONA BEHÖVER KALIBRERAS PÅ NÅGOT SÄTT
	//Send_Servo_Delaytime();
	Send_Servo_LED();
	//Send_Servo_Angle_Limit();
	sei(); //Aktivera avbrott öht (MSB=1 i SREG)
	
	////Send_Servo_Position({0x07,0x66,0x01});
	//_delay_ms(1000);
	////Send_Servo_Position({0x07,0xFF,0x01});
	//_delay_ms(1000);
	////Send_Servo_Position({0x07,0x67,0x01});
	//_delay_ms(1000);
	////Send_Servo_Position({0x07,0xFF,0x01});
	//_delay_ms(1000);
	//Send_Servo_Position({0x07,0x65,0x01});

	//unsigned char servos[9] = {12,10,8,18,16,14, 6,4,2};
	//for (uint8_t i = 0; i < 9; i++)
	//{		
		//unsigned char ID_and_position[3] = {servos[i], 0x90, 0x01};
		//Send_Servo_Position(ID_and_position);
		//_delay_ms(1000);
	//}
	
	while(1)
	{
    }
}