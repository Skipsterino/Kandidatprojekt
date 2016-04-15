/*
 * testloop.c
 *
 * Created: 4/15/2016 4:17:57 PM
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
#include "gangstilar.h"

//TESTVERSION av ny main
int test_main(void)
{
	Init();
	
	//KÖR CONFIGURE-FUNKTIONERNA NÄR SERVONA BEHÖVER KALIBRERAS PÅ NÅGOT SÄTT
	//Configure_Servos_Delaytime();
	//Configure_Servos_LED();
	//Configure_Servos_No_Response();
	Configure_Servos_Angle_Limit();
	//
	
	Send_Inner_P1_Velocity(0x00F0); //DESSA SEX ANROP MÅSTE ALLTID KÖRAS EFTERSOM HASTIGHETEN LIGGER I RAM!!!
	Send_Middle_P1_Velocity(0x0100);//
	Send_Outer_P1_Velocity(0x0100);//
	Send_Inner_P2_Velocity(0x00F0);//
	Send_Middle_P2_Velocity(0x0100);//
	Send_Outer_P2_Velocity(0x0100);//
	
	sei(); //Aktivera avbrott öht (MSB=1 i SREG). Anropas EFTER all konfigurering klar!
	
	while(1)
	{
		unsigned char first_kom_byte = fromKom[0];
		
		if (first_kom_byte & 0b00000011) //Skickas vinkel & intensitet?
		{
			unsigned char angle = fromKom[1];
			unsigned char intensity = fromKom[2];
			Walk_Cycle(13, 2, 11, intensity, angle, 16);
		}
	
	}
}