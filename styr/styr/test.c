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

typedef enum  {
	
	AUTO,
	MANUAL,
	RACE
	
} CONTROL_MODE;

//TESTVERSION av ny main
int main(void)
{
	CONTROL_MODE cm = MANUAL; //Representerar aktuellt läge hos roboten
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
	
	memset(fromKom, 0, sizeof(fromKom)); //Nollställer fromKom & fromSen (tar bort ev skräp på minnesplatserna) så koden inte ballar ur innan första avbrottet kommit. Lägg ev in i Init!
	memset(fromSen, 0, sizeof(fromSen));
	
	sei(); //Aktivera avbrott öht (MSB=1 i SREG). Anropas EFTER all konfigurering klar!
	
	while(1)
	{
		switch(cm)
		{
			case MANUAL:
				unsigned char first_kom_byte = fromKom[0];
			
				if (first_kom_byte & 0b00000011) //Skickas vinkel & intensitet?
				{
					unsigned char intensity = fromKom[2]*6/100; //100 på kontroll -> 6 i speed
					char angle = fromKom[1]*0.57/128; //128 på kontroll -> 0.57 i vinkel
					Walk_Half_Cycle(intensity, angle,11,13);
				} 
				break;
			case AUTO:
				//Autonomt läge
				break;
			case RACE:
				//Vänta på knapptryck -> cm blir AUTO
				break;
			default:
				break;	
		}
		
	}
}