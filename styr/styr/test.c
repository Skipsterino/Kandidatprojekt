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

float angle;
int intensity;
int8_t intensity_byte;
int8_t angle_byte;
float height;
float delta_h;
	
//TESTVERSION av ny main
int main_poop(void)
{
	CONTROL_MODE cm = MANUAL; //Representerar aktuellt läge hos roboten
	angle = 0;
	intensity = 0;
	intensity_byte = 100;
	angle_byte = 100;
	height = 11;
	delta_h = 0.01;
	
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
	
	unsigned char first_kom_byte;
	
	while(1)
	{
		switch(cm)
		{
			case MANUAL:
				first_kom_byte = fromKom[0];
				intensity_byte = 100;
				angle_byte = 100;
			
				if (first_kom_byte & 0b00000011) //Skickas vinkel & intensitet?
				{
					
					intensity_byte = fromKom[2] - 100;
					intensity = (float)(intensity_byte)*((float)6)/((float)100); //100 på kontroll -> 6 i speed
					angle_byte = 0;
					//memcpy(angle_byte, fromKom[1], sizeof(angle_byte));
					angle_byte = fromKom[1] - 100;
					
					angle = (float)(angle_byte)*((float)0.57)/((float)100); //128 på kontroll -> 0.57 i vinkel	
					
					Walk_Half_Cycle(intensity, angle,height);
				}
				if (first_kom_byte & 0b00000100) //Höj/sänk gångstil?
				{
					unsigned char change_height = fromKom[3];
					
					if (change_height == 1) //Sänk?
					{
						height -= delta_h;
					}
					else //Annars måste change_height == 2, dvs höj roboten (ty vet här att höjning/sänkning skickats)
					{
						height += delta_h;
					}
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