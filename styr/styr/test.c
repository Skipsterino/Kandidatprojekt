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
#include "reglering.h"
#include "state_machine.h"


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

int main(void)
{
	CONTROL_MODE cm = MANUAL; //Representerar aktuellt läge hos roboten
	ROBOT_STATE = CORRIDOR;
	
	//Defaultvärden
	angle = 0;
	intensity = 0;
	intensity_byte = 100;
	angle_byte = 100;
	height = 11;
	delta_h = 0.1;
	Kp = 0.001;
	Kd = 0.25;
	
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
	Walk_Half_Cycle(0, 0,height);
	
	while(1)
	{
		first_kom_byte = fromKom[0];
		
		if (first_kom_byte & 0b00001000) //Växla läge?
		{
			unsigned char change_mode = fromKom[4];
			
			if (change_mode == 0) //Byt till MANUAL?
			{
				cm = MANUAL;
			}
			
			else if (change_mode == 1) //Byt till AUTO?
			{
				cm = AUTO;
				ROBOT_STATE = CORRIDOR;
			}
			else if (change_mode == 2) //Byt till RACE?
			{
				cm = RACE;
			}
		}
		
		switch(cm)
		{
			case MANUAL: //Manuellt läge
			intensity_byte = 100;
			angle_byte = 100;
			
			if (first_kom_byte & 0b00000011) //Skickas vinkel & intensitet?
			{
				if((fromKom[1] < 20) || (fromKom[1] > 220)){
					angle_byte = 0;
				}
				else{
					angle_byte = fromKom[1] - 120;
				}
				
				if((fromKom[2] < 20) || (fromKom[2] > 220)){
					intensity_byte = 0;
				}
				else{
					intensity_byte = fromKom[2] - 120;
				}
				
				intensity = (float)(intensity_byte)*((float)6)/((float)100); //100 på kontroll -> 6 i speed
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
				else if (change_height == 2) //Höj?
				{
					height += delta_h;
				}
				Walk_Half_Cycle(0, 0,height); //Genomför höjdändringen
			}
			if (first_kom_byte & 0b00010000) //Nytt Kp?
			{
				Kp = ((float)fromKom[5])/100; //Kp skickas som 1000 ggr det önskade värdet!!!
			}
			if (first_kom_byte & 0b00100000) //Nytt Kd?
			{
				Kd = ((float)fromKom[6])/100; //Kd skickas som 1000 ggr det önskade värdet!!!
			}
			break;
			case AUTO: //Autonomt läge
			update_state();
			run_state(height);
			break;
			case RACE:
			//if (PIND3 == 0) //Har knapp tryckts ned? PIN ist. för PORT eftersom in-port ist. för ut-port???
			//{
			//cm = AUTO;
			//}
			break;
			default:
			break;
		}
	}
}