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
#include "state_machine.h"



typedef enum  {
	AUTO,
	MANUAL,
	RACE
} CONTROL_MODE;

volatile unsigned char lastPacket[16];

float angle;
float speed;
int8_t intensity_byte;
int8_t angle_byte;
float height;
float delta_h;

int main(void)
{
	memset(lastPacket, 0, sizeof(lastPacket));
	
	CONTROL_MODE cm = MANUAL; //Representerar aktuellt läge hos roboten
	ROBOT_STATE = CORRIDOR;
	
	//Defaultvärden
	angle = 0;
	speed = 0;
	intensity_byte = 120;
	angle_byte = 120;
	height = 11;
	delta_h = 0.1;
	Kp = 1;
	Kd = 1;
	
	Init();
	
	//KÖR CONFIGURE-FUNKTIONERNA NÄR SERVONA BEHÖVER KALIBRERAS PÅ NÅGOT SÄTT
	//Configure_Servos_Delaytime();
	//Configure_Servos_LED();
	//Configure_Servos_No_Response();
	Configure_Servos_Angle_Limit();
	//
	
	Send_Inner_P1_Velocity(0x0010); //DESSA SEX ANROP MÅSTE ALLTID KÖRAS EFTERSOM HASTIGHETEN LIGGER I RAM!!!
	Send_Middle_P1_Velocity(0x0010);//
	Send_Outer_P1_Velocity(0x0010);//
	Send_Inner_P2_Velocity(0x0010);//
	Send_Middle_P2_Velocity(0x0010);//
	Send_Outer_P2_Velocity(0x0010);//
	
	sei(); //Aktivera avbrott öht (MSB=1 i SREG). Anropas EFTER all konfigurering klar!

	unsigned char first_kom_byte;
	Walk_Half_Cycle(0, 0,height);
	
	while(1)
	{
		memcpy(lastPacket, lastValidPacket, sizeof(lastPacket));
		
		first_kom_byte = lastPacket[0];
		
		if (first_kom_byte & 0b00001000) //Växla läge?
		{
			unsigned char change_mode = lastPacket[4];
			
			if (change_mode == 0b00001111) //Byt till MANUAL?
			{
				cm = MANUAL;
			}
			
			else if (change_mode == 0b00111100) //Byt till AUTO?
			{
				cm = AUTO;
				ROBOT_STATE = CORRIDOR;
			}
			else if (change_mode == 0b11110000) //Byt till RACE?
			{
				cm = RACE;
			}
		}
		
		switch(cm)
		{
			case MANUAL: //Manuellt läge
			intensity_byte = 120;
			angle_byte = 120;
			
			if (first_kom_byte & 0b00000011) //Skickas vinkel & intensitet?
			{
				if((lastPacket[1] < 20) || (lastPacket[1] > 220)){
					angle_byte = 0;
				}
				else{
					angle_byte = lastPacket[1] - 120;
				}
				
				if((lastPacket[2] < 20) || (lastPacket[2] > 220)){
					intensity_byte = 0;
				}
				else{
					intensity_byte = lastPacket[2] - 120;
				}
				
				speed = (float)(intensity_byte)*((float)6)/((float)100); //100 på kontroll -> 6 i speed
				angle = (float)(angle_byte)*((float)0.57)/((float)100); //128 på kontroll -> 0.57 i vinkel
				
				Walk_Half_Cycle(speed, angle,height);
			}
			if (first_kom_byte & 0b00000100) //Höj/sänk gångstil?
			{
				unsigned char change_height = lastPacket[3];
				
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
				Kp = ((float)lastPacket[5])/1000.f; //Kp skickas som 1000 ggr det önskade värdet!!!
			}
			if (first_kom_byte & 0b00100000) //Nytt Kd?
			{
				Kd = ((float)lastPacket[6])/1000.f; //Kd skickas som 1000 ggr det önskade värdet!!!
			}
			break;
			
			case AUTO: //Autonomt läge
			update_state();
			run_state(height);
			break;
			
			case RACE:
			if ((PIND & (1 << PIND3)) == 0) //Har knapp tryckts ned? PIN ist. för PORT eftersom in-port ist. för ut-port???
			{
				cm = AUTO;
			}
			break;
			default:
			break;
		}
	}
}