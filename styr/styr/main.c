/*
 *        File: main.c
 *     Version: 1.0
 * Last edited: 20/5/2016 
 *     Authors: erilj291, joabe408
 */ 


#ifndef F_CPU
#define F_CPU 16000000UL		// 16 MHz klockfrekvens
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

CONTROL_MODE cm;
float angle;
float speed;
int8_t intensity_byte;
int8_t angle_byte;
float height;
float delta_h;
int8_t dx;

volatile unsigned char lastPacket[16];

//Funktionsdeklarationer
void update_mode();
void update_speed_and_angle();
void update_height();


////////////////////////////////////////////////////////////////////////////////////////////
////									MAIN											////
////////////////////////////////////////////////////////////////////////////////////////////

int main(void)
{
	memset(lastPacket, 0, sizeof(lastPacket));
	
	//Initiera robotens styrläge
	cm = RACE;
	//Default-tillstånd hos roboten
	ROBOT_STATE = CORRIDOR;
	
	//Defaultvärden
	angle = 0;
	speed = 0;
	intensity_byte = 120;
	angle_byte = 120;
	height = 11;
	delta_h = 0.4;
	dx = 0;
	
	//Defaultvärden för state_machine
	Kp = 0.005;
	Kd = 0.45;
	on_top_of_obstacle = false;
	trust_sensors = true;
	low = false;
	
	Init();
	
	//Konfigurering
	Configure_Servos_Delaytime();
	Configure_Servos_LED();
	Configure_Servos_No_Response();
	Configure_Servos_Angle_Limit('r'); // Måste köras ty begränsningarna tappas ibland trots att de ligger i ROM
	Configure_Servos_Max_Torque();
	
	//Inledande begränsning av hastigheterna så inget otrevligt sker vid start
	Send_Inner_P1_Velocity(0x0010);
	Send_Middle_P1_Velocity(0x0010);
	Send_Outer_P1_Velocity(0x0010);
	Send_Inner_P2_Velocity(0x0010);
	Send_Middle_P2_Velocity(0x0010);
	Send_Outer_P2_Velocity(0x0010);

	sei(); //Aktivera avbrott öht (MSB=1 i SREG). Anropas EFTER all konfigurering är klar.

	unsigned char first_kom_byte;
	
	Walk_Half_Cycle(1, 0.01, height,0); //Ställ in default-höjd

	while(1)
	{
		//Hämta det senast giltiga paketet
		cli();
		memcpy(lastPacket, lastValidPacket, sizeof(lastPacket));
		sei();
		
		first_kom_byte = lastPacket[0];
		
		if (first_kom_byte & 0b00001000) //Växla läge?
		{
			update_mode();
		}
		
		switch(cm)
		{
			case MANUAL: //Manuellt läge
			ROBOT_STATE = CORRIDOR; //Ha CORRIDOR som default-tillstånd
			speed = 0; //Nollställ speed och angle vid manuellt läge
			angle = 0;
			
			if (first_kom_byte & 0b00000011) //Skickas vinkel eller intensitet?
			{
				update_speed_and_angle();
			}
			if (first_kom_byte & 0b00000100) //Höj/sänk gångstil?
			{
				update_height();
			}
			
			if (first_kom_byte & 0b00010000) //Nytt Kp?
			{
				Kp = ((float)lastPacket[5])/1000.f; //Kp skickas som 1000 ggr det önskade värdet, därav divisionen
			}
			if (first_kom_byte & 0b00100000) //Nytt Kd?
			{
				Kd = ((float)lastPacket[6])/100.f; //Kd skickas som 100 ggr det önskade värdet, därav divisionen
			}
			
			if((!(first_kom_byte & 0b10000000)) && dance_r > 1)  // Är kroppen inte i utgångsläge?
			{
				Dance(0,0); //För tillbaka kroppen till utgångsläge
			}
			else
			{
				if ((first_kom_byte & 0b01000000) && lastPacket[7] == 1)
				{
					dx = -6;
				}
				else if ((first_kom_byte & 0b01000000) && lastPacket[7] == 2)
				{
					dx = 6;
				}
				else
				{
					dx = 0;
				}
				
				if(first_kom_byte & 0b10000000) //else if
				{
					Dance(((float)((lastPacket[8] & 0b11110000) >> 4)-8), -(((float)(lastPacket[8] & 0b00001111))-8));
				}
				else
				{
					Walk_Half_Cycle(speed,angle,height,dx);
				}
			}
			break;
			
			case AUTO: //Autonomt läge
			//height = 11;
			update_state();
			run_state();
			break;
			
			case RACE:
			if ((PIND & (1 << PIND3)) == 0) //Har knapp tryckts ned?
			{
				_delay_ms(1000);
				cm = AUTO;
			}
			break;
			
			default:
			break;
		}
	}
}

void update_mode()
{
	unsigned char change_mode = lastPacket[4];
	
	if (change_mode == 0b00001111) //Byt till MANUAL?
	{
		cm = MANUAL;
		height = 11;
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

void update_speed_and_angle()
{
	intensity_byte = 120; //Återställer till default så offset blir rätt
	angle_byte = 120;
	
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
	int8_t sgn_angle = (angle_byte >= 0) - (angle_byte < 0);
	angle = sgn_angle * 0.57 * angle_byte * angle_byte/10000; //Kvadratisk skalning av mottaget värde till vinkel
}

void update_height()
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
}
