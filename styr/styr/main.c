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

CONTROL_MODE cm;
float angle;
float speed;
int8_t intensity_byte;
int8_t angle_byte;
float height;
float delta_h;

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
	
	cm = RACE; //Aktuellt styrläge hos roboten
	ROBOT_STATE = CORRIDOR; //Default-tillstånd hos roboten
	
	//Defaultvärden
	angle = 0;
	speed = 0;
	intensity_byte = 120;
	angle_byte = 120;
	height = 11;
	delta_h = 0.4;
	
	//Defaultvärden för state_machine
	Kp = 0.01;
	Kd = 0.55;
	on_top_of_obstacle = false;
	trust_sensors = true;
	
	Init();
	
	//init_fuck();
	
	//KÖR CONFIGURE-FUNKTIONERNA NÄR SERVONA BEHÖVER KALIBRERAS PÅ NÅGOT SÄTT
	Configure_Servos_Delaytime();
	Configure_Servos_LED();
	Configure_Servos_No_Response();
	Configure_Servos_Angle_Limit('r'); // ta inte bort!!!!!!!!!!!!! allt kan faila......
	Configure_Servos_Max_Torque();
	
	//
	
	Send_Inner_P1_Velocity(0x0010); //DESSA SEX ANROP MÅSTE ALLTID KÖRAS EFTERSOM HASTIGHETEN LIGGER I RAM!!!
	Send_Middle_P1_Velocity(0x0010);//
	Send_Outer_P1_Velocity(0x0010);//
	Send_Inner_P2_Velocity(0x0010);//
	Send_Middle_P2_Velocity(0x0010);//
	Send_Outer_P2_Velocity(0x0010);//

	
	

	
	sei(); //Aktivera avbrott öht (MSB=1 i SREG). Anropas EFTER all konfigurering klar!

	//_delay_ms(100);
	////Send_Leg3_Kar(22,0,0);
	////_delay_ms(1);
	////Send_Leg1_Kar(22,0,0);
	////_delay_ms(1);
	////Send_Leg2_Kar(22,0,0);
	////_delay_ms(1);
	////Send_Leg4_Kar(22,0,0);
	////_delay_ms(1);
	////Send_Leg5_Kar(22,0,0);
	////_delay_ms(1);
	////Send_Leg6_Kar(22,0,0);
	////_delay_ms(1);
	//Send_Servo_Position(1,0x01FF);
	//Send_Servo_Position(2,0x01FF);
	//Send_Servo_Position(3,0x01FF-0xA0);
	//Send_Servo_Position(4,0x01FF+0xA0);
	//Send_Servo_Position(5,0x01FF+0xA0);
	//Send_Servo_Position(6,0x01FF-0xA0);
	//Send_Servo_Position(7,0x01FF);
	//Send_Servo_Position(8,0x01FF);
	//Send_Servo_Position(9,0x01FF-0xA0);
	//Send_Servo_Position(10,0x01FF+0xA0);s
	//Send_Servo_Position(11,0x01FF+0xA0);
	//Send_Servo_Position(12,0x01FF-0xA0);
	//Send_Servo_Position(13,0x01FF);
	//Send_Servo_Position(14,0x01FF);
	//Send_Servo_Position(15,0x01FF-0xA0);
	//Send_Servo_Position(16,0x01FF+0xA0);
	//Send_Servo_Position(17,0x01FF+0xA0);
	//Send_Servo_Position(18,0x01FF-0xA0);
	//
	//while(1)
	//{
	//}
	
	unsigned char first_kom_byte;
	
	Walk_Half_Cycle(1, 0.01, height); //Ställ in default-höjd

	while(1)
	{
		
		//Hämta det senaste giltiga paketet
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
			ROBOT_STATE = CORRIDOR; //Ha CORRIDOR som default state
			speed = 0;
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
				Kp = ((float)lastPacket[5])/1000.f; //Kp skickas som 1000 ggr det önskade värdet!!!
			}
			if (first_kom_byte & 0b00100000) //Nytt Kd?
			{
				Kd = ((float)lastPacket[6])/100.f; //Kd skickas som 100 ggr det önskade värdet!!!
			}
			
			if((!(first_kom_byte & 0b10000000)) && dance_r > 1)
			{
				Dance(0,0);
			}
			else
			{
				if ((first_kom_byte & 0b01000000) && lastPacket[7] == 1)
				{
					Walk_Half_Crab_Cycle(-6);
				}
				else if ((first_kom_byte & 0b01000000) && lastPacket[7] == 2)
				{
					Walk_Half_Crab_Cycle(6);
				}
				else if(first_kom_byte & 0b10000000)
				{
					Dance(((float)((lastPacket[8] & 0b11110000) >> 4)-8), -(((float)(lastPacket[8] & 0b00001111))-8));
				}
				else
				{
					Walk_Half_Cycle(speed,angle,height);
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
	//angle = (float)(angle_byte)*((float)0.57)/((float)100); //128 på kontroll -> 0.57 i vinkel
	int8_t sgn_angle = (angle_byte >= 0) - (angle_byte < 0);
	angle = sgn_angle * 0.57 * angle_byte * angle_byte/10000;
}

void update_height()
{
	unsigned char change_height = lastPacket[3];
	
	if (change_height == 1) //Sänk?
	{
		//height -= delta_h;
	}
	else if (change_height == 2) //Höj?
	{
		//height += delta_h;
	}
}
