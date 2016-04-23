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
#include "gangstilar.h"
#include "over_hinder.h"
unsigned int loadd[100];
unsigned int message[100][8];

int main_old(void)
{
	Init();
	//Configure_Servos_No_Response();
	_delay_ms(1000);
	
	Send_Servo_Position(5, 0x200);
	_delay_ms(2000);
	//for(int i=0; i<100; i++)
	//{
		//
		//unsigned char message1[6];
		//unsigned char load_LSByte;
		//unsigned char load_MSByte;
		//
		//message1[0] = 5;
		//message1[1] = 0x04;
		//message1[2] = 0x02;
		//message1[3] = 0x24; //Läser ut Present load
		//message1[4] = 0x02;
		//
		//Send_Servo_Message(message1, 2);
		//
		//_delay_ms(0.02); //Lite extra tidsmarginal så bussen hinner bli ledig innan riktning ändras!!!
		//PORTD &= ~(1<<PORTD2); //Välj riktning "från servon" i tri-state
//
		//
		//message[i][0]=USART_Receive(); //mp är "triggerhappy"
		//message[i][1]=USART_Receive(); //Första startbyten
		//message[i][2]=USART_Receive(); //Andra startbyten
		//message[i][3]=USART_Receive(); //ID
		//message[i][4]=USART_Receive(); //Längd
		//message[i][5]=USART_Receive(); //Error
		//message[i][6]= USART_Receive(); //LS Byte av load
		//message[i][7]= USART_Receive(); //MS Byte av load
		////message[i][8]=USART_Receive(); //Checksum
		//_delay_ms(0.05); //Lite extra tidsmarginal så bussen hinner bli ledig innan riktning ändras!!!
		//
		//unsigned int load = (((unsigned int)load_MSByte) << 8) | load_LSByte;
		//PORTD |= 1<<PORTD2; //Välj riktning "till servon" i tri-state
	//}
	

	for(int i; i<100; ++i)
	{
		loadd[i]=Get_Servo_Load(5);
	}
	
	Send_Leg5_Kar(20,0,-5);
	
	//Walk_Up();

	while(1)
	{
		//Walk_Up();
	}
	
	//KÖR CONFIGURE-FUNKTIONERNA NÄR SERVONA BEHÖVER KALIBRERAS PÅ NÅGOT SÄTT
	//Configure_Servos_Delaytime();
	//Configure_Servos_LED();
	
	Configure_Servos_No_Response();
	Configure_Servos_Angle_Limit();
	
	Send_Inner_P1_Velocity(0x0010); //DESSA SEX ANROP MÅSTE ALLTID KÖRAS EFTERSOM HASTIGHETEN LIGGER I RAM!!!
	Send_Middle_P1_Velocity(0x0010);//
	Send_Outer_P1_Velocity(0x0010);//
	Send_Inner_P2_Velocity(0x0010);//
	Send_Middle_P2_Velocity(0x0010);//
	Send_Outer_P2_Velocity(0x0010);//
	
	sei(); //Aktivera avbrott öht (MSB=1 i SREG). Anropas EFTER all konfigurering klar!
	
	triple_uint positioner = Kar_To_Pos(24.5, 0.5, 0);

	while(1)
	{
		Walk_Half_Cycle(3, 0,6);
		Walk_Half_Cycle(3, 0,8);
		Walk_Half_Cycle(3, 0,10);
		Walk_Half_Cycle(3, 0,12);
		Walk_Half_Cycle(3, 0,16);// max speed =+-6 , max theta +-0.57, ändra inte h när den står på marken.
		//Walk_Half_Crab_Cycle(4,0,15,10);//bara testad med dessa värden, ska ha mycket mindre h
	}

}
