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
unsigned int load;

int main_old(void)
{
	Init();
	
	
	Send_Leg2_Kar(25,0,0);
	while(1)
	{
	Walk_Up();
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