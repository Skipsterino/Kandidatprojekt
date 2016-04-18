/*
 * servo_UART.h
 *
 * Created: 4/8/2016 3:03:52 PM
 *  Author: chrma018
 */ 


#ifndef SERVO_UART_H_
#define SERVO_UART_H_

#ifndef F_CPU
#define F_CPU 16000000UL		// 16 MHz
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "structs.h"

void USART_Transmit(unsigned char);
unsigned char USART_Receive(void);
unsigned char checksum_calc(unsigned char param[], uint8_t num_of_par);

void Configure_Servos_Delaytime(void);
void Configure_Servos_LED(void);
void Configure_Servos_Angle_Limit(void);
void Configure_Servos_No_Response(void);

unsigned int Get_Servo_Load(void);
double_uchar Get_Servo_Position(unsigned char ID);
//unsigned char[] Receive_Servo_Statuspackage(uint8_t size); FUNKAR EJ TY KAN EJ RETURNERA STRUCT
//void Check_Servo_Temp(unsigned char ID)

void Send_Servo_Message(unsigned char message[], uint8_t num_of_par);
void Send_Servo_Position(unsigned char ID, unsigned int pos);
void Send_Servo_Velocity(unsigned char ID, unsigned int vel);
void Send_Servo_Angle_Limit(unsigned char ID, unsigned int lower, unsigned int higher);

void Send_Outer_P1_Velocity(unsigned int vel); 
void Send_Outer_P2_Velocity(unsigned int vel); 
void Send_Middle_P1_Velocity(unsigned int vel); 
void Send_Middle_P2_Velocity(unsigned int vel); 
void Send_Inner_P1_Velocity(unsigned int vel); 
void Send_Inner_P2_Velocity(unsigned int vel); 

void Send_Leg1_Kar(float,float,float);
void Send_Leg2_Kar(float,float,float);
void Send_Leg3_Kar(float,float,float);
void Send_Leg4_Kar(float,float,float);
void Send_Leg5_Kar(float,float,float);
void Send_Leg6_Kar(float,float,float);

void Send_Leg1_Cyl(float,float,float);
void Send_Leg2_Cyl(float,float,float);
void Send_Leg3_Cyl(float,float,float);
void Send_Leg4_Cyl(float,float,float);
void Send_Leg5_Cyl(float,float,float);
void Send_Leg6_Cyl(float,float,float);

void Send_Servo_Position_And_Velocity(unsigned char ID, unsigned int pos, unsigned int vel);
void Send_Leg1_Kar_And_Velocity(float x, float y, float z, unsigned int inner, unsigned int middle, unsigned int outer);
void Send_Leg2_Kar_And_Velocity(float x, float y, float z, unsigned int inner, unsigned int middle, unsigned int outer);
void Send_Leg3_Kar_And_Velocity(float x, float y, float z, unsigned int inner, unsigned int middle, unsigned int outer);
void Send_Leg4_Kar_And_Velocity(float x, float y, float z, unsigned int inner, unsigned int middle, unsigned int outer);
void Send_Leg5_Kar_And_Velocity(float x, float y, float z, unsigned int inner, unsigned int middle, unsigned int outer);
void Send_Leg6_Kar_And_Velocity(float x, float y, float z, unsigned int inner, unsigned int middle, unsigned int outer);

void Send_Leg1_Cyl_And_Velocity(float r, float th, float z, unsigned int inner, unsigned int middle, unsigned int outer);
void Send_Leg2_Cyl_And_Velocity(float r, float th, float z, unsigned int inner, unsigned int middle, unsigned int outer);
void Send_Leg3_Cyl_And_Velocity(float r, float th, float z, unsigned int inner, unsigned int middle, unsigned int outer);
void Send_Leg4_Cyl_And_Velocity(float r, float th, float z, unsigned int inner, unsigned int middle, unsigned int outer);
void Send_Leg5_Cyl_And_Velocity(float r, float th, float z, unsigned int inner, unsigned int middle, unsigned int outer);
void Send_Leg6_Cyl_And_Velocity(float r, float th, float z, unsigned int inner, unsigned int middle, unsigned int outer);

#endif /* SERVO_UART_H_ */