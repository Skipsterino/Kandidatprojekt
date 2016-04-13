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
unsigned char USART_Receive( void );
unsigned char checksum_calc(unsigned char param[], uint8_t num_of_par);

void Send_Servo_Delaytime(void);
void Send_Servo_LED(void);
void Send_Servo_Angle_Limit(void);

void Send_Servo_Message(unsigned char message[], uint8_t num_of_par);
void Send_Servo_Position(unsigned char ID_and_pos[]);
void Send_Servo_Velocity(unsigned char ID_and_vel[]);

double_uchar Get_Servo_Position(unsigned char ID);
//unsigned char[] Receive_Servo_Statuspackage(uint8_t size); FUNKAR EJ TY KAN EJ RETURNERA STRUCT

#endif /* SERVO_UART_H_ */