/*
* USART.h
*
* Created: 2016-04-06 09:06:16
*  Author: Joakim
*/

#ifndef USART_H_
#define USART_H_

#include <avr/io.h>
#include <avr/interrupt.h>

#include "Buffers.h"

volatile int BTtimeout;
volatile uint8_t BTcounter; 

void initUSART();

unsigned char USART_Recieve();
void USART_Transmit(unsigned char data);
void USART_Transmit_Array(unsigned char array[], uint8_t size);
void USART_Flush(void);

#endif /* USART_H_ */