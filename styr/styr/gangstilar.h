/*
 * gangstilar.h
 *
 * Created: 4/8/2016 3:03:52 PM
 *  Author: chrma018
 */ 


#ifndef GANGSTILAR_H_
#define GANGSTILAR_H_

#ifndef F_CPU
#define F_CPU 16000000UL		// 16 MHz
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "invers_kinematik.h"
#include "structs.h"
#include "servo_UART.h"

triple_float Tripod(float l, float s,float h,uint8_t speed, uint8_t m, uint8_t n);
void Walk_Cycle(float l, float s,float h, uint8_t speed, float th, uint8_t m);

#endif /* GANGSTILAR_H_ */