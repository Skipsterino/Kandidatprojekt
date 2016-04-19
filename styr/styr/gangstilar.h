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

int max_speed(float theta,int sgn_theta);
triple_float Tripod(float x, float s, float h, uint8_t m, uint8_t n);
void Walk_Half_Cycle(int speed, float th, float h);
void Walk_Half_Crab_Cycle(int speed, float th, float h, float l );


#endif /* GANGSTILAR_H_ */

