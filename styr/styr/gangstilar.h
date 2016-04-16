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

uint8_t max_speed(float theta);
triple_float Tripod(uint8_t speed, float x, float s, float h, uint8_t m, uint8_t n);
void Walk__Half_Cycle(uint8_t speed, float th = 0, float h = 11, float l = 13);
void Walk__Half__crab_Cycle(uint8_t speed, float th = 0, float h = 11, float l = 13);

#endif /* GANGSTILAR_H_ */

