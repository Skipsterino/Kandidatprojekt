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

float LP_Filter_And_Limit_Input(float speed, int sgn_speed, float theta, int sgn_theta, float height);
int max_speed(float theta,int sgn_theta);
void Adjust_Servo_Speed(float theta, int sgn_theta);
void Adjust_Height(float l, float height_step, float corner_pitch);
void Send_Legs_Kar(triple_float kar_p1, triple_float kar_p2, float corner_pitch);
void Rotate_And_Send_Legs(triple_float kar_p1, triple_float kar_p2, float corner_pitch, int sgn_speed,float theta);
double_float Limit_Theta(float speed, int sgn_speed, float theta, int sgn_theta );
float Limit_Speed(float speed, int sgn_speed);
float Limit_Height(float height);


triple_float Tripod(float x, float stroke, float height, uint8_t n);
triple_float Tripod2(float x, float stroke, float height, uint8_t n);
void Walk_Half_Cycle(float speed, float th, float height);
void Walk_Half_Crab_Cycle(float speed);
void Walk_Half_Crab_Cycle2(float speed);
void Walk_Half_Safe_Cycle(float speed, float th, float height) ;


#endif /* GANGSTILAR_H_ */

