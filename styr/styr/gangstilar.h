/*
 * gangstilar.h
 *
 * Created: 4/8/2016 3:03:52 PM
 *  Author: jenma421 
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


float height;//meingslös (?)

void Adjust_Stance_Climbed(char direction);

/**
* @brief Limits changes in walking parameters
*
* Sets global speed and theta to limited values
* 
* @param speed Input speed
* @param sgn_speed Signum of input speed
* @param theta Input rotation angle
* 
* @return Limited heigth
*/
float LP_Filter_And_Limit_Input(float speed, int sgn_speed, float theta, int sgn_theta, float height);

int max_speed(float theta,int sgn_theta);

/**
* @brief Calculates appropriate servo speeds
*
* Calculates appropriate servo speeds given theta
* 
* @param theta Input rotation angle
* @param sgn_theta Signum of rotation angle
* @param leg_down Tells if legs are in transition from swing to support phase
* 
* @return Appropriate speed for inner, middle and outer servo
*/
triple_float Adjust_Servo_Speed(float theta, int sgn_theta, int8_t leg_down);

void Adjust_Height(float l, float height_step, float corner_pitch);

/**
* @brief Sends position and servo speed to legs 
*
* Sends legs position in kartesian and speed for their servos.
* 
* @param kar_p1 Kartesian coordinates for legs in pair 1
* @param kar_p2 Kartesian coordinates for legs in pair 2
* @param corner_pitch Working point offset in x-direction for corner legs
* @param speed_p1 Servo speed for inner, middle and outer servo in pair 1
* @param speed_p2 Servo speed for inner, middle and outer servo in pair 2
* 
*/
void Send_Legs_Kar(triple_float kar_p1, triple_float kar_p2, float corner_pitch, triple_float speed_p1, triple_float speed_p2); 

/**
* @brief Sends position with added rotation and servo speed to legs 
*
* Sends legs position in cylindrical coordinates with added rotation and speed for their servos.
* 
* @param kar_p1 Kartesian coordinates for legs in pair 1
* @param kar_p2 Kartesian coordinates for legs in pair 2
* @param sgn speed Signum of input speed
* @param theta Rotation angle
* @param corner_pitch Working point offset in x-direction for corner legs
* @param speed_p1 Servo speed for inner, middle and outer servo in pair 1
* @param speed_p2 Servo speed for inner, middle and outer servo in pair 2
* 
*/
void Rotate_And_Send_Legs(triple_float kar_p1, triple_float kar_p2, float corner_pitch, int sgn_speed,float theta, triple_float speed_p1, triple_float speed_p2);

double_float Limit_Theta(float speed, int sgn_speed, float theta, int sgn_theta );
float Limit_Speed(float speed, int sgn_speed);
float Limit_Height(float height);
void Set_Last_Heigth(float heigth);


triple_float Tripod(float x, float stroke, float height,float lift, uint8_t n);
//triple_float Tripod2(float x, float stroke, float height, uint8_t n); 
void Walk_Half_Cycle(float speed, float th, float height);
void Walk_Half_Crab_Cycle(float speed);
//void Walk_Half_Crab_Cycle2(float speed);
//void Walk_Half_Safe_Cycle(float speed, float th, float height) ;


#endif /* GANGSTILAR_H_ */

