/**
* @file gangstilar.h
* @author Jens
* @date 8 april 2016
* @brief Functions for walking
*
* Functions for walking and rotation
*
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


float height;

/**
* @brief Sets parameters for smooth transition with over_hinder
*
* Sets height and walking cycle phase correctly
* 
* @param direction Tells if robot just finished climbing up or climbing down
* 
*/
void Adjust_Stance_Climbed(char direction);

/**
* @brief Limits changes in walking input parameters
*
* Sets global last_speed and last_theta to limited values and global theta_max to max allowed rotation.
* 
* @param speed Input speed
* @param sgn_speed Signum of input speed
* @param theta Input rotation angle
* @param sgn_theta Signum of rotation angle
* 
* @return Limited height
*/
float LP_Filter_And_Limit_Input(float speed, int sgn_speed, float theta, int sgn_theta, float height);

int max_speed(float theta,int sgn_theta);//nonono?

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

void Adjust_Height(float l, float height_step, float corner_pitch); //nonono?

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
* @param sgn _peed Signum of input speed
* @param theta Rotation angle
* @param corner_pitch Working point offset in x-direction for corner legs
* @param speed_p1 Servo speed for inner, middle and outer servo in pair 1
* @param speed_p2 Servo speed for inner, middle and outer servo in pair 2
* 
*/
void Rotate_And_Send_Legs(triple_float kar_p1, triple_float kar_p2, float corner_pitch, int sgn_speed,float theta, triple_float speed_p1, triple_float speed_p2);

/**
* @breif Limits rotation
*
* Limits theta depending on speed
* 
* @param speed Input speed
* @param sgn_speed Signum of speed
* @param theta Input rotation
* @param sgn_theta Signum of rotation
*
* @return Limited theta and max allowed theta
*
*/
double_float Limit_Theta(float speed, int sgn_speed, float theta, int sgn_theta );

/**
* @Limits speed [-6, 6]
*
* Sets speed to 6 or -6 if outside [-6, 6]
* 
* @param speed Input speed
* @param sgn_speed Signum of speed
*
* @return Limited speed
* 
*/
float Limit_Speed(float speed, int sgn_speed);

/**
* @breif Limits Height [7, 14]
*
* Sets hieght to 7 or 14 if outside [7, 14]
* 
* @param heigth input height
*
* @return Limited height
* 
*/
float Limit_Height(float height);


/**
* @breif Generates leg positions for tripod gait 
*
* Generatets tripod gait leg positions with overlapping support phases. 
* Global support_l sets the length of support phase and global swing_l sets the length of swing phase.
* 
* @param x Foot distance from body in x-direction 
* @param stroke Leg stroke
* @param height Robot height
* @param lift Lift height for legs during swing phase
* @param n [1, support_l + swing_l] Index of walk cycle, the step of walk cycle to be generated.
*
*/
triple_float Tripod(float x, float stroke, float height,float lift, uint8_t n);

/**
* @breif Walks half cycle of tripod wave gait 
*
* Huvudfunktion för att gå
* 
* @param speed [-6, 6] Possitive value = forward
* @param theta [-0.33, 0,33] Rotation angle
* @param heigth [7, 14] 
*
*/
void Walk_Half_Cycle(float speed, float theta, float height);

/**
* @breif Walks half cycle of tripod mammal-kick gait
*
* It's the crabs
* 
* @param speed  höger vänster just nu.
* 
*/
void Walk_Half_Crab_Cycle(int8_t speed);


#endif /* GANGSTILAR_H_ */

