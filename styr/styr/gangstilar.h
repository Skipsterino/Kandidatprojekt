/**
* @file gangstilar.h
* @author Jens, Christoffer
* @date 20 may 2016
* @brief Functions for walking
*
* Functions for walking and rotation.
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


//float height; //vems?, inte min 
float dance_r; //kommentera chris

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
*
*/
float LP_Filter_And_Limit_Input(float speed, int sgn_speed, float theta, int sgn_theta, float height, float dx, int sgn_dx);

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
triple_float Calc_Servo_Speed(float theta, int sgn_theta, int8_t leg_down);

/**
* @brief Sends position and servo speed to legs 
*
* Sends legs position in kartesian and speed for their servos.
* 
* @param kar_p1 Kartesian coordinates for legs in pair 1
* @param kar_p2 Kartesian coordinates for legs in pair 2
* @param corner_pitch Working point offset in y-direction for corner legs
* @param speed_p1 Servo speed for inner, middle and outer servo in pair 1
* @param speed_p2 Servo speed for inner, middle and outer servo in pair 2
* 
*/
void Send_Legs_Kar(triple_float kar_p1, triple_float kar_p2, float corner_pitch, triple_float speed_p1, triple_float speed_p2, float dx_p1, float dx_p2); 

/**
* @brief Sends position with added rotation and servo speed to legs 
*
* Sends legs position in cylindrical coordinates with added rotation and speed for their servos.
* 
* @param kar_p1 Kartesian coordinates for legs in pair 1
* @param kar_p2 Kartesian coordinates for legs in pair 2
* @param sgn _speed Signum of input speed
* @param theta Rotation angle
* @param corner_pitch Working point offset in y-direction for corner legs
* @param speed_p1 Servo speed for inner, middle and outer servo in pair 1
* @param speed_p2 Servo speed for inner, middle and outer servo in pair 2
* 
*/
void Rotate_And_Send_Legs(triple_float kar_p1, triple_float kar_p2, float corner_pitch, int sgn_speed,float theta, triple_float speed_p1, triple_float speed_p2, float dx_p1, float dx_p2);

/**
* @breif Limits rotation depending on speed
*
* 
* @param speed Input speed
* @param sgn_speed Signum of speed
* @param theta Input rotation
* @param sgn_theta Signum of rotation
*
* @return Limited theta and max allowed theta for given speed
*
*/
double_float Limit_Theta(float speed, int sgn_speed, float theta, int sgn_theta );

/**
* @Limits speed 
*
* Limits speed to [-6, 6] or [-5, 5] if height >= 12 or <=9
* 
* @param speed Input speed
* @param sgn_speed Signum of speed
*
* @return Limited speed
* 
*/
float Limit_Speed(float speed, int sgn_speed, float dx, int sgn_dx);

/**
* @breif Limits height [7, 14]
*
* Sets height to 7 or 14 if outside [7, 14]
* 
* @param height input height
*
* @return Limited height
* 
*/
float Limit_Height(float height);

/**
* @breif Generates leg positions for tripod gait 
*
* Generates tripod gait leg positions with overlapping support phases. 
* Global support_l sets the length of support phase and global swing_l sets the length of swing phase.
* 
* @param x Foot distance from body in x-direction 
* @param stroke Leg stroke
* @param height Robot height
* @param lift Lift height for legs during swing phase
* @param n Index of walk cycle, the step of walk cycle to be generated.
*
* @return Leg position in kartesian
*
*/
triple_float Tripod(float x, float stroke, float height,float lift, uint8_t n);

/**
* @breif Walks half cycle of tripod wave gait 
*
* Can walk forward, backward, turn and rotate 
* 
* @param speed Walking speed, positive = forward
* @param theta Rotation angle, positive = counterclockwise
* @param height Robot height
*
*/
//void Walk_Half_Cycle(float speed, float theta, float height);//dummy
void Walk_Half_Cycle(float speed, float theta, float height, float dx);

/**
* @breif Walks half cycle of tripod crab-walk
*
* Walks sideways
* 
* @param speed  Walking speed positive = right 
* 
*/
void Walk_Half_Crab_Cycle(int8_t speed);

//kommentera chris
void Dance(float x, float y);
void Dance_Cyl(float r, float th);
void Victory_Dance(void);
#endif /* GANGSTILAR_H_ */

