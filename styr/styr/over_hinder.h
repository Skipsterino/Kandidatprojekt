/**
* @file over_hinder.h
* @author Christoffer, Erik
* @date 5 may 2016
* @brief Climbing up/down
*
* Functions for climbing onto and down from obstacles of height approx. eq. to 6,5 cm.
*/ 

#ifndef OVER_HINDER_H_
#define OVER_HINDER_H_

#include "servo_UART.h"
#include "gangstilar.h"

#include <util/delay.h>

/**
* @brief Keeps track on the lenght previous step
*
* The lenght of the previous steps in a climb is needed for making
* the following steps smooth. 
* @param new_step The lenght of the next step.
*/
void update_step(float new_step); 

/**
* @brief Makes the robot climb an obstacle of the height 6.5 cm
*
* A sereis of hardcoded movements for acending a step with the height 6.5 cm 
*/
void Walk_Up(void); 

/**
* @brief Makes the robot climb down from an obstacle of the height 6.5 cm
*
* A sereis of hardcoded movements for decenting a step with the height 6.5 cm
*/
void Walk_Down(void); 

/**
* @brief Prepares the robot for a climb
*/
void To_Climbing_Stance(void);

/**
* @brief Moves the first leg to the top of an obstacle
* 
* Moves the first leg to the top of an obstacle and
* moves the robot forwards. 
*/
void First_Leg(char direction);

/**
* @brief Moves the second leg to the top of an obstacle
*
* Moves the second leg to the top of an obstacle and
* moves the robot forwards.
*/
void Second_Leg(char direction);

/**
* @brief Prepares the robot for moving the third leg to the top of the obstacle
*/
void First_Body_Adjust(void);

/**
* @brief Moves the third leg to the top of an obstacle
*
* Moves the third leg to the top of an obstacle and
* moves the robot forwards.
*/
void Third_Leg(char direction);

/**
* @brief Moves the fourth leg to the top of an obstacle
*
* Moves the fourth leg to the top of an obstacle and
* moves the robot forwards.
*/
void Fourth_Leg(char direction);

/**
* @brief Prepares the robot for moving the fifth leg to the top of the obstacle
*/
void Second_Body_Adjust(void);

/**
* @brief Moves the fifth leg to the top of an obstacle
*
* Moves the fifth leg to the top of an obstacle and
* moves the robot forwards.
*/
void Fifth_Leg(char direction);

/**
* @brief Moves the sixth leg to the top of an obstacle
*
* Moves the sixth leg to the top of an obstacle and
* moves the robot forwards.
*/
void Sixth_Leg(char direction);


/**
* @brief Makes the robot return to the default stance after an acent or decent
*/
void To_Default_Stance(void);

#endif /* OVER_HINDER_H_ */