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
//unsigned int load;
uint8_t number_of_steps;

void Probe_Walk(void);
void Probe_Height_Up(void); 
void Probe_Height_Down(void); 

void Walk_Up(void); 
void Walk_Up_Hard(void);
void Walk_Top(void);

void Walk_Down(void); 
void Walk_Down_Hard(void);

void To_Climbing_Stance(void);
void To_Default_Stance(void);

void First_Leg(char direction);
void Second_Leg(char direction);
void First_Body_Adjust(void);
void Third_Leg(char direction);
void Fourth_Leg(char direction);
void Second_Body_Adjust(void);
void Fifth_Leg(char direction);
void Sixth_Leg(char direction);

#endif /* OVER_HINDER_H_ */