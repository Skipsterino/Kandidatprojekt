/*
 * over_hinder.h
 *
 * Created: 4/18/2016 1:57:34 PM
 *  Author: chrma018
 */ 


#ifndef OVER_HINDER_H_
#define OVER_HINDER_H_

#include "servo_UART.h"
#include <util/delay.h>

unsigned int load;

void Probe_Walk(void);
void Probe_Height_Up(void); 
void Walk_Up(void); 
void Walk_Top(void);
void Probe_Height_Down(void); 
void Walk_Down(void); 


#endif /* OVER_HINDER_H_ */