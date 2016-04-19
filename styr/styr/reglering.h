/*
 * reglering.h
 *
 * Created: 4/13/2016 5:42:52 PM
 *  Author: chrma018
 */ 

#ifndef REGLERING_H_
#define REGLERING_H_

#include "SPI.h"

uint8_t Kp;
uint8_t Kd;
float alpha; //Vinkel som roboten ska vrida med (ev mult med skalfaktor)

void update_alpha(void);

#endif /* REGLERING_H_ */