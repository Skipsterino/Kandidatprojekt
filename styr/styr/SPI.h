/*
 * SPI.h
 *
 * Created: 4/8/2016 2:53:17 PM
 *  Author: chrma018
 */ 


#ifndef SPI_H_
#define SPI_H_

#ifndef F_CPU
#define F_CPU 16000000UL		// 16 MHz
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define SS_sen 3
#define SS_kom 4

volatile unsigned char fromSen[16];
volatile unsigned char fromKom[16];

void SPI_init_master();
void SPI_sen_transmit_master();
void Set_SS_sen_kom(uint8_t toSen, uint8_t toKom);

#endif /* SPI_H_ */