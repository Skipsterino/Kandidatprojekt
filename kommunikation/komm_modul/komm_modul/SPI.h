/*
* SPI.h
*
* Created: 2016-04-06 16:32:24
*  Author: Joakim
*/


#ifndef SPI_H_
#define SPI_H_

#include <avr/io.h>
#include <avr/interrupt.h>

#include "Buffers.h"

void SPI_init_master();
void SPI_init_slave();

#endif /* SPI_H_ */