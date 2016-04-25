/**
* @file SPI.h
* @author Joakim
* @date 25 apr 2016
* @brief SPI bus interface
*
* Defines functions to communicate over SPI with the other modules.
*/

#ifndef SPI_H_
#define SPI_H_

#include <avr/io.h>
#include <avr/interrupt.h>

#include "Buffers.h"

/**
* @brief Initiates the SPI interface.
*
* Initiates the SPI interface as a slave unit on the bus. Communication
* is handled by using interupts.
*/
void SPI_init_slave();

#endif /* SPI_H_ */