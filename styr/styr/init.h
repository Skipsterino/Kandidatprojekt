/**
* @file init.h
* @author Erik
* @date 25 apr 2016
* @brief Config and init functions
*
*/

#ifndef INIT_H_
#define INIT_H_

#include <avr/io.h>

/**
* @brief Configuration and initialization
*
* Configures ports, initializes UART and initializes SPI.
*/
void Init(void);

/**
* @brief Initialization of UART
*
* This function is used by Init() to initialize UART.
*/
void UART_Init(void);

#include "SPI.h"

#endif /* INIT_H_ */