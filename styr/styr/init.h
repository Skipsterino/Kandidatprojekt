/**
* @file init.h
* @author erilj291
* @date 15/4/2016 
* @version 1.0
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