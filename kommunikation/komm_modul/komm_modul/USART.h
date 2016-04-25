/**
* @file USART.h
* @author Joakim
* @date 25 apr 2016
* @brief USART driver
*
* Defines commands for interfacing over USART protocol.
* Used in the project to communicate over bluetooth.
*/


#ifndef USART_H_
#define USART_H_

#include <avr/io.h>
#include <avr/interrupt.h>

#include "Buffers.h"

volatile int BTtimeout; /**< Timeout for the communication over bluetooth. */
volatile uint8_t BTcounter; /**< Counter for keeping track of the current byte sent over bluetooth. */

/**
* @brief Initiates USART
*
* Initiates USART to be used on USART0 on the ATmega1284P microprocessor.
* Configures the protocol to follow 115200, 8, N, 1 for use with the FireFly
* bluetooth circuit.
*/
void initUSART();

/**
* @brief Recieves one byte over USART
*
* Waits until one byte has been recieved and returns it.
* @return The recieved byte.
*/
unsigned char USART_Recieve();

/**
* @brief Transmits one byte over USART
*
* Waits until the other unit is ready to recieve, and then sends one byte.
* @param data The byte to be sent.
*/
void USART_Transmit(unsigned char data);

/**
* @brief Transmits multiple bytes over USART
*
* Sends an array of a given size over USART.
* @see USART_Transmit
* @param array The array to be sent.
* @param size Size of the array.
*/
void USART_Transmit_Array(unsigned char array[], uint8_t size);

/**
* @brief Flushes the USART buffer.
*
* Repeatedly reads from the buffer as long as there is data in the buffer.
* @warning Erases data.
*/
void USART_Flush(void);

#endif /* USART_H_ */