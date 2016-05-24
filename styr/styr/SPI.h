/**
* @file SPI.h
* @author kevkj515
* @date 15/4/2016 
* @brief SPI Driver
*
* Defines communication through the SPI bus given a triggered interrupt.
*/

#ifndef SPI_H_
#define SPI_H_

#ifndef F_CPU
#define F_CPU 16000000UL //*< Speed of processor (16 Mhz) */
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define SS_sen 3 //*< Offset of the slave-select bit to Sensor module */
#define SS_kom 4 //*< Offset of the slave-select bit to Kommunacation module */

volatile unsigned char fromSen[16]; //*< Array in which data from Sensor module is stored */
volatile unsigned char fromKom[16]; //*< Array in which data form Kommunication module is stored */

volatile unsigned char lastValidPacket[16]; //*< Array in which the last checksum-valid fromKom array is stored */

/**
* @brief Initiation
*
* Initiates the outputs and SPI configuration as
* a master on the bus
*/
void SPI_init_master();

/**
* @brief Master transmit
*
* Transmits the first byte to Sensor Module
* starting the chain of 16
*/
void SPI_sen_transmit_master();

/**
* @brief Change slave select
*
* Sets the slave select ports to their respective
* value given 'bools' of what they should be
*/
void Set_SS_sen_kom(uint8_t toSen, uint8_t toKom);

#endif /* SPI_H_ */
