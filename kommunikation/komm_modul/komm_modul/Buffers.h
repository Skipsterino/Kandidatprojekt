/**
* @file Buffers.h
* @author Joakim
* @date 25 apr 2016
* @brief Buffer Definitions
*
* Buffers in the communication module for handling data
* passed between modules.
*/


#ifndef BUFFERS_H_
#define BUFFERS_H_


volatile unsigned char toBluetooth[16]; /**< Buffer containing data to be sent over Bluetooth. */
volatile unsigned char toSPI[16]; /**< Buffer containing data to be sent over the bus. */


#endif /* BUFFERS_H_ */