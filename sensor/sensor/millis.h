﻿/**
* @file millis.h
* @author Fredrik, Jonas
* @date 26 apr 2016
* @brief The code required to get a timestamp
*
* The code enabling the timestamp required by the DMP library.
*/

#ifndef MILLIS_H_
#define MILLIS_H_

#include <util/atomic.h>

volatile unsigned long timer0_millis;			/**< The timer value in ms */
volatile unsigned char timer0_fract;			/**<  */
volatile unsigned long timer0_overflow_count;	/**< Number of overflow interrupts generated by the timer */
#define MILLIS_INC (1024 / 1000)				/**< Number with which the timer0_millis should increase every overflow */
#define FRACT_INC ((1024 % 1000) >> 3)			/**< Number with which the timer0_fract should increase every overflow */
#define FRACT_MAX (1000 >> 3)					/**< Number when if reached timer0_millis should be increased */

/**
* @brief Gives a timestamp in ms
*
* A function that gives a timestamp since the start of the program in ms.
* Required by the DMP library.
*/
unsigned long millis();

#endif /* MILLIS_H_ */