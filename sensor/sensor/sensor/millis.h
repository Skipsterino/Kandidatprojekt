/*
 * millis.h
 *
 * Created: 4/12/2016 9:30:34 AM
 *  Author: fregu856
 */ 

#ifndef MILLIS_H_
#define MILLIS_H_

#include <util/atomic.h>

unsigned long timer0_millis;
unsigned char timer0_fract;
unsigned long timer0_overflow_count;
#define MILLIS_INC (1024 / 1000)
#define FRACT_INC ((1024 % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

unsigned long millis();

#endif /* MILLIS_H_ */