/*
 * millis.c
 *
 * Created: 4/12/2016 9:29:39 AM
 *  Author: fregu856
 */ 

#include "millis.h"

unsigned long millis()
{
	unsigned long m;
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		m = timer0_millis;
	}
	return m;
}

SIGNAL(TIMER0_OVF_vect)
{
	unsigned long m = timer0_millis;
	unsigned char f = timer0_fract;
	
	m += MILLIS_INC;
	f += FRACT_INC;
	if (f >= FRACT_MAX)
	{
		f -= FRACT_MAX;
		m += 1;
	}
		
	timer0_fract = f;
	timer0_millis = m;
	timer0_overflow_count++;
		
}
