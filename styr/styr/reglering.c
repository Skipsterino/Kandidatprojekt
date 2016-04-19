/*
 * reglering.c
 *
 * Created: 4/14/2016 8:35:29 AM
 *  Author: erilj291
 */ 

#include "reglering.h"

void update_alpha(void)
{
	if(fromKom[0] & (1 << 4)) //Har nytt Kp skickats?
	{
		Kp = fromKom[5];
	}
	
	if(fromKom[0] & (1 << 5)) //Har nytt Kd skickats?
	{
		Kd = fromKom[6];
	}
	
	uint8_t x_hb = fromSen[3];
	uint8_t x_vf = fromSen[6];
	uint8_t x_hf = fromSen[2];
	
	alpha = Kp*(x_hb-x_vf) + Kd*(x_hf-x_hb); //Uppdatera vridvinkeln alpha
	
}