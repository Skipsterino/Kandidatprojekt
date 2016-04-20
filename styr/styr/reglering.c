/*
 * reglering.c
 *
 * Created: 4/14/2016 8:35:29 AM
 *  Author: erilj291
 */ 

#include "reglering.h"

float update_alpha(float Yaw_value, float x_hb, float x_vf)
{
	if(fromKom[0] & (1 << 4)) //Har nytt Kp skickats?
	{
		Kp = fromKom[5];
	}
	
	if(fromKom[0] & (1 << 5)) //Har nytt Kd skickats?
	{
		Kd = fromKom[6];
	}
	
	//alpha = Kp*(x_hb-x_vf) + Kd*Yaw_value; //Uppdatera vridvinkeln alpha. RETURNERA ISTÄLLET DIREKT
	alpha = Kd*Yaw_value; //Uppdatera vridvinkeln alpha. RETURNERA ISTÄLLET DIREKT
	//alpha_d = Kd*Yaw_value; //D-delen
	
	return alpha;
}