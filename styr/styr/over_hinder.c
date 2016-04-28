/*
 * over_hinder.c
 *
 * Created: 4/18/2016 1:55:26 PM
 *  Author: chrma018
 */ 

// Höjden på hinder kan ligga globalt...
#include "over_hinder.h"

unsigned int load;
float z;
void Walk_Up(void)
{
	
	//Denna kan man ju testa som probe start. 
	unsigned int contact_load = 0X04a0; //Mät fram denna
	z = 3;
	float probe_step = 0.4; 

	load = 0x000b;
	
	Send_Leg2_Kar(20, 0,0); //För att se att vi lyckats med load-test
	
	Send_Leg6_Kar(20, 0, 3); // upp med ben åt sidan 
	_delay_ms(1000);
	Send_Leg6_Kar(18, 4, 3); // fram med ben 
	_delay_ms(1000); 
	
	
	while(load < contact_load)
	{
		z= z - probe_step;
		_delay_ms(500);
		Send_Leg6_Kar(18, 4, z);
		load = Get_Servo_Load(3); // ID 3 är mittenservot på ben 1 
		//load = load && 0x3FF;
		_delay_ms(500);	
		if( z < -5) // Test om man missar ytan
		{
			z=3;
		}
	}
	
	Send_Leg2_Kar(20, 0,-10); //För att se att vi lyckats med load-test
	_delay_ms(3000);
	
}