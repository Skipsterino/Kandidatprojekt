/*
 * over_hinder.c
 *
 * Created: 4/18/2016 1:55:26 PM
 *  Author: chrma018
 */ 

// Höjden på hinder kan ligga globalt...
#include "over_hinder.h"



void Walk_Up(void)
{
	
	//Denna kan man ju testa som probe start. 
	unsigned int load; 
	unsigned int contact_load = 0X00D0; //Mät fram denna
	unsigned int z = 3;
	float probe_step = 0.4; 

	load = 0;
	
	Send_Leg1_Kar(20, 0, 3); // upp med ben åt sidan 
	_delay_ms(1000);
	Send_Leg1_Kar(18, 18, 3); // fram med ben 
	_delay_ms(1000); 
	
	
	while(load < contact_load)
	{
		z= z - probe_step;
		Send_Leg1_Kar(18, 18, z);
		load = Get_Servo_Load(10); // ID 3 är mittenservot på ben 1 
		load = load && 0x3FF;
		_delay_ms(2000);	
		if( z < -5) // Test om man missar ytan
		{
			z=3;
		}
	}
	
	Send_Leg2_Kar(20, 0,-10); //För att se att vi lyckats med load-test
	_delay_ms(3000);
	
}