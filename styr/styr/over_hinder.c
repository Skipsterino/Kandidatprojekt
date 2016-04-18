/*
 * over_hinder.c
 *
 * Created: 4/18/2016 1:55:26 PM
 *  Author: chrma018
 */ 

// Höjden på hinder kan ligga globalt...


void Walk_Up(void)
{
	//Denna kan man ju testa som probe start. 
	
	unsigned int contact_load = 0X01FF; //Mät fram denna
	unsigned int load; 
	unsigned int z = 3;
	float probe_step = 0.4; 
	
	Send_Leg1_Kar_And_Velocity(20, 0, 3, 0x0100, 0x0100, 0x0100); // upp med ben åt sidan 
	_delay_ms(1000);
	Send_Leg1_Kar(18, 18, 3); // fram med ben 
	_delay_ms(1000); 
	
	
	while(load < contact_load)
	{
	z= z - probe_step;
	Send_Leg1_Kar_And_Velocity(18, 18, z, 0x0010, 0x0010, 0x0010);
	load = Get_Servo_Load(1);
	_delay_ms(5);	
	if( z < -5) // Test om man missar ytan
	{
		Probe_Walk();
	}
	}
	
	Send_Leg2_Kar_And_Velocity(20, 0, 3, 0x0100, 0x0100, 0x0100);
	
	
}