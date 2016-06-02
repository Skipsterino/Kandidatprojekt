/*
 *        File: servo_UART.c
 *     Version: 1.0
 * Last edited: 12/5/2016 
 *     Authors: chrma018, erilj291
 */ 


#include "servo_UART.h"
#include "invers_kinematik.h"

#define FIRST_SERVO_OFFSET 0x00
#define SECOND_SERVO_OFFSET 0x2F
#define THIRD_SERVO_OFFSET 0x79

void UART_Transmit( unsigned char data )
{
	// Vänta på att bufferten ska bli tom
	while ( !( UCSR0A & (1<<UDRE0)) );
	// Skicka datan
	UDR0 = data;
}

volatile unsigned char UART_Receive( void )
{
	float i=0;
	// Vänta på att data mottas
	while ((!(UCSR0A & (1<<RXC0))))
	{
		i = i + 1; 
		// Timer för mottagningen
		if(i>50)
		{
			return 0xBB;
		}
	}
	// Hämta och returnera mottagen data från bufferten
	return UDR0;
}

unsigned char checksum_calc(unsigned char message[], uint8_t num_of_par)
{
	uint8_t sum = 0;
	for (uint8_t i = 0; i< num_of_par+3; i++) //Ta med parametrarna + ID + instruktion + längd
	{
		sum += message[i];
	}
	uint8_t checksum = ~sum; //inverserad summa 
	return (unsigned char)checksum;
}

void Configure_Servos_Delaytime(void)
{
	//till alla servon
	for (uint8_t i = 1; i < 19; i++)
	{
		unsigned char return_delay_time[] = {i, 0x04, 0x03, 0x05, 0x2D}; //delaytime = 2D
		Send_Servo_Message(return_delay_time, 2);
		
		_delay_ms(1);
	}
}


void Configure_Servos_LED(void)
{
	//till alla servon
	for (uint8_t i = 1; i < 19; i++)
	{
		unsigned char LED_settings[] = {i, 0x04, 0x03, 0x11, 0xFF}; //LED  är på 
		Send_Servo_Message(LED_settings, 2);
		
		_delay_ms(1);
	}
}


void Configure_Servos_Angle_Limit(char mode)
{
	//Vilka servon sitter var 
	uint8_t inner_middle[] = {13,14};
	uint8_t inner_lf_rb[] = {1,8};
	uint8_t inner_rf_lb[] = {2,7};
	uint8_t middle[] = {3,4,9,10,15,16};
	uint8_t outer_left[] = {6,12,18};
	uint8_t outer_right[] = {5,11,17};
	
	//till alla servon
	for (uint8_t i = 0; i < 6; i++)
	{
		// pos 1FF -+ 1023/300pos/vinkl*90grader= 0x00CD,0x0331
		Send_Servo_Angle_Limit(middle[i], 0x00CD, 0x0331); 
		_delay_ms(1);
	}
	
	for (uint8_t i = 0; i < 3; i++)
	{
		// pos 1FF + (1023/300pos/vinkl*(+40grader eller - 90 grader)= 0x00CD (- 0x1F för vi behöver extraspelet),0x0287
		Send_Servo_Angle_Limit(outer_left[i], 0x00AE, 0x0287); 
		_delay_ms(1);
		 // pos 1FF + (1023/300pos/vinkl*(-40grader eller + 90 grader)= 0x0177,0x0331 (+ 0x1F för vi behöver extraspelet)
		Send_Servo_Angle_Limit(outer_right[i], 0x0177, 0x0350);
		_delay_ms(1);
	}
	
	if (mode == 'c') //"Climbing mode"?
	{
		for (uint8_t i = 0; i < 2; i++)
		{
			//Inga begränsningar för de inre servona i klättrings läge 
			Send_Servo_Angle_Limit(inner_lf_rb[i], 0x0000, 0x03ff);  
			_delay_ms(1);
			Send_Servo_Angle_Limit(inner_middle[i], 0x0000, 0x03ff); 
			_delay_ms(1);
			Send_Servo_Angle_Limit(inner_rf_lb[i], 0x0000, 0x03ff); 
			_delay_ms(1);
		}
	}
	
	else //Annars antas mode == 'r', dvs "regular mode".
	{
		for (uint8_t i = 0; i < 2; i++)
		{
			// pos 1FF + (1023/300pos/vinkl*(-45grader eller + 60 grader)= 0x0166,0x02CB
			Send_Servo_Angle_Limit(inner_lf_rb[i], 0x0166, 0x02CB);  
			_delay_ms(1);
			// pos 1FF + (1023/300pos/vinkl*(-15grader eller + 15 grader)= 0x01BB,0x0243 -> 0x019F, 0x025F (lite marginal)
			Send_Servo_Angle_Limit(inner_middle[i], 0x019F, 0x025F); 
			_delay_ms(1);
			// pos 1FF + (1023/300pos/vinkl*(-60grader eller + 45 grader)= 0x0131,0x0298
			Send_Servo_Angle_Limit(inner_rf_lb[i], 0x0131, 0x0298); 
			_delay_ms(1);
		}
	}
}

void Configure_Servos_No_Response(void)
{
	//Skicka till alla servon
	for (uint8_t i = 1; i < 19; i++)
	{
		unsigned char response_settings[] = {i, 0x04, 0x03, 0x10, 0x01}; //0x01 betyder att servot inte svarar om det inte explicit ombeds att göra det
		Send_Servo_Message(response_settings, 2);
		
		_delay_ms(1);
	}
}

void Configure_Servos_Max_Torque(void)
{
	//Skicka till alla servon
	for (uint8_t i = 1; i < 19; i++)
	{
		unsigned char response_settings[] = {i, 0x05, 0x03, 0x0E, 0x90, 0x02}; //Ställer in maximum torque = 0x290 
		Send_Servo_Message(response_settings, 3);
		
		_delay_ms(1);
	}
}

unsigned int Get_Servo_Load(unsigned char ID)
{	
	unsigned char message[5];
	unsigned char load_LSByte;
	unsigned char load_MSByte;
	volatile unsigned char start_byte1= 0;
	volatile unsigned char start_byte2= 0; 
	uint8_t timer = 0; 
	unsigned int load;
	
	message[0] = ID;
	message[1] = 0x04; //Length
	message[2] = 0x02; //Läs
	message[3] = 0x28; //Läser där Present load finns  
	message[4] = 0x02; //Läs två byts
	
	Send_Servo_Message(message, 2);

	PORTD &= ~(1<<PORTD2); //Välj riktning "från servon" i tri-state
	
	while((start_byte1 != 0xFF) && (start_byte2 != 0xFF)) // Vänta på start bytes 0xFF, 0xFF
	{
		start_byte2 = start_byte1; 
		start_byte1 = UART_Receive();
		++timer;
		if(timer>10) //Timer
		{
			break;
		}
	}
	UART_Receive(); //Hantera förskjutningen i bufferten
	UART_Receive(); //ID
	UART_Receive(); //Längd
	UART_Receive(); //Error
	load_LSByte = UART_Receive(); //LS Byte av load
	load_MSByte = UART_Receive(); //MS Byte av load
	UART_Receive(); //Checksum
	
	_delay_ms(0.05); //Lite extra tidsmarginal så bussen hinner bli ledig innan riktning ändras
	
	PORTD |= 1<<PORTD2; //Välj riktning "till servon" i tri-state
	
	load = (((unsigned int)load_MSByte) << 8) | load_LSByte;
	return load;
}

void Send_Servo_Velocity(unsigned char ID, unsigned int vel)
{
	//Dela upp hastyigheter i LS respektive MS Byte
	unsigned char velLS = vel; //Plockar ut LS Byte av hastigheten för det första servot (översta servot)
	unsigned char velMS = (vel>>8); //Som ovan men med MS Byte
	unsigned char message[6];
	message[0] = ID;
	message[1] = 0x05;	//längd 
	message[2] = 0x03;	//Skriv 
	message[3] = 0x20;	//Skriv i hastighet 
	message[4] = velLS; //hastighet LS byte
	message[5] = velMS; //hastighet MS byte
	
	Send_Servo_Message(message, 3);
}

void Send_Servo_Angle_Limit(unsigned char ID, unsigned int lower, unsigned int higher)
{
	//Dela upp vinklar i LS respektive MS Byte
	unsigned char lowerLS = lower; //Plockar ut LS Byte av vinkel för det första servot (översta servot)
	unsigned char lowerMS = (lower>>8); //Som ovan men med MS Byte
	
	//Dela upp vinklar i LS respektive MS Byte
	unsigned char higherLS = higher; //Plockar ut LS Byte av vinkel för det första servot (översta servot)
	unsigned char higherMS = (higher>>8); //Som ovan men med MS Byte
	
	unsigned char limits[] = {ID, 0x07, 0x03, 0x06, lowerLS, lowerMS, higherLS, higherMS};
	Send_Servo_Message(limits, 5);
}



void Send_Inner_P1_Velocity(unsigned int vel)
{	
	unsigned char inner_servos[]={2,8,13}; //Skickar till dessa servon 
		
	for (uint8_t i = 0; i < 3; i++)
	{
		Send_Servo_Velocity(inner_servos[i], vel);
	}
}


void Send_Inner_P2_Velocity(unsigned int vel)
{	
	unsigned char inner_servos[]={1,7,14}; //Skickar till dessa servon 
		
	for (uint8_t i = 0; i < 3; i++)
	{
		Send_Servo_Velocity(inner_servos[i], vel);
	}
}


void Send_Middle_P1_Velocity(unsigned int vel)
{	
	unsigned char middle_servos[]={4,10,15}; //Skickar till dessa servon 
		
	for (uint8_t i = 0; i < 3; i++)
	{
		Send_Servo_Velocity(middle_servos[i], vel);
	}
}


void Send_Middle_P2_Velocity(unsigned int vel)
{	
	unsigned char middle_servos[]={9,16,3}; //Skickar till dessa servon 
		
	for (uint8_t i = 0; i < 3; i++)
	{
		Send_Servo_Velocity(middle_servos[i], vel);
	}
}


void Send_Outer_P1_Velocity(unsigned int vel)
{	
	unsigned char outer_servos[]={6,12,17}; //Skickar till dessa servon 
		
	for (uint8_t i = 0; i < 3; i++)
	{
		Send_Servo_Velocity(outer_servos[i], vel);
	}
}


void Send_Outer_P2_Velocity(unsigned int vel)
{	
	unsigned char outer_servos[]={5,11,18}; //Skickar till dessa servon 
		
	for (uint8_t i = 0; i < 3; i++)
	{
		Send_Servo_Velocity(outer_servos[i], vel);
	}
}


void Send_Servo_Message(unsigned char message[], uint8_t num_of_par)
{
	unsigned char checksum = checksum_calc(message, num_of_par); //Beräknar checksum
	
	cli(); //Deaktivera avbrott
	
	UART_Transmit(0xFF); //2 st Startbytes
	UART_Transmit(0xFF);
	
	// Här skickas meddelandet (exkl checksum)
	for(uint8_t i=0; i < num_of_par+3; i++)
	{
		UART_Transmit(message[i]);
	}
	
	UART_Transmit(checksum); //Checksum
	
	while(!( UCSR0A & (1<<TXC0))); //Vänta på att UART är redo att skicka ny data
	_delay_ms(0.06); //Lite extra tidsmarginal så överföringen verkligen hinner bli klar
	
	sei(); //Aktivera avbrott igen
}

//Skickar önskad position till servo. Inargument = (ID, position)
void Send_Servo_Position(unsigned char ID, unsigned int pos)
{
	//Dela upp positionen i LS respektive MS Byte
	unsigned char posLS = pos; 
	unsigned char posMS = (pos>>8);
	
	unsigned char message[6];
	message[0] = ID;
	message[1] = 0x05; //Längd 
	message[2] = 0x03; //Skriv 
	message[3] = 0x1E; //Skriv vid Position
	message[4] = posLS;//Position LS byte 
	message[5] = posMS;//Position MS byte
	
	Send_Servo_Message(message, 3);
}

//Konverterar de givna kart. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg1_Kar(float x, float y, float z)
{
	//Invers kinematik
	triple_uint pos = Kar_To_Pos(x, y, z);
	//Servo-offsets och ev. invertering 
	unsigned int pos11 = -(pos.a - FIRST_SERVO_OFFSET) + 0x99 + 0x01FF;
	unsigned int pos12 = -(pos.b - SECOND_SERVO_OFFSET) + 0x01FF;
	unsigned int pos13 = -(pos.c - THIRD_SERVO_OFFSET) + 0x01FF;
	
	Send_Servo_Position(8, pos11); //Servona i ben 1
	Send_Servo_Position(10, pos12);
	Send_Servo_Position(12, pos13);
}

//Konverterar de givna kart. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg2_Kar(float x, float y, float z)
{
	//Invers kinematik
	triple_uint pos = Kar_To_Pos(x, y, z);
	//Servo-offsets och ev. invertering  
	unsigned int pos21 = pos.a - FIRST_SERVO_OFFSET - 0x99 + 0x01FF;
	unsigned int pos22 = pos.b - SECOND_SERVO_OFFSET + 0x01FF;
	unsigned int pos23 = pos.c - THIRD_SERVO_OFFSET + 0x01FF;
	
	Send_Servo_Position(7, pos21); //Servona i ben 2
	Send_Servo_Position(9, pos22);
	Send_Servo_Position(11, pos23);
}

//Konverterar de givna kart. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg3_Kar(float x, float y, float z)
{
	//Invers kinematik
	triple_uint pos = Kar_To_Pos(x, y, z);
	//Servo-offsets och ev. invertering  
	unsigned int pos31 = -(pos.a - FIRST_SERVO_OFFSET) + 0x01FF;
	unsigned int pos32 = -(pos.b - SECOND_SERVO_OFFSET) + 0x01FF;
	unsigned int pos33 = -(pos.c - THIRD_SERVO_OFFSET) + 0x01FF;
	
	Send_Servo_Position(14, pos31); //Servona i ben 3
	Send_Servo_Position(16, pos32);
	Send_Servo_Position(18, pos33);
}

//Konverterar de givna kart. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg4_Kar(float x, float y, float z)
{	
	//Invers kinematik
	triple_uint pos = Kar_To_Pos(x, y, z);
	//Servo-offsets och ev. invertering 
	unsigned int pos41 = pos.a - FIRST_SERVO_OFFSET + 0x01FF;
	unsigned int pos42 = pos.b - SECOND_SERVO_OFFSET + 0x01FF;
	unsigned int pos43 = pos.c - THIRD_SERVO_OFFSET + 0x01FF;
	
	Send_Servo_Position(13, pos41); //Servona i ben 4
	Send_Servo_Position(15, pos42);
	Send_Servo_Position(17, pos43);
}

//Konverterar de givna kart. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg5_Kar(float x, float y, float z)
{
	//Invers kinematik
	triple_uint pos = Kar_To_Pos(x, y, z);
	//Servo-offsets och ev. invertering 
	unsigned int pos51 = -(pos.a - FIRST_SERVO_OFFSET) - 0x99 + 0x01FF;
	unsigned int pos52 = -(pos.b - SECOND_SERVO_OFFSET) + 0x01FF;
	unsigned int pos53 = -(pos.c - THIRD_SERVO_OFFSET) + 0x01FF;
	
	Send_Servo_Position(2, pos51); //Servona i ben 5
	Send_Servo_Position(4, pos52);
	Send_Servo_Position(6, pos53);
}

//Konverterar de givna kart. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg6_Kar(float x, float y, float z)
{
	//Invers kinematik
	triple_uint pos = Kar_To_Pos(x, y, z);
	//Servo-offsets och ev. invertering  
	unsigned int pos61 = pos.a - FIRST_SERVO_OFFSET + 0x99 + 0x01FF;
	unsigned int pos62 = pos.b - SECOND_SERVO_OFFSET + 0x01FF;
	unsigned int pos63 = pos.c - THIRD_SERVO_OFFSET + 0x01FF;
	
	Send_Servo_Position(1, pos61); //Servona i ben 6
	Send_Servo_Position(3, pos62);
	Send_Servo_Position(5, pos63);
}

//Konverterar de givna cyl. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg1_Cyl(float r, float th, float z)
{
	//Ben-offset och koordinatbyte
	float x = r*sin(th) - 6;
	float y = r*cos(th) - 12;
	//invers kinematik 
	triple_uint pos = Kar_To_Pos(x, y, z);
	//Servo-offsets och ev. invertering 
	unsigned int pos1 = -(pos.a - FIRST_SERVO_OFFSET) + 0x99 + 0x01FF;
	unsigned int pos2 = -(pos.b - SECOND_SERVO_OFFSET) + 0x01FF;
	unsigned int pos3 = -(pos.c - THIRD_SERVO_OFFSET) + 0x01FF;
	
	Send_Servo_Position(8,pos1); //Servona i ben 1
	Send_Servo_Position(10, pos2);
	Send_Servo_Position(12, pos3);
}

//Konverterar de givna cyl. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg2_Cyl(float r, float th, float z)
{
	//Ben-offset och koordinatbyte
	float x = -r*sin(th) - 6;
	float y = r*cos(th) - 12; 
	//invers kinematik 
	triple_uint pos = Kar_To_Pos(x, y, z);
	//Servo-offsets och ev. invertering 
	unsigned int pos1 = pos.a - FIRST_SERVO_OFFSET - 0x99 + 0x01FF;
	unsigned int pos2 = pos.b - SECOND_SERVO_OFFSET + 0x01FF;
	unsigned int pos3 = pos.c - THIRD_SERVO_OFFSET + 0x01FF;
	
	Send_Servo_Position(7, pos1); //Servona i ben 2
	Send_Servo_Position(9, pos2);
	Send_Servo_Position(11, pos3);
}

//Konverterar de givna cyl. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg3_Cyl(float r, float th, float z)
{
	//Ben-offset och koordinatbyte
	float x = r*sin(th) - 10; 
	float y = r*cos(th);
	//invers kinematik 
	triple_uint pos = Kar_To_Pos(x, y, z);
	//Servo-offsets och ev. invertering 
	unsigned int pos1 = -(pos.a - FIRST_SERVO_OFFSET) + 0x01FF;
	unsigned int pos2 = -(pos.b - SECOND_SERVO_OFFSET) + 0x01FF;
	unsigned int pos3 = -(pos.c - THIRD_SERVO_OFFSET) + 0x01FF;

	Send_Servo_Position(14, pos1); //Servona i ben 3
	Send_Servo_Position(16, pos2);
	Send_Servo_Position(18, pos3);
}

//Konverterar de givna cyl. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg4_Cyl(float r, float th, float z)
{
	//Ben-offset och koordinatbyte
	float x = -r*sin(th) - 10;
	float y = r*cos(th);
	//invers kinematik 
	triple_uint pos = Kar_To_Pos(x, y, z);
	//Servo-offsets och ev. invertering 
	unsigned int pos1 = pos.a - FIRST_SERVO_OFFSET + 0x01FF;
	unsigned int pos2 = pos.b - SECOND_SERVO_OFFSET + 0x01FF;
	unsigned int pos3 = pos.c - THIRD_SERVO_OFFSET + 0x01FF;
	
	Send_Servo_Position(13, pos1); //Servona i ben 4
	Send_Servo_Position(15, pos2);
	Send_Servo_Position(17, pos3);
}

//Konverterar de givna cyl. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg5_Cyl(float r, float th, float z)
{
	//Ben-offset och koordinatbyte
	float x = r*sin(th) - 6;
	float y = r*cos(th) + 12;
	//invers kinematik 
	triple_uint pos = Kar_To_Pos(x, y, z);
	//Servo-offsets och ev. invertering 
	unsigned int pos1 = -(pos.a - FIRST_SERVO_OFFSET) - 0x99 + 0x01FF;
	unsigned int pos2 = -(pos.b - SECOND_SERVO_OFFSET) + 0x01FF;
	unsigned int pos3 = -(pos.c - THIRD_SERVO_OFFSET) + 0x01FF;
	
	Send_Servo_Position(2, pos1); //Servona i ben 5
	Send_Servo_Position(4, pos2);
	Send_Servo_Position(6, pos3);
}

//Konverterar de givna cyl. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg6_Cyl(float r, float th, float z)
{
	//Ben-offset och koordinatbyte
	float x = -r*sin(th) - 6;
	float y = r*cos(th) + 12;
	//invers kinematik 
	triple_uint pos = Kar_To_Pos(x, y, z);
	//Servo-offsets och ev. invertering 
	unsigned int pos1 = pos.a - FIRST_SERVO_OFFSET + 0x99 + 0x01FF;
	unsigned int pos2 = pos.b - SECOND_SERVO_OFFSET + 0x01FF;
	unsigned int pos3 = pos.c - THIRD_SERVO_OFFSET + 0x01FF;
	
	Send_Servo_Position(1, pos1); //Servona i ben 6
	Send_Servo_Position(3, pos2);
	Send_Servo_Position(5, pos3);
}

void Send_Servo_Position_And_Velocity(unsigned char ID, unsigned int pos, unsigned int vel)
{
	//Dela upp hastigheter i LS respektive MS Byte
	unsigned char velLS = vel; //Plockar ut LS Byte av hastigheten för det första servot (översta servot)
	unsigned char velMS = (vel>>8); //Som ovan men med MS Byte
	
	//Dela upp hastigheter i LS respektive MS Byte
	unsigned char posLS = pos; //Plockar ut LS Byte av hastigheten för det första servot (översta servot)
	unsigned char posMS = (pos>>8); //Som ovan men med MS Byte
	
	unsigned char message[8];
	message[0] = ID;
	message[1] = 0x07;	//Längd 
	message[2] = 0x03;  //Skriv
	message[3] = 0x1E;	//Skriv där postion börjar
	message[4] = posLS; //position LS byte 
	message[5] = posMS; //position MS byte
	message[6] = velLS; //hastighet LS byte
	message[7] = velMS; //hastighet LS byte
	Send_Servo_Message(message, 5);
}

void Send_Leg1_Kar_And_Velocity(float x, float y, float z, unsigned int inner, unsigned int middle, unsigned int outer)
{
	//invers kinematik 
	triple_uint pos = Kar_To_Pos(x, y, z);
	//Servo-offsets och ev. invertering 
	unsigned int pos1 = -(pos.a - FIRST_SERVO_OFFSET) + 0x99 + 0x01FF;
	unsigned int pos2 = -(pos.b - SECOND_SERVO_OFFSET) + 0x01FF;
	unsigned int pos3 = -(pos.c - THIRD_SERVO_OFFSET) + 0x01FF;
	
	Send_Servo_Position_And_Velocity(8,pos1,inner); //Servona i ben 1
	Send_Servo_Position_And_Velocity(10, pos2,middle);
	Send_Servo_Position_And_Velocity(12, pos3,outer);
}

//Konverterar de givna kart. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg2_Kar_And_Velocity(float x, float y, float z, unsigned int inner, unsigned int middle, unsigned int outer)
{
	//invers kinematik 
	triple_uint pos = Kar_To_Pos(x, y, z);
	//Servo-offsets och ev. invertering 
	unsigned int pos1 = pos.a - FIRST_SERVO_OFFSET - 0x99 + 0x01FF;
	unsigned int pos2 = pos.b - SECOND_SERVO_OFFSET + 0x01FF;
	unsigned int pos3 = pos.c - THIRD_SERVO_OFFSET + 0x01FF;
	
	Send_Servo_Position_And_Velocity(7, pos1, inner); //Servona i ben 2
	Send_Servo_Position_And_Velocity(9, pos2, middle);
	Send_Servo_Position_And_Velocity(11, pos3, outer);
}

//Konverterar de givna kart. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg3_Kar_And_Velocity(float x, float y, float z, unsigned int inner, unsigned int middle, unsigned int outer)
{
	//invers kinematik 
	triple_uint pos = Kar_To_Pos(x, y, z);
	//Servo-offsets och ev. invertering 
	unsigned int pos1 = -(pos.a - FIRST_SERVO_OFFSET) + 0x01FF;
	unsigned int pos2 = -(pos.b - SECOND_SERVO_OFFSET) + 0x01FF;
	unsigned int pos3 = -(pos.c - THIRD_SERVO_OFFSET) + 0x01FF;

	Send_Servo_Position_And_Velocity(14, pos1, inner); //Servona i ben 3
	Send_Servo_Position_And_Velocity(16, pos2, middle);
	Send_Servo_Position_And_Velocity(18, pos3, outer);
}

//Konverterar de givna kart. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg4_Kar_And_Velocity(float x, float y, float z, unsigned int inner, unsigned int middle, unsigned int outer)
{	
	//invers kinematik 
	triple_uint pos = Kar_To_Pos(x, y, z);
	//Servo-offsets och ev. invertering 
	unsigned int pos1 = pos.a - FIRST_SERVO_OFFSET + 0x01FF;
	unsigned int pos2 = pos.b - SECOND_SERVO_OFFSET + 0x01FF;
	unsigned int pos3 = pos.c - THIRD_SERVO_OFFSET + 0x01FF;
	
	Send_Servo_Position_And_Velocity(13, pos1, inner); //Servona i ben 4
	Send_Servo_Position_And_Velocity(15, pos2, middle);
	Send_Servo_Position_And_Velocity(17, pos3, outer);
}

//Konverterar de givna kart. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg5_Kar_And_Velocity(float x, float y, float z, unsigned int inner, unsigned int middle, unsigned int outer)
{
	//invers kinematik 
	triple_uint pos = Kar_To_Pos(x, y, z);
	//Servo-offsets och ev. invertering 
	unsigned int pos1 = -(pos.a - FIRST_SERVO_OFFSET) - 0x99 + 0x01FF;
	unsigned int pos2 = -(pos.b - SECOND_SERVO_OFFSET) + 0x01FF;
	unsigned int pos3 = -(pos.c - THIRD_SERVO_OFFSET) + 0x01FF;
	
	Send_Servo_Position_And_Velocity(2, pos1, inner); //Servona i ben 5
	Send_Servo_Position_And_Velocity(4, pos2, middle);
	Send_Servo_Position_And_Velocity(6, pos3, outer);
}

//Konverterar de givna kart. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg6_Kar_And_Velocity(float x, float y, float z, unsigned int inner, unsigned int middle, unsigned int outer)
{
	//invers kinematik 
	triple_uint pos = Kar_To_Pos(x, y, z);
	//Servo-offsets och ev. invertering 
	unsigned int pos1 = pos.a - FIRST_SERVO_OFFSET + 0x99 + 0x01FF;
	unsigned int pos2 = pos.b - SECOND_SERVO_OFFSET + 0x01FF;
	unsigned int pos3 = pos.c - THIRD_SERVO_OFFSET + 0x01FF;
	
	Send_Servo_Position_And_Velocity(1, pos1, inner); //Servona i ben 6
	Send_Servo_Position_And_Velocity(3, pos2, middle);
	Send_Servo_Position_And_Velocity(5, pos3, outer);
}

//Konverterar de givna cyl. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg1_Cyl_And_Velocity(float r, float th, float z, unsigned int inner, unsigned int middle, unsigned int outer)
{
	//Ben-offset och koordinatbyte
	float x = r*sin(th) - 6;
	float y = r*cos(th) - 12;
	//invers kinematik 
	triple_uint pos = Kar_To_Pos(x, y, z);
	//Servo-offsets och ev. invertering 
	unsigned int pos1 = -(pos.a - FIRST_SERVO_OFFSET) + 0x99 + 0x01FF;
	unsigned int pos2 = -(pos.b - SECOND_SERVO_OFFSET) + 0x01FF;
	unsigned int pos3 = -(pos.c - THIRD_SERVO_OFFSET) + 0x01FF;
	
	Send_Servo_Position_And_Velocity(8,pos1, inner); //Servona i ben 1
	Send_Servo_Position_And_Velocity(10, pos2, middle);
	Send_Servo_Position_And_Velocity(12, pos3, outer);
}

//Konverterar de givna cyl. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg2_Cyl_And_Velocity(float r, float th, float z, unsigned int inner, unsigned int middle, unsigned int outer)
{
	//Ben-offset och koordinatbyte
	float x = -r*sin(th) - 6;
	float y = r*cos(th) - 12; 
	//invers kinematik 
	triple_uint pos = Kar_To_Pos(x, y, z);
	//Servo-offsets och ev. invertering 
	unsigned int pos1 = pos.a - FIRST_SERVO_OFFSET - 0x99 + 0x01FF;
	unsigned int pos2 = pos.b - SECOND_SERVO_OFFSET + 0x01FF;
	unsigned int pos3 = pos.c - THIRD_SERVO_OFFSET + 0x01FF;
	
	Send_Servo_Position_And_Velocity(7, pos1, inner); //Servona i ben 2
	Send_Servo_Position_And_Velocity(9, pos2, middle);
	Send_Servo_Position_And_Velocity(11, pos3,outer);
}

//Konverterar de givna cyl. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg3_Cyl_And_Velocity(float r, float th, float z, unsigned int inner, unsigned int middle, unsigned int outer)
{
	//Ben-offset och koordinatbyte
	float x = r*sin(th) - 10; 
	float y = r*cos(th);
	//invers kinematik 
	triple_uint pos = Kar_To_Pos(x, y, z);
	//Servo-offsets och ev. invertering 
	unsigned int pos1 = -(pos.a - FIRST_SERVO_OFFSET) + 0x01FF;
	unsigned int pos2 = -(pos.b - SECOND_SERVO_OFFSET) + 0x01FF;
	unsigned int pos3 = -(pos.c - THIRD_SERVO_OFFSET) + 0x01FF; 

	Send_Servo_Position_And_Velocity(14, pos1, inner); //Servona i ben 3
	Send_Servo_Position_And_Velocity(16, pos2, middle);
	Send_Servo_Position_And_Velocity(18, pos3, outer);
}

//Konverterar de givna cyl. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg4_Cyl_And_Velocity(float r, float th, float z, unsigned int inner, unsigned int middle, unsigned int outer)
{
	//Ben-offset och koordinatbyte
	float x = -r*sin(th) - 10;
	float y = r*cos(th);
	//invers kinematik 
	triple_uint pos = Kar_To_Pos(x, y, z);
	//Servo-offsets och ev. invertering 
	unsigned int pos1 = pos.a - FIRST_SERVO_OFFSET + 0x01FF;
	unsigned int pos2 = pos.b - SECOND_SERVO_OFFSET + 0x01FF;
	unsigned int pos3 = pos.c - THIRD_SERVO_OFFSET + 0x01FF;
	
	Send_Servo_Position_And_Velocity(13, pos1, inner); //Servona i ben 4
	Send_Servo_Position_And_Velocity(15, pos2, middle);
	Send_Servo_Position_And_Velocity(17, pos3, outer);
}

//Konverterar de givna cyl. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg5_Cyl_And_Velocity(float r, float th, float z, unsigned int inner, unsigned int middle, unsigned int outer)
{
	//Ben-offset och koordinatbyte
	float x = r*sin(th) - 6;
	float y = r*cos(th) + 12;
	//invers kinematik 
	triple_uint pos = Kar_To_Pos(x, y, z);
	//Servo-offsets och ev. invertering 
	unsigned int pos1 = -(pos.a - FIRST_SERVO_OFFSET) - 0x99 + 0x01FF;
	unsigned int pos2 = -(pos.b - SECOND_SERVO_OFFSET) + 0x01FF;
	unsigned int pos3 = -(pos.c - THIRD_SERVO_OFFSET) + 0x01FF;
	
	Send_Servo_Position_And_Velocity(2, pos1, inner); //Servona i ben 5
	Send_Servo_Position_And_Velocity(4, pos2, middle);
	Send_Servo_Position_And_Velocity(6, pos3, outer);
}

//Konverterar de givna cyl. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg6_Cyl_And_Velocity(float r, float th, float z, unsigned int inner, unsigned int middle, unsigned int outer)
{
	//Ben-offset och koordinatbyte
	float x = -r*sin(th) - 6;
	float y = r*cos(th) + 12;
	//invers kinematik 
	triple_uint pos = Kar_To_Pos(x, y, z);
	//Servo-offsets och ev. invertering 
	unsigned int pos1 = pos.a - FIRST_SERVO_OFFSET + 0x99 + 0x01FF;
	unsigned int pos2 = pos.b - SECOND_SERVO_OFFSET + 0x01FF;
	unsigned int pos3 = pos.c - THIRD_SERVO_OFFSET + 0x01FF;
	
	Send_Servo_Position_And_Velocity(1, pos1, inner); //Servona i ben 6
	Send_Servo_Position_And_Velocity(3, pos2, middle);
	Send_Servo_Position_And_Velocity(5, pos3, outer);
}
