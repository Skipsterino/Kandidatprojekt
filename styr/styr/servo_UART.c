
#include "servo_UART.h"
#include "invers_kinematik.h"

#define first_servo_offset 0x00
#define second_servo_offset 0x2F
#define third_servo_offset 0x79

void UART_Transmit( unsigned char data )
{
	// Wait for empty transmit buffer
	while ( !( UCSR0A & (1<<UDRE0)) );
	// Put data into buffer, sends the data
	UDR0 = data;
}

unsigned char UART_Receive( void )
{
	float i=0;
	// Wait for data to be received
	while ((!(UCSR0A & (1<<RXC0))))
	{
		i = i + 1; 
		if(i>50)
		{
			return 0xBB;
		}
	}
	// Get and return received data from buffer
	return UDR0;
}

unsigned char checksum_calc(unsigned char message[], uint8_t num_of_par)
{
	uint8_t sum = 0;
	for (uint8_t i = 0; i< num_of_par+3; i++) //Ta med parametrarna + ID + instruktion + längd
	{
		sum += message[i];
	}
	uint8_t checksum = ~sum;
	return (unsigned char)checksum;
}

void Configure_Servos_Delaytime(void)
{
	for (uint8_t i = 1; i < 19; i++)
	{
		unsigned char return_delay_time[] = {i, 0x04, 0x03, 0x05, 0x2D};
		Send_Servo_Message(return_delay_time, 2);
		
		_delay_ms(1);
	}
}


void Configure_Servos_LED(void)
{
	for (uint8_t i = 1; i < 19; i++)
	{
		unsigned char LED_settings[] = {i, 0x04, 0x03, 0x11, 0xFF};
		Send_Servo_Message(LED_settings, 2);
		
		_delay_ms(1);
	}
}


void Configure_Servos_Angle_Limit(void)
{
	uint8_t inner_middle[] = {13,14};
	uint8_t inner_lf_rb[] = {1,8};
	uint8_t inner_rf_lb[] = {2,7};
	uint8_t middle[] = {3,4,9,10,15,16};
	uint8_t outer_left[] = {6,12,18};
	uint8_t outer_right[] = {5,11,17};
	
	for (uint8_t i = 0; i < 6; i++)
	{
		Send_Servo_Angle_Limit(middle[i], 0x00CD, 0x0331); // pos 1FF -+ 1023/300pos/vinkl*90grader= 0x00CD,0x0331
		_delay_ms(1);
	}
	for (uint8_t i = 0; i < 3; i++)
	{
		Send_Servo_Angle_Limit(outer_left[i], 0x00CD, 0x0287); // pos 1FF + (1023/300pos/vinkl*(+40grader eller - 90 grader)= 0x00CD,0x0287
		_delay_ms(1);
		Send_Servo_Angle_Limit(outer_right[i], 0x0177, 0x0331); // pos 1FF + (1023/300pos/vinkl*(-40grader eller + 90 grader)= 0x0177,0x0331
		_delay_ms(1);
	}
	for (uint8_t i = 0; i < 2; i++)
	{
		Send_Servo_Angle_Limit(inner_lf_rb[i], 0x0166, 0x02CB);  // pos 1FF + (1023/300pos/vinkl*(-45grader eller + 60 grader)= 0x0166,0x02CB
		_delay_ms(1);
		Send_Servo_Angle_Limit(inner_middle[i], 0x019F, 0x025F); // pos 1FF + (1023/300pos/vinkl*(-15grader eller + 15 grader)= 0x01BB,0x0243
		_delay_ms(1);
		Send_Servo_Angle_Limit(inner_rf_lb[i], 0x0131, 0x0298); // pos 1FF + (1023/300pos/vinkl*(-60grader eller + 45 grader)= 0x0131,0x0298
		_delay_ms(1);
	}
		
}


void Configure_Servos_No_Response(void)
{
	for (uint8_t i = 1; i < 19; i++)
	{
		unsigned char response_settings[] = {i, 0x04, 0x03, 0x10, 0x01};
		Send_Servo_Message(response_settings, 2);
		
		_delay_ms(1);
	}
}

void Configure_Servos_Max_Torque(void)
{
	for (uint8_t i = 1; i < 19; i++)
	{
		unsigned char response_settings[] = {i, 0x05, 0x03, 0x0E, 0xFF, 0x01};
		Send_Servo_Message(response_settings, 3);
		
		_delay_ms(1);
	}
}

unsigned int Get_Servo_Load(unsigned char ID)
{	
	unsigned char message[5];
	unsigned char load_LSByte;
	unsigned char load_MSByte;
	unsigned char start_byte1= 0;
	unsigned char start_byte2= 0; 
	unsigned char length; 
	unsigned char error; 
	unsigned char checksum;
	float timer = 0; 
	
	message[0] = ID;
	message[1] = 0x04;
	message[2] = 0x02;
	message[3] = 0x28; //Läser ut Present load 
	message[4] = 0x02;
	
	Send_Servo_Message(message, 2);
	
	
	_delay_ms(0.02); //Lite extra tidsmarginal så bussen hinner bli ledig innan riktning ändras!!!
	PORTD &= ~(1<<PORTD2); //Välj riktning "från servon" i tri-state
	
	while((start_byte1 != 0xFF) && (start_byte2 != 0xFF))
	{
		start_byte2 = start_byte1; 
		start_byte1 = UART_Receive();
		timer = timer + 1; 
		if(timer>10)
		{
			return 0xBB;
		}
	}
	UART_Receive(); //dont know lol
	ID = UART_Receive(); //ID
	length = UART_Receive(); //Längd
	error = UART_Receive(); //Error
	load_LSByte = UART_Receive(); //LS Byte av load
	load_MSByte = UART_Receive(); //MS Byte av load
	checksum = UART_Receive(); //Checksum
	
	_delay_ms(0.05); //Lite extra tidsmarginal så bussen hinner bli ledig innan riktning ändras!!!
	
	unsigned int load = (((unsigned int)load_MSByte) << 8) | load_LSByte;
	PORTD |= 1<<PORTD2; //Välj riktning "till servon" i tri-state
	
	return load;
}

unsigned char Get_Servo_Temp(unsigned char ID)
{	
	unsigned char message[5];
	unsigned char temp;
	unsigned char start_byte1= 0;
	unsigned char start_byte2= 0; 
	unsigned char length; 
	unsigned char error; 
	unsigned char checksum;
	float timer = 0; 
	
	message[0] = ID;
	message[1] = 0x04;
	message[2] = 0x02;
	message[3] = 0x2B; //Läser ut Present load 
	message[4] = 0x01;
	
	Send_Servo_Message(message, 2);
	
	
	_delay_ms(0.02); //Lite extra tidsmarginal så bussen hinner bli ledig innan riktning ändras!!!
	PORTD &= ~(1<<PORTD2); //Välj riktning "från servon" i tri-state
	
	while((start_byte1 != 0xFF) && (start_byte2 != 0xFF))
	{
		start_byte2 = start_byte1; 
		start_byte1 = UART_Receive();
		timer = timer + 1; 
		if(timer>10)
		{
			return 0xBB;
		}
	}
	UART_Receive(); //dont know lol
	ID = UART_Receive(); //ID
	length = UART_Receive(); //Längd
	error = UART_Receive(); //Error
	temp = UART_Receive(); //LS Byte av load
	checksum = UART_Receive(); //Checksum
	
	_delay_ms(0.05); //Lite extra tidsmarginal så bussen hinner bli ledig innan riktning ändras!!!
	
	PORTD |= 1<<PORTD2; //Välj riktning "till servon" i tri-state
	
	return temp;
}


	

void Send_Servo_Velocity(unsigned char ID, unsigned int vel)
{
	//Dela upp hastyigheter i LS respektive MS Byte, spara i unsigned chars (unsigned chars passar bättre vid kontakt med servona)
	unsigned char velLS = vel; //Plockar ut LS Byte av hastigheten för det första servot (översta servot)
	unsigned char velMS = (vel>>8); //Som ovan men med MS Byte
	unsigned char message[6];
	message[0] = ID;
	message[1] = 0x05;	
	message[2] = 0x03;
	message[3] = 0x20;
	message[4] = velLS;
	message[5] = velMS;
	
	Send_Servo_Message(message, 3);
}

void Send_Servo_Angle_Limit(unsigned char ID, unsigned int lower, unsigned int higher)
{
	//Dela upp vinklar i LS respektive MS Byte, spara i unsigned chars (unsigned chars passar bättre vid kontakt med servona)
	unsigned char lowerLS = lower; //Plockar ut LS Byte av vinkel för det första servot (översta servot)
	unsigned char lowerMS = (lower>>8); //Som ovan men med MS Byte
	
	//Dela upp vinklar i LS respektive MS Byte, spara i unsigned chars (unsigned chars passar bättre vid kontakt med servona)
	unsigned char higherLS = higher; //Plockar ut LS Byte av vinkel för det första servot (översta servot)
	unsigned char higherMS = (higher>>8); //Som ovan men med MS Byte
	
	unsigned char limits[] = {ID, 0x07, 0x03, 0x06, lowerLS, lowerMS, higherLS, higherMS};
	Send_Servo_Message(limits, 5);
}



void Send_Inner_P1_Velocity(unsigned int vel)
{	
	unsigned char inner_servos[]={2,8,13};
		
	for (uint8_t i = 0; i < 3; i++)
	{
		Send_Servo_Velocity(inner_servos[i], vel);
		//_delay_ms(1);
	}
}


void Send_Inner_P2_Velocity(unsigned int vel)
{	
	unsigned char inner_servos[]={1,7,14};
		
	for (uint8_t i = 0; i < 3; i++)
	{
		Send_Servo_Velocity(inner_servos[i], vel);
		//_delay_ms(1);
	}
}


void Send_Middle_P1_Velocity(unsigned int vel)
{	
	unsigned char middle_servos[]={4,10,15};
		
	for (uint8_t i = 0; i < 3; i++)
	{
		Send_Servo_Velocity(middle_servos[i], vel);
		//_delay_ms(1);
	}
}


void Send_Middle_P2_Velocity(unsigned int vel)
{	
	unsigned char middle_servos[]={9,16,3};
		
	for (uint8_t i = 0; i < 3; i++)
	{
		Send_Servo_Velocity(middle_servos[i], vel);
		//_delay_ms(1);
	}
}


void Send_Outer_P1_Velocity(unsigned int vel)
{	
	unsigned char outer_servos[]={6,12,17};
		
	for (uint8_t i = 0; i < 3; i++)
	{
		Send_Servo_Velocity(outer_servos[i], vel);
		//_delay_ms(1);
	}
}


void Send_Outer_P2_Velocity(unsigned int vel)
{	
	unsigned char outer_servos[]={5,11,18};
		
	for (uint8_t i = 0; i < 3; i++)
	{
		Send_Servo_Velocity(outer_servos[i], vel);
		//_delay_ms(1);
	}
}


void Send_Servo_Message(unsigned char message[], uint8_t num_of_par)
{
	//PORTD |= 1<<PORTD2; //Välj riktning "till servon" i tri-state
	unsigned char checksum = checksum_calc(message, num_of_par);
	cli();
	UART_Transmit(0xFF); //2 st Startbytes
	UART_Transmit(0xFF);

	// här skickas meddelandet (exkl checksum)
	for(uint8_t i=0; i < num_of_par+3; i++)
	{
		UART_Transmit(message[i]);
	}
	
	//cli(); //Deaktivera avbrott så överföringen avslutas korrekt. BEHÖVS EJ???
	UART_Transmit(checksum); //Checksum
	while(!( UCSR0A & (1<<TXC0))); //Vänta på att överföringen klar (redo att skicka ny data)
	_delay_ms(0.06); //Lite extra tidsmarginal så överföringen verkligen hinner bli klar innan riktning ändras!!!
	//PORTD &= ~(1<<PORTD2); //Välj riktning "från servon" i tri-state
	sei(); //Aktivera avbrott igen. BEHÖVS EJ???
}

//Skickar önskad position till servo. Inargument = (ID, position) OBS! LS Byte först...
void Send_Servo_Position(unsigned char ID, unsigned int pos)
{
	//Dela upp positionen i LS respektive MS Byte, spara i unsigned chars (unsigned chars passar bättre vid kontakt med servona)
	unsigned char posLS = pos; //Plockar ut LS Byte av positionen för det första servot (översta servot)
	unsigned char posMS = (pos>>8); //Som ovan men med MS Byte
	unsigned char message[6];
	message[0] = ID;
	message[1] = 0x05;
	message[2] = 0x03;
	message[3] = 0x1E;
	message[4] = posLS;
	message[5] = posMS;
	
	Send_Servo_Message(message, 3);
}



//FUNKAR EJ PGA KAN EJ RETURNERA STRUC
//// Tar emot ett meddelande från ett servon. Retunerar(ID, Length, Error, Parametrar).
//unsigned char[] Receive_Servo_Statuspackage(uint8_t size)
//{
//unsigned char message[];
//unsigned char checksum;
//unsigned char Start1 = USART_Receive();
//unsigned char Start2 = USART_Receive();
//unsigned char ID = USART_Receive();
//unsigned char length = USART_Receive();
//
//for(int i=2; i < length + 1; i++)
//{
//message[i] = USART_Receive();
//}
//checksum = USART_Receive();
//
//message[0] = ID;
//message[1] = length;
//
//// Följande rader är en kontroll av checksum. Använd vid behov
////checksum = checksum + ID + length;
////for(i=1; i < length; i++)
////{
////checksum = checksum + message[i];
////}
////if(checksum != 0x0F)
////{
////KASTA SKIT ELLER VAD MAN NU KAN GORA I C
////}
//
//message[0] = ID;
//message[1] = length;
//
//return message;
//}

//void Check_Servo_Temp(unsigned char ID)
//{
//unsigned char message[5];
//unsigned char temp;
//unsigned char maxtemp = 130;
//
//message[0] = ID;
//message[1] = 0x04;
//message[2] = 0x02;
//message[3] = 0x2B; //Läser ut Present Temp
//message[4] = 0x01;
//
//Send_Servo_Message(message, 2);
//
//_delay_ms(0.06); //Lite extra tidsmarginal så överföringen verkligen hinner bli klar innan riktning ändras!!!
//PORTD &= ~(1<<PORTD2); //Välj riktning "från servon" i tri-state
//
//USART_Receive(); //Första startbyten
//USART_Receive(); //Andra startbyten
//USART_Receive(); //ID
//USART_Receive(); //Längd
//USART_Receive(); //Error
//temp = USART_Receive(); // Temp
//USART_Receive(); //Checksum
//
//if(temp > maxtemp)
//{
////Do Stuff
//}
//}
unsigned int pos11;
unsigned int pos12;
unsigned int pos13;
unsigned int pos31;
unsigned int pos32;
unsigned int pos33;
unsigned int pos51;
unsigned int pos52;
unsigned int pos53;

unsigned int pos21;
unsigned int pos22;
unsigned int pos23;
unsigned int pos41;
unsigned int pos42;
unsigned int pos43;
unsigned int pos61;
unsigned int pos62;
unsigned int pos63;
//Konverterar de givna kart. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg1_Kar(float x, float y, float z)
{
	triple_uint pos = Kar_To_Pos(x, y, z);

	 pos11 = -(pos.a - first_servo_offset) + 0x99 + 0x01FF;
	 pos12 = -(pos.b - second_servo_offset) + 0x01FF;
	 pos13 = -(pos.c - third_servo_offset) + 0x01FF;
	
	Send_Servo_Position(8, pos11);
	//_delay_ms(1);
	Send_Servo_Position(10, pos12);
	//_delay_ms(1);
	Send_Servo_Position(12, pos13);
	//_delay_ms(1);
}

//Konverterar de givna kart. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg2_Kar(float x, float y, float z)
{
	triple_uint pos = Kar_To_Pos(x, y, z);
	
	 pos21 = pos.a - first_servo_offset - 0x99 + 0x01FF;
	pos22 = pos.b - second_servo_offset + 0x01FF;
	 pos23 = pos.c - third_servo_offset + 0x01FF;
	
	Send_Servo_Position(7, pos21);
	//_delay_ms(1);
	Send_Servo_Position(9, pos22);
	//_delay_ms(1);
	Send_Servo_Position(11, pos23);
	//_delay_ms(1);
}



//Konverterar de givna kart. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg3_Kar(float x, float y, float z)
{
	triple_uint pos = Kar_To_Pos(x, y, z);
	
	pos31 = -(pos.a - first_servo_offset) + 0x01FF;
	pos32 = -(pos.b - second_servo_offset) + 0x01FF;
	pos33 = -(pos.c - third_servo_offset) + 0x01FF;
	

	Send_Servo_Position(14, pos31);
	//_delay_ms(1);
	Send_Servo_Position(16, pos32);
	//_delay_ms(1);
	Send_Servo_Position(18, pos33);
	//_delay_ms(1);
}

//Konverterar de givna kart. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg4_Kar(float x, float y, float z)
{	
	triple_uint pos = Kar_To_Pos(x, y, z);
	
	pos41 = pos.a - first_servo_offset + 0x01FF;
	pos42 = pos.b - second_servo_offset + 0x01FF;
	pos43 = pos.c - third_servo_offset + 0x01FF;
	
	Send_Servo_Position(13, pos41);
	//_delay_ms(1);
	Send_Servo_Position(15, pos42);
	//_delay_ms(1);
	Send_Servo_Position(17, pos43);
	//_delay_ms(1);
}

//Konverterar de givna kart. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg5_Kar(float x, float y, float z)
{
	triple_uint pos = Kar_To_Pos(x, y, z);
	
	pos51 = -(pos.a - first_servo_offset) - 0x99 + 0x01FF;
	pos52 = -(pos.b - second_servo_offset) + 0x01FF;
	pos53 = -(pos.c - third_servo_offset) + 0x01FF;
	
	Send_Servo_Position(2, pos51);
	//_delay_ms(1);
	Send_Servo_Position(4, pos52);
	//_delay_ms(1);
	Send_Servo_Position(6, pos53);
	//_delay_ms(1);
}

//Konverterar de givna kart. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg6_Kar(float x, float y, float z)
{
	triple_uint pos = Kar_To_Pos(x, y, z);
	
	pos61 = pos.a - first_servo_offset + 0x99 + 0x01FF;
	 pos62 = pos.b - second_servo_offset + 0x01FF;
	pos63 = pos.c - third_servo_offset + 0x01FF;
	
	Send_Servo_Position(1, pos61);
	//_delay_ms(1);
	Send_Servo_Position(3, pos62);
	//_delay_ms(1);
	Send_Servo_Position(5, pos63);
	//_delay_ms(1);
}

//Konverterar de givna cyl. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg1_Cyl(float r, float th, float z)
{
	float x = r*sin(th) - 6;
	float y = r*cos(th) - 12;
	triple_uint pos = Kar_To_Pos(x, y, z);

	
	unsigned int pos1 = -(pos.a - first_servo_offset) + 0x99 + 0x01FF;
	unsigned int pos2 = -(pos.b - second_servo_offset) + 0x01FF;
	unsigned int pos3 = -(pos.c - third_servo_offset) + 0x01FF;
	
	Send_Servo_Position(8,pos1);
	//_delay_ms(1);
	Send_Servo_Position(10, pos2);
	//_delay_ms(1);
	Send_Servo_Position(12, pos3);
	//_delay_ms(1);
}

//Konverterar de givna cyl. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg2_Cyl(float r, float th, float z)
{
	float x = -r*sin(th) - 6;
	float y = r*cos(th) - 12; 
	triple_uint pos = Kar_To_Pos(x, y, z);
	
	unsigned int pos1 = pos.a - first_servo_offset - 0x99 + 0x01FF;
	unsigned int pos2 = pos.b - second_servo_offset + 0x01FF;
	unsigned int pos3 = pos.c - third_servo_offset + 0x01FF;
	
	Send_Servo_Position(7, pos1);
	//_delay_ms(1);
	Send_Servo_Position(9, pos2);
	//_delay_ms(1);
	Send_Servo_Position(11, pos3);
	//_delay_ms(1);
}

//Konverterar de givna cyl. koord. till positioner för benets servon och skickar positionerna till servona.
unsigned int pos1fg; //0x019F, 0x025F
void Send_Leg3_Cyl(float r, float th, float z)
{
	float x;
	float y;
	
	x = r*sin(th) - 10; 
	y = r*cos(th);
	triple_uint pos = Kar_To_Pos(x, y, z);
	
	pos1fg = -(pos.a - first_servo_offset) + 0x01FF;
	unsigned int pos2 = -(pos.b - second_servo_offset) + 0x01FF;
	unsigned int pos3 = -(pos.c - third_servo_offset) + 0x01FF;

	Send_Servo_Position(14, pos1fg);
	//_delay_ms(1);
	Send_Servo_Position(16, pos2);
	//_delay_ms(1);
	Send_Servo_Position(18, pos3);
	//_delay_ms(1);
}

//Konverterar de givna cyl. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg4_Cyl(float r, float th, float z)
{
	float x = -r*sin(th) - 10;
	float y = r*cos(th);
	triple_uint pos = Kar_To_Pos(x, y, z);
	
	unsigned int pos1 = pos.a - first_servo_offset + 0x01FF;
	unsigned int pos2 = pos.b - second_servo_offset + 0x01FF;
	unsigned int pos3 = pos.c - third_servo_offset + 0x01FF;
	
	Send_Servo_Position(13, pos1);
	//_delay_ms(1);
	Send_Servo_Position(15, pos2);
	//_delay_ms(1);
	Send_Servo_Position(17, pos3);
	//_delay_ms(1);
}

//Konverterar de givna cyl. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg5_Cyl(float r, float th, float z)
{
	float x = r*sin(th) - 6;
	float y = r*cos(th) + 12;
	triple_uint pos = Kar_To_Pos(x, y, z);
	
	unsigned int pos1 = -(pos.a - first_servo_offset) - 0x99 + 0x01FF;
	unsigned int pos2 = -(pos.b - second_servo_offset) + 0x01FF;
	unsigned int pos3 = -(pos.c - third_servo_offset) + 0x01FF;
	
	Send_Servo_Position(2, pos1);
	//_delay_ms(1);
	Send_Servo_Position(4, pos2);
	//_delay_ms(1);
	Send_Servo_Position(6, pos3);
	//_delay_ms(1);
}

//Konverterar de givna cyl. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg6_Cyl(float r, float th, float z)
{
	float x = -r*sin(th) - 6;
	float y = r*cos(th) + 12;
	triple_uint pos = Kar_To_Pos(x, y, z);
	
	unsigned int pos1 = pos.a - first_servo_offset + 0x99 + 0x01FF;
	unsigned int pos2 = pos.b - second_servo_offset + 0x01FF;
	unsigned int pos3 = pos.c - third_servo_offset + 0x01FF;
	
	Send_Servo_Position(1, pos1);
	//_delay_ms(1);
	Send_Servo_Position(3, pos2);
	//_delay_ms(1);
	Send_Servo_Position(5, pos3);
	//_delay_ms(1);
}

void Send_Servo_Position_And_Velocity(unsigned char ID, unsigned int pos, unsigned int vel)
{
	//Dela upp hastigheter i LS respektive MS Byte, spara i unsigned chars (unsigned chars passar bättre vid kontakt med servona)
	unsigned char velLS = vel; //Plockar ut LS Byte av hastigheten för det första servot (översta servot)
	unsigned char velMS = (vel>>8); //Som ovan men med MS Byte
	
	//Dela upp hastigheter i LS respektive MS Byte, spara i unsigned chars (unsigned chars passar bättre vid kontakt med servona)
	unsigned char posLS = pos; //Plockar ut LS Byte av hastigheten för det första servot (översta servot)
	unsigned char posMS = (pos>>8); //Som ovan men med MS Byte
	
	unsigned char message[8];
	message[0] = ID;
	message[1] = 0x07;	
	message[2] = 0x03;
	message[3] = 0x1E;
	message[4] = posLS;
	message[5] = posMS;
	message[6] = velLS;
	message[7] = velMS;
	Send_Servo_Message(message, 5);
}



void Send_Leg1_Kar_And_Velocity(float x, float y, float z, unsigned int inner, unsigned int middle, unsigned int outer)
{
	triple_uint pos = Kar_To_Pos(x, y, z);

	unsigned int pos1 = -(pos.a - first_servo_offset) + 0x99 + 0x01FF;
	unsigned int pos2 = -(pos.b - second_servo_offset) + 0x01FF;
	unsigned int pos3 = -(pos.c - third_servo_offset) + 0x01FF;
	
	Send_Servo_Position_And_Velocity(8,pos1,inner);
	//_delay_ms(1);
	Send_Servo_Position_And_Velocity(10, pos2,middle);
	//_delay_ms(1);
	Send_Servo_Position_And_Velocity(12, pos3,outer);
	//_delay_ms(1);
}

//Konverterar de givna kart. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg2_Kar_And_Velocity(float x, float y, float z, unsigned int inner, unsigned int middle, unsigned int outer)
{
	triple_uint pos = Kar_To_Pos(x, y, z);
	
	unsigned int pos1 = pos.a - first_servo_offset - 0x99 + 0x01FF;
	unsigned int pos2 = pos.b - second_servo_offset + 0x01FF;
	unsigned int pos3 = pos.c - third_servo_offset + 0x01FF;
	
	Send_Servo_Position_And_Velocity(7, pos1, inner);
	//_delay_ms(1);
	Send_Servo_Position_And_Velocity(9, pos2, middle);
	//_delay_ms(1);
	Send_Servo_Position_And_Velocity(11, pos3, outer);
	//_delay_ms(1);
}

//Konverterar de givna kart. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg3_Kar_And_Velocity(float x, float y, float z, unsigned int inner, unsigned int middle, unsigned int outer)
{
	triple_uint pos = Kar_To_Pos(x, y, z);
	
	unsigned int pos1 = -(pos.a - first_servo_offset) + 0x01FF;
	unsigned int pos2 = -(pos.b - second_servo_offset) + 0x01FF;
	unsigned int pos3 = -(pos.c - third_servo_offset) + 0x01FF;

	Send_Servo_Position_And_Velocity(14, pos1, inner);
	//_delay_ms(1);
	Send_Servo_Position_And_Velocity(16, pos2, middle);
	//_delay_ms(1);
	Send_Servo_Position_And_Velocity(18, pos3, outer);
	//_delay_ms(1);
}

//Konverterar de givna kart. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg4_Kar_And_Velocity(float x, float y, float z, unsigned int inner, unsigned int middle, unsigned int outer)
{	
	triple_uint pos = Kar_To_Pos(x, y, z);
	
	unsigned int pos1 = pos.a - first_servo_offset + 0x01FF;
	unsigned int pos2 = pos.b - second_servo_offset + 0x01FF;
	unsigned int pos3 = pos.c - third_servo_offset + 0x01FF;
	
	Send_Servo_Position_And_Velocity(13, pos1, inner);
	//_delay_ms(1);
	Send_Servo_Position_And_Velocity(15, pos2, middle);
	//_delay_ms(1);
	Send_Servo_Position_And_Velocity(17, pos3, outer);
	//_delay_ms(1);
}

//Konverterar de givna kart. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg5_Kar_And_Velocity(float x, float y, float z, unsigned int inner, unsigned int middle, unsigned int outer)
{
	triple_uint pos = Kar_To_Pos(x, y, z);
	
	unsigned int pos1 = -(pos.a - first_servo_offset) - 0x99 + 0x01FF;
	unsigned int pos2 = -(pos.b - second_servo_offset) + 0x01FF;
	unsigned int pos3 = -(pos.c - third_servo_offset) + 0x01FF;
	
	Send_Servo_Position_And_Velocity(2, pos1, inner);
	//_delay_ms(1);
	Send_Servo_Position_And_Velocity(4, pos2, middle);
	//_delay_ms(1);
	Send_Servo_Position_And_Velocity(6, pos3, outer);
	//_delay_ms(1);
}

//Konverterar de givna kart. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg6_Kar_And_Velocity(float x, float y, float z, unsigned int inner, unsigned int middle, unsigned int outer)
{
	triple_uint pos = Kar_To_Pos(x, y, z);
	
	unsigned int pos1 = pos.a - first_servo_offset + 0x99 + 0x01FF;
	unsigned int pos2 = pos.b - second_servo_offset + 0x01FF;
	unsigned int pos3 = pos.c - third_servo_offset + 0x01FF;
	
	Send_Servo_Position_And_Velocity(1, pos1, inner);
	//_delay_ms(1);
	Send_Servo_Position_And_Velocity(3, pos2, middle);
	//_delay_ms(1);
	Send_Servo_Position_And_Velocity(5, pos3, outer);
	//_delay_ms(1);
}

//Konverterar de givna cyl. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg1_Cyl_And_Velocity(float r, float th, float z, unsigned int inner, unsigned int middle, unsigned int outer)
{
	float x = r*sin(th) - 6;
	float y = r*cos(th) - 12;
	triple_uint pos = Kar_To_Pos(x, y, z);
	
	unsigned int pos1 = -(pos.a - first_servo_offset) + 0x99 + 0x01FF;
	unsigned int pos2 = -(pos.b - second_servo_offset) + 0x01FF;
	unsigned int pos3 = -(pos.c - third_servo_offset) + 0x01FF;
	
	Send_Servo_Position_And_Velocity(8,pos1, inner);
	//_delay_ms(1);
	Send_Servo_Position_And_Velocity(10, pos2, middle);
	//_delay_ms(1);
	Send_Servo_Position_And_Velocity(12, pos3, outer);
	//_delay_ms(1);
}

//Konverterar de givna cyl. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg2_Cyl_And_Velocity(float r, float th, float z, unsigned int inner, unsigned int middle, unsigned int outer)
{
	float x = -r*sin(th) - 6;
	float y = r*cos(th) - 12; 
	triple_uint pos = Kar_To_Pos(x, y, z);
	
	unsigned int pos1 = pos.a - first_servo_offset - 0x99 + 0x01FF;
	unsigned int pos2 = pos.b - second_servo_offset + 0x01FF;
	unsigned int pos3 = pos.c - third_servo_offset + 0x01FF;
	
	Send_Servo_Position_And_Velocity(7, pos1, inner);
	//_delay_ms(1);
	Send_Servo_Position_And_Velocity(9, pos2, middle);
	//_delay_ms(1);
	Send_Servo_Position_And_Velocity(11, pos3,outer);
	//_delay_ms(1);
}

//Konverterar de givna cyl. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg3_Cyl_And_Velocity(float r, float th, float z, unsigned int inner, unsigned int middle, unsigned int outer)
{
	float x;
	float y;
	
	x = r*sin(th) - 10; 
	y = r*cos(th);
	triple_uint pos = Kar_To_Pos(x, y, z);
	
	unsigned int pos1 = -(pos.a - first_servo_offset) + 0x01FF;
	unsigned int pos2 = -(pos.b - second_servo_offset) + 0x01FF;
	unsigned int pos3 = -(pos.c - third_servo_offset) + 0x01FF;

	Send_Servo_Position_And_Velocity(14, pos1, inner);
	//_delay_ms(1);
	Send_Servo_Position_And_Velocity(16, pos2, middle);
	//_delay_ms(1);
	Send_Servo_Position_And_Velocity(18, pos3, outer);
	//_delay_ms(1);
}

//Konverterar de givna cyl. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg4_Cyl_And_Velocity(float r, float th, float z, unsigned int inner, unsigned int middle, unsigned int outer)
{
	float x = -r*sin(th) - 10;
	float y = r*cos(th);
	triple_uint pos = Kar_To_Pos(x, y, z);
	
	unsigned int pos1 = pos.a - first_servo_offset + 0x01FF;
	unsigned int pos2 = pos.b - second_servo_offset + 0x01FF;
	unsigned int pos3 = pos.c - third_servo_offset + 0x01FF;
	
	Send_Servo_Position_And_Velocity(13, pos1, inner);
	//_delay_ms(1);
	Send_Servo_Position_And_Velocity(15, pos2, middle);
	//_delay_ms(1);
	Send_Servo_Position_And_Velocity(17, pos3, outer);
	//_delay_ms(1);
}

//Konverterar de givna cyl. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg5_Cyl_And_Velocity(float r, float th, float z, unsigned int inner, unsigned int middle, unsigned int outer)
{
	float x = r*sin(th) - 6;
	float y = r*cos(th) + 12;
	triple_uint pos = Kar_To_Pos(x, y, z);
	
	unsigned int pos1 = -(pos.a - first_servo_offset) - 0x99 + 0x01FF;
	unsigned int pos2 = -(pos.b - second_servo_offset) + 0x01FF;
	unsigned int pos3 = -(pos.c - third_servo_offset) + 0x01FF;
	
	Send_Servo_Position_And_Velocity(2, pos1, inner);
	//_delay_ms(1);
	Send_Servo_Position_And_Velocity(4, pos2, middle);
	//_delay_ms(1);
	Send_Servo_Position_And_Velocity(6, pos3, outer);
	//_delay_ms(1);
}

//Konverterar de givna cyl. koord. till positioner för benets servon och skickar positionerna till servona.
void Send_Leg6_Cyl_And_Velocity(float r, float th, float z, unsigned int inner, unsigned int middle, unsigned int outer)
{
	float x = -r*sin(th) - 6;
	float y = r*cos(th) + 12;
	triple_uint pos = Kar_To_Pos(x, y, z);
	
	unsigned int pos1 = pos.a - first_servo_offset + 0x99 + 0x01FF;
	unsigned int pos2 = pos.b - second_servo_offset + 0x01FF;
	unsigned int pos3 = pos.c - third_servo_offset + 0x01FF;
	
	Send_Servo_Position_And_Velocity(1, pos1, inner);
	//_delay_ms(1);
	Send_Servo_Position_And_Velocity(3, pos2, middle);
	//_delay_ms(1);
	Send_Servo_Position_And_Velocity(5, pos3, outer);
	//_delay_ms(1);
}

void Rise_Robot_Height(float height)
{
	triple_float leg1 = Pos_To_Kar(Get_Servo_Position(8), Get_Servo_Position(10), Get_Servo_Position(12));
	triple_float leg2 = Pos_To_Kar(Get_Servo_Position(7), Get_Servo_Position(9), Get_Servo_Position(11));
	triple_float leg3 = Pos_To_Kar(Get_Servo_Position(14), Get_Servo_Position(16), Get_Servo_Position(18));
	triple_float leg4 = Pos_To_Kar(Get_Servo_Position(13), Get_Servo_Position(15), Get_Servo_Position(17));
	triple_float leg5 = Pos_To_Kar(Get_Servo_Position(2), Get_Servo_Position(4), Get_Servo_Position(6));
	triple_float leg6 = Pos_To_Kar(Get_Servo_Position(1), Get_Servo_Position(3), Get_Servo_Position(5));
	
	Send_Leg1_Kar(leg1.a, height, leg1.c);
	Send_Leg2_Kar(leg2.a, height, leg2.c);
	Send_Leg3_Kar(leg3.a, height, leg3.c);
	Send_Leg4_Kar(leg4.a, height, leg4.c);
	Send_Leg5_Kar(leg5.a, height, leg5.c);
	Send_Leg6_Kar(leg6.a, height, leg6.c);
}