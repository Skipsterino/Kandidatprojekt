/*
 * servo_UART.c
 *
 * Created: 4/8/2016 3:04:25 PM
 *  Author: chrma018
 */ 
#include "servo_UART.h"

void USART_Transmit( unsigned char data )
{
	// Wait for empty transmit buffer
	while ( !( UCSR0A & (1<<UDRE0)) );
	// Put data into buffer, sends the data
	UDR0 = data;
}

unsigned char USART_Receive( void )
{
	// Wait for data to be received
	while ( !(UCSR0A & (1<<RXC0)) );
	// Get and return received data from buffer
	return UDR0;
}

//Beräknar checksum
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

//Konfigurerar alla servon med rätt return delay time
void Send_Servo_Delaytime(void)
{
	for (uint8_t i = 1; i < 19; i++)
	{
		unsigned char return_delay_time[] = {i, 0x04, 0x03, 0x05, 0x2D};
		Send_Servo_Message(return_delay_time, 2);
		
		_delay_ms(500);
	}
}

//Konfigurerar alla servon så att deras LED-lampor blinkar så fort något slags fel har uppstått
void Send_Servo_LED(void)
{
	for (uint8_t i = 1; i < 19; i++)
	{
		unsigned char LED_settings[] = {i, 0x04, 0x03, 0x11, 0xFF};
		Send_Servo_Message(LED_settings, 2);
		
		_delay_ms(500);
	}
}

//Konfigurerar alla servon med vinkelbegränsningar
void Send_Servo_Angle_Limit(void)
{
	uint8_t inner_middle[] = {13,14};
	uint8_t inner_lf_rb[] = {1,8};
	uint8_t inner_rf_lb[] = {2,7};
	uint8_t middle[] = {3,4,9,10,15,16};
	uint8_t outer_left[] = {6,12,18};
	uint8_t outer_right[] = {5,11,17};

	for (uint8_t i = 0; i < 6; i++)
	{
		unsigned char limits[] = {middle[i], 0x07, 0x03, 0x06, 0xCD, 0x00, 0X31, 0x03}; // pos 1FF -+ 1024/300pos/vinkl*90grader= 0x00CD,0x0331
		Send_Servo_Message(limits, 5);
		_delay_ms(500);
	}
	for (uint8_t i = 0; i < 3; i++)
	{
		unsigned char limits[] = {outer_left[i], 0x07, 0x03, 0x06, 0x55, 0x01, 0x31, 0x03}; // pos 1FF + (1024/300pos/vinkl*(-50grader eller + 90 grader)= 0x0155,0x0331
		Send_Servo_Message(limits, 2);
		_delay_ms(500);
		unsigned char limits2 [] = {outer_right[i], 0x07, 0x03, 0x06, 0xCD, 0x00, 0xA9, 0x02}; // pos 1FF + (1024/300pos/vinkl*(+50grader eller - 90 grader)= 0x00CD,0x02A9
		Send_Servo_Message(limits2, 2);
		_delay_ms(500);
	}
	for (uint8_t i = 0; i < 2; i++)
	{
		unsigned char limits[] = {inner_lf_rb[i], 0x07, 0x03, 0x06, 0x55, 0x01, 0x31, 0x03}; // pos 1FF + (1024/300pos/vinkl*(-60grader eller + 45 grader)= 0x01EE,0x0298
		Send_Servo_Message(limits, 2);
		_delay_ms(500);
		unsigned char limits2 [] = {inner_middle[i], 0x07, 0x03, 0x06, 0x55, 0x01, 0x31, 0x03}; // pos 1FF + (1024/300pos/vinkl*(-15grader eller + 15 grader)= 0x01CC,0x0232
		Send_Servo_Message(limits2, 2);
		_delay_ms(500);
		unsigned char limits3 [] = {inner_rf_lb[i], 0x07, 0x03, 0x06, 0x55, 0x01, 0x31, 0x03}; // pos 1FF + (1024/300pos/vinkl*(-45grader eller + 60 grader)= 0x0166,0x02CB
		Send_Servo_Message(limits3, 2);
		_delay_ms(500);
	}
}

// id,instruktion,startposition, och parametrar i en array och ta med antalet parametrar som en uint8
void Send_Servo_Message(unsigned char message[], uint8_t num_of_par)
{
	PORTD |= 1<<PORTD2; //Välj riktning "till servon" i tri-state
	unsigned char checksum = checksum_calc(message, num_of_par);

	USART_Transmit(0xFF); //2 st Startbytes
	USART_Transmit(0xFF);

	// här skickas meddelandet (exkl checksum)
	for(uint8_t i=0; i < num_of_par+3; i++)
	{
		USART_Transmit(message[i]);
	}
	
	cli(); //Deaktivera avbrott så överföringen avslutas korrekt
	USART_Transmit(checksum); //Checksum
	while(!( UCSR0A & (1<<TXC0))); //Vänta på att överföringen klar
	_delay_ms(0.06); //Lite extra tidsmarginal så överföringen verkligen hinner bli klar innan riktning ändras
	PORTD &= ~(1<<PORTD2); //Välj riktning "från servon" i tri-state
	sei(); //Aktivera avbrott igen
}

// Skickar önskad position till servo. Inargument = (ID, position) OBS! LS Byte först...
void Send_Servo_Position(unsigned char ID_and_pos[])
{
	unsigned char message[6];
	message[0] = ID_and_pos[0];
	message[1] = 0x05;
	message[2] = 0x03;
	message[3] = 0x1E;
	message[4] = ID_and_pos[1];
	message[5] = ID_and_pos[2];
	
	Send_Servo_Message(message, 3);
}

//Skickar önskad hastighet till servo. Inargument = (ID, hastighet) OBS! LS Byte först...
void Send_Servo_Velocity(unsigned char ID_and_vel[])
{
	unsigned char message[6];
	message[0] = ID_and_vel[0];
	message[1] = 0x05;
	message[2] = 0x03;
	message[3] = 0x20;
	message[4] = ID_and_vel[1];
	message[5] = ID_and_vel[2];
	
	Send_Servo_Message(message, 3);
}

// Hämtar positionen hos servo med angivet ID, returnerar som en double_uchar.
double_uchar Get_Servo_Position(unsigned char ID) //FUNKAR ATT RETURNERA SÅHÄR?
{
	unsigned char message[6];
	unsigned char position_LSByte;
	unsigned char position_MSByte;
	
	message[0] = ID;
	message[1] = 0x04;
	message[2] = 0x02;
	message[3] = 0x24; //Läser ut Present Position (önskas istället Goal Position får man ändra här till 0x1E)
	message[4] = 0x02;
	
	Send_Servo_Message(message, 2);
	
	USART_Receive(); //Första startbyten
	USART_Receive(); //Andra startbyten
	USART_Receive(); //ID
	USART_Receive(); //Längd
	USART_Receive(); //Error
	position_LSByte = USART_Receive(); //LS Byte av positionen
	position_MSByte = USART_Receive(); //MS Byte av positionen
	USART_Receive(); //Checksum

	double_uchar position = create_double_uchar(position_LSByte, position_MSByte);
	return position;
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








