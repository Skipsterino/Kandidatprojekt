/*
 * Styrprojekt.c
 *
 * Created: 4/5/2016 8:44:50 AM
 *  Author: erilj291
 */ 

#define F_CPU 16000000UL		// 16 MHz
#define BAUD_PRESCALER 0		
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <math.h>
#include <util/delay.h>

typedef struct{
	float a;
	float b;
	float c;
} triple_float;

typedef struct{
	unsigned char a;
	unsigned char b;
	unsigned char c;
} triple_uchar;

typedef struct{
	unsigned char a;
	unsigned char b;
} double_uchar;

void Init(void);
void USART_Init(unsigned long);
void USART_Transmit(unsigned char);
unsigned char USART_Receive( void );
void USART_Flush(void); 
unsigned char checksum_calc(unsigned char param[], uint8_t num_of_par);
void Send_Servo_Message(unsigned char message[], uint8_t num_of_par);
void Send_Servo_Delaytime(void);
void Send_Servo_Position(unsigned char ID_and_pos[]);
void Send_Servo_LED(void);
triple_uchar Kar_To_Pos(float kar[]);
triple_float Pos_To_Kar(unsigned char pos[]);
triple_float create_triple_float(float a, float b, float c);
triple_uchar create_triple_uchar(unsigned char a, unsigned char b, unsigned char c);
double_uchar create_double_float(unsigned char a, unsigned char b);
void Send_Servo_Velocity(unsigned char ID_and_vel[]);
double_uchar Get_Servo_Position(unsigned char ID); 


	

//unsigned char message[]* Receive_Servo_Statuspackage(uint8_t size);

//Dessa ska INTE vara globala sen (om de används senare)! Är globala endast vid testning så komp. inte slänger dem pga oanvända
//unsigned char r1; //I dessa lagras statuspaketets bytes
//unsigned char r2;
//unsigned char r3;
//unsigned char r4;
//unsigned char r5;
//unsigned char r6;
	
int main(void)
{
	//INIT
	DDRD = 1<<DDD2;
	DDRC = 0; //JTAG, alla väljs till ingångar
	DDRB = (1<<DDB3) | (1<<DDB4) | (1<<DDB5) | (0<<DDB6) | (1<<DDB7); //SPI, allt ut förutom PB6
	USART_Init(BAUD_PRESCALER);
	sei(); //Aktivera avbrott öht (MSB=1 i SREG)
	//KÖR FÖLJANDE FUNKTIONER NÄR SERVONA BEHÖVER KALIBRERAS PÅ NÅGOT SÄTT
	//Send_Servo_Delaytime();
	//Send_Servo_LED();
	
	unsigned char servos[9] = {12,10,8,18,16,14, 6,4,2};
	for (uint8_t i = 0; i < 9; i++)
	{		
		unsigned char ID_and_position[3] = {servos[i], 0x90, 0x01};
		Send_Servo_Position(ID_and_position);
		_delay_ms(1000);
	}
	while(1)
	{
    }
}

void USART_Init( unsigned long prescaler )
{
	//Se till att baud-inställningarna blir rätt
	UCSR0A &= ~(1<<U2X0);
	/* Set baud rate prescaler*/
	UBRR0H = (prescaler>>8);
	UBRR0L = prescaler;
	/* Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0); //(X) Motsvarar den bortkommenterade raden lägst upp
	/* Set frame format: 8data, 1stop bit */
	UCSR0C = (0<<USBS0)|(3<<UCSZ00);
}

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


// id, length, instruktion,startposition, och parametrar i en array och ta med antalet parametrar som en uint8 
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

//Räknar ut kartesiska koordinater (kar) mha servoposition (pos)
triple_float Pos_To_Kar(unsigned char pos[])
{
	float pos_to_ang_scale = 0.0051182676011; //Skalfaktor mellan pos och vinkel i RADIANER
	unsigned char first_servo_offset = 0x01; //Lika med 0?
	unsigned char second_servo_offset = 0x01; //Lika med 0?
	unsigned char third_servo_offset = 0x01; //Skild från 0
	float leg_offset_x = 15;
	float leg_offset_y = 10;
	float leg_offset_z = 0; 
	
	float L1 = 5; //Benlängder i cm
	float L2 = 7; 
	float L3 = 12;
	 
	unsigned char p1 = pos[1];
	unsigned char p2 = pos[2];
	unsigned char p3 = pos[3];
		
	float v1= pos_to_ang_scale * (float)(p1 - 0x1FF + first_servo_offset);
	float v2= pos_to_ang_scale * (float)(p2 - 0x1FF + second_servo_offset);
	float v3= pos_to_ang_scale * (float)(p3 - 0x1FF + third_servo_offset);
	 
	float x = L1*sin(v1) + L2*sin(v1)*cos(v2) + L3*cos(v1)*cos(v2 + v3) + leg_offset_x;
	float y = L1*cos(v1) + L2*cos(v1)*cos(v2) + L3*cos(v1)*cos(v2 + v3) + leg_offset_y;
	float z = -L2*sin(v2) - L3*sin(v2 + v3) + leg_offset_z;
	triple_float kar = create_triple_float(x,y,z); 
	return kar;
}

triple_uchar Kar_To_Pos(float kar[])
{
	float ang_to_pos_scale = 195.37860814;
	float pi = 3.1415926536;
	
	unsigned char first_servo_offset = 0x01; //Lika med 0?
	unsigned char second_servo_offset = 0x01; //Lika med 0?
	unsigned char third_servo_offset = 0x01; //Skild från 0
	
	float L1 = 5; //Benlängder i cm
	float L2 = 7;
	float L2_power_of_2 = 49;
	float L3 = 12;
	float L3_power_of_2 = 144;
	
	float leg_offset_x = 15;
	float leg_offset_y = 10;

	
	float x = kar[0];
	float y = kar[1];
	float z = kar[2];
	
	//hjälpvariabler
	float r_leg = sqrt(pow((x-leg_offset_x),2)+pow((y-leg_offset_y),2));
	float r_outer_leg = r_leg - L1;
	float r_outer_leg_power_of_2 = pow(r_outer_leg,2); 
	float z_power_of_2= pow(z,2);
	float big_ass_thing = asin((L3_power_of_2-r_outer_leg_power_of_2-z_power_of_2-L2_power_of_2)/(-2*L2)/sqrt(r_outer_leg_power_of_2+z_power_of_2));

	//inverskinematik
	float v1 = asin((y-leg_offset_y)/r_leg);
	float v2 = big_ass_thing + atan(r_outer_leg/z);
	float v3 = pi -big_ass_thing + asin((L2_power_of_2-r_outer_leg_power_of_2-z_power_of_2-L3_power_of_2)/(-2*L3)/sqrt(r_outer_leg_power_of_2+z_power_of_2));

	 
	unsigned char pos1 =(unsigned char)(ang_to_pos_scale*v1) + 0x1FF + first_servo_offset;
	unsigned char pos2 =(unsigned char)(ang_to_pos_scale*v2) + 0x1FF + second_servo_offset;
	unsigned char pos3 =(unsigned char)(ang_to_pos_scale*v3) + 0x1FF + third_servo_offset; 
	triple_uchar pos = create_triple_uchar(pos1, pos2, pos3);
	return pos; 
	
}

triple_float create_triple_float(float a, float b, float c)
{
	triple_float this; 
	this.a = a;
	this.b = b;
	this.c = c;
	return this; 
}

triple_uchar create_triple_uchar(unsigned char a, unsigned char b, unsigned char c)
{
	triple_uchar this;
	this.a = a;
	this.b = b;
	this.c = c;
	return this;
}


double_uchar create_double_uchar(unsigned char a, unsigned char b)
{
	double_uchar this;
	this.a = a;
	this.b = b;
	return this;
}

//Konfigurerar portar, initierar UART och aktiverar avbrott öht
void Init(void)
{
	DDRD = 1<<DDD2;
	DDRC = 0; //JTAG, alla väljs till ingångar
	DDRB = (1<<DDB3) | (1<<DDB4) | (1<<DDB5) | (0<<DDB6) | (1<<DDB7); //SPI, allt ut förutom PB6
	
	USART_Init(BAUD_PRESCALER);
	
	sei(); //Aktivera avbrott öht (MSB=1 i SREG)
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

//Konfigurerar alla servon med vinkelbegränsningar
void Send_Servo_Angle_Limit(void)
{
	for (uint8_t i = 1; i < 19; i++)
	{
		unsigned char return_delay_time[] = {i, 0x06, 0x03, 0x06, 0x2D, mpohfgl,dgkmr};
		}
		Send_Servo_Message(return_delay_time, 2);
		
		_delay_ms(500);
	}
}


		
		
