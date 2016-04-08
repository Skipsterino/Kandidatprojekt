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

#define SS_sen 3
#define SS_kom 4


//structs som används i koden
typedef struct{
	float a;
	float b;
	float c;
} triple_float;

typedef struct{
	unsigned char a;
	unsigned char b;
	unsigned char c;
	unsigned char d;
	unsigned char e;
	unsigned char f;
} six_uchar;

typedef struct{
	unsigned char a;
	unsigned char b;
} double_uchar;

//Deklarationer
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
void Send_Servo_Velocity(unsigned char ID_and_vel[]);
double_uchar Get_Servo_Position(unsigned char ID);

six_uchar Kar_To_Pos(triple_float kar);
triple_float Pos_To_Kar(six_uchar pos);

triple_float create_triple_float(float a, float b, float c);
six_uchar create_six_uchar(unsigned char a, unsigned char b, unsigned char c, unsigned char d, unsigned char e, unsigned char f);
double_uchar create_double_uchar(unsigned char a, unsigned char b);

void SPI_init_master();
void SPI_sen_transmit_master();
void Set_SS_sen_kom(uint8_t toSen, uint8_t toKom);

//Dessa ska INTE vara globala sen (om de används senare)! Är globala endast vid testning så komp. inte slänger dem pga oanvända
//unsigned char r1; //I dessa lagras statuspaketets bytes
//unsigned char r2;
//unsigned char r3;
//unsigned char r4;
//unsigned char r5;
//unsigned char r6;
	
int main(void)
{
	Init();
	USART_Init(BAUD_PRESCALER);
	SPI_init_master();
	//KÖR FÖLJANDE FUNKTIONER NÄR SERVONA BEHÖVER KALIBRERAS PÅ NÅGOT SÄTT
	//Send_Servo_Delaytime();
	Send_Servo_LED();
	//Send_Servo_Angle_Limit();
	////Send_Servo_Position({0x07,0x66,0x01});
	//_delay_ms(1000);
	////Send_Servo_Position({0x07,0xFF,0x01});
	//_delay_ms(1000);
	////Send_Servo_Position({0x07,0x66,0x01});
	//_delay_ms(1000);
	////Send_Servo_Position({0x07,0xFF,0x01});
	//_delay_ms(1000);
	//Send_Servo_Position({0x07,0x65,0x01});

	sei(); //Aktivera avbrott öht (MSB=1 i SREG)
	

	//unsigned char servos[9] = {12,10,8,18,16,14, 6,4,2};
	//for (uint8_t i = 0; i < 9; i++)
	//{		
		//unsigned char ID_and_position[3] = {servos[i], 0x90, 0x01};
		//Send_Servo_Position(ID_and_position);
		//_delay_ms(1000);
	//}
	
	while(1)
	{
    }
}

//Konfigurerar portar, initierar UART och aktiverar avbrott öht
void Init(void)
{
	DDRD = 1<<DDD2;
	DDRC = 0; //JTAG, alla väljs till ingångar
	DDRB = (1<<DDB3) | (1<<DDB4) | (1<<DDB5) | (0<<DDB6) | (1<<DDB7); //SPI, allt ut förutom PB6
	
	USART_Init(BAUD_PRESCALER);
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

//Räknar ut kartesiska koordinater (kar) mha servoposition (pos). Returnerar kar som en triple_float.
triple_float Pos_To_Kar(six_uchar pos)
{
	float pos_to_ang_scale = 0.0051182676011; //Skalfaktor mellan pos och vinkel (som mäts i RADIANER)
	unsigned int first_servo_offset = 0x01; //Lika med 0?
	unsigned int second_servo_offset = 0x01; //Lika med 0?
	unsigned int third_servo_offset = 0x01; //Skild från 0
	float leg_offset_x = 15;
	float leg_offset_y = 10;
	float leg_offset_z = 0; 
	
	float L1 = 5; //Benlängder i cm
	float L2 = 7; 
	float L3 = 12;
	
	//Plocka fram LS resp. MS Byte av servopositionerna
	unsigned char p1LS = pos.a;
	unsigned char p1MS = pos.b;
	unsigned char p2LS = pos.c;
	unsigned char p2MS = pos.d;
	unsigned char p3LS = pos.e;
	unsigned char p3MS = pos.f;
	
	//Kombinera dessa till unsigned ints ty passar bättre för nedanstående beräkningar
	unsigned int p1 = p1MS; //OBS, här hamnar MS på LS-platserna i p1, varför nedanstående skiftning görs så MS hamnar rätt i p1
	p1 = (p1<<8);
	p1 |= (unsigned int)p1LS; //Adderar till LS på LS-platserna i p1
	unsigned int p2 = p2MS; 
	p2 = (p2<<8);
	p2 |= (unsigned int)p2LS;
	unsigned int p3 = p3MS;
	p3 = (p3<<8);
	p3 |= (unsigned int)p3LS;
		
	float v1= pos_to_ang_scale * (float)(p1 - 0x1FF + first_servo_offset);
	float v2= pos_to_ang_scale * (float)(p2 - 0x1FF + second_servo_offset);
	float v3= pos_to_ang_scale * (float)(p3 - 0x1FF + third_servo_offset);
	 
	float x = L1*sin(v1) + L2*sin(v1)*cos(v2) + L3*cos(v1)*cos(v2 + v3) + leg_offset_x;
	float y = L1*cos(v1) + L2*cos(v1)*cos(v2) + L3*cos(v1)*cos(v2 + v3) + leg_offset_y;
	float z = -L2*sin(v2) - L3*sin(v2 + v3) + leg_offset_z;
	triple_float kar = create_triple_float(x,y,z); 
	return kar;
}

//Räknar ut servopositioner (pos) mha kartesiska koordinater (kar). Returnerar positionerna som en six_uchar. LS Byte först för varje servoposition! (pos1LS, pos1MS,...)
six_uchar Kar_To_Pos(triple_float kar)
{
	float ang_to_pos_scale = 195.37860814;
	float pi = 3.1415926536;
	
	unsigned int first_servo_offset = 0x01; //Lika med 0?
	unsigned int second_servo_offset = 0x01; //Lika med 0?
	unsigned int third_servo_offset = 0x01; //Skild från 0
	
	float L1 = 5; //Benlängder i cm
	float L2 = 7;
	float L2_sq = 49;
	float L3 = 12;
	float L3_sq = 144;
	
	float leg_offset_x = 15;
	float leg_offset_y = 10;
	
	float x = kar.a;
	float y = kar.b;
	float z = kar.c;
	
	//Hjälpvariabler
	float r_leg = sqrt(pow((x-leg_offset_x),2)+pow((y-leg_offset_y),2));
	float r_outer_leg = r_leg - L1;
	float r_outer_leg_sq = pow(r_outer_leg,2); 
	float z_sq= pow(z,2);
	float big_ass_thing = asin((L3_sq-r_outer_leg_sq-z_sq-L2_sq)/(-2*L2)/sqrt(r_outer_leg_sq+z_sq));

	//Invers kinematik
	float v1 = asin((y-leg_offset_y)/r_leg);
	float v2 = big_ass_thing + atan(r_outer_leg/z);
	float v3 = pi -big_ass_thing + asin((L2_sq-r_outer_leg_sq-z_sq-L3_sq)/(-2*L3)/sqrt(r_outer_leg_sq+z_sq));

	//Spara först positionerna som unsigned int (beräkningarna passar bättre med att hantera både LS och MS byte samtidigt som en unsigned int) 
	unsigned int pos1 =(unsigned int)(ang_to_pos_scale*v1) + 0x1FF + first_servo_offset;
	unsigned int pos2 =(unsigned int)(ang_to_pos_scale*v2) + 0x1FF + second_servo_offset;
	unsigned int pos3 =(unsigned int)(ang_to_pos_scale*v3) + 0x1FF + third_servo_offset;
	
	//Dela upp positionen i LS respektive MS Byte, spara i unsigned chars (unsigned chars passar bättre vid kontakt med servona)
	unsigned char pos1LS = pos1; //Plockar ut LS Byte av positionen för det första servot (översta servot) 
	unsigned char pos1MS = (pos1>>8); //Som ovan men med MS Byte
	unsigned char pos2LS = pos2;
	unsigned char pos2MS = (pos2>>8);
	unsigned char pos3LS = pos3;
	unsigned char pos3MS = (pos3>>8);
	
	six_uchar pos = create_six_uchar(pos1LS, pos1MS, pos2LS, pos2MS, pos3LS, pos3MS);
	return pos; 
}

//Returnerar en triple_float bestående av tre givna floats.
triple_float create_triple_float(float a, float b, float c)
{
	triple_float this; 
	this.a = a;
	this.b = b;
	this.c = c;
	return this; 
}

//Returnerar en six_uchar bestående av sex givna unsigned chars.
six_uchar create_six_uchar(unsigned char a, unsigned char b, unsigned char c, unsigned char d, unsigned char e, unsigned char f)
{
	six_uchar this;
	this.a = a;
	this.b = b;
	this.c = c;
	this.d = d;
	this.e = e;
	this.f = f;
	return this;
}

//Returnerar en double_uchar bestående av två givna unsigned chars.
double_uchar create_double_uchar(unsigned char a, unsigned char b)
{
	double_uchar this;
	this.a = a;
	this.b = b;
	return this;
}

//Konfigurerar alla servon med vinkelbegränsningar
//void Send_Servo_Angle_Limit(void)
//{
	//uint8_t inners_middle[] = {13,14};
	//uint8_t inners_lf_rb[] = {1,8};
	//uint8_t inners_rf_lb[] = {2,7};
	//uint8_t middle[] = {3,4,9,10,15,16};
	//uint8_t outer_left[] = {6,12,18};
	//uint8_t outer_right[] = {5,11,17};
//
	//
	//for (uint8_t i = 0; i < 6; i++)
	//{
		//unsigned char return_delay_time[] = {middle[i], 0x07, 0x03, 0x06, 0xCD, 0x00, 0X31, 0x03}; // pos 1FF -+ 1024/300pos/vinkl*90grader= 0x00CD,0x0331
		//Send_Servo_Message(return_delay_time, 5);
		//_delay_ms(500);
	//}
	//for (uint8_t i = 0; i < 3; i++) 
	//{
		//unsigned char return_delay_time[] = {outer_left[i], 0x07, 0x03, 0x06, 0x55 0x01, 0x31 0x03}; // pos 1FF + (1024/300pos/vinkl*(-50grader eller + 90 grader)= 0x0155,0x0331
		//Send_Servo_Message(return_delay_time, 2);
		//_delay_ms(500);
		//unsigned char return_delay_time[] = {outer_right[i], 0x07, 0x03, 0x06, 0xCD, 0x00, 0xA9, 0x02}; // pos 1FF + (1024/300pos/vinkl*(+50grader eller - 90 grader)= 0x00CD,0x02A9
		//Send_Servo_Message(return_delay_time, 2);
		//_delay_ms(500);
	//}
	//for (uint8_t i = 0; i < 2; i++) 
	//{
		//unsigned char return_delay_time[] = {inner_lf_rb[i], 0x07, 0x03, 0x06, 0x55 0x01, 0x31 0x03}; // pos 1FF + (1024/300pos/vinkl*(-60grader eller + 45 grader)= 0x01EE,0x0298
		//Send_Servo_Message(return_delay_time, 2);
		//_delay_ms(500);
		//unsigned char return_delay_time[] = {inner_middle[i], 0x07, 0x03, 0x06, 0x55 0x01, 0x31 0x03}; // pos 1FF + (1024/300pos/vinkl*(-15grader eller + 15 grader)= 0x01CC,0x0232
		//Send_Servo_Message(return_delay_time, 2);
		//_delay_ms(500);
		//unsigned char return_delay_time[] = {inner_rf_lb[i], 0x07, 0x03, 0x06, 0x55 0x01, 0x31 0x03}; // pos 1FF + (1024/300pos/vinkl*(-45grader eller + 60 grader)= 0x0166,0x02CB
		//Send_Servo_Message(return_delay_time, 2);
		//_delay_ms(500);
	//}
	//
//}


//////////////////////////////////////////////////////////////////////////
//						KOD FÖR SPI										//
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
unsigned char fromSen[16];
unsigned char fromKom[16];
uint8_t SPIcounter;
uint8_t toSen;
uint8_t toKom;
uint8_t overflow;
// timer0 interruppt funktion
ISR(TIMER0_OVF_vect)
{
	++overflow;
	
	if(overflow >= 2000){
		SPI_sen_transmit_master();
	overflow = 0;
	}
}
void SPI_sen_transmit_master()
{
	SPIcounter = 0;
	//Sätter SS för styr->sen
	toSen = 0;
	toKom = 1;
	Set_SS_sen_kom(toSen, toKom);
	// skickar noll till Sen
	SPDR = fromSen[0];
}
ISR(SPI_STC_vect)
{
	fromSen[0] = 14;
	fromSen[1] = 15;
	fromSen[2] = 36;
	fromSen[3] = 47;

	/*
	if(toSen == 1){
		fromSen[SPIcounter] = SPDR;
		
		if(SPIcounter < 15){
			++SPIcounter;
			SPDR = 0;
		}
		else{
			SPIcounter = 0;
			//sätter SS till styr->kom
			toSen = 0;
			toKom = 1;
			Set_SS_sen_kom(toSen, toKom);
			//Startar styr->kom buss
			SPDR = fromSen[SPIcounter];
		}
	}
	else if(toKom == 1){
		fromKom[SPIcounter] = SPDR;
		
		if(SPIcounter < 15){
			SPDR = fromSen[SPIcounter + 1];
			++SPIcounter;
		}
		else{
			SPIcounter = 0;
			toSen = 0;
			toKom = 0;
			Set_SS_sen_kom(toSen, toKom);
		}
	}
	*/
	if(toKom == 1){
		fromKom[SPIcounter] = SPDR;
		
		if(SPIcounter < 15){
			_delay_ms(1);
			SPDR = fromSen[SPIcounter + 1];
			++SPIcounter;
		}
		else{
			SPIcounter = 0;
			toSen = 0;
			toKom = 0;
			Set_SS_sen_kom(toSen, toKom);
		}
	}
}
void Set_SS_sen_kom(uint8_t toSen, uint8_t toKom)
{
	if(toSen == 1){
		PORTB &= ~(1 << SS_sen);
		PORTB |= (1 << SS_kom);
	}
	else if(toKom == 1){
		PORTB |= (1 << SS_sen);
		PORTB &= ~(1 << SS_kom);
	}
	else{
		PORTB |= (1 << SS_sen);
		PORTB |= (1 << SS_kom);
	}
}
void SPI_init_master()
{
	//MOSI, SCK , SS0 och SS1 är utgångar
	DDRB |= 0b10111000;	
	overflow = 0;
	//SPI Enable
	//SPI Master
	//SPI clock f/16
	//SPI interrupt enable
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0)|(1<<SPIE);
	
	// Enable timer0 interruppt
	TIMSK0 = (1<<TOIE0);
	// set timer0 counter initial value to 0
	TCNT0=0x00;
	// start timer0 with /1024 prescaler
	TCCR0B = (1<<CS02) | (1<<CS00);
	
	SPIcounter = 0;
	uint8_t toSen = 0;
	uint8_t toKom = 0;
	
	Set_SS_sen_kom(toSen, toKom);
}
