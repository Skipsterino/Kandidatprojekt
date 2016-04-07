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

void USART_Init(unsigned long);
void USART_Transmit(unsigned char);
unsigned char USART_Receive( void );
void USART_Flush(void); 
unsigned char checksum_calc(unsigned char param[], uint8_t num_of_par);
void Send_Servo_Message(unsigned char message[], uint8_t num_of_par);
void Send_Servo_Delaytime(void);

//Dessa ska INTE vara globala sen! Är globala endast vid testning så komp. inte slänger dem pga oanvända
unsigned char r1; //I dessa lagras statuspaketets bytes
unsigned char r2;
unsigned char r3;
unsigned char r4;
unsigned char r5;
unsigned char r6;
	
int main(void)
{
	//init
	DDRD = 1<<DDD2;
	DDRC = 0; //JTAG, alla väljs till ingångar
	DDRB = (1<<DDB3) | (1<<DDB4) | (1<<DDB5) | (0<<DDB6) | (1<<DDB7); //SPI, allt ut förutom PB6
	USART_Init(BAUD_PRESCALER);
	sei(); //Aktivera avbrott öht (MSB=1 i SREG)
	
	//Send_Servo_Delaytime();
	
	unsigned char servos[3] = {12,10,8};
		
	////TEST TEST
	//PORTD |= 1<<PORTD2; //Välj riktning "till servon" i tri-state
	//_delay_ms(0.1);
	//PORTD &= ~(1<<PORTD2); //Välj riktning "från servon" i tri-state
	////SLUT TEST TEST
	
	//for (int i = 0; i < 4; i++)
	//unsigned char return_delay_time[] = {servos[0], 0x04, 0x03, 0x05, 0x2D};
	//Send_Servo_Message(return_delay_time, 2);
	//
	
	
	
	while(1)
	{	
		unsigned char return_delay_time[] = {servos[0], 0x04, 0x03, 0x05, 0x1E};  	
		unsigned char send_position_and_velocity[] = {servos[0], 0x07, 0x03, 0x1E, 0x00, 0x02, 0x00, 0x00};

		unsigned char get_temperature[] = {servos[1], 0x04, 0x02, 0x2B, 0x01};
		Send_Servo_Message(send_position_and_velocity, 5);
		
		
		//PORTD |= 1<<PORTD2; //Välj riktning "till servon" i tri-state
		//
		//unsigned char ID = servos[i];
		//unsigned char length = 0x07;
		//unsigned char instr = 0x03;
		//unsigned char param1 = 0x1E;
		//unsigned char param2 = 0x80;
		//unsigned char param3 = 0x01;
		//unsigned char param4 = 0x80;
		//unsigned char param5 = 0x00;
		////Nedan beräknas checksum mha checksum_calc
		//unsigned char param[8] = {ID, length, instr, param1, param2, param3, param4, param5};
		//unsigned char checksum = checksum_calc(param, 8);
		//
		//USART_Transmit(0xFF); //2 st Startbytes
		//USART_Transmit(0xFF);
		//USART_Transmit(ID); //Väljer servo-ID
		//USART_Transmit(length); //Length = #parametrar + 2
		//USART_Transmit(instr); //Instruktion 3, dvs WRITE DATA
		//USART_Transmit(param1); //Skriv på Goal Position
		//USART_Transmit(param2); // GoalP LS byte
		//USART_Transmit(param3); // GoalP MS byte
		//USART_Transmit(param4);
		//USART_Transmit(param5);
		//cli(); //Disable interrupts för att inte överföringen ska ta för lång tid
		//USART_Transmit(checksum); //Checksum

		//while(!( UCSR0A & (1<<TXC0))) //Vänta på att överföringen klar
		//;
		//PORTD &= ~(1<<PORTD2); //Välj riktning "från servon" i tri-state
		//sei(); //Aktivera avbrott igen
		//
		//
		r1 = USART_Receive();
		r2 = USART_Receive();
		r3 = USART_Receive();
		r4 = USART_Receive();
		r5 = USART_Receive();
		r6 = USART_Receive();
		
		//Vänta så skicka OCH ta emot hinner genomföras innan nästa sändning påbörjas
		_delay_ms(1);
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
	while ( !( UCSR0A & (1<<UDRE0)) )
	;
	// Put data into buffer, sends the data
	UDR0 = data;
}

unsigned char USART_Receive( void )
{
	// Wait for data to be received
	while ( !(UCSR0A & (1<<RXC0)) )
	{
	};
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
	for(int i=0; i < num_of_par+3; i++)
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


// Tar emot ett meddelande från ett servon. Retunerar(ID, Length, Error, Parametrar). 
unsigned char[] Receive_Servo_Statuspackage(uint8_t size)
{
	unsigned char message[];
	unsigned char checksum;
	unsigned char Start1 = USART_Receive();
	unsigned char Start2 = USART_Receive();
	unsigned char ID = USART_Receive();
	unsigned char length = USART_Receive();
	
	for(int i=2; i < length + 1; i++)
	{
	message[i] = USART_Receive();
	}
	checksum = USART_Receive();
	
	message[0] = ID;
	message[1] = length;
	
	// Följande rader är en kontroll av checksum. Använd vid behov 
	//checksum = checksum + ID + length; 
	//for(i=1; i < length; i++)
	//{
		//checksum = checksum + message[i];
	//}
	//if(checksum != 0x0F)
	//{
		//KASTA SKIT ELLER VAD MAN NU KAN GORA I C 
	//}
	
	message[0] = ID; 
	message[1] = length;
	
	return message;
}


//Beräknar checksum 
unsigned char checksum_calc(unsigned char message[], uint8_t num_of_par)
{
	uint8_t sum = 0;
	for (int i = 0; i< num_of_par+3; i++) //Ta med parametrarna + ID + instruktion + längd
	{
		sum += message[i];
	}
	uint8_t checksum = ~sum;
	return (unsigned char)checksum;
}

//Konfigurerar alla servon med rätt return delay time
void Send_Servo_Delaytime(void)
{
	for (unsigned char i = 1; i < 19; i++)
	{
		unsigned char return_delay_time[] = {i, 0x04, 0x03, 0x05, 0x2D};
		Send_Servo_Message(return_delay_time, 2);
		
		_delay_ms(500);
	}
}
