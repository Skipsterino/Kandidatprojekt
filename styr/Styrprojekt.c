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

int main(void)
{
	//init
	DDRD = 1<<DDD2;
	DDRC = 0; //JTAG, alla väljs till ingångar
	DDRB = (1<<DDB3) | (1<<DDB4) | (1<<DDB5) | (0<<DDB6) | (1<<DDB7); //SPI, allt ut förutom PB6
	
	USART_Init(BAUD_PRESCALER);
	
	sei(); //Aktivera avbrott öht (MSB=1 i SREG)
	
	unsigned char r1; //I dessa lagras statuspaketets bytes
	unsigned char r2;
	unsigned char r3;
	unsigned char r4;
	unsigned char r5;
	unsigned char r6;
	
	
	unsigned char servos[3] = {12,10,8};
	
	for (int i = 0; i < 3; i++)
	{
		unsigned char message1[] = {servos[i], 0x04, 0x03, 0x05,0x00};
		Send_Servo_Message(message1, 2);
		
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
		r1 = USART_Receive();
		r2 = USART_Receive();
		r3 = USART_Receive();
		r4 = USART_Receive();
		r5 = USART_Receive();
		r6 = USART_Receive();
		
		//_delay_ms(1);
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
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) )
	;
	/* Put data into buffer, sends the data */
	UDR0 = data;
}

unsigned char USART_Receive( void )
{
	/* Wait for data to be received */
	while ( !(UCSR0A & (1<<RXC0)) )
	;
	/* Get and return received data from buffer */
	return UDR0;
}


// id,instruktion,startpostition, och parametrar i en array och ta med antalet parametrar som en unit8 
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
	PORTD &= ~(1<<PORTD2); //Välj riktning "från servon" i tri-state
	sei(); //Aktivera avbrott igen
}

//unsigned char Receive_Servo_Statuspackage(void)
//{
	//PORTD &= ~(1<<PORTD2); //Välj riktning "från servon" i tri-state
	//int i; 
	//while(???)
	//{
		//message[i] = USART_Receive();
		//i++;
	//}
//}

unsigned char checksum_calc(unsigned char message[], uint8_t num_of_par)
{
	uint8_t sum = 0;
	for (int i = 0; i< num_of_par+3; i++)
	{
		sum += message[i];
	}
	uint8_t checksum = ~sum;
	return (unsigned char)checksum;
}
