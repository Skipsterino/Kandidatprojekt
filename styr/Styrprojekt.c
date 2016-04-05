/*
 * Styrprojekt.c
 *
 * Created: 4/5/2016 8:44:50 AM
 *  Author: erilj291
 */ 

#define F_CPU 16000000UL		// 16 MHz
#define BAUD_RATE 1000000UL		
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <math.h>
#include <util/delay.h>

void USART_Init(unsigned long);
void USART_Transmit(unsigned char);
unsigned char USART_Receive( void );

int main(void)
{
	
	DDRD = 1<<DDD2;
	DDRC = 0; //JTAG, alla väljs till ingångar
	DDRB = (1<<DDB3) | (1<<DDB4) | (1<<DDB5) | (0<<DDB6) | (1<<DDB7); //SPI, allt ut förutom PB6
	
	USART_Init(BAUD_RATE);
	
	sei(); //Aktivera avbrott öht (MSB=1 i SREG)
	
	int r1; //I dessa lagras statuspaketets bytes
	int r2;
	int r3;
	int r4;
	int r5;
	int r6;
	
    while(1)
    {
		PORTD |= 1<<PORTD2; //Välj riktning "till servon" i tri-state
		
		USART_Transmit(0xFF); //2 st Startbytes
		USART_Transmit(0xFF);
		USART_Transmit(0x5); //Väljer random servo
		USART_Transmit(0x5); //Length = #parametrar + 2
		USART_Transmit(0x3); //Instruktion 3, dvs WRITE DATA
		USART_Transmit(0x1E); //Skriv på Goal Position 
		USART_Transmit(0x00); // GoalP LS
		USART_Transmit(0x02); // GoalP MS
		USART_Transmit(~(0x2D)); //Check sum
		
		//PORTD &= ~(1<<PORTD2); //Välj riktning "från servon" i tri-state
		
		//r1 = USART_Receive();
		//r2 = USART_Receive();
		//r3 = USART_Receive();
		//r4 = USART_Receive();
		//r5 = USART_Receive();
		//r6 = USART_Receive();
    }
}

void USART_Init( unsigned long baud )
{
	/* Set baud rate */
	UBRR0H = (unsigned char)(baud>>8);
	UBRR0L = (unsigned char)baud;
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