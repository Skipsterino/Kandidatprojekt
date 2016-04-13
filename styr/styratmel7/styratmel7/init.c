#include "init.h"

//Konfigurerar portar, initierar UART och aktiverar avbrott �ht
void Init(void)
{
	DDRD = 1<<DDD2;
	DDRC = 0; //JTAG, alla v�ljs till ing�ngar
	DDRB = (1<<DDB3) | (1<<DDB4) | (1<<DDB5) | (0<<DDB6) | (1<<DDB7); //SPI, allt ut f�rutom PB6
	
	USART_Init(BAUD_PRESCALER);
}

void USART_Init( unsigned long prescaler )
{
	//Se till att baud-inst�llningarna blir r�tt
	UCSR0A &= ~(1<<U2X0);
	/* Set baud rate prescaler*/
	UBRR0H = (prescaler>>8);
	UBRR0L = prescaler;
	/* Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0); //(X) Motsvarar den bortkommenterade raden l�gst upp
	/* Set frame format: 8data, 1stop bit */
	UCSR0C = (0<<USBS0)|(3<<UCSZ00);
}