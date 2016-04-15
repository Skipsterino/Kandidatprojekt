#include "init.h"

//Konfigurerar portar, initierar UART och initierar SPI
void Init(void)
{
	DDRD = 1<<DDD2;
	DDRC = 0; //JTAG, alla v�ljs till ing�ngar
	DDRB = (1<<DDB3) | (1<<DDB4) | (1<<DDB5) | (0<<DDB6) | (1<<DDB7); //SPI, allt ut f�rutom PB6
	
	PORTD |= 1<<PORTD2; //V�lj riktning "till servon" i tri-state
	
	USART_Init();
	SPI_init_master();
}

void USART_Init(void)
{
	//Se till att baud-inst�llningarna blir r�tt
	UCSR0A &= ~(1<<U2X0);
	//S�tt baud-prescaler=0 s� att baud-rate blir 1MBPS. M�STE HA 1MBPS ty servona har det!!!
	UBRR0H = 0;
	UBRR0L = 0;
	//St�ll in processorn som s�ndare och mottagare p� bussen
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	//8data, 1 stopbit. M�STE HA 1 stopbit ty servona vill ha det!!!
	UCSR0C = (0<<USBS0)|(3<<UCSZ00);
}