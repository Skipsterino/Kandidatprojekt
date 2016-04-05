/*
 * komm_modul.c
 *
 * Created: 2016-04-04 15:31:53
 * Author : Joakim
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>


#define F_CPU 14745600UL
#define BAUD_RATE 115200

unsigned char USART_Recieve();
void USART_Transmit(unsigned char data);
void USART_Flush(void);

ISR(USART0_TX_vect)
{
	
}

ISR(USART0_RX_vect)  
{
	unsigned char data = 0;
	data = USART_Recieve();
	USART_Transmit(data);
}

void initLCD()
{
	
}

void initSPI()
{
	
}

void initBluetooth()
{
	uint16_t ubrr_val = 7;
	
	//Sätt baud rate prescaler
	UBRR0L = ubrr_val;
	UBRR0H = (ubrr_val>>8);
	
	//Aktivera RxD och TxD samt aktivera avbrott på dessa
	UCSR0B = 0b11011000;
	
	//Async
	//No parity
	//1 stop bit
	//8 char size
	UCSR0C = 0b00000110;
	
	USART_Flush();
}

void USART_Transmit(unsigned char data)
{
	while(!(UCSR0A & (1<<UDRE0)));
	
	UDR0 = data;
	
}

unsigned char USART_Recieve(void)
{
	while(!(UCSR0A & (1<<RXC0)));
	
	return UDR0;
	
}

void USART_Flush(void)
{
	unsigned char dummy;
	while(UCSR0A & (1<<RXC0)) dummy = UDR0;
}

int main(void)
{
	cli();
	//Initiera LCD
	initLCD();
	
	//Initiera buss	
	initSPI();
	
	//Initiera BT
	initBluetooth();
	
	sei();
	//Sleep
	
	//unsigned char data = USART_Recieve();
	
	//USART_Transmit(4);
	
	
    /* Replace with your application code */
    while (1) 
    {
    }
}

