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
void USART_Transmit_Array(unsigned char array[], uint8_t size);
void USART_Flush(void);

//Avbrottsrutin som körs då enheten har skickat data.
ISR(USART0_TX_vect)
{
	
}

//Avbrottsrutin som körs då det finns inkommande data.
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

//Sätter upp alla nödvändiga register för BT
void initBluetooth()
{
	//Enl. datablad 14.7456MHz och 115200BPS
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
	
	//Töm bufferten
	USART_Flush();
}

//Skicka ett tecken
void USART_Transmit(unsigned char data)
{
	while(!(UCSR0A & (1<<UDRE0)));
	
	UDR0 = data;
	
}

//Ta emot ett tecken
unsigned char USART_Recieve(void)
{
	while(!(UCSR0A & (1<<RXC0)));
	
	return UDR0;
	
}

//Skickar en lista med tecken
void USART_Transmit_Array(unsigned char array[], uint8_t size)
{
	int i;
	for(i; i < size; ++i)
	{
		USART_Transmit(array[i]);
	}
}

//töm databufferten
void USART_Flush(void)
{
	unsigned char dummy;
	while(UCSR0A & (1<<RXC0)) dummy = UDR0;
}

//Mainfunktion för komm-cpu
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
	
	unsigned char packet[16] = { 66,111,111,116,105,110,103,46,46,46,32,32,32,32,32,32};
	
	USART_Transmit_Array(packet, 16);
  
   while(1);
}

