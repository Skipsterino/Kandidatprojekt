/*
* USART.c
*
* Created: 2016-04-06 09:08:09
*  Author: Joakim
*/

#include "USART.h"

uint8_t BTcounter; 

//Sätter upp alla nödvändiga register för BT
void initUSART()
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
	
	BTcounter = 0;
}

//Ta emot ett tecken
unsigned char USART_Recieve(void)
{
	while(!(UCSR0A & (1<<RXC0)));
	
	return UDR0;
	
}

//Skicka ett tecken
void USART_Transmit(unsigned char data)
{
	while(!(UCSR0A & (1<<UDRE0)));
	
	UDR0 = data;
	
}

//Skickar en lista med tecken
void USART_Transmit_Array(unsigned char array[], uint8_t size)
{
	int i = 0;
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

//Avbrottsrutin som körs då enheten har skickat data.
ISR(USART0_TX_vect)
{
	
}

//Avbrottsrutin som körs då det finns inkommande data.
ISR(USART0_RX_vect)
{
	toSPI[BTcounter] = USART_Recieve();
	
	if(BTcounter < 15){
		++BTcounter;
	}
	else{
		BTcounter = 0;
		SPDR = toSPI[0];
		//char digits[16];
		//sprintf(digits,"%d", toSPI[0]);
		//LCD_controller_put_line(1,digits);
		BTtimeout = 0;
	}
	/*unsigned char data = 0;
	data = USART_Recieve();
	USART_Transmit(data);*/
}
