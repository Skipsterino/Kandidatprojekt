#include "init.h"
#include "structs.h"
#include "SPI.h"
#include "servo_UART.h"
#include "invers_kinematik.h"
	
int main(void)
{
	Init();
	USART_Init(BAUD_PRESCALER);
	SPI_init_master();
	//KÖR FÖLJANDE FUNKTIONER NÄR SERVONA BEHÖVER KALIBRERAS PÅ NÅGOT SÄTT
	//Send_Servo_Delaytime();
	//Send_Servo_LED();
	//Send_Servo_Angle_Limit();
	sei(); //Aktivera avbrott öht (MSB=1 i SREG)
	
	triple_float kartesiska = create_triple_float(23, 2, 2);
	six_uchar positioner = Kar_To_Pos(kartesiska);
	unsigned char message7[]= {0x11, positioner.a, positioner.b};
	Send_Servo_Position(message7);
	_delay_ms(1);
	//unsigned char message6[] = {0x09, positioner.c, positioner.d};
	//Send_Servo_Position(message6);
	//_delay_ms(1);
	//unsigned char message11[] = {0x0B, positioner.e, positioner.f};
	//Send_Servo_Position(message11);
	//_delay_ms(1);
			//
		 
	
	
	//unsigned char message1[] = {0x0D, 0xFF, 0x01};
	//Send_Servo_Position(message1);
	//unsigned char message2[] = {0x0F, 0xD0, 0x01};
	//Send_Servo_Position(message2);
	//unsigned char message3[] = {0x12, 0x55, 0x01};
	//Send_Servo_Position(message3);
	
	
	
	while(1)
	{
    }
}
