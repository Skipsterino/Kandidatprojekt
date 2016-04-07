//Konfigurerar portar, initierar UART och aktiverar avbrott öht
void Init(void)
{
	DDRD = 1<<DDD2;
	DDRC = 0; //JTAG, alla väljs till ingångar
	DDRB = (1<<DDB3) | (1<<DDB4) | (1<<DDB5) | (0<<DDB6) | (1<<DDB7); //SPI, allt ut förutom PB6
	
	USART_Init(BAUD_PRESCALER);
	
	sei(); //Aktivera avbrott öht (MSB=1 i SREG)
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
	
	struct double_uchar position = create_double_uchar(position_LSByte, position_MSByte);
	return position;
}
