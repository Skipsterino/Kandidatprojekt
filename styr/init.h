#ifndef INIT_H_
#define INIT_H_

#ifndef BAUD_PRESCALER
#define BAUD_PRESCALER 0
#endif

#include <avr/io.h>

void Init(void);
void USART_Init( unsigned long prescaler );

#endif /* INIT_H_ */