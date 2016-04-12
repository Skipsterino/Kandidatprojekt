/*
 * I2C.h
 *
 * Created: 4/8/2016 9:46:05 AM
 *  Author: joneh084
 */ 


#ifndef I2C_H_
#define I2C_H_

#include <util/atomic.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define F_CPU 16000000UL

#define I2Cstatus_START 0x08

#define I2Cstatus_MT_SLA_ACK 0x18		// Master Transmitter Mode
#define I2Cstatus_MT_SLA_NACK 0x20
#define I2Cstatus_MT_DATA_ACK 0x28
#define I2Cstatus_MT_DATA_NACK 0x30

#define I2Cstatus_MR_SLA_ACK 0x40		// Master Receiver Mode
#define I2Cstatus_MR_SLA_NACK 0x48
#define I2Cstatus_MR_DATA_ACK 0x50
#define I2Cstatus_MR_DATA_NACK 0x58

#define SLA_W 0xD0
#define SLA_R 0xD1

unsigned long timer0_millis;
unsigned char timer0_fract;
unsigned long timer0_overflow_count;
//#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
//#define clockCyclesToMicroseconds ( ((64 * 256) * 1000L) / (F_CPU / 1000L) )
//#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))
// the whole number of milliseconds per timer0 overflow
#define MILLIS_INC (1024 / 1000)
// the fractional number of milliseconds per timer0 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((1024 % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

void I2C_start();
void I2C_stop();

int error();

int i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data);
int i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);

unsigned long millis();
#define min(a,b) ((a<b)?a:b)

#endif /* I2C_H_ */