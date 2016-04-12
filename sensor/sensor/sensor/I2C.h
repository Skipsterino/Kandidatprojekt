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

void I2C_start();
void I2C_stop();

int error();

int i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data);
int i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);

#endif /* I2C_H_ */
