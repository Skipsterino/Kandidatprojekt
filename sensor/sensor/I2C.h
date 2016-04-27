/**
* @file I2C.h
* @author Fredrik, Jonas
* @date 26 apr 2016
* @brief The code for the I2C-bus
*
* The code used by the sensor module to communicate with the IMU by the I2C-bus.
*/


#ifndef I2C_H_
#define I2C_H_

#include <util/atomic.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define F_CPU 16000000UL				/**< Define the frequency for the CPU as 16 MHz */

#define I2Cstatus_START 0x08			/**< Define the start state for the I2C-bus */

#define I2Cstatus_MT_SLA_ACK 0x18		/**< Master Transmitter Mode addressing the slave ACK */
#define I2Cstatus_MT_SLA_NACK 0x20		/**< Master Transmitter Mode addressing the slave NACK */
#define I2Cstatus_MT_DATA_ACK 0x28		/**< Master Transmitter Mode sending data ACK */
#define I2Cstatus_MT_DATA_NACK 0x30		/**< Master Transmitter Mode sending data NACK */

#define I2Cstatus_MR_SLA_ACK 0x40		/**< Master Receiver Mode addressing the slave ACK */
#define I2Cstatus_MR_SLA_NACK 0x48		/**< Master Receiver Mode addressing the slave NACK */
#define I2Cstatus_MR_DATA_ACK 0x50		/**< Master Receiver Mode recieving data ACK */
#define I2Cstatus_MR_DATA_NACK 0x58		/**< Master Receiver Mode recieving data NACK */

#define SLA_W 0xD0						/**< Slave address + write */
#define SLA_R 0xD1						/**< Slave address + read  */

/**
* @brief Opens up the I2C-bus
*
* Opens the bus for communication. All transmissions must be started with this command.
* Leaves the data line high.
*/
void I2C_start();

/**
* @brief Shuts down the I2C-bus
*
* Sends the stop command for the bus. Leaves the data line high.
*/
void I2C_stop();

/**
* @brief An error function for the I2C-bus
*
* An error function called if a problem is encountered during I2C-communication. 
*/
int error();

/**
* @brief Writes data to a register at slave
*
* Sends some data using the I2C-bus to a register at the slave address.
* @see I2C-start
* @param slave_addr The address of the slave
* @param reg_addr The register address of the register to be written to at the slave
* @param length The length of the transmission
* @param data The data to be sent
*/
int i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data);

/**
* @brief Reads data from a register at slave
*
* Reads some data from a register at the slave address.
* @see I2C-start
* @param slave_addr The address of the slave
* @param reg_addr The register address of the register to be read from at the slave
* @param length The length of the transmission
* @param data The data recieved
*/
int i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);

#endif /* I2C_H_ */
