/**
* @file servo_UART.h
* @author Erik, Christoffer
* @date 25 apr 2016
* @brief Servo communication 
*
* Functions that send different messages to the servos via UART.  
*
*/


#ifndef SERVO_UART_H_
#define SERVO_UART_H_

#ifndef F_CPU
#define F_CPU 16000000UL	
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "structs.h"


/**
* @brief Transmits one byte on the UART
*
* Sends one byte on the UART
* @param data The data to be transmitted.
*/
void UART_Transmit(unsigned char data);

/**
* @brief Receives one byte from the UART
*
* Tries to receive one byte on the UART
* Fail returned after reached time-limit 
*
* @return The received data, 0xBB if fail. 
*/
volatile unsigned char UART_Receive(void);

/**
* @brief Calculates the checksum
*
* @param param Message written in the servoprotocol 
* without start and stopp bytes 
* @param num_of_param Number of parameters in the transfer 
*
* @return The checksum, which is the final byte in a transmitt. 
*/
unsigned char checksum_calc(unsigned char param[], uint8_t num_of_par);

/**
* @brief Configures the servos response delaytime
*/
void Configure_Servos_Delaytime(void);

/**
* @brief Configures the servos LED messages 
*
* Configures which inforamtion that is transmitted on the servo LED-lights
*/
void Configure_Servos_LED(void);

/**
* @brief Configures the servo angle limits
*
* Configures servo angle limitations in different ways for different values of mode.
* The value 'r' indicates "regular mode", i.e. corridor, turn etc.
* The value 'c' indicates "climbing mode", i.e. climbing on top of low obstacles.
* If a servo is told to exceed a limit, it will move to the limit, i.e. as close as possible.
*
* @param mode Indicates which angle limits is to be used. Can take on the values 'r' and 'c'.
*/
void Configure_Servos_Angle_Limit(char mode);

/**
* @brief Configures the servo responses
*
* Configures servos only to respond when explicitly asked to. 
*/
void Configure_Servos_No_Response(void);

/**
* @brief Configures maximum torque
*
* Configures the maximum torque that each servo can support.
*/
void Configure_Servos_Max_Torque(void);

/**
* @brief Gets the current load on a specific servo
* 
* @return The load as an int. 0x0000 is min 0x03FF i max
*/
unsigned int Get_Servo_Load(unsigned char ID);

/**
* @brief Gets the current temperature of a specific servo
*
* @return The temperature as an unsigned char in celsius
*/
unsigned char Get_Servo_Temp(unsigned char ID);


//unsigned char[] Receive_Servo_Statuspackage(uint8_t size); FUNKAR EJ TY KAN EJ RETURNERA STRUCT
//void Check_Servo_Temp(unsigned char ID)

/**
* @brief Sends a message to the servos
*
* Order of message: ID, Lenght, Instruction, Parameter1, Parameter2 ... ... ParameterN
*
* @param message The message written accordingly to servo protocol without the start and stopp bytes. 
* @param num_of_par The number of parameters in the message (OBS != the length of message).
*/
void Send_Servo_Message(unsigned char message[], uint8_t num_of_par);

/**
* @brief Sends a position to a servo
*
* @param ID The ID of the specific servo 
* @param pos The desired position of the servo. 0x000 is min 0x03FF is max 
*/
void Send_Servo_Position(unsigned char ID, unsigned int pos);

/**
* @brief Sends a velicity to a servo
*
* @param ID The ID of the specific servo
* @param vel The desired velocity of the servo. 0x000 is min 0x03FF is max
*/
void Send_Servo_Velocity(unsigned char ID, unsigned int vel);

/**
* @brief Sends a angle limitation to a servo
*
* If a servo is told to exceed these limits, it will go as close as possible
* 
* @param ID The ID of the specific servo
* @param lower The desired lower angle limit of the servo. 0x000 is min 0x03FF is max
* @param higher The desired higher angle limit of the servo. 0x000 is min 0x03FF is max
*/
void Send_Servo_Angle_Limit(unsigned char ID, unsigned int lower, unsigned int higher);

/**
* @brief Sends a velicity to Servo 6, 12, 17
*
* @param vel The desired velocity of the servos. 0x000 is min 0x03FF is max
*/
void Send_Outer_P1_Velocity(unsigned int vel); 

/**
* @brief Sends a velicity to Servo 5, 11, 18
*
* @param vel The desired velocity of the servos. 0x000 is min 0x03FF is max
*/
void Send_Outer_P2_Velocity(unsigned int vel); 

/**
* @brief Sends a velicity to Servo 4, 10, 15
*
* @param vel The desired velocity of the servos. 0x000 is min 0x03FF is max
*/
void Send_Middle_P1_Velocity(unsigned int vel); 

/**
* @brief Sends a velicity to Servo 3, 9, 16 
*
* @param vel The desired velocity of the servos. 0x000 is min 0x03FF is max
*/
void Send_Middle_P2_Velocity(unsigned int vel); 

/**
* @brief Sends a velicity to Servo 2, 8, 13
*
* @param vel The desired velocity of the servos. 0x000 is min 0x03FF is max
*/
void Send_Inner_P1_Velocity(unsigned int vel); 

/**
* @brief Sends a velicity to Servo 1, 7, 14
*
* @param vel The desired velocity of the servos. 0x000 is min 0x03FF is max
*/
void Send_Inner_P2_Velocity(unsigned int vel); 

/**
* @brief Sends desired position to leg-1 in Cartesian koodinates
*
* @param x The length in x-direction in cm
* @param y The length in y-direction in cm
* @param z The length in z-direction in cm
*/
void Send_Leg1_Kar(float,float,float);

/**
* @brief Sends desired position to leg-2 in Cartesian koodinates
*
* @param x The length in x-direction in cm
* @param y The length in y-direction in cm
* @param z The length in z-direction in cm
*/
void Send_Leg2_Kar(float,float,float);

/**
* @brief Sends desired position to leg-3 in Cartesian koodinates
*
* @param x The length in x-direction in cm
* @param y The length in y-direction in cm
* @param z The length in z-direction in cm
*/
void Send_Leg3_Kar(float,float,float);

/**
* @brief Sends desired position to leg-4 in Cartesian koodinates
*
* @param x The length in x-direction in cm
* @param y The length in y-direction in cm
* @param z The length in z-direction in cm
*/
void Send_Leg4_Kar(float,float,float);

/**
* @brief Sends desired position to leg-5 in Cartesian koodinates
*
* @param x The length in x-direction in cm
* @param y The length in y-direction in cm
* @param z The length in z-direction in cm
*/
void Send_Leg5_Kar(float,float,float);

/**
* @brief Sends desired position to leg-6 in Cartesian koodinates
*
* @param x The length in x-direction in cm
* @param y The length in y-direction in cm
* @param z The length in z-direction in cm
*/
void Send_Leg6_Kar(float,float,float);

/**
* @brief Sends desired position to leg-1 in Cylindrical koodinates
*
* @param r The length in r-direction in cm
* @param th The angles in th-direction in radians
* @param z The length in z-direction in cm
*/
void Send_Leg1_Cyl(float,float,float);

/**
* @brief Sends desired position to leg-2 in Cylindrical koodinates
*
* @param r The length in r-direction in cm
* @param th The angles in th-direction in radians
* @param z The length in z-direction in cm
*/
void Send_Leg2_Cyl(float,float,float);

/**
* @brief Sends desired position to leg-3 in Cylindrical koodinates
*
* @param r The length in r-direction in cm
* @param th The angles in th-direction in radians
* @param z The length in z-direction in cm
*/
void Send_Leg3_Cyl(float,float,float);

/**
* @brief Sends desired position to leg-4 in Cylindrical koodinates
*
* @param r The length in r-direction in cm
* @param th The angles in th-direction in radians
* @param z The length in z-direction in cm
*/
void Send_Leg4_Cyl(float,float,float);

/**
* @brief Sends desired position to leg-5 in Cylindrical koodinates
*
* @param r The length in r-direction in cm
* @param th The angles in th-direction in radians
* @param z The length in z-direction in cm
*/
void Send_Leg5_Cyl(float,float,float);

/**
* @brief Sends desired position to leg-6 in Cylindrical koodinates
*
* @param r The length in r-direction in cm
* @param th The angles in th-direction in radians
* @param z The length in z-direction in cm
*/
void Send_Leg6_Cyl(float,float,float);

/**
* @brief Sends desired position and velicity to a servo
*
* @param ID The ID of the specific servo
* @param pos The desired position of the servo. 0x000 is min 0x03FF is max
* @param vel The desired velocity of the servo. 0x000 is min 0x03FF is max
*/
void Send_Servo_Position_And_Velocity(unsigned char ID, unsigned int pos, unsigned int vel);

// KOMMENTERA DESSA CHRIPPETY!
void Send_Leg1_Kar_And_Velocity(float x, float y, float z, unsigned int inner, unsigned int middle, unsigned int outer);
void Send_Leg2_Kar_And_Velocity(float x, float y, float z, unsigned int inner, unsigned int middle, unsigned int outer);
void Send_Leg3_Kar_And_Velocity(float x, float y, float z, unsigned int inner, unsigned int middle, unsigned int outer);
void Send_Leg4_Kar_And_Velocity(float x, float y, float z, unsigned int inner, unsigned int middle, unsigned int outer);
void Send_Leg5_Kar_And_Velocity(float x, float y, float z, unsigned int inner, unsigned int middle, unsigned int outer);
void Send_Leg6_Kar_And_Velocity(float x, float y, float z, unsigned int inner, unsigned int middle, unsigned int outer);

// KOMMENTERA!!!
void Send_Leg1_Cyl_And_Velocity(float r, float th, float z, unsigned int inner, unsigned int middle, unsigned int outer);
void Send_Leg2_Cyl_And_Velocity(float r, float th, float z, unsigned int inner, unsigned int middle, unsigned int outer);
void Send_Leg3_Cyl_And_Velocity(float r, float th, float z, unsigned int inner, unsigned int middle, unsigned int outer);
void Send_Leg4_Cyl_And_Velocity(float r, float th, float z, unsigned int inner, unsigned int middle, unsigned int outer);
void Send_Leg5_Cyl_And_Velocity(float r, float th, float z, unsigned int inner, unsigned int middle, unsigned int outer);
void Send_Leg6_Cyl_And_Velocity(float r, float th, float z, unsigned int inner, unsigned int middle, unsigned int outer);

#endif /* SERVO_UART_H_ */