/**
* @file LCD.h
* @author Joakim
* @date 25 apr 2016
* @brief LCD driver
*
* Defines commands for interfacing with the LCD
*/

#ifndef LCD_H_
#define LCD_H_

#ifndef F_CPU
#define F_CPU 14745600UL
#endif

#include <avr/io.h>
#include <util/delay.h>

#define LCD_NUMBER_OF_LINES 4 /**< The number of lines the LCD can show. */
#define LCD_LINE_WIDTH 16 /**< The number of columns the LCD can show. */

/**
* @brief Clears the screen
*
* Clears the screen by using the built-in command and
* set the cursor to the start position.
*/
void LCD_clear();

/**
* @brief Prints a number on the screen
*
* Prints an unsigned number at the current cursor position
* @param num The number to be printed
*/
void LCD_put_num_u(unsigned int num);

/**
* @brief Prints a character on the screen
*
* Prints an ascii character at the current cursor position
* @param c The character to be printed
*/
void LCD_putc(char c);

/**
* @brief Prints 4 strings to the LCD
*
* Prints four strings to 4 lines of the LCD.
* @param line1 The string to be printed at line1
* @param line2 The string to be printed at line2
* @param line3 The string to be printed at line3
* @param line4 The string to be printed at line4
*/
void LCD_print_string(char line1[LCD_LINE_WIDTH], char line2[LCD_LINE_WIDTH], char line3[LCD_LINE_WIDTH], char line4[LCD_LINE_WIDTH]);

/**
* @brief Check if the LCD is busy
*
* Checks the LCD busy flag.
* @return 1 if LCD is busy, otherwise 0
*/
int LCD_busy();

/**
* @brief Sends a command to the LCD
*
* Sends a command to the LCD processing unit
* @param command The command to be sent
*/
void LCD_send_command(unsigned char command);

/**
* @brief Initiates the LCD
*
* Setup the processor in 4 line mode, etc
*/
void LCD_init();

#endif /* LCD_H_ */