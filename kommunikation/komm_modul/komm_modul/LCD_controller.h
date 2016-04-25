/**
* @file LCD_controller.h
* @author Joakim
* @date 25 apr 2016
* @brief LCD controller
*
* Defines a LCD layer that presents information on a number of pages
* on the LCD. The pages switches based on a timer.
*/

#ifndef LCD_CONTROLLER_H_
#define LCD_CONTROLLER_H_

#include <avr/interrupt.h>
#include <avr/io.h>

#include "LCD.h"

#define CONTROLLER_NUMBER_OF_PAGES 4 /**< The number of pages the controller handles. */

volatile char lines[CONTROLLER_NUMBER_OF_PAGES*LCD_NUMBER_OF_LINES][LCD_LINE_WIDTH]; /**< The buffer containing the strings to be displayed. */
volatile uint8_t currentLine; /**< Index of the line on the top of the current page. */
volatile uint8_t overflow_counter; /**< Counter keeping track of the current number of times the timer has overflowed. */
volatile uint8_t spiOverflow;  /**< Counter keeping track of the current number of times the timer has overflowed. */

/**
* @brief Initiates the LCD controller
*
* Initiates the LCD controller with the specified number of pages.
*/
void LCD_controller_init();

/**
* @brief Put a string in the line buffer.
*
* Put a string in the LCD controller's buffer at a given place.
* Line 0 is the top of the first page.
* @param line The index of the line to be placed in.
* @param string The string to put in the buffer.
*/
void LCD_controller_put_line(uint8_t line, char string[LCD_LINE_WIDTH]);


#endif /* LCD_CONTROLLER_H_ */