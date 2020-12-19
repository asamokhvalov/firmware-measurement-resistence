#ifndef USER_SETTINGS_H
#define USER_SETTINGS_H

#include "stm32f0xx.h"
// port defines
#define NUMBER_MULTIPLIERS			(uint8_t)0x04

// Also, has x1 multiplier
#define MULTIPLY_X_2_PORT				GPIO_ODR_4	// PA
#define MULTIPLY_X_11_PORT			GPIO_ODR_3	// PA
#define MULTIPLY_X_101_PORT			GPIO_ODR_2	// PA
#define MULTIPLY_X_1001_PORT		GPIO_ODR_1	// PA

#define BITMASK_MULTIPLY_PORTS	(uint16_t)0x001E //0b00011110
#define BITMASK_X_2_PORT				(uint16_t)0x0001
#define BITMASK_X_11_PORT				(uint16_t)0x0002
#define BITMASK_X_101_PORT			(uint16_t)0x0004
#define BITMASK_X_1001_PORT			(uint16_t)0x0008

#define MULTIPLY_X_1						(uint8_t)0x31
#define MULTIPLY_X_2						(uint8_t)0x32
#define MULTIPLY_X_11						(uint8_t)0x33
#define MULTIPLY_X_101					(uint8_t)0x34
#define MULTIPLY_X_1001					(uint8_t)0x35

#define ADC_INPUT_PORT					GPIO_ODR_0	// PB

#define USER_LED								GPIO_ODR_3	// PB

#define ADC_MAX_VALUE						(uint16_t)4095

// protocol defines
#define START_PACKAGE			0x3A
#define NEW_LINE 					0x0A
#define CARRIAGE_RETURN 	0x0D

#define SIZE_UART_BUF_RX 12
#define SIZE_UART_BUF_TX 10

/*
	protocol transfer adc info to PC

	[0] - 0x3A
	
	if data
	[1] - '+' or '$'
	[2] - code of multiplier
			'1' - x1
			'2' - x2
			'3' - x11
			'4' - x101
			'5' - x1001
			
	[3,4,5,6] - data from adc. ex '4', '0', '9', '5'
	[7,8] - crc from [1-6]
	[9]		- \r 0x0D
	[10]	- \n 0x0A
	
	if cmd
	[1] - '$'
	[2,3,4,5] - command ex 'S', 'T', 'O', 'P'
	[6,7] - crc from [1-5]
	[8]	- \r 0x0D
	[9]	- \n 0x0A	
*/

#endif
