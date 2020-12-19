#ifndef CRC_CALCULATION
#define CRC_CALCULATION

#ifdef STM32F031
	#include "stm32f0xx.h"
#endif
#ifdef STM32F303
	#include "stm32f3xx.h"
#endif
#ifdef STM32F407
	#include "stm32f4xx.h"
#endif
#ifdef STM32H743
	#include "stm32h743xx.h"
#endif
#ifdef STM32H750
	#include "stm32h750xx.h"
#endif

#define ST_CRC_8_VAL	(uint8_t)0x00
#define CRC_8_POL		(uint8_t)0x0D

#define ST_CRC_16_VAL	(uint16_t)0x0000
#define CRC_16_POL		(uint16_t)0x1021

uint8_t calculate_crc_8 (uint8_t * array, uint8_t st_index, uint8_t lenght, uint8_t polynom, uint8_t init_value);
uint16_t calculate_crc_16 (uint8_t * array, uint8_t st_index, uint8_t lenght, uint16_t polynom, uint16_t init_value);

#endif
