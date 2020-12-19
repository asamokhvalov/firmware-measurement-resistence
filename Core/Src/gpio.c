/**
  ******************************************************************************
  * File Name          : gpio.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
/* USER CODE BEGIN 0 */

#include "dev-info.h"

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS_TERMO_1_Pin|CS_TERMO_2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MULTIPLE_X_2_Pin|DIVIDER_10_M_Pin|MULTIPLE_X_5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, DIVIDER_47_M_Pin|MULTIPLE_X_10_Pin|DIVIDER_1_M_Pin|MULTIPLE_X_100_Pin
                          |DIVIDER_100_K_Pin|MULTIPLE_X_1000_Pin|DIVIDER_10_K_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PEPin PEPin PEPin PEPin
                           PEPin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PEPin PEPin PEPin PEPin
                           PEPin PEPin PEPin PEPin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD7_Pin
                          |LD9_Pin|LD10_Pin|LD8_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin */
  GPIO_InitStruct.Pin = CS_TERMO_1_Pin|CS_TERMO_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PCPin PCPin PCPin */
  GPIO_InitStruct.Pin = MULTIPLE_X_2_Pin|DIVIDER_10_M_Pin|MULTIPLE_X_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PDPin PDPin PDPin PDPin
                           PDPin PDPin PDPin */
  GPIO_InitStruct.Pin = DIVIDER_47_M_Pin|MULTIPLE_X_10_Pin|DIVIDER_1_M_Pin|MULTIPLE_X_100_Pin
                          |DIVIDER_100_K_Pin|MULTIPLE_X_1000_Pin|DIVIDER_10_K_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */
void Reset_All_MUL_DIV (void)
{
	HAL_GPIO_WritePin (MULTIPLE_X_2_GPIO_Port, MULTIPLE_X_2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin (MULTIPLE_X_5_GPIO_Port, MULTIPLE_X_5_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin (MULTIPLE_X_10_GPIO_Port, MULTIPLE_X_10_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin (MULTIPLE_X_100_GPIO_Port, MULTIPLE_X_100_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin (MULTIPLE_X_1000_GPIO_Port, MULTIPLE_X_1000_Pin, GPIO_PIN_RESET);
	
	HAL_GPIO_WritePin (DIVIDER_10_K_GPIO_Port, DIVIDER_10_K_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin (DIVIDER_100_K_GPIO_Port, DIVIDER_100_K_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin (DIVIDER_1_M_GPIO_Port, DIVIDER_1_M_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin (DIVIDER_47_M_GPIO_Port, DIVIDER_47_M_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin (DIVIDER_10_M_GPIO_Port, DIVIDER_10_M_Pin, GPIO_PIN_RESET);
}

void Reset_All_MUL (void)
{
	HAL_GPIO_WritePin (MULTIPLE_X_2_GPIO_Port, MULTIPLE_X_2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin (MULTIPLE_X_5_GPIO_Port, MULTIPLE_X_5_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin (MULTIPLE_X_10_GPIO_Port, MULTIPLE_X_10_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin (MULTIPLE_X_100_GPIO_Port, MULTIPLE_X_100_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin (MULTIPLE_X_1000_GPIO_Port, MULTIPLE_X_1000_Pin, GPIO_PIN_RESET);
}

void Reset_All_DIV (void)
{
	HAL_GPIO_WritePin (DIVIDER_10_K_GPIO_Port, DIVIDER_10_K_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin (DIVIDER_100_K_GPIO_Port, DIVIDER_100_K_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin (DIVIDER_1_M_GPIO_Port, DIVIDER_1_M_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin (DIVIDER_47_M_GPIO_Port, DIVIDER_47_M_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin (DIVIDER_10_M_GPIO_Port, DIVIDER_10_M_Pin, GPIO_PIN_RESET);
}

void Write_MUL (uint8_t mul_val, GPIO_PinState out_val)
{
	switch (mul_val) {
		case MULTIPLE_X_1: {
			Reset_All_MUL ();
			break;
		}
		case MULTIPLE_X_2: {
			HAL_GPIO_WritePin (MULTIPLE_X_2_GPIO_Port, MULTIPLE_X_2_Pin, out_val);
			break;
		}
		case MULTIPLE_X_5: {
			HAL_GPIO_WritePin (MULTIPLE_X_5_GPIO_Port, MULTIPLE_X_5_Pin, out_val);
			break;
		}
		case MULTIPLE_X_10: {
			HAL_GPIO_WritePin (MULTIPLE_X_10_GPIO_Port, MULTIPLE_X_10_Pin, out_val);
			break;
		}
		case MULTIPLE_X_100: {
			HAL_GPIO_WritePin (MULTIPLE_X_100_GPIO_Port, MULTIPLE_X_100_Pin, out_val);
			break;
		}
		case MULTIPLE_X_1000: {
			HAL_GPIO_WritePin (MULTIPLE_X_1000_GPIO_Port, MULTIPLE_X_1000_Pin, out_val);
			break;
		}
		default: {
			/* Set to x1. */
			Reset_All_MUL ();
			break;
		}
	}
}

void Write_DIV (uint8_t div_val, GPIO_PinState out_val)
{
	switch (div_val) {
		case DIVIDER_10_K: {
			HAL_GPIO_WritePin (DIVIDER_10_K_GPIO_Port, DIVIDER_10_K_Pin, out_val);
			break;
		}
		case DIVIDER_100_K: {
			HAL_GPIO_WritePin (DIVIDER_100_K_GPIO_Port, DIVIDER_100_K_Pin, out_val);
			break;
		}
		case DIVIDER_1_M: {
			HAL_GPIO_WritePin (DIVIDER_1_M_GPIO_Port, DIVIDER_1_M_Pin, out_val);
			break;
		}
		case DIVIDER_10_M: {
			HAL_GPIO_WritePin (DIVIDER_10_M_GPIO_Port, DIVIDER_10_M_Pin, out_val);
			break;
		}
		case DIVIDER_47_M: {
			HAL_GPIO_WritePin (DIVIDER_47_M_GPIO_Port, DIVIDER_47_M_Pin, out_val);
			break;
		}
		default: {
			/* Set to 47M. */
			HAL_GPIO_WritePin (DIVIDER_47_M_GPIO_Port, DIVIDER_47_M_Pin, out_val);
			break;
		}
	}
}

/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
