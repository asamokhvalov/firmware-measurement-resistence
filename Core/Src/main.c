/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "d-protocol.h"
#include "dev-info.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void calculate_average (void);
void change_div_mul (void);

void change_mul_div_up (uint8_t cur_mul, uint8_t div_mul);
void change_mul_div_down (uint8_t cur_mul, uint8_t div_mul);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t cur_time = 0;
uint16_t prev_time = 0;
uint16_t calc_time = 0;


uint16_t tx[1];
uint16_t rx[1];
uint16_t temp;

	uint8_t asd[5] = {0x11, 0x22, 0x33, 0x44, 0x55};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USB_PCD_Init();
  MX_SPI2_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

	HAL_ADC_Start (&hadc1);
	HAL_ADC_Start (&hadc2);
	
  Reset_All_MUL_DIV ();

  dev_init (&dev_r);

	set_r_multiple (&dev_r, MULTIPLE_X_1);
	Write_MUL (MULTIPLE_X_1, GPIO_PIN_SET);

	set_r_divide (&dev_r, DIVIDER_10_M);
	Write_DIV (DIVIDER_10_M, GPIO_PIN_SET);
	
	HAL_UART_Transmit (&huart1, (uint8_t *)asd, 5, 1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		cur_time = get_sys_time (&dev_r);
		
		if (cur_time > prev_time) {
			calc_time = cur_time - prev_time;
		}
		else {
			calc_time = MAX_SYS_TICK_MS - prev_time + cur_time;
		}
		
		if (calc_time >= PERIOD_SEND_DELAY) {
			/*execute many times when counters are zero*/
			prev_time = cur_time;
			
			uint8_t stage = get_status_dev (&dev_r);
			
			HAL_GPIO_WritePin (LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET); // test speed measurement
			get_measure ();		
			calculate_average ();

			HAL_GPIO_WritePin (CS_TERMO_1_GPIO_Port, CS_TERMO_1_Pin, GPIO_PIN_RESET);
			HAL_SPI_TransmitReceive (&hspi2, (uint8_t *)tx, (uint8_t *)rx, 1, 1000);
			HAL_GPIO_WritePin (CS_TERMO_1_GPIO_Port, CS_TERMO_1_Pin, GPIO_PIN_SET);
						
			temp = rx[0] >> 3;
			rx[0] = 0;
			
			set_thermocouple_1 (&dev_r, temp);
			
			HAL_GPIO_WritePin (CS_TERMO_2_GPIO_Port, CS_TERMO_2_Pin, GPIO_PIN_RESET);
			HAL_SPI_TransmitReceive (&hspi2, (uint8_t *)tx, (uint8_t *)rx, 1, 1000);
	
			HAL_GPIO_WritePin (CS_TERMO_2_GPIO_Port, CS_TERMO_2_Pin, GPIO_PIN_SET);
						
			temp = rx[0] >> 3;
			rx[0] = 0;	
			
			set_thermocouple_2 (&dev_r, temp);
			
			send_message (&data_resistance);

			change_div_mul ();
			
			HAL_GPIO_WritePin (LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET); // test speed measurement
			
			__NOP();
		}
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */

void calculate_average (void)
{
	uint32_t average_adc_data_meas = 0;
	uint32_t average_adc_ref_volt = 0;

	uint16_t adc_data_meas = 0;
	uint16_t adc_ref_volt = 0;

	for (uint8_t i = 0; i < CAPTURE_DATA_SIZE; i++)
	{
		average_adc_data_meas += get_capture_cur_adc_data_meas (&dev_r, i);
		average_adc_ref_volt += get_capture_cur_adc_data_ref_volt (&dev_r, i);
	}

	adc_data_meas = (uint16_t)(average_adc_data_meas / CAPTURE_DATA_SIZE);
	adc_ref_volt = (uint16_t)(average_adc_ref_volt / CAPTURE_DATA_SIZE);

	set_adc_data_meas (&dev_r, adc_data_meas);
	set_adc_data_ref_volt (&dev_r, adc_ref_volt);
}

void change_div_mul (void)
{
	uint16_t adc_data_meas = 0;
	uint8_t gpio_mul = 0;
	uint8_t gpio_div = 0;

	uint16_t adc_ref_volt = 0;

	adc_data_meas = get_adc_data_meas (&dev_r);
//	adc_data_meas = get_adc_data_ref_volt (&dev_r);
	
	gpio_mul = get_r_multiple (&dev_r);
	gpio_div = get_r_divide (&dev_r);

//	adc_ref_volt = get_adc_data_ref_volt (&dev_r);

	/* change divide and multiply resistors depends on adc_ref_out value. */
	/*
	 * ? ?????? ??????????????? ???????? ?? ??????? (divider (D)).
	 * ????? ?????????? ??????????? ???????? ?? ?? (multiple (M)).
	 * ? ???????? ????????? ????????? D ? M ?????????? ????????? ???????:
	 * 	D = const, ? M ?????????? ?? ??? ???, ???? ??????????? ???????????:
	 * 			START_VALUE_SIGNAL_MIN < adc_ref_volt < START_VALUE_SIGNAL_MAX (1)
	 * 	????? (1) ?? ???????????, ?? ??????? ?????????? M, ? ????? M ????????? min ??? max,
	 * 							  ?? D ?????????????? ????????????? ??? ???????????.
	 * */

	/* Go throught range. */
	if (adc_data_meas < START_VALUE_SIGNAL_MIN) {
		/* D=max, M=max. */
		if ((gpio_div == DIVIDER_10_K) && (gpio_mul == MULTIPLE_X_1000))
		{
			/* We have got the highest input voltage. */
			Write_MUL (MULTIPLE_X_1000, GPIO_PIN_RESET);
			set_r_multiple (&dev_r, MULTIPLE_X_1);
			Write_MUL (MULTIPLE_X_1, GPIO_PIN_SET);

			Write_DIV (DIVIDER_10_K, GPIO_PIN_RESET);
			set_r_divide (&dev_r, DIVIDER_100_K);
			Write_DIV (DIVIDER_100_K, GPIO_PIN_SET);
		}
		/* D=max, M=min. */
		else if ((gpio_div == DIVIDER_10_K) && (gpio_mul == MULTIPLE_X_1))
		{
			Write_MUL (MULTIPLE_X_1, GPIO_PIN_RESET);
			set_r_multiple (&dev_r, MULTIPLE_X_2);
			Write_MUL (MULTIPLE_X_2, GPIO_PIN_SET);
		}
		/* D=min, M=max. */
		else if ((gpio_div == DIVIDER_47_M) && (gpio_mul == MULTIPLE_X_1000))
		{
			Write_MUL (MULTIPLE_X_1000, GPIO_PIN_RESET);
			set_r_multiple (&dev_r, MULTIPLE_X_1);
			Write_MUL (MULTIPLE_X_1, GPIO_PIN_SET);

			Write_DIV (DIVIDER_47_M, GPIO_PIN_RESET);
			set_r_divide (&dev_r, DIVIDER_10_M);
			Write_DIV (DIVIDER_10_M, GPIO_PIN_SET);
		}
		/* D=min, M=min. */
		else if ((gpio_div == DIVIDER_47_M) && (gpio_mul == MULTIPLE_X_1))
		{
			Write_MUL (MULTIPLE_X_1, GPIO_PIN_RESET);
			set_r_multiple (&dev_r, MULTIPLE_X_2);
			Write_MUL (MULTIPLE_X_2, GPIO_PIN_SET);
		}
		/* Other casees. */
		else
		{
			// ??????????? ?????? M
			change_mul_div_up (gpio_mul, gpio_div);
		}
	}
	else if (adc_data_meas > START_VALUE_SIGNAL_MAX) {
		/* D=max, M=max. */
		if ((gpio_div == DIVIDER_10_K) && (gpio_mul == MULTIPLE_X_1000))
		{
			Write_MUL (MULTIPLE_X_1000, GPIO_PIN_RESET);
			set_r_multiple (&dev_r, MULTIPLE_X_1);
			Write_MUL (MULTIPLE_X_1, GPIO_PIN_SET);

			Write_DIV (DIVIDER_10_K, GPIO_PIN_RESET);
			set_r_divide (&dev_r, DIVIDER_100_K);
			Write_DIV (DIVIDER_100_K, GPIO_PIN_SET);
		}
		/* D=max, M=min. */
		else if ((gpio_div == DIVIDER_10_K) && (gpio_mul == MULTIPLE_X_1))
		{
			/* We have got the lowest input voltage. */
		}
		/* D=min, M=max. */
		else if ((gpio_div == DIVIDER_47_M) && (gpio_mul == MULTIPLE_X_1000))
		{
			Write_MUL (MULTIPLE_X_1000, GPIO_PIN_RESET);
			set_r_multiple (&dev_r, MULTIPLE_X_1);
			Write_MUL (MULTIPLE_X_1, GPIO_PIN_SET);

			Write_DIV (DIVIDER_47_M, GPIO_PIN_RESET);
			set_r_divide (&dev_r, DIVIDER_10_M);
			Write_DIV (DIVIDER_10_M, GPIO_PIN_SET);
		}
		/* D=min, M=min. */
		else if ((gpio_div == DIVIDER_47_M) && (gpio_mul == MULTIPLE_X_1))
		{
			Write_DIV (DIVIDER_47_M, GPIO_PIN_RESET);
			set_r_divide (&dev_r, DIVIDER_10_M);
			Write_DIV (DIVIDER_10_M, GPIO_PIN_SET);
		}
		/* Other casees. */
		else
		{
			// ??????????? ?????? M
			change_mul_div_down (gpio_mul, gpio_div);
		}
	}
}

void change_mul_div_up (uint8_t cur_mul, uint8_t div_mul)
{
	switch (cur_mul)
	{
		case MULTIPLE_X_1: {
			Write_MUL (MULTIPLE_X_1, GPIO_PIN_RESET);
			
			set_r_multiple (&dev_r, MULTIPLE_X_2);
			Write_MUL (MULTIPLE_X_2, GPIO_PIN_SET);
			break;
		}
		case MULTIPLE_X_2: {
			Write_MUL (MULTIPLE_X_2, GPIO_PIN_RESET);
			
			set_r_multiple (&dev_r, MULTIPLE_X_5);
			Write_MUL (MULTIPLE_X_5, GPIO_PIN_SET);
			break;
		}
		case MULTIPLE_X_5: {
			Write_MUL (MULTIPLE_X_5, GPIO_PIN_RESET);
			
			set_r_multiple (&dev_r, MULTIPLE_X_10);
			Write_MUL (MULTIPLE_X_10, GPIO_PIN_SET);
			break;
		}
		case MULTIPLE_X_10: {
			Write_MUL (MULTIPLE_X_10, GPIO_PIN_RESET);
			
			set_r_multiple (&dev_r, MULTIPLE_X_100);
			Write_MUL (MULTIPLE_X_100, GPIO_PIN_SET);
			break;
		}
		case MULTIPLE_X_100: {
			Write_MUL (MULTIPLE_X_100, GPIO_PIN_RESET);
			
			set_r_multiple (&dev_r, MULTIPLE_X_1000);
			Write_MUL (MULTIPLE_X_1000, GPIO_PIN_SET);
			break;
		}
		default: {
			/*
			 * ?????????? ????? ??????? (?1000) ?????????
			 * ????? ???????? ?????? ???????? ?????????? ? ??????? ?????????? ????????? ??????????
			 * */
			switch (div_mul)
			{
				case DIVIDER_10_K: {
					Write_DIV (DIVIDER_10_K, GPIO_PIN_RESET);
					
					set_r_divide (&dev_r, DIVIDER_100_K);
					Write_DIV (DIVIDER_100_K, GPIO_PIN_SET);
					break;
				}
				case DIVIDER_100_K: {
					Write_DIV (DIVIDER_100_K, GPIO_PIN_RESET);
					
					set_r_divide (&dev_r, DIVIDER_1_M);
					Write_DIV (DIVIDER_1_M, GPIO_PIN_SET);
//					set_r_divide (&dev_r, DIVIDER_10_K);
//					Write_DIV (DIVIDER_10_K, GPIO_PIN_SET);
					break;
				}
				case DIVIDER_1_M: {
					Write_DIV (DIVIDER_1_M, GPIO_PIN_RESET);
					
					set_r_divide (&dev_r, DIVIDER_10_M);
					Write_DIV (DIVIDER_10_M, GPIO_PIN_SET);
//					set_r_divide (&dev_r, DIVIDER_100_K);
//					Write_DIV (DIVIDER_100_K, GPIO_PIN_SET);
					break;
				}
				case DIVIDER_10_M: {
					Write_DIV (DIVIDER_10_M, GPIO_PIN_RESET);

					set_r_divide (&dev_r, DIVIDER_47_M);
					Write_DIV (DIVIDER_47_M, GPIO_PIN_SET);					
//					set_r_divide (&dev_r, DIVIDER_1_M);
//					Write_DIV (DIVIDER_1_M, GPIO_PIN_SET);
					break;
				}
//				case DIVIDER_47_M: {
//					Write_DIV (DIVIDER_47_M, GPIO_PIN_RESET);
//					
//					set_r_divide (&dev_r, DIVIDER_10_M);
//					Write_DIV (DIVIDER_10_M, GPIO_PIN_SET);
//					break;
//				}
				default: {
					// ?? ?????? ?????????, ?????? ??? ??? ??????? ?????? (10K)
					break;
				}
			}
			break;
		}
	}
}

void change_mul_div_down (uint8_t cur_mul, uint8_t div_mul)
{
	switch (cur_mul)
	{
		case MULTIPLE_X_2: {
			Write_MUL (MULTIPLE_X_2, GPIO_PIN_RESET);
			
			set_r_multiple (&dev_r, MULTIPLE_X_1);
			Write_MUL (MULTIPLE_X_1, GPIO_PIN_SET);
			break;
		}
		case MULTIPLE_X_5: {
			Write_MUL (MULTIPLE_X_5, GPIO_PIN_RESET);
			
			set_r_multiple (&dev_r, MULTIPLE_X_2);
			Write_MUL (MULTIPLE_X_2, GPIO_PIN_SET);
			break;
		}
		case MULTIPLE_X_10: {
			Write_MUL (MULTIPLE_X_10, GPIO_PIN_RESET);
			
			set_r_multiple (&dev_r, MULTIPLE_X_5);
			Write_MUL (MULTIPLE_X_5, GPIO_PIN_SET);
			break;
		}
		case MULTIPLE_X_100: {
			Write_MUL (MULTIPLE_X_100, GPIO_PIN_RESET);
			
			set_r_multiple (&dev_r, MULTIPLE_X_10);
			Write_MUL (MULTIPLE_X_10, GPIO_PIN_SET);
			break;
		}
		case MULTIPLE_X_1000: {
			Write_MUL (MULTIPLE_X_1000, GPIO_PIN_RESET);
			
			set_r_multiple (&dev_r, MULTIPLE_X_100);
			Write_MUL (MULTIPLE_X_100, GPIO_PIN_SET);
			break;
		}
		default: {
			/*
			 * ?????????? ????? ??????? (?1) ?????????
			 * ????? ???????? ?????? ???????? ?????????? ? ??????? ?????????? ????????? ??????????
			 * */
			switch (div_mul)
			{
//				case DIVIDER_10_K: {
//					Write_DIV (DIVIDER_10_K, GPIO_PIN_RESET);
//					set_r_divide (&dev_r, DIVIDER_100_K);
//					Write_DIV (DIVIDER_100_K, GPIO_PIN_SET);
//					break;
//				}
				case DIVIDER_100_K: {
					Write_DIV (DIVIDER_100_K, GPIO_PIN_RESET);
					
					set_r_divide (&dev_r, DIVIDER_10_K);
					Write_DIV (DIVIDER_10_K, GPIO_PIN_SET);
//					set_r_divide (&dev_r, DIVIDER_1_M);
//					Write_DIV (DIVIDER_1_M, GPIO_PIN_SET);
					break;
				}
				case DIVIDER_1_M: {
					Write_DIV (DIVIDER_1_M, GPIO_PIN_RESET);
					
					set_r_divide (&dev_r, DIVIDER_100_K);
					Write_DIV (DIVIDER_100_K, GPIO_PIN_SET);					
//					set_r_divide (&dev_r, DIVIDER_10_M);
//					Write_DIV (DIVIDER_10_M, GPIO_PIN_SET);
					break;
				}
				case DIVIDER_10_M: {
					Write_DIV (DIVIDER_10_M, GPIO_PIN_RESET);
					
					set_r_divide (&dev_r, DIVIDER_1_M);
					Write_DIV (DIVIDER_1_M, GPIO_PIN_SET);
//					set_r_divide (&dev_r, DIVIDER_47_M);
//					Write_DIV (DIVIDER_47_M, GPIO_PIN_SET);
					break;
				}
				case DIVIDER_47_M: {
					Write_DIV (DIVIDER_47_M, GPIO_PIN_RESET);
					
					set_r_divide (&dev_r, DIVIDER_10_M);
					Write_DIV (DIVIDER_10_M, GPIO_PIN_SET);
//					set_r_divide (&dev_r, DIVIDER_47_M);
//					Write_DIV (DIVIDER_47_M, GPIO_PIN_SET);
					break;
				}
				default: {
					// ?? ?????? ?????????, ?????? ??? ??? ??????? ??????
					break;
				}
			}

			break;
		}
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
