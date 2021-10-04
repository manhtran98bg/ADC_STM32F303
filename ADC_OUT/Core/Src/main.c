/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint16_t adc_val[2]={0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void ADC_Select_CH1 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_1;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void ADC_Select_CH2 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_2;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}
void turn_off_port(){
	HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUT2_GPIO_Port,OUT2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUT3_GPIO_Port,OUT3_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUT4_GPIO_Port,OUT4_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUT5_GPIO_Port,OUT5_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUT6_GPIO_Port,OUT6_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUT7_GPIO_Port,OUT7_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUT8_GPIO_Port,OUT8_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUT9_GPIO_Port,OUT9_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUT10_GPIO_Port,OUT10_Pin,GPIO_PIN_RESET);
}
void turn_on_pin(uint16_t pin_num, GPIO_TypeDef *GPIO_port){
	turn_off_port();
	HAL_GPIO_WritePin(GPIO_port,pin_num, GPIO_PIN_SET);
}
void adc_read_value(uint16_t *adc_value)
{	
	ADC_Select_CH1();
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	adc_value[0] = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);	
	//Uncomment if want to use Channel 2
//	ADC_Select_CH2();
//	HAL_ADC_Start(&hadc1);
//	HAL_ADC_PollForConversion(&hadc1, 1000);
//	adc_value[1] = HAL_ADC_GetValue(&hadc1);
//	HAL_ADC_Stop(&hadc1);	
	
}
void process_data(uint16_t adc_value){
	if ((adc_value>=0) && (adc_value<=322)) {
		HAL_UART_Transmit(&huart1,(uint8_t*)"Turn on Port 1\n",sizeof("Turn on Port 1\n"),100);
		turn_on_pin(OUT1_Pin,OUT1_GPIO_Port);
		return;
	}
	if ((adc_value>322) && (adc_value<=645)) {
		HAL_UART_Transmit(&huart1,(uint8_t*)"Turn on Port 2\n",sizeof("Turn on Port 2\n"),100);
		turn_on_pin(OUT2_Pin,OUT2_GPIO_Port);
		return;
	}
	if ((adc_value>645) && (adc_value<=968)) {
		HAL_UART_Transmit(&huart1,(uint8_t*)"Turn on Port 3\n",sizeof("Turn on Port 3\n"),100);
		turn_on_pin(OUT3_Pin,OUT3_GPIO_Port);
		return;
	}
	if ((adc_value>968) && (adc_value<=1291)) {
		HAL_UART_Transmit(&huart1,(uint8_t*)"Turn on Port 4\n",sizeof("Turn on Port 4\n"),100);
		turn_on_pin(OUT4_Pin,OUT4_GPIO_Port);
		return;
	}
	if ((adc_value>1291) && (adc_value<=1613)) {
		HAL_UART_Transmit(&huart1,(uint8_t*)"Turn on Port 5\n",sizeof("Turn on Port 5\n"),100);
		turn_on_pin(OUT5_Pin,OUT5_GPIO_Port);
		return;
	}
	if ((adc_value>1613) && (adc_value<=1936)) {
		HAL_UART_Transmit(&huart1,(uint8_t*)"Turn on Port 6\n",sizeof("Turn on Port 6\n"),100);
		turn_on_pin(OUT6_Pin,OUT6_GPIO_Port);
		return;
	}
	if ((adc_value>1936) && (adc_value<=2259)) {
		HAL_UART_Transmit(&huart1,(uint8_t*)"Turn on Port 7\n",sizeof("Turn on Port 7\n"),100);
		turn_on_pin(OUT7_Pin,OUT7_GPIO_Port);
		return;
	}
	if ((adc_value>2259) && (adc_value<=2582)) {
		HAL_UART_Transmit(&huart1,(uint8_t*)"Turn on Port 8\n",sizeof("Turn on Port 8\n"),100);
		turn_on_pin(OUT8_Pin,OUT8_GPIO_Port);
		return;
	}
	if ((adc_value>2582) && (adc_value<=2904)) {
		HAL_UART_Transmit(&huart1,(uint8_t*)"Turn on Port 9\n",sizeof("Turn on Port 9\n"),100);
		turn_on_pin(OUT9_Pin,OUT9_GPIO_Port);
		return;
	}
	if ((adc_value>2904) && (adc_value<=3327)) {
		HAL_UART_Transmit(&huart1,(uint8_t*)"Turn on Port 10\n",sizeof("Turn on Port 10\n"),100);
		turn_on_pin(OUT10_Pin,OUT10_GPIO_Port);
		return;
	}
	turn_off_port();
	return;

}
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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		adc_read_value(adc_val); 
		process_data(adc_val[0]);
		HAL_Delay(100);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
//  /** Configure Regular Channel
//  */
//  sConfig.Channel = ADC_CHANNEL_1;
//  sConfig.Rank = ADC_REGULAR_RANK_1;
//  sConfig.SingleDiff = ADC_SINGLE_ENDED;
//  sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
//  sConfig.OffsetNumber = ADC_OFFSET_NONE;
//  sConfig.Offset = 0;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure Regular Channel
//  */
//  sConfig.Channel = ADC_CHANNEL_2;
//  sConfig.Rank = ADC_REGULAR_RANK_2;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OUT10_GPIO_Port, OUT10_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, OUT9_Pin|OUT8_Pin|OUT7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OUT6_GPIO_Port, OUT6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OUT5_Pin|OUT4_Pin|OUT3_Pin|OUT2_Pin
                          |OUT1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : OUT10_Pin */
  GPIO_InitStruct.Pin = OUT10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(OUT10_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT9_Pin OUT8_Pin OUT7_Pin */
  GPIO_InitStruct.Pin = OUT9_Pin|OUT8_Pin|OUT7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OUT6_Pin */
  GPIO_InitStruct.Pin = OUT6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(OUT6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT5_Pin OUT4_Pin OUT3_Pin OUT2_Pin
                           OUT1_Pin */
  GPIO_InitStruct.Pin = OUT5_Pin|OUT4_Pin|OUT3_Pin|OUT2_Pin
                          |OUT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
