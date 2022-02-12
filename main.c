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
#include "STM_MY_LCD16X2.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define usTIM TIM4
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void usDelay(uint32_t uSec);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Speed of sound divided by two as 2 directions
const float speedOfSound = 0.0343/2;
float distance;
uint32_t adcValue;
uint8_t icFlag = 0;
uint8_t captureIdx=0;
uint32_t edge1Time=0, edge2Time=0;

// For SPI
uint8_t spiTxBuf[2];
uint8_t spiRxBuf[6];
uint16_t x_data;
uint16_t y_data;
uint16_t z_data;
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
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	LCD1602_Begin4BIT(GPIOB, RS_Pin, EN_Pin, GPIOE, D4_Pin, D5_Pin, D6_Pin, D7_Pin);
  LCD1602_noCursor();
  LCD1602_noBlink();
  LCD1602_clear();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  char student_name[] = "Ciaran Mallory";
  char student_number[] = "K00238250";
	//uint32_t numTicks = 0;
	float adcVoltage = 0.0;
	uint32_t duty_cycle = 0;
	uint8_t count = 0;
	//GPIO_PinState PinState;
	
  HAL_GPIO_WritePin(GPIOC, LCD_LED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
  
	// Printing Name and K number to LCD
	HAL_Delay(500);
	LCD1602_print(student_name);
	LCD1602_2ndLine();
	LCD1602_print(student_number);

	HAL_Delay(3000);
	LCD1602_clear();
			
	// Start PWM Channel
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
			// Set up PWM channel
			htim4.Instance->CCR4 = adcValue;
			//while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 0);
			//HAL_Delay(10);
			//count++;
			//count = count % 5;
			count++;
		
			switch(count)
			{
				case(1):
					LCD1602_clear();
					//LCD1602_clear();
					// Reading value from ADC pin PC5
					HAL_ADC_Start(&hadc1);
					if(HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK)
					{
						adcValue = HAL_ADC_GetValue(&hadc1);
					}
					HAL_ADC_Stop(&hadc1);
					HAL_Delay(100);
					
					LCD1602_1stLine();
					LCD1602_print("ADC Value: ");
					LCD1602_PrintInt(adcValue);
					adcVoltage = (float)adcValue * (3.0/4096.0);
					LCD1602_2ndLine();
					LCD1602_print("ADC Voltage: ");
					LCD1602_PrintFloat(adcVoltage,4);
					
					HAL_Delay(500);
					//LCD1602_clear();
						
				case(2):
					HAL_Delay(5000);
					LCD1602_clear();
					// Reading value from ADC pin PC5
					HAL_ADC_Start(&hadc1);
					if(HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK)
					{
						adcValue = HAL_ADC_GetValue(&hadc1);
					}
					HAL_ADC_Stop(&hadc1);
					HAL_Delay(100);
					
					//LCD1602_clear();
					duty_cycle = (uint32_t)(adcValue * 100/4096);
					adcVoltage = (float)adcValue * (3.0/4096.0);
					LCD1602_1stLine();
					LCD1602_print("ADC Voltage: ");
					LCD1602_PrintFloat(adcVoltage,4);
					LCD1602_2ndLine();
					LCD1602_print("Duty Cycle: ");
					LCD1602_PrintInt(duty_cycle);
					LCD1602_print("%");
					
					HAL_Delay(500);
					//LCD1602_clear();
				
				case(3):
					HAL_Delay(5000);
					LCD1602_clear();
					// Ultrasonic input capture method
					//Set TRIG to LOW for few uSec
					HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_RESET);
					usDelay(3);
					
					//*** START Ultrasonic measure routine ***//
					//1. Output 10 usec TRIG
					HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_SET);
					usDelay(10);
					HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_RESET);
					
					//2. ECHO signal pulse width
					//Start IC timer
					HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_4);
					//Wait for IC flag
					uint32_t startTick = HAL_GetTick();
					do
					{
						if(icFlag) break;
					}while((HAL_GetTick() - startTick) < 500);  //500ms
					icFlag = 0;
					HAL_TIM_IC_Stop_IT(&htim5, TIM_CHANNEL_4);
					
					//Calculate distance in cm
					if(edge2Time > edge1Time)
					{
						distance = ((edge2Time - edge1Time) + 0.0f)*speedOfSound;
					}
					else
					{
						distance = 0.0f;
					}
					
					// Print to LCD
					LCD1602_1stLine();
					LCD1602_print("Distance: ");
					LCD1602_PrintFloat(distance,4);
					HAL_Delay(500);
					//LCD1602_clear();
					//while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1);
					//HAL_Delay(10);
					
				case(4):
					LCD1602_clear();
					// SPI Transmit
					// 1. Bring slave select line low
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
					// 2. Transmit register address & data
					spiTxBuf[0] = 0x23; // Register Address 0x20 -- CTRL REG4
					spiRxBuf[1] = 0xC8; // Enable x,y,z axis and set Data rate to 3.125Hz
					HAL_SPI_Transmit(&hspi1, spiTxBuf, 2, 50); // Send 2 bytes
					// 3. Bring slave select line high
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
					
					// SPI Receive
					// 1. Bring slave select line high
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
					// 2. Transmit register address + 0x80 (set MSB to high for read mode)
					spiTxBuf[0] = 0x20 |0x80; // Register address 0x20 -- CTRL REG4 in read mode
					HAL_SPI_Transmit(&hspi1, spiTxBuf, 1, 50); // send 1 byte
					// 3. Receive data
					HAL_SPI_Receive(&hspi1, spiRxBuf, 6, 50); // Receive 1 byte
							x_data = (spiRxBuf[1]<<8) | spiRxBuf[0];
							y_data = (spiRxBuf[3]<<8) | spiRxBuf[2];
							z_data = (spiRxBuf[5]<<8) | spiRxBuf[4];
					// 4. Bring slave select line high
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
					HAL_Delay(500);
					
					// Print to LCD
					LCD1602_1stLine();
					LCD1602_print(" X: ");
					LCD1602_PrintFloat(x_data,1);
					LCD1602_print(" Y: ");
					LCD1602_PrintFloat(y_data,1);
					LCD1602_2ndLine();
					LCD1602_print(" Z: ");
					LCD1602_PrintFloat(z_data,1);
					HAL_Delay(500);
					count = 1;
			
			// Toggling LED
      HAL_GPIO_TogglePin(GPIOC, LCD_LED_Pin);
      HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);

      HAL_Delay(500);
      
      HAL_GPIO_TogglePin(GPIOD, LCD_LED_Pin);
      HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);

  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 16-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 4;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, D5_Pin|D7_Pin|D6_Pin|D4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_LED_GPIO_Port, LCD_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EN_Pin|RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : D5_Pin PE3 D7_Pin D6_Pin
                           D4_Pin */
  GPIO_InitStruct.Pin = D5_Pin|GPIO_PIN_3|D7_Pin|D6_Pin
                          |D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_LED_Pin */
  GPIO_InitStruct.Pin = LCD_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB_Pin */
  GPIO_InitStruct.Pin = PB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Trigger_Pin */
  GPIO_InitStruct.Pin = Trigger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Trigger_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_Pin RS_Pin */
  GPIO_InitStruct.Pin = EN_Pin|RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
void usDelay(uint32_t uSec)
{
	if(uSec < 2) uSec = 2;
	usTIM->ARR = uSec - 1;
	usTIM->EGR = 1;
	usTIM->SR &= ~1;
	usTIM->CR1 |= 1;
	while((usTIM->SR&0x0001) != 1);
	usTIM->SR &= ~(0x0001);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

		if(captureIdx == 0) //Fisrt edge
		{
			edge1Time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4); //__HAL_TIM_GetCounter(&htim3);//
			
			captureIdx = 1;
		}
		else if(captureIdx == 1) //Second edge
		{
			edge2Time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
			captureIdx = 0;
			icFlag = 1;
		}
}
/*
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	UNUSED(GPIO_Pin);
	
	// SPI Receive
	// 1. Bring slave select line high
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	// 2. Transmit register address + 0x80 (set MSB to high for read mode)
	spiTxBuf[0] = 0x28 |0x80; // Register address 0x20 -- CTRL REG4 in read mode
	HAL_SPI_Transmit(&hspi1, spiTxBuf, 1, 50); // send 1 byte
	// 3. Receive data
	HAL_SPI_Receive(&hspi1, spiRxBuf, 6, 50); // Receive 1 byte
			x_data = (spiRxBuf[1]<<8) | spiRxBuf[0];
			y_data = (spiRxBuf[3]<<8) | spiRxBuf[2];
			z_data = (spiRxBuf[5]<<8) | spiRxBuf[4];
	// 4. Bring slave select line high
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
	HAL_Delay(500);
	
	// Print to LCD
	LCD1602_1stLine();
	LCD1602_print(" X: ");
	LCD1602_PrintFloat(x_data,1);
	LCD1602_print(" Y: ");
	LCD1602_PrintFloat(y_data,1);
	LCD1602_2ndLine();
	LCD1602_print(" Z: ");
	LCD1602_PrintFloat(z_data,1);
	HAL_Delay(500);
}*/
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
