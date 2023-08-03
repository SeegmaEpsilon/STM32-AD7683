/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#define CS_LOW() HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
#define CS_HIGH() HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* --- PRINTF_BYTE_TO_BINARY macro's --- */
#define PRINTF_BINARY_PATTERN_INT8 "%c%c%c%c%c%c%c%c"
#define PRINTF_BYTE_TO_BINARY_INT8(i)    \
    (((i) & 0x80ll) ? '1' : '0'), \
    (((i) & 0x40ll) ? '1' : '0'), \
    (((i) & 0x20ll) ? '1' : '0'), \
    (((i) & 0x10ll) ? '1' : '0'), \
    (((i) & 0x08ll) ? '1' : '0'), \
    (((i) & 0x04ll) ? '1' : '0'), \
    (((i) & 0x02ll) ? '1' : '0'), \
    (((i) & 0x01ll) ? '1' : '0')

#define PRINTF_BINARY_PATTERN_INT16 \
    PRINTF_BINARY_PATTERN_INT8              PRINTF_BINARY_PATTERN_INT8
#define PRINTF_BYTE_TO_BINARY_INT16(i) \
    PRINTF_BYTE_TO_BINARY_INT8((i) >> 8),   PRINTF_BYTE_TO_BINARY_INT8(i)
#define PRINTF_BINARY_PATTERN_INT32 \
    PRINTF_BINARY_PATTERN_INT16             PRINTF_BINARY_PATTERN_INT16
#define PRINTF_BYTE_TO_BINARY_INT32(i) \
    PRINTF_BYTE_TO_BINARY_INT16((i) >> 16), PRINTF_BYTE_TO_BINARY_INT16(i)
#define PRINTF_BINARY_PATTERN_INT64    \
    PRINTF_BINARY_PATTERN_INT32             PRINTF_BINARY_PATTERN_INT32
#define PRINTF_BYTE_TO_BINARY_INT64(i) \
    PRINTF_BYTE_TO_BINARY_INT32((i) >> 32), PRINTF_BYTE_TO_BINARY_INT32(i)
/* --- end macros --- */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t ADC_value_global = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Delay for at least max possible conversion time (0.5us)
void AD7683_Conversion_Delay(void)
{
  // 84 MHz CPU, 40*(1/84*10^6) = 0.5us
  for(uint8_t i = 0; i < 22; i++)
  {
    __asm__("nop");
  }
}

// 1 mhz clk
//uint16_t AD7683_GetValues_SPI(void)
//{
//  uint16_t byte_array[2] = {0};
//
//  uint16_t high_byte = 0;
//  uint16_t middle_byte = 0;
//  uint16_t low_byte = 0;
//
//  uint32_t adcValue = 0;
//
//  // Trigger a conversion (CNV High)
//  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
//
//  // Wait 0.5us (Max conversion time)
//  //AD7683_Conversion_Delay();
//
//  // Clock out the 16 bits for ADC1
//  HAL_SPI_Receive(&hspi1, (uint8_t*)&byte_array, 3, 500);
//  //--------------------------------------
//  // Reset CNV line
//  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
//
//  high_byte = byte_array[0];
//  //middle_byte = byte_array[1];
//  low_byte = byte_array[1];
//
//  char divider[20] = "---------------\n\r";
//  char string_high[100];
//  //char string_middle[100];
//  char string_low[100];
//
//  sprintf(string_high, "high byte: " PRINTF_BINARY_PATTERN_INT16 "\n\r", PRINTF_BYTE_TO_BINARY_INT16(high_byte));
//  //sprintf(string_middle, "middle byte: " PRINTF_BINARY_PATTERN_INT8 "\n\r", PRINTF_BYTE_TO_BINARY_INT8(middle_byte));
//  sprintf(string_low, "low byte: " PRINTF_BINARY_PATTERN_INT16 "\n\r", PRINTF_BYTE_TO_BINARY_INT16(low_byte));
//
//  HAL_UART_Transmit(&huart2, (uint8_t*)divider, strlen(divider), 1000);
//
//  HAL_UART_Transmit(&huart2, (uint8_t*)string_high, strlen(string_high), 1000);
//  //HAL_UART_Transmit(&huart2, (uint8_t*)string_middle, strlen(string_middle), 1000);
//  HAL_UART_Transmit(&huart2, (uint8_t*)string_low, strlen(string_low), 1000);
//
//  HAL_UART_Transmit(&huart2, (uint8_t*)divider, strlen(divider), 1000);
//
//  adcValue = (high_byte | low_byte) >> 10;
//
//  char string_value[100];
//  sprintf(string_value, "ADC_value,%d\n\r", adcValue);
//  HAL_UART_Transmit(&huart2, (uint8_t*)string_value, strlen(string_value), 1000);
//
//  return adcValue;
//}

uint16_t ADC_getData(void)
{
  uint16_t spi_data = 0;

  CS_LOW();
  for(int i = 0; i < 22; i++)
  {
    HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_SET);
    for(int i = 0; i < 22; i++) __asm__("nop");
    spi_data |= HAL_GPIO_ReadPin(MISO_GPIO_Port, MISO_Pin);
    HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_RESET);
    for(int i = 0; i < 22; i++) __asm__("nop");
    if(i != 21) spi_data <<= 1;
  }
  CS_HIGH();
  return spi_data;
}

#define DATA_SIZE 32000
uint16_t adcData[DATA_SIZE] = { 0 };
uint16_t counter = 0;

double calculateRMS()
{
  double dataAverage = 0;
  double squaresSum = 0;
  char string_value[100];
  double RMS = 0;

  for(int i = 0; i < DATA_SIZE; i++)
  {
    dataAverage += adcData[i];
  }
  dataAverage /= DATA_SIZE;

  for(int i = 0; i < DATA_SIZE; i++)
  {
    squaresSum += (adcData[i] - dataAverage) * (adcData[i] - dataAverage);
  }

  RMS = sqrt(squaresSum / DATA_SIZE);

  sprintf(string_value, "ADC_value_RMS,%f\n\r", RMS);
  HAL_UART_Transmit(&huart2, (uint8_t*)string_value, strlen(string_value), 1000);

  return RMS;
}

/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM1) //check if the interrupt comes from TIM1
  {
    char string_value[100];
    uint16_t adcValue = ADC_getData();

    adcData[counter] = adcValue;
    counter++;

    if(counter >= DATA_SIZE)
    {
      counter = 0;
      calculateRMS();
    }
    else
    {
      sprintf(string_value, "ADC_value,%d\n\r", adcValue);
      HAL_UART_Transmit(&huart2, (uint8_t*)string_value, strlen(string_value), 1000);
    }

  }
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1)
  {
    //AD7683_GetValues_SPI();
    //HAL_Delay(500);
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
  RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
  RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
  TIM_MasterConfigTypeDef sMasterConfig = { 0 };

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 10;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 125;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if(HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if(HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if(HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if(HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_Pin */
  GPIO_InitStruct.Pin = CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(CLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MISO_Pin */
  GPIO_InitStruct.Pin = MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MISO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

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
  while(1)
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
