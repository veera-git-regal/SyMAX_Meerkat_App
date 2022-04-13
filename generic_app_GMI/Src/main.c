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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_CRC_Init(void);
//static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
//static void MX_USART2_UART_Init(void);
//static void MX_ADC1_Init(void); //SPA
//static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  if (SysTick_Config(SystemCoreClock/1000))
  { 
    /* Capture error */ 
    while (1);
  }

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
  MX_CRC_Init();
  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  //MX_GPIO_Init();
  MX_DMA_Init();
  //MX_USART2_UART_Init();
  //MX_ADC1_Init(); //SPA
  //MX_USART1_UART_Init(); //SPA
  /* USER CODE BEGIN 2 */
  LL_SYSTICK_EnableIT();
  Sched_Initialize();
  Sched_Run();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
  {
    Error_Handler();  
  };

  /* HSI configuration and activation */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  };

  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 8, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_Enable();
  LL_RCC_PLL_EnableDomain_SYS();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  };

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Sysclk activation on the main PLL */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  };

  /* Set APB1 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  
  LL_Init1msTick(64000000); //SPA
  
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(64000000);

//   /* Update the time base */
//  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
//  {
//    Error_Handler();  
//  };
  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK1);
  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSOURCE_SYSCLK);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_ADC1_Init(void)
//{
//
//  /* USER CODE BEGIN ADC1_Init 0 */
//
//  /* USER CODE END ADC1_Init 0 */
//
//  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
//  LL_ADC_InitTypeDef ADC_InitStruct = {0};
//
//  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//  /* Peripheral clock enable */
//  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC);
//  
//  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
//  /**ADC1 GPIO Configuration  
//  PA0   ------> ADC1_IN0
//  PA1   ------> ADC1_IN1 
//  */
//  GPIO_InitStruct.Pin = ANALOG_0_10V_Pin;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(ANALOG_0_10V_GPIO_Port, &GPIO_InitStruct);
//
//  GPIO_InitStruct.Pin = ANALOG_4_20MA_Pin;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(ANALOG_4_20MA_GPIO_Port, &GPIO_InitStruct);
//
//  /* ADC1 DMA Init */
//  
//  /* ADC1 Init */
//  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_ADC1);
//
//  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
//
//  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);
//
//  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);
//
//  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);
//
//  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);
//
//  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);
//
//  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);
//
//  /* ADC1 interrupt Init */
//  NVIC_SetPriority(ADC1_IRQn, 2);
//  NVIC_EnableIRQ(ADC1_IRQn);
//
//  /* USER CODE BEGIN ADC1_Init 1 */
//
//  /* USER CODE END ADC1_Init 1 */
//  /** Configure Regular Channel 
//  */
//  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CHANNEL_TEMPSENSOR
//                              );
//  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
//  */
//  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
//  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_3RANKS;
//  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
//  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS;
//  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_LIMITED;
//  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
//  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
//  LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
//  LL_ADC_SetTriggerFrequencyMode(ADC1, LL_ADC_CLOCK_FREQ_MODE_LOW);
//  LL_ADC_REG_SetSequencerConfigurable(ADC1, LL_ADC_REG_SEQ_CONFIGURABLE);
//  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_COMMON_1, LL_ADC_SAMPLINGTIME_160CYCLES_5);
//  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_COMMON_2, LL_ADC_SAMPLINGTIME_160CYCLES_5);
//  LL_ADC_DisableIT_EOC(ADC1);
//  LL_ADC_DisableIT_EOS(ADC1);
//  ADC_InitStruct.Clock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
//  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
//  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
//  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
//  LL_ADC_Init(ADC1, &ADC_InitStruct);
//  /** Configure Regular Channel 
//  */
//  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_0);
//  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_0, LL_ADC_SAMPLINGTIME_COMMON_1);
//  /** Configure Regular Channel 
//  */
//  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_1);
//  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_COMMON_1);
//  /** Configure Regular Channel 
//  */
//  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_TEMPSENSOR);
//  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_TEMPSENSOR, LL_ADC_SAMPLINGTIME_COMMON_1);
//  /* USER CODE BEGIN ADC1_Init 2 */
// //LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)adc1Result); //SPA
// //LL_DMA_EnableChannel(DMA_TypeDef *DMA1, uint32_t LL_DMA_CHANNEL_1 ) //SPA
//  /* USER CODE END ADC1_Init 2 */
//
//}

///**
//  * @brief USART1 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_USART1_UART_Init(void)
//{
//
//  /* USER CODE BEGIN USART1_Init 0 */
//
//  /* USER CODE END USART1_Init 0 */
//
//  LL_USART_InitTypeDef USART_InitStruct = {0};
//
//  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//  /* Peripheral clock enable */
//  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
//  
//  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
//  /**USART1 GPIO Configuration  
//  PA9   ------> USART1_TX
//  PA10   ------> USART1_RX
//  PA12 [PA10]   ------> USART1_DE 
//  */
//  GPIO_InitStruct.Pin = MODBUS_TX_Pin;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
//  LL_GPIO_Init(MODBUS_TX_GPIO_Port, &GPIO_InitStruct);
//
//  GPIO_InitStruct.Pin = MODBUS_RX_Pin;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
//  LL_GPIO_Init(MODBUS_RX_GPIO_Port, &GPIO_InitStruct);
//
//  GPIO_InitStruct.Pin = MODBUS_EN_Pin;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
//  LL_GPIO_Init(MODBUS_EN_GPIO_Port, &GPIO_InitStruct);
//
//  /* USART1 interrupt Init */
//  NVIC_SetPriority(USART1_IRQn, 0);
//  NVIC_EnableIRQ(USART1_IRQn);
//
//  /* USER CODE BEGIN USART1_Init 1 */
//
//  /* USER CODE END USART1_Init 1 */
//  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
//  USART_InitStruct.BaudRate = 115200;
//  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
//  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
//  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
//  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
//  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
//  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
//  LL_USART_Init(USART1, &USART_InitStruct);
//  LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
//  LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
//  LL_USART_EnableDEMode(USART1);
//  LL_USART_SetDESignalPolarity(USART1, LL_USART_DE_POLARITY_HIGH);
//  LL_USART_SetDEAssertionTime(USART1, 0);
//  LL_USART_SetDEDeassertionTime(USART1, 0);
//  LL_USART_DisableFIFO(USART1);
//  LL_USART_ConfigAsyncMode(USART1);
//
//  /* USER CODE BEGIN WKUPType USART1 */
//
//  /* USER CODE END WKUPType USART1 */
//
//  LL_USART_Enable(USART1);
//
//  /* Polling USART1 initialisation */
//  while((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))))
//  {
//  }
//  /* USER CODE BEGIN USART1_Init 2 */
//
//  /* USER CODE END USART1_Init 2 */
//
//}


/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* Peripheral clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC);

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  //CRC-16/CCITT-FALSE 
  LL_CRC_SetInputDataReverseMode(CRC, LL_CRC_INDATA_REVERSE_NONE);
  LL_CRC_SetOutputDataReverseMode(CRC, LL_CRC_OUTDATA_REVERSE_NONE);
  LL_CRC_SetPolynomialCoef(CRC, 0x1021);                                              //LL_CRC_DEFAULT_CRC32_POLY);
  LL_CRC_SetPolynomialSize(CRC, LL_CRC_POLYLENGTH_16B);                                //LL_CRC_POLYLENGTH_32B);
  LL_CRC_SetInitialData(CRC, 0xFFFF);                                                  //LL_CRC_DEFAULT_CRC_INITVALUE);
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

///**
//  * @brief GPIO Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_GPIO_Init(void)
//{
//  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
//  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//  /* GPIO Ports Clock Enable */
//  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
//  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
//  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
//
//  /**/
//  LL_GPIO_ResetOutputPin(DOUT1_GPIO_Port, DOUT1_Pin);
//
//  /**/
//  LL_GPIO_ResetOutputPin(LED_ONBOARD_GPIO_Port, LED_ONBOARD_Pin);
//
//  /**/
//  LL_GPIO_ResetOutputPin(LED_OUT_GPIO_Port, LED_OUT_Pin);
//
//  /**/
//  LL_GPIO_ResetOutputPin(RELAY_OUT_GPIO_Port, RELAY_OUT_Pin);
//
//  /**/
//  GPIO_InitStruct.Pin = DOUT1_Pin;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(DOUT1_GPIO_Port, &GPIO_InitStruct);
//
//  /**/
//  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
//  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//  /**/
//  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
//  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//  /**/
//  GPIO_InitStruct.Pin = DIN1_Pin;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(DIN1_GPIO_Port, &GPIO_InitStruct);
//
//  /**/
//  GPIO_InitStruct.Pin = DIN2_Pin;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(DIN2_GPIO_Port, &GPIO_InitStruct);
//
//  /**/
//  GPIO_InitStruct.Pin = DIN3_Pin;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(DIN3_GPIO_Port, &GPIO_InitStruct);
//
//  /**/
//  GPIO_InitStruct.Pin = LED_ONBOARD_Pin;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(LED_ONBOARD_GPIO_Port, &GPIO_InitStruct);
//
//  /**/
//  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
//  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//  /**/
//  GPIO_InitStruct.Pin = LED_OUT_Pin;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(LED_OUT_GPIO_Port, &GPIO_InitStruct);
//
//  /**/
//  GPIO_InitStruct.Pin = RELAY_OUT_Pin;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(RELAY_OUT_GPIO_Port, &GPIO_InitStruct);
//
//  /**/
//  LL_EXTI_SetEXTISource(LL_EXTI_CONFIG_PORTA, LL_EXTI_CONFIG_LINE8);
//
//  /**/
//  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_8;
//  EXTI_InitStruct.LineCommand = ENABLE;
//  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
//  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
//  LL_EXTI_Init(&EXTI_InitStruct);
//
//  /**/
//  LL_GPIO_SetPinPull(PWM_IN_GPIO_Port, PWM_IN_Pin, LL_GPIO_PULL_NO);
//
//  /**/
//  LL_GPIO_SetPinMode(PWM_IN_GPIO_Port, PWM_IN_Pin, LL_GPIO_MODE_INPUT);
//
//}

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
