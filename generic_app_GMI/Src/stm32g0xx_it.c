/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g0xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32g0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "driver_adc1.h" //SPA
#include "driver_usart2.h" //SPA
#include "driver_pwm_input.h"
#include "driver_spi1.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
volatile uint64_t tickCounter=0; //SPA
/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;  // TODO: Pwm Input: Move if possible to FlexMouse location
//extern SPI_HandleTypeDef htim2;  // TODO: Pwm Input: Move if possible to FlexMouse location
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  tickCounter++;
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel 1 interrupt.
  */
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */
  
  /* USER CODE END EXTI4_15_IRQn 0 */
  if (LL_EXTI_IsActiveRisingFlag_0_31(LL_EXTI_LINE_8) != RESET)
  {
    LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_8);
    /* USER CODE BEGIN LL_EXTI_LINE_8_RISING */

    /* USER CODE END LL_EXTI_LINE_8_RISING */
  }
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */
}

/**
  * @brief This function handles ADC1 interrupt.
  */
void ADC1_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_IRQn 0 */
//if(LL_ADC_IsActiveFlag_EOC(ADC1) != 0)
  //{
    /* Clear flag ADC group regular end of unitary conversion */
   // LL_ADC_ClearFlag_EOC(ADC1);
    
    /* Call interruption treatment function */
    //AdcGrpRegularUnitaryConvComplete_Callback();
  //}
  /* USER CODE END ADC1_IRQn 0 */
  /* USER CODE BEGIN ADC1_IRQn 1 */

  /* USER CODE END ADC1_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break, update, trigger and commutation interrupts.
  */
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_UP_TRG_COM_IRQn 0 */

  /* USER CODE END TIM1_BRK_UP_TRG_COM_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1); // TODO: Pwm Input: Move if possible to FlexMouse location
  /* USER CODE BEGIN TIM1_BRK_UP_TRG_COM_IRQn 1 */

//   /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE END TIM1_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1); // TODO: Pwm Input: Move if possible to FlexMouse location
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */
}

/**
  * @brief This function handles SPI1 global interrupt.
  */
void SPI1_IRQHandler(void)
{
  SPI1_Interrupt_Handler();
}

/**
//  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
//  */
// void USART1_IRQHandler(void)
// {
//  /* USER CODE BEGIN USART1_IRQn 0 */
//
//  /* USER CODE END USART1_IRQn 0 */
//  /* USER CODE BEGIN USART1_IRQn 1 */
//
//  /* USER CODE END USART1_IRQn 1 */
// }

///**
//  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
//  */
//void USART2_IRQHandler(void)
//{
//  /* USER CODE BEGIN USART2_IRQn 0 */
//  /* Check RXNE flag value in ISR register */
//  if (LL_USART_IsActiveFlag_RXNE(USART2) && LL_USART_IsEnabledIT_RXNE(USART2))
//  {
//    /* RXNE flag will be cleared by reading of RDR register (done in call) */
//    /* Call function in charge of handling Character reception */
//    USART_CharReception_Callback();
//  }
//  
//  if (LL_USART_IsEnabledIT_TXE(USART2) && LL_USART_IsActiveFlag_TXE(USART2))
//  {
//    /* TXE flag will be automatically cleared when writing new data in TDR register */
//
//    /* Call function in charge of handling empty DR => will lead to transmission of next character */
//    USART_TXEmpty_Callback();
//  }
//
//  if (LL_USART_IsEnabledIT_TC(USART2) && LL_USART_IsActiveFlag_TC(USART2))
//  {
//    /* Clear TC flag */
//    LL_USART_ClearFlag_TC(USART2);
//    /* Call function in charge of handling end of transmission of sent character
//       and prepare next charcater transmission */
//    USART_CharTransmitComplete_Callback();
//  }
//  
//  if (LL_USART_IsEnabledIT_ERROR(USART2) && LL_USART_IsActiveFlag_NE(USART2))
//  {
//    /* Call Error function */
//    Error_Callback();
//  }
//  /* USER CODE END USART2_IRQn 0 */
//  /* USER CODE BEGIN USART2_IRQn 1 */
//
//  /* USER CODE END USART2_IRQn 1 */
//}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
