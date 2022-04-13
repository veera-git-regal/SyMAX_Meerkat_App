/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"
#include "stm32g0xx_ll_adc.h"
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_system.h"
#include "stm32g0xx_ll_exti.h"
#include "stm32g0xx_ll_cortex.h"
#include "stm32g0xx_ll_utils.h"
#include "stm32g0xx_ll_pwr.h"
#include "stm32g0xx_ll_dma.h"
#include "stm32g0xx_ll_usart.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_crc.h"
#include "stm32g0xx_ll_spi.h"
  
#include "scheduler.h" //SPA
#include "hardware_config.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/*
#define ANALOG_0_10V_Pin LL_GPIO_PIN_0
#define ANALOG_0_10V_GPIO_Port GPIOA
#define ANALOG_4_20MA_Pin LL_GPIO_PIN_1
#define ANALOG_4_20MA_GPIO_Port GPIOA
#define USART_TX_Pin LL_GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin LL_GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
//#define DIN1_Pin LL_GPIO_PIN_0
//#define DIN1_GPIO_Port GPIOB
//#define DIN2_Pin LL_GPIO_PIN_1
//#define DIN2_GPIO_Port GPIOB
//#define DIN3_Pin LL_GPIO_PIN_2
//#define DIN3_GPIO_Port GPIOB
//#define PWM_IN_Pin LL_GPIO_PIN_8
//#define PWM_IN_GPIO_Port GPIOA
#define MODBUS_TX_Pin LL_GPIO_PIN_9
#define MODBUS_TX_GPIO_Port GPIOA

#define MODBUS_RX_Pin LL_GPIO_PIN_10
#define MODBUS_RX_GPIO_Port GPIOA
#define MODBUS_EN_Pin LL_GPIO_PIN_12
#define MODBUS_EN_GPIO_Port GPIOA*/


/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
