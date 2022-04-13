/**
  ********************************************************************************************************************************
  * @file    driver_gpio.h 
  * @author  Pamela Lee
  * @brief   Header of Driver function/s for serial protocol with Usart1 hardware
  * @details Protocol Usart1, after decode a whole valid frame from serial port1,
  *          trigger the system control to execute the relative APP in the int stage the Rx data is in Usart1RxPipe.
  *          To Transmitt data : put data into Usart1TxPipe, and call this function USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
  ********************************************************************************************************************************
  */

/* Define to prevent recursive inclusion ---------------------------------------------------------------------------------------*/
#ifndef _DRV_GPIO_H_
#define _DRV_GPIO_H_

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include <stdint.h>

#include "typedef.h"

#include "scheduler.h"
#include "sequential_memory.h"
#include "structured_memory.h"

#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_tim.h"

/* Content ---------------------------------------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* User parameters -------------------------------------------------------------------------------------------------------------*/
#define VALUE_INDICATING_TRIGGERED_IRQ 0x55

// #define SIZE_OF_GPIO_SEQ_MEM 32
#define ACCESS_MODE
  
/*
// GPIO Input Ports/Pins
#define DIN1_Pin LL_GPIO_PIN_0
#define DIN1_GPIO_Port GPIOB
#define DIN2_Pin LL_GPIO_PIN_1
#define DIN2_GPIO_Port GPIOB
#define DIN3_Pin LL_GPIO_PIN_2
#define DIN3_GPIO_Port GPIOB
#define PWM_IN_Pin LL_GPIO_PIN_8
#define PWM_IN_GPIO_Port GPIOA

// GPIO Output Ports/Pins
#define DOUT1_Pin LL_GPIO_PIN_9
#define DOUT1_GPIO_Port GPIOB
#define LED_ONBOARD_Pin LL_GPIO_PIN_6
#define LED_ONBOARD_GPIO_Port GPIOC
#define LED_OUT_Pin LL_GPIO_PIN_6
#define LED_OUT_GPIO_Port GPIOB
#define RELAY_OUT_Pin LL_GPIO_PIN_7
#define RELAY_OUT_GPIO_Port GPIOB
*/
  
#define TIM3_CH1_PWM_OUT_PRESCALER_VALUE 19
#define TIM3_CH1_PWM_OUT_PERIOD 1000
#define PWM_OUT_PULSE_VALUE 500
  
#define CLOCK_FREQ 64000000
  
#define TOTAL_DIGITAL_INPUTS  5  // Number of digital inputs actually in use
#define MAX_DIGITAL_INPUTS	8	// maximum number of possible digital inputs
#define TOTAL_DIGITAL_OUTPUTS 10 // Number of digital outputs
  
#define NO_ERROR 0
#define ERROR_WRONG_PIN 1

/* Setup -----------------------------------------------------------------------------------------------------------------------*/
static Ring_Buf_Handle gpioSeqMem_u32;
static Ram_Buf_Handle gpioStructMem_u32;

// GPIO pin state
enum
{
  PIN_RESET = 0U,
  PIN_SET = 1U
};



/* Function Declarations -------------------------------------------------------------------------------------------------------*/
/**
  ********************************************************************************************************************************
  * @brief    
  * @details 
  ********************************************************************************************************************************
  */
void GPIOInit(void);

/**
  ********************************************************************************************************************************
  * @brief    
  * @details 
  ********************************************************************************************************************************
  */
void Driver_GPIO_Dout1_Init(uint8_t input_no_u8);

/**
  ********************************************************************************************************************************
  * @brief   Read GPIO State
  * @details Read input state of GPIO and send status back
  ********************************************************************************************************************************
  */
uint8_t Gpio_Driver_ReadGpioInputState(uint8_t);

/**
  ********************************************************************************************************************************
  * @brief   Read GPIO Ouput State
  * @details Read output state of GPIO and send status back
  ********************************************************************************************************************************
  */
uint8_t Driver_Gpio_ReadGpioOutputState(uint8_t output_num_u8);

/**
  ********************************************************************************************************************************
  * @brief   Toggle GPIO output State
  * @details 
  ********************************************************************************************************************************
*/
void Driver_Gpio_TogglePin(uint8_t output_num_u8);

/**
  ********************************************************************************************************************************
  * @brief   Write GPIO State
  * @details Write output state of GPIO
  ********************************************************************************************************************************
*/
void Driver_Gpio_WriteGpioState(uint8_t output_num_u8, uint8_t PinState);

/**
  ********************************************************************************************************************************
  * @brief   Start PWM output
  * @details 
  * @param   None 
  * @return  None
  ********************************************************************************************************************************
*/
void Driver_Gpio_StartPwmOut(uint8_t output_num_u8);

/**
  ********************************************************************************************************************************
  * @brief   Stop PWM output
  * @details 
  * @param   None 
  * @return  None
  ********************************************************************************************************************************
*/
void Driver_Gpio_StopPwmOut(uint8_t output_num_u8);

/**
  ********************************************************************************************************************************
  * @brief   Set PWM output polairty
  * @details Assign structured memory for App module
  * @param   None 
  * @return  None
  ********************************************************************************************************************************
*/
void Driver_Gpio_SetPwmOutPolarity(uint8_t output_num_u8, uint8_t polarity_value);

/**
  ********************************************************************************************************************************
  * @brief   Set PWM Frequency
  * @details 
  * @param   pwm_frequency 
  * @return  None
  ********************************************************************************************************************************
*/
void Driver_Gpio_SetPwmOutPeriod(uint8_t output_num_u8, uint16_t pwm_frequency_u16);

/**
  ********************************************************************************************************************************
  * @brief   Set PWM output duty cycle
  * @details 
  * @param   pwm_duty_cycle 
  * @return  None
  ********************************************************************************************************************************
*/
void Driver_Gpio_SetPwmOutDutyCycle(uint8_t output_num_u8, uint16_t pwm_duty_cycle_u16);

/**
  ********************************************************************************************************************************
  * @brief   This function handles EXTI line[9:5] interrupts.
  * @details 
  * @param   None 
  * @return  None
  ********************************************************************************************************************************
*/
void EXTI9_5_IRQHandler(void);



#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _DRV_GPIO_H_ */