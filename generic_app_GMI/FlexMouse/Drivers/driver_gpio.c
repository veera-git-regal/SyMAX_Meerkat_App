/**
  ********************************************************************************************************************************
  * @file    drv_gpio.c 
  * @author  Pamela Lee
  * @brief   Main Driver function/s for GPIO setup for particular port/s
  * @details    
  ********************************************************************************************************************************
  */

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "driver_gpio.h"
#include "main.h"
#include "macros.h"

#include <stdio.h>

/* Content ---------------------------------------------------------------------------------------------------------------------*/
extern ProcessInfo processInfoTable[];
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);

uint32_t pwmPeriod_u32 = 0;

typedef struct{
  GPIO_TypeDef *gpioPort;
  uint16_t pin;
}Gpio_Ports_Pins;

// GPIO Inputs ports/pins
#if HARDWARE_VERSION == HARDWARE_VERSION_SYMAX_SRI
Gpio_Ports_Pins gpioInputPortsPins[] = {
  {DIR_ROT_GPIO_Port, DIR_ROT_Pin},
  {DIN1_GPIO_Port, DIN1_Pin},
  {DIN2_GPIO_Port, DIN2_Pin},
  {DIN3_GPIO_Port, DIN3_Pin},
  {PWM_IN_GPIO_Port, PWM_IN_Pin},
};
#else
// GPIO Inputs ports/pins
Gpio_Ports_Pins gpioInputPortsPins[] = {
  {DIN1_GPIO_Port, DIN1_Pin},
  {DIN2_GPIO_Port, DIN2_Pin },
  {DIN3_GPIO_Port, DIN3_Pin},
  {PWM_IN_GPIO_Port, PWM_IN_Pin},
};
#endif

// GPIO Outputs ports/pins
Gpio_Ports_Pins gpioOutputPortsPins[] = {
  {DOUT1_GPIO_Port, DOUT1_Pin},
  {LED_ONBOARD_GPIO_Port, LED_ONBOARD_Pin },
  {LED_OUT_GPIO_Port, LED_OUT_Pin},
  {RELAY_OUT_GPIO_Port, RELAY_OUT_Pin},
  {MODBUS_EN_GPIO_Port, MODBUS_EN_Pin},
};

/**
  ********************************************************************************************************************************
  * @brief   GPIO Initialization Function
  * @details 
  * @param   None
  * @return  None
  ********************************************************************************************************************************
*/
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  /**/
  LL_GPIO_ResetOutputPin(LED_ONBOARD_GPIO_Port, LED_ONBOARD_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED_OUT_GPIO_Port, LED_OUT_Pin);

  /**/
  LL_GPIO_ResetOutputPin(RELAY_OUT_GPIO_Port, RELAY_OUT_Pin);

  // /**/    
  // GPIO_InitStruct.Pin = SPI1_MISO_Pin;
  // GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  // GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  // GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  // GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  // GPIO_InitStruct.Alternate = SPI1_GPIO_AF;
  // LL_GPIO_Init(SPI1_MISO_Port, &GPIO_InitStruct);

  // /**/
  // GPIO_InitStruct.Pin = SPI1_MOSI_Pin;
  // GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  // GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  // GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  // GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  // GPIO_InitStruct.Alternate = SPI1_GPIO_AF;
  // LL_GPIO_Init(SPI1_MOSI_Port, &GPIO_InitStruct);
  
  //  /**/
  // GPIO_InitStruct.Pin = SPI1_SCK_Pin;
  // GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  // GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  // GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  // GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  // GPIO_InitStruct.Alternate = SPI1_GPIO_AF;
  // LL_GPIO_Init(SPI1_SCK_Port, &GPIO_InitStruct);

#if HARDWARE_VERSION == HARDWARE_VERSION_SYMAX_SRI
  GPIO_InitStruct.Pin = DIR_ROT_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(DIR_ROT_GPIO_Port, &GPIO_InitStruct);  
#endif
  
  /**/
  GPIO_InitStruct.Pin = DIN1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(DIN1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = DIN2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(DIN2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = DIN3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(DIN3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED_ONBOARD_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED_ONBOARD_GPIO_Port, &GPIO_InitStruct);
 
  /**/
  GPIO_InitStruct.Pin = LED_OUT_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED_OUT_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = RELAY_OUT_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(RELAY_OUT_GPIO_Port, &GPIO_InitStruct);
}

/**
  ********************************************************************************************************************************
  * @brief   GPIO Initialization Function
  * @details Wrapper function for ST GPIO init function
  * @param   None 
  * @return  None
  ********************************************************************************************************************************
*/
void GPIOInit(void)
{
  MX_GPIO_Init();  
}

/**
  ********************************************************************************************************************************
  * @brief   Timer 17 init
  * @details Init Timer 17 that is used for PWM output
  * @param   None 
  * @return  None
  ********************************************************************************************************************************
*/
static void MX_TIM3_Init(void)
{
  TIM_HandleTypeDef htim3;
  __HAL_RCC_TIM3_CLK_ENABLE();
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 19;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
 
  __HAL_RCC_GPIOB_CLK_ENABLE();
  /**TIM3 GPIO Configuration
  PB5     ------> TIM3_CH2
  */
  GPIO_InitStruct.Pin = DOUT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = DOUT1_LL_GPIO_AF;
  HAL_GPIO_Init(DOUT1_GPIO_Port, &GPIO_InitStruct);
 HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
}


/**
  * @brief  Init DOUT1 GPIO
  * @param  Initilize GPIO DOUT1 as digital ouput
  * @param  output_num_u8
  * @retval None
  */
void Driver_GPIO_Dout1_Init(uint8_t output_num_u8){
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  LL_GPIO_ResetOutputPin(DOUT1_GPIO_Port, DOUT1_Pin);
  
  if(gpioOutputPortsPins[output_num_u8].pin == DOUT1_Pin)
  {
    GPIO_InitStruct.Pin = DOUT1_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(DOUT1_GPIO_Port, &GPIO_InitStruct);
  }
}

/**
  ********************************************************************************************************************************
  * @brief   Read GPIO input state
  * @details Read GPIO input state of GPIO input number output_num_u8
  * @param   input_no_u8
  * @return  None
  ********************************************************************************************************************************
*/
uint8_t Gpio_Driver_ReadGpioInputState(uint8_t input_no_u8){
  uint8_t bitstatus = PIN_RESET;
  
  /* Check the parameters */
  assert_param(IS_GPIO_PIN(gpioInputPortsPins[input_no_u8].pin));

  if ((gpioInputPortsPins[input_no_u8].gpioPort->IDR & gpioInputPortsPins[input_no_u8].pin) != 0x00u)
  {
    bitstatus = (uint8_t)PIN_SET;
  }
  else
  {
    bitstatus = (uint8_t)PIN_RESET;
  }
  return bitstatus;  
}

/**
  ********************************************************************************************************************************
  * @brief   Read GPIO output state
  * @details Read GPIO output state of GPIO output number output_num_u8
  * @param   output_num_u8
  * @return  None
  ********************************************************************************************************************************
*/
uint8_t Driver_Gpio_ReadGpioOutputState(uint8_t output_num_u8){
  uint8_t current_state_u8;
  uint32_t bit_value_u32;
  bit_value_u32 = (gpioOutputPortsPins[output_num_u8].gpioPort->ODR);
  bit_value_u32 &= (gpioOutputPortsPins[output_num_u8].pin);
  if( bit_value_u32 != 0x00u)
  {
    current_state_u8 = (uint8_t)PIN_SET;
  }
  else
  {
    current_state_u8 = (uint8_t)PIN_RESET;
  }
  return(current_state_u8);
  
}

/**
  ********************************************************************************************************************************
  * @brief   Toggle the specified GPIO pin.
  * @details 
  * @param   GPIO_Pin specifies the pin to be toggled.
  * @return  None
  ********************************************************************************************************************************
*/
void Driver_Gpio_TogglePin(uint8_t output_num_u8)//GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
  /* Check the parameters */
  assert_param(IS_GPIO_PIN(IS_GPIO_PIN(gpioOutputPortsPins[output_num_u8].pin)));
  if (( (gpioOutputPortsPins[output_num_u8].gpioPort->ODR) & (gpioOutputPortsPins[output_num_u8].pin) ) != 0x00u)
  {
    gpioOutputPortsPins[output_num_u8].gpioPort->BRR |= (uint32_t)gpioOutputPortsPins[output_num_u8].pin;
  } else {
    gpioOutputPortsPins[output_num_u8].gpioPort->BSRR |=  (uint32_t)gpioOutputPortsPins[output_num_u8].pin;
  }
}

/**
  ********************************************************************************************************************************
  * @brief   Set GPIO output state
  * @details Set GPIO output to high or low for the GPIO output number output_num_u8
  * @param   output_num_u8, PinState
  * @return  None
  ********************************************************************************************************************************
*/
void Driver_Gpio_WriteGpioState(uint8_t output_num_u8, uint8_t PinState){
  
    /* Check the parameters */
  assert_param(IS_GPIO_PIN(gpioOutputPortsPins[output_num_u8].pin));
  assert_param(IS_GPIO_PIN_ACTION(PinState));
  
  if (PinState != PIN_RESET) // Since high state 
  {
    gpioOutputPortsPins[output_num_u8].gpioPort->BSRR = (uint32_t)gpioOutputPortsPins[output_num_u8].pin;
  }
  else
  {
    gpioOutputPortsPins[output_num_u8].gpioPort->BRR = (uint32_t)gpioOutputPortsPins[output_num_u8].pin;
  }
}

/**
  ********************************************************************************************************************************
  * @brief   Start PWM output
  * @details Initilize TIM3 and enable PWM ouptut
  * @param   None 
  * @return  None
  ********************************************************************************************************************************
*/
void Driver_Gpio_StartPwmOut(uint8_t output_num_u8){
  if(gpioOutputPortsPins[output_num_u8].pin == DOUT1_Pin)
  {
    MX_TIM3_Init(); // Initilize the timer for PWM output
  
  
    
  }
}

/**
  ********************************************************************************************************************************
  * @brief   Stop PWM ouput
  * @details PWM output is disabled
  * @param   None 
  * @return  None
  ********************************************************************************************************************************
*/
void Driver_Gpio_StopPwmOut(uint8_t output_num_u8){
  if(gpioOutputPortsPins[output_num_u8].pin == DOUT1_Pin)
  { // Only Dout1 pin support timer for PWM 
    // Main output disable
    TIM3->BDTR |= (uint32_t)((0x1UL << TIM_BDTR_MOE_Pos));
  }
}

/**
  ********************************************************************************************************************************
  * @brief   Set PWM ouput polarity
  * @details Set  PWM output poloarity to ACTIVE_LOW or ACTIVE_HIGH
  * @param   polarity_value 
  * @return  None
  ********************************************************************************************************************************
*/
void Driver_Gpio_SetPwmOutPolarity(uint8_t output_num_u8, uint8_t polarity_value){
  if(gpioOutputPortsPins[output_num_u8].pin == DOUT1_Pin)
  { // Only Dout1 pin support timer for PWM 
    if(polarity_value == ACTIVE_LOW){ // Active Low
      TIM3->CCER |= TIM_CCER_CC2P; // 1
    } else{ // Active High
      TIM3->CCER |= (0x0UL << TIM_CCER_CC2P_Pos); // 0
    }
  }
}

/**
  ********************************************************************************************************************************
  * @brief   Set PWM output frequency
  * @details Set PWM output frequency.
  * @param   output_num_u8, pwm_frequency_u16 (in Hz)
  * @return  None
  ********************************************************************************************************************************
*/
void Driver_Gpio_SetPwmOutPeriod(uint8_t output_num_u8, uint16_t pwm_frequency_u16){
  if(gpioOutputPortsPins[output_num_u8].pin == DOUT1_Pin)
  { // Only Dout1 pin support timer for PWM 
    pwmPeriod_u32 = (uint32_t)((CLOCK_FREQ)* (1/((float)(TIM3_CH1_PWM_OUT_PRESCALER_VALUE+1))));
    pwmPeriod_u32 = (uint32_t)(pwmPeriod_u32 * (1/((float)pwm_frequency_u16)));
    TIM3->ARR = (uint32_t)pwmPeriod_u32;
  }
}

/**
  ********************************************************************************************************************************
  * @brief   Set PWM output duty cycle
  * @details Set PWM output duty cycle.
  * @param   output_num_u8, pwm_duty_cycle_u16 (xxxyy = xxx.yy%)
  * @return  None
  ********************************************************************************************************************************
*/
void Driver_Gpio_SetPwmOutDutyCycle(uint8_t output_num_u8, uint16_t pwm_duty_cycle_u16){
  if(gpioOutputPortsPins[output_num_u8].pin == DOUT1_Pin)
  { // Only Dout1 pin support timer for PWM 
    //Set Duty Cycle to 0 to disable PWM output. Any value above zero will start the PWM
    float duty_cycle_period_f= 0;
    duty_cycle_period_f = (float)(pwm_duty_cycle_u16/((float)100));
    duty_cycle_period_f = (uint32_t)(pwmPeriod_u32*duty_cycle_period_f/((float)100));
    TIM3->CCR2 = (uint32_t)duty_cycle_period_f;
  }
}