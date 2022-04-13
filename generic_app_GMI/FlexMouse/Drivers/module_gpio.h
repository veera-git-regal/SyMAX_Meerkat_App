/**
  *****************************************************************************
  * @file    module_gpio.h 
  * @author  Pamela Lee
  * @brief   Main driver module for GPIO
  *****************************************************************************
  */

// Define to prevent recursive inclusion --------------------------------------
#ifndef _MODULE_GPIO_H_
#define _MODULE_GPIO_H_

// Includes -------------------------------------------------------------------
#include "main.h"
#include "typedef.h"

#include "driver_gpio.h" 

#include "sequential_memory.h"
#include "structured_memory.h"
#include "scheduler.h"

//------------------- GPIO Control (inside shared memory) ---------------------  
// GPIO status are stored here
struct Gpio_Settings
{
  uint8_t debounceCountLimit_u8;
};

struct Gpio_Result
{
  uint16_t gpio_Status_u16;
  uint8_t pwm_Status_u8;
  uint8_t  errorCode_u8; 
};

//Main structure used by other modules
typedef struct{
 struct Gpio_Settings gpio_Settings;
 struct Gpio_Result gpio_Result ; 
}Gpio_Control;

//------------------ end of GPIO Control (inside shared memory) --------------- 

// Global funciton prototypes -------------------------------------------------
void Module_Gpio_StartPwmOut(uint8_t output_num_u8);
void Module_Gpio_StopPwmOut(uint8_t output_num_u8);
void Module_Gpio_SetPwmOutPolarity(uint8_t output_num_u8, uint8_t polarity_value);
void Module_Gpio_SetPwmOutPeriod(uint8_t output_num_u8, uint16_t pwm_frequency);
void Module_Gpio_SetPwmOutDutyCycle(uint8_t output_num_u8, uint16_t pwm_duty_cycle);
uint8_t Module_Gpio_ReadGpioOutputState(uint8_t output_num_u8);
void Module_Gpio_TogglePin(uint8_t output_num_u8);
void Module_Gpio_WriteGpioState(uint8_t output_num_u8, uint8_t pin_State_u8);
void Module_Gpio_Dout1_Init(uint8_t input_no_u8);

// End of content -------------------------------------------------------------
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _MODULE_GPIO_H_ */