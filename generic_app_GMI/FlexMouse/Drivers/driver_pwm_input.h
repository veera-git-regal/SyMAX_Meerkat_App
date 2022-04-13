/**
  ********************************************************************************************************************************
  * @file    driver_pwm_input.h 
  * @author  Logan Schaufler
  * @brief   PWM input driver which provides duty cycle and frequency of input pwm
  * @details 
  ********************************************************************************************************************************
  */

/* Define to prevent recursive inclusion ---------------------------------------------------------------------------------------*/
#ifndef _DRIVER_PWM_INPUT_H_
#define _DRIVER_PWM_INPUT_H_

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include <stdint.h>
#include "typedef.h"

#define PWM_INPUT_AVERAGING_BUF_SIZE 5 // number of samples to take per stored averaged duty cycle

//******************* TIM1 Control (inside shared memory) *******************************************************************************************************************************  
//Average Results from TIM1 are stored here
struct TIM1_ResultAvg
{
  float    PWMInputDutyCycleAverageCalc_f;     //pwm TIM1 average duty cycle
  uint16_t PWMInputFrequencyAverageCalc_u16;   //pwm TIM1 average frequency
  //uint8_t  PWMInputDigitalInputState_u8;     //pwm TIM1 state for when digital input is selected as alternate function
  uint8_t  errorCode_u8; 
};


//Main structure used by other modules
typedef struct{
 struct TIM1_ResultAvg tim1_ResultAvg;  
}TIM1_Control;

/**
  ********************************************************************************************************************************
  * @brief    
  * @details 
  ********************************************************************************************************************************
  */
void PWMInputTimerInit(void);
void CalculateSamplingExponentPWMInput(void);
void PWMInput_GPIO_DigitalInput_Init();
void PWMInput_GPIO_PwmInput_Init();

#endif // _DRIVER_PWM_INPUT_H_
