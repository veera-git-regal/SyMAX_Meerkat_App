/**
  ********************************************************************************************************************************
  * @file    driver_timed_input.c 
  * @author  Logan Schaufler
  * @brief   Digital Input timed for use as PWM Input
  * @details    
  ********************************************************************************************************************************
  */
/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "driver_pwm_input.h"
#include "main.h"

/* Content --------------------------------------------------------------------------------------------------------------------*/

/* Private Variables */
TIM_HandleTypeDef htim1;

TIM1_Control tim1_Control;

/* Captured Values (in TIM1 referenced milliseconds from original htim1 starting point) */
float pwm_input_falling_edge_capture = 0;
float pwm_input_rising_edge_capture = 0;

/* Calculations */


/* User defined number of pwm duty cycle and frequency values sampled to be averaged */
uint8_t TIM1_PWM_Average_Index = 0; //used to calculate pwm duty cycle and frequency average over specified number of samples
//uint8_t avg_exponent_u8 = 0;
//float tim1_Result_Sum_f[2] = {0,0};
//uint64_t tim1_Result_Sum_u16[2] = {0,0};
//uint8_t TIM1_PWM_averaging_index = 0;
uint8_t is_rising_edge_triggered = FALSE;
uint8_t is_falling_edge_triggered = FALSE;
uint64_t pwm_input_falling_edge_capture_sum_u64 = 0;
uint64_t pwm_input_rising_edge_capture_sum_u64 = 0;


//TIM1_Control tim1_Control;
static void MX_TIM1_Init(void);

/**
  * @brief TIM Initialization Function
  * @param None
  * @retval None
  */
void PWMInputTimerInit(void)
{
  MX_TIM1_Init();
}

/**
  * @brief Init input as a PWM input
  * @param None
  * @retval None
  */
void PWMInput_GPIO_PwmInput_Init(){
  
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  
   /**/
  //LL_EXTI_SetEXTISource(LL_EXTI_CONFIG_PORTA, LL_EXTI_CONFIG_LINE8);
  LL_EXTI_SetEXTISource(PWM_IN_EXTI_CONFIG_Port, PWN_IN_EXTI_CONFIG_Line);

  /**/
  EXTI_InitStruct.Line_0_31 = PWM_IN_EXTI_Line;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  LL_GPIO_SetPinPull(PWM_IN_GPIO_Port, PWM_IN_Pin, LL_GPIO_PULL_NO);

  /**/
  LL_GPIO_SetPinMode(PWM_IN_GPIO_Port, PWM_IN_Pin, LL_GPIO_MODE_INPUT); 
}


/**
  * @brief Init input as a digital input
  * @param None
  * @retval None
  */
void PWMInput_GPIO_DigitalInput_Init()
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  GPIO_InitStruct.Pin = PWM_IN_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(PWM_IN_GPIO_Port, &GPIO_InitStruct);
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

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 20; // 20 supports functional pwm input 50hz - 10khz (50Hz max) 
  // - 50Hz = Period count of 61100 (overflow at 65536)
  // - 47Hz = Period count of 65011
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xFFFF;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

      /*## Start the Input Capture in interrupt mode ##########################*/
  if (HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }

  /*## Start the Input Capture in interrupt mode ##########################*/
  if (HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }

  /* USER CODE END TIM1_Init 2 */

}

/*void CalculateSamplingExponentPWMInput()  //Does nothing currently, for future use in optimizing code
{
  uint8_t result_u8 = PWM_INPUT_AVERAGING_BUF_SIZE;
  do{
    avg_exponent_u8++;
    result_u8 =(uint8_t)(result_u8/2);
  }while(result_u8);
  avg_exponent_u8--;
}*/

/**
  * @brief  Input Capture callback in non blocking mode; LS All Timer Input captures are handled here via HAL_TIM_ACTIVE_CHANNEL_x 
  * @param  htim : TIM IC handle
  * @retval None
  */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  // Rising edge triggerred
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    is_rising_edge_triggered = TRUE;

    if (is_falling_edge_triggered)
    {
      /* Get the Input Capture value */
      if (TIM1_PWM_Average_Index < PWM_INPUT_AVERAGING_BUF_SIZE)
      {
        // Captured rising edge value stored in terms of ticks
        pwm_input_rising_edge_capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        if (pwm_input_rising_edge_capture != 0)
        {
          pwm_input_falling_edge_capture_sum_u64 += pwm_input_falling_edge_capture; //Total sum of falling edge ticks
          pwm_input_rising_edge_capture_sum_u64 += pwm_input_rising_edge_capture;   //Total sum of rising edge ticks
          TIM1_PWM_Average_Index++;
        } 
      }        
        
      if (TIM1_PWM_Average_Index == PWM_INPUT_AVERAGING_BUF_SIZE)
      {
        //FOR FUTURE USE             
        //tim1_Result_Sum_u16[0] = (uint64_t)(tim1_Result_Sum_f[0]);
        //tim1_Result_Sum_u16[1] = (uint64_t)(tim1_Result_Sum_f[1]);
        //tim1_Control.tim1_ResultAvg.PWMInputDutyCycleAverageCalc_f = (float)(tim1_Result_Sum_u16[0] >> avg_exponent_u8 );
        //tim1_Control.tim1_ResultAvg.PWMInputFrequencyAverageCalc_u16 = (uint16_t)(tim1_Result_Sum_u16[1] >> avg_exponent_u8 );

        tim1_Control.tim1_ResultAvg.PWMInputDutyCycleAverageCalc_f = (float)((pwm_input_falling_edge_capture_sum_u64 * 100) / (float)pwm_input_rising_edge_capture_sum_u64);    //PWM Input Duty Cycle calculation // Divition automatically does the averagind of the captures
        tim1_Control.tim1_ResultAvg.PWMInputFrequencyAverageCalc_u16 = (uint16_t)((HAL_RCC_GetSysClockFreq() * PWM_INPUT_AVERAGING_BUF_SIZE) / (float)((htim1.Init.Prescaler + 1) * pwm_input_rising_edge_capture_sum_u64));  //PWM Input Frequency Calculation

        TIM1_PWM_Average_Index = 0;
        pwm_input_falling_edge_capture_sum_u64 = 0;
        pwm_input_rising_edge_capture_sum_u64 = 0;
      } 
    }
  }
      
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
  {
    // Falling edge triggerred
    if (is_rising_edge_triggered)
    {
      // Captured falling edge value stored in terms of ticks
      pwm_input_falling_edge_capture = (HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2));
      is_falling_edge_triggered = TRUE;
    }
  }
}
