/**
********************************************************************************************************************************
* @file    module_gpio.c 
* @author  Satya Akkina
* @brief   Main driver module for GPIO
* @details    
********************************************************************************************************************************
*/

/* Includes --------------------------------------------------------------------------------------------------------------------*/
//#include "driver_gpio.h"
#include "module_gpio.h"

//#include <stdio.h> // Only needed when using printf

/* scheduler handle declaration */
#include "scheduler.h"
#include "structured_memory.h"

/* Content ---------------------------------------------------------------------------------------------------------------------*/
// Function prototypes
uint16_t updateGpioStatus(uint16_t current_value, uint8_t position, uint8_t new_bit_value);
void AssignModuleMemGpio(void);
uint8_t GetDebouncedDigitalInputValue(uint8_t input_no_u8, uint8_t new_bit_value);
void InitGpioSettingsData(void);

// -- Module States
enum {
  MEMORY_INIT_MODULE,
  INIT_MODULE,
  RUN_MODULE,
  // additional states to be added here as necessary.
  IRQ_MODULE = DEFAULT_IRQ_STATE,
  KILL_MODULE = KILL_APP
};

// - External Variables
extern Ram_Buf sharedMemArray[STRUCT_MEM_ARRAY_SIZE];
extern ProcessInfo processInfoTable[]; 

// - Global variables specific to this module
Gpio_Control gpio_Control;
static Ram_Buf_Handle gpio_StructMem_u32;
uint8_t current_debounce_count[TOTAL_DIGITAL_INPUTS];

/**
********************************************************************************************************************************
* @brief   State machine for gpio module
* @details
* @param   drv_identifier_u8, previous_state_u8, next_stat_u8, irq_identfier_u8
* @retval  return_state_u8
********************************************************************************************************************************
*/
uint8_t moduleGPIO_u32(uint8_t drv_id_u8, uint8_t prev_state_u8, uint8_t next_state_u8, uint8_t irq_id_u8) 
{
  uint8_t return_state_u8 = MEMORY_INIT_MODULE;
  switch (next_state_u8) 
  {
  case MEMORY_INIT_MODULE:
    {
      AssignModuleMemGpio(); // Assign structured memory
      return_state_u8 = INIT_MODULE;
      break;
    }
  case INIT_MODULE: 
    {
      //AssignModuleMemGpio();
      InitGpioSettingsData();
      // Initialize GPIO.
      GPIOInit();
      return_state_u8 = RUN_MODULE;
      break;
    }
  case RUN_MODULE: 
    {
      // Loop through all inputs and update the status variable. 
      for (uint8_t input_number_u8 = 0; input_number_u8 < TOTAL_DIGITAL_INPUTS; input_number_u8++){
        uint8_t state_value_u8=0;
        state_value_u8 = Gpio_Driver_ReadGpioInputState(input_number_u8);
        state_value_u8 = GetDebouncedDigitalInputValue(input_number_u8, state_value_u8); // Debounce input state
        gpio_Control.gpio_Result.gpio_Status_u16 = updateGpioStatus(gpio_Control.gpio_Result.gpio_Status_u16, input_number_u8, state_value_u8);            
      }          
      return_state_u8 = RUN_MODULE;
      break;
    }
  case KILL_MODULE: 
    {
      // The GPIO driver module must only be executed once.
      // Setting processStatus_u8 to PROCESS_STATUS_KILLED prevents the scheduler main loop from calling this module again.
      uint8_t table_index_u8 = getProcessInfoIndex(drv_id_u8);
      if (table_index_u8 != INDEX_NOT_FOUND) {
        processInfoTable[table_index_u8].Sched_DrvData.processStatus_u8 = PROCESS_STATUS_KILLED;
      }
      return_state_u8 = KILL_MODULE;
      break;
    }
  default:
    {
      return_state_u8 = 10;
      break;
    }
  }
  return return_state_u8;
}


/**
********************************************************************************************************************************
* @brief   Assign structured memory
* @details Assign structured memory gpio_Control
* @param   None
* @retval  None
********************************************************************************************************************************
*/
void AssignModuleMemGpio(void){   
  gpio_StructMem_u32 =  StructMem_CreateInstance(MODULE_GPIO, sizeof(Gpio_Control), ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);
  (*gpio_StructMem_u32).p_ramBuf_u8 = (uint8_t *)&gpio_Control ;    // Map the ADC1 memory into the structured memory
  uint8_t module_gpio_index_u8 = getProcessInfoIndex(MODULE_GPIO);
  processInfoTable[module_gpio_index_u8].Sched_ModuleData.p_masterSharedMem_u32 = (Ram_Buf_Handle)gpio_StructMem_u32;
}


void InitGpioSettingsData(void)
{
  gpio_Control.gpio_Settings.debounceCountLimit_u8 = 4;
  
  // Init local variables
  for (uint8_t input_number_u8 = 0; input_number_u8 < TOTAL_DIGITAL_INPUTS; input_number_u8++){
    current_debounce_count[input_number_u8] = 0;
  }
}

/**
********************************************************************************************************************************
* @brief   Debounce Digital Inputs
* @details Apply debounce to input. If the new value does not stay the same for at least debounceCountLimit_u8, revert to previous know good value
* @param   input_num_u8, new_bit_value
* @param   
* @retval  debounced_value_u8
********************************************************************************************************************************
*/
uint8_t GetDebouncedDigitalInputValue(uint8_t input_num_u8, uint8_t new_bit_value){
  uint16_t mask = 1 << input_num_u8; 
  uint8_t stable_value_u8 =   (gpio_Control.gpio_Result.gpio_Status_u16 & mask) >> input_num_u8;
  
  uint8_t debounced_value_u8 = stable_value_u8;
  if(new_bit_value != stable_value_u8) // Input state changed.
  {
    current_debounce_count[input_num_u8]++;
    if(current_debounce_count[input_num_u8] >= gpio_Control.gpio_Settings.debounceCountLimit_u8)
    { // Debounce count satisified
      debounced_value_u8 = new_bit_value;
    }
  } else{ // No change in input state.
    current_debounce_count[input_num_u8]=0;
    debounced_value_u8 = stable_value_u8;
  } 
  return(debounced_value_u8);
}

/**
********************************************************************************************************************************
* @brief   Get status of GPIO input and update the status flags
* @details 
* @param   current_value, position, new_bit_value
* @retval  new value of status flags
********************************************************************************************************************************
*/
uint16_t updateGpioStatus(uint16_t current_value, uint8_t position, uint8_t new_bit_value) 
{ 
  uint16_t mask = 1 << position; 
  return (current_value & ~mask) | (((uint16_t)(new_bit_value << position)) & mask); 
} 

/**
********************************************************************************************************************************
* @brief   Start PWM output
* @details 
* @param   output_num_u8
* @retval  None
********************************************************************************************************************************
*/
void Module_Gpio_StartPwmOut(uint8_t output_num_u8){
  Driver_Gpio_StartPwmOut(output_num_u8);
}

/**
********************************************************************************************************************************
* @brief   Stop PWM output
* @details 
* @param   output_num_u8
* @retval  None
********************************************************************************************************************************
*/
void Module_Gpio_StopPwmOut(uint8_t output_num_u8){
  Driver_Gpio_StopPwmOut(output_num_u8);
}

/**
********************************************************************************************************************************
* @brief   Set PWM output polarity
* @details 
* @param   output_num_u8, polarity_value
* @retval  None
********************************************************************************************************************************
*/
void Module_Gpio_SetPwmOutPolarity(uint8_t output_num_u8, uint8_t polarity_value){
  Driver_Gpio_SetPwmOutPolarity(output_num_u8, polarity_value);
}

/**
********************************************************************************************************************************
* @brief   Set PWM output period
* @details 
* @param   output_num_u8, pwm_frequency
* @retval  None
********************************************************************************************************************************
*/
void Module_Gpio_SetPwmOutPeriod(uint8_t output_num_u8, uint16_t pwm_frequency){
  Driver_Gpio_SetPwmOutPeriod(output_num_u8, pwm_frequency);
}

/**
********************************************************************************************************************************
* @brief   Set PWM output duty cycle
* @details 
* @param   output_num_u8, pwm_period
* @retval  None
********************************************************************************************************************************
*/
void Module_Gpio_SetPwmOutDutyCycle(uint8_t output_num_u8, uint16_t pwm_period){
  Driver_Gpio_SetPwmOutDutyCycle(output_num_u8, pwm_period);
}

/**
********************************************************************************************************************************
* @brief   Read GPIO state
* @details 
* @param   output_num_u8
* @retval  GPIO state
********************************************************************************************************************************
*/
uint8_t Module_Gpio_ReadGpioOutputState(uint8_t output_num_u8)
{
  return(Driver_Gpio_ReadGpioOutputState(output_num_u8));
}

/**
********************************************************************************************************************************
* @brief   Toggle GPIO output state
* @details 
* @param   output_num_u8
* @retval  None
********************************************************************************************************************************
*/
void Module_Gpio_TogglePin(uint8_t output_num_u8){
  Driver_Gpio_TogglePin(output_num_u8);
}

/**
********************************************************************************************************************************
* @brief   Set GPIO output state
* @details 
* @param   output_num_u8, pin_State_u8
* @retval  None
********************************************************************************************************************************
*/
void Module_Gpio_WriteGpioState(uint8_t output_num_u8, uint8_t pin_State_u8){
  Driver_Gpio_WriteGpioState(output_num_u8, pin_State_u8);  
}

/**
********************************************************************************************************************************
* @brief   Init Dout1 as GPIO output
* @details 
* @param   output_num_u8, pin_State_u8
* @retval  None
********************************************************************************************************************************
*/
void Module_Gpio_Dout1_Init(uint8_t input_no_u8)
{
  Driver_GPIO_Dout1_Init(input_no_u8);
}