/**
********************************************************************************************************************************
* @file    module_digital_inputs.c 
* @author  Satya Akkina
* @brief   This is a module that handles digital inputs.
* @details Reads digital inputs from driver module and provides status based on settings and input state.
********************************************************************************************************************************
*/

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "module_digital_inputs.h"

#include <stdio.h>

#include "macros.h"

/* Content ---------------------------------------------------------------------------------------------------------------------*/
extern Ram_Buf sharedMemArray[TOTAL_NUM_OF_STRUCT_MEM_INSTANCES];
extern ProcessInfo processInfoTable[];
static  Ram_Buf_Handle digital_Inputs_StructMem_u32;

// Local structure pointers
Gpio_Control* gpio_LocalControl;
DigitalInputs_Control  digitalInputs_Control;

// Local Function Definitions
void AssignModuleMemDigitalInputs(void);
void FindDiscreteDemandInputs(void);
void InitDigtialInputSetting(void);
void UpdateInputState(uint8_t input_no_u8);
void ProcessDigitalInputs(uint8_t input_no_u8, uint8_t state_value_u8);
void ProcessDiscreteDemand(void);
uint8_t GetDebouncedValue(uint8_t input_no_u8, uint8_t current_value_u8);

// Local variables
uint64_t module_digital_inputs_poll_time_u64 =0; // Poll time for the module
uint8_t debouce_count_u8[TOTAL_DIGITAL_INPUTS]; // Stores current debounce count for each input
uint8_t previous_input_state_u8[TOTAL_DIGITAL_INPUTS]; // Store previous state on inputs
uint8_t discrete_demand_input_nos_u8[MAX_DISCRETE_DEMAND_INPUTS]; // store the digital input no's corresponding to discrete demand inputs
uint8_t discrete_demand_inputs_count_u8 = 0; // Count of digital inputs set to discrete demand input mode

enum {
  MEMORY_INIT_MODULE,
  INIT_MODULE,
  RUN_MODULE,
  // Additional states to be added here as necessary.
  IRQ_MODULE = DEFAULT_IRQ_STATE,
  KILL_MODULE = KILL_APP
};


/**
********************************************************************************************************************************
* @brief   State machine for Ditial Inputs module
* @details
* @param   drv_identifier_u8, previous_state_u8, next_stat_u8, irq_identfier_u8
* @retval  return_state_u8
********************************************************************************************************************************
*/
uint8_t module_Digital_Inputs_u32(uint8_t drv_identifier_u8, uint8_t previous_state_u8, uint8_t next_state_u8,
                                  uint8_t irq_identifier_u8)
{
  uint8_t return_state_u8 = MEMORY_INIT_MODULE;
  switch (next_state_u8)
  {
  case MEMORY_INIT_MODULE:
    {
      AssignModuleMemDigitalInputs(); // Assign structured memory
      return_state_u8 = INIT_MODULE;
      break;
    }
  case INIT_MODULE:
    {     
      AssignModuleMemDigitalInputs(); // Assign structured memory to DigitalInputs setting and data
      
      InitDigtialInputSetting(); // Initilize DigtialInputs settings      
      
      // Get structured memory for GPIO module data
      uint8_t module_gpio_index_u8 = getProcessInfoIndex(MODULE_GPIO);
      gpio_LocalControl = (Gpio_Control*)((*(processInfoTable[module_gpio_index_u8].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8);
      
      module_digital_inputs_poll_time_u64 = getSysCount() + DIGITAL_INPUT_POLL_TIME; // Digital input poll period // Store time tick value
      
      return_state_u8 = RUN_MODULE;
      break;
      
    }   
  case RUN_MODULE:                                                             
    {
      // Process digital inputs every "DIGITAL_INPUT_POLL_TIME" mSec
      if (getSysCount() >= module_digital_inputs_poll_time_u64) // Time above poll time
      {
        uint8_t state_value_u8 = BIT_LOW;        
        
        digitalInputs_Control.digitalInputs_Data.digitalInputStatus_u16 = (*gpio_LocalControl).gpio_Result.gpio_Status_u16 ; // Get input status from driver module
        for (uint8_t input_no_u8 = 0; input_no_u8 < TOTAL_DIGITAL_INPUTS; input_no_u8++)
        { // Update the digital input flags
          UpdateInputState(input_no_u8);
          
          state_value_u8 = (uint8_t)(((digitalInputs_Control.digitalInputs_Data.digitalInputStatusFiltered_u16) >> input_no_u8) & 1 );
          ProcessDigitalInputs(input_no_u8, state_value_u8); 
        }
        if (discrete_demand_inputs_count_u8 != 0) // One of the input is set for discrete demand inputs
        {
          ProcessDiscreteDemand(); // Find the demand
        }
        module_digital_inputs_poll_time_u64 = getSysCount() + DIGITAL_INPUT_POLL_TIME; // Next poll time
      }
      return_state_u8 = RUN_MODULE;
      break;
    }
  case IRQ_MODULE: 
    {
      // If there are more than one interrupts, from different drivers, you can identify each individually by:
      // tableIndex_u8 = getProcessInfoIndex(irq_identifier_u8);
      // Then use processInfoTable[tableIndex_u8] to tailor your response appropriately.
      return_state_u8 = RUN_MODULE;
      break;
    }
    
  case KILL_MODULE: 
    {
      // Setting processStatus_u8 to PROCESS_STATUS_KILLED prevents the scheduler main loop from calling this module again.
      uint8_t table_index_u8 = getProcessInfoIndex(drv_identifier_u8);
      if (table_index_u8 != INDEX_NOT_FOUND) {
        processInfoTable[table_index_u8].Sched_DrvData.processStatus_u8 = PROCESS_STATUS_KILLED;
      }
      return_state_u8 = INIT_MODULE;
      break;
    }
  default:
    {
      return_state_u8 = KILL_MODULE; //10; 
      break;
    }
  }
  return return_state_u8;
} 

/**
********************************************************************************************************************************
* @brief   Assign structured memory
* @details Assign structured memory for Digital Input control
* @param   None 
* @return  None
********************************************************************************************************************************
*/
//
void AssignModuleMemDigitalInputs(void){   
  digital_Inputs_StructMem_u32 =  StructMem_CreateInstance(MODULE_DIGITAL_INPUTS, sizeof(DigitalInputs_Control), ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);
  (*digital_Inputs_StructMem_u32).p_ramBuf_u8 = (uint8_t *)&digitalInputs_Control ;    // Map the ADC1 memory into the structured memory
  uint8_t module_digital_inputs_index_u8 = getProcessInfoIndex(MODULE_DIGITAL_INPUTS);
  processInfoTable[module_digital_inputs_index_u8].Sched_ModuleData.p_masterSharedMem_u32 = (Ram_Buf_Handle)digital_Inputs_StructMem_u32;
}

/**
********************************************************************************************************************************
* @brief   Initilize all digital input settings and live data
* @details Read settings from the RAM and initilize the settings. This function need run in order to update the input function/mode.
* @param   None 
* @retval  None
********************************************************************************************************************************
*/
void InitDigtialInputSetting(void){   
  // Init digital input settings
  digitalInputs_Control.digitalInputs_Setting.inputFunction[0]= SET_DIRECTION; // Function of each digital input
  digitalInputs_Control.digitalInputs_Setting.inputFunction[1]= SET_DEMAND_INPUT_0; // Function of each digital input
  digitalInputs_Control.digitalInputs_Setting.inputFunction[2]= SET_DEMAND_INPUT_1; // Function of each digital input
  digitalInputs_Control.digitalInputs_Setting.inputFunction[3]= SET_DEMAND_INPUT_2; // Function of each digital input
 // digitalInputs_Control.digitalInputs_Setting.inputFunction[4]= SET_DIRECTION; // Function of each digital input
  digitalInputs_Control.digitalInputs_Setting.debounceCountLimit_u16 = 4; // # of counts digital input should be active high/low to consider it as active high/low
  digitalInputs_Control.digitalInputs_Setting.inputPolarity_u16 = 0;   // Each bit indicates the polarity of the input. 0= Normally Open, 1 = Normally closed. Bit 0 = Digital input 0 polarity.
  digitalInputs_Control.digitalInputs_Setting.inputEnable_u16 = 0xFFFF;  // Each bit enables an individual digital input. Bit 0 = Digital input 0 enable. 
  
  digitalInputs_Control.digitalInputs_Setting.demandPercent[0] = 0;    // 0%
  digitalInputs_Control.digitalInputs_Setting.demandPercent[1] = 4000; // Percentage*100 (ex. 3000 = 30.00%)
  digitalInputs_Control.digitalInputs_Setting.demandPercent[2] = 5000; //
  digitalInputs_Control.digitalInputs_Setting.demandPercent[3] = 6000; //
  digitalInputs_Control.digitalInputs_Setting.demandPercent[4] = 7000; //
  digitalInputs_Control.digitalInputs_Setting.demandPercent[5] = 8000; //
  digitalInputs_Control.digitalInputs_Setting.demandPercent[6] = 9000; //
  digitalInputs_Control.digitalInputs_Setting.demandPercent[7] = 10000; //
  
  // Init metering data
  digitalInputs_Control.digitalInputs_Data.digitalInputStatus_u16 = 0;
  digitalInputs_Control.digitalInputs_Data.digitalInputStatusFiltered_u16 = 0;
  digitalInputs_Control.digitalInputs_Data.discreteDemandPercent = 0;
  digitalInputs_Control.digitalInputs_Data.discretes_u16.is_invertDirection = FALSE;
  uint8_t count_motor_enable_input_u8 = 0;
  for (uint8_t index_u8 = 0; index_u8 < MAX_DIGITAL_INPUTS; index_u8++)
  {
    digitalInputs_Control.digitalInputs_Data.digitalInputsValue[index_u8] = 0;
    discrete_demand_input_nos_u8[index_u8] = 0xFF;
    if(digitalInputs_Control.digitalInputs_Setting.inputFunction[index_u8] == MOTOR_ENABLE)
    {
      count_motor_enable_input_u8++;
    }    
  }
  if(count_motor_enable_input_u8 > 0) // Check if one of the inputs is MOTOR_ENABLE
  {
    digitalInputs_Control.digitalInputs_Data.discretes_u16.is_motorEnabled = FALSE;
  } else {
    digitalInputs_Control.digitalInputs_Data.discretes_u16.is_motorEnabled = TRUE;
  }
  
  // Initilize local variables
  FindDiscreteDemandInputs();  // Find the input that are set for discrete demand
  
  for (uint8_t index_u8= 0; index_u8 < TOTAL_DIGITAL_INPUTS; index_u8++){ 
    previous_input_state_u8[index_u8]=0;
    debouce_count_u8[index_u8] = 0;
  }
}

/**
********************************************************************************************************************************
* @brief   Update digital input flags
* @details Update digital input flags based on settings (enable and polarity).
* @param   input_no_u8, Digial input no 
* @param   state_value_u8, 
* @retval  None
********************************************************************************************************************************
*/
void UpdateInputState(uint8_t input_no_u8){
  uint8_t state_value_u8 = BIT_LOW;
  if(((digitalInputs_Control.digitalInputs_Setting.inputEnable_u16) >> input_no_u8) & 1 == BIT_HI){
    state_value_u8 = (uint8_t)(((digitalInputs_Control.digitalInputs_Data.digitalInputStatus_u16) >> input_no_u8) & 1 ); // Get current input state value
    //state_value_u8 = GetDebouncedValue(input_no_u8, state_value_u8); // Check for debounce // Debounce is implemented in module_gpio
    if(((digitalInputs_Control.digitalInputs_Setting.inputPolarity_u16) >> input_no_u8) & 1 == BIT_HI) //Invert polarity is enabled
    { 
      // Flip the read state
      if(state_value_u8 == 0){
        state_value_u8 = BIT_HI;
      } else{
        state_value_u8 = BIT_LOW;
      }
    }
    // Update filtered digital input state
    if(state_value_u8 == BIT_HI){
      BIT_SET(digitalInputs_Control.digitalInputs_Data.digitalInputStatusFiltered_u16, input_no_u8);
    } else{
      BIT_CLEAR(digitalInputs_Control.digitalInputs_Data.digitalInputStatusFiltered_u16, input_no_u8);
    }
  } else{ // Digital input disabled
    if(((digitalInputs_Control.digitalInputs_Setting.inputPolarity_u16) >> input_no_u8) & 1 == BIT_HI) //Invert Polarity
    {
      state_value_u8 = BIT_HI; // Flip the read state since the polarity bit is Set.
      BIT_SET(digitalInputs_Control.digitalInputs_Data.digitalInputStatusFiltered_u16, input_no_u8);
    }
  }
}

/**
********************************************************************************************************************************
* @brief   Read inputs and set response
* @details Read inputs and input function to sets output appropriatly.
* @param   input_no_u8, state_value_u8
* @retval  None
********************************************************************************************************************************
*/
void ProcessDigitalInputs(uint8_t input_no_u8, uint8_t state_value_u8){
  
  uint16_t input_function_u16;
  
  input_function_u16 = digitalInputs_Control.digitalInputs_Setting.inputFunction[input_no_u8];
  
  switch(input_function_u16){
  case MOTOR_ENABLE:    // 0
    digitalInputs_Control.digitalInputs_Data.digitalInputsValue[input_no_u8] = state_value_u8;
    digitalInputs_Control.digitalInputs_Data.discretes_u16.is_motorEnabled = state_value_u8;
    break;
    
  case MOTOR_START:     // 1
    digitalInputs_Control.digitalInputs_Data.digitalInputsValue[input_no_u8] = state_value_u8;
    break;
    
  case SET_DIRECTION:   // 2
    digitalInputs_Control.digitalInputs_Data.digitalInputsValue[input_no_u8] = state_value_u8;
    digitalInputs_Control.digitalInputs_Data.discretes_u16.is_invertDirection = state_value_u8;
    break;
    
  case FIREMODE_ENABLE: // 3
    digitalInputs_Control.digitalInputs_Data.digitalInputsValue[input_no_u8] = state_value_u8;
    break;
    
  case ALARM_RESET:     // 4
    digitalInputs_Control.digitalInputs_Data.digitalInputsValue[input_no_u8] = state_value_u8;
    break;      
    
  case SET_DEMAND_INPUT_0: // 5    
  case SET_DEMAND_INPUT_1: // 6    
  case SET_DEMAND_INPUT_2: // 7
  case SET_DEMAND_INPUT_3: // 8
    break;
    
  default: 
    break;    
  }  
}

/**
********************************************************************************************************************************
* @brief   Find discrete demand inputs
* @details Find the input no's correspond to the discrete demand inputs and find the number of discrete demand input
* @param   None 
* @retval  None
********************************************************************************************************************************
*/
void FindDiscreteDemandInputs(){
  uint8_t count_u8=0; // Index to store input # which is set at discrete demand input
  for (uint8_t input_no_u8 = 0; input_no_u8 < TOTAL_DIGITAL_INPUTS; input_no_u8++){
    if(digitalInputs_Control.digitalInputs_Setting.inputFunction[input_no_u8] == SET_DEMAND_INPUT_0) // 3    
    {
      discrete_demand_input_nos_u8[count_u8] =  input_no_u8;
      count_u8++;
    }
    if(digitalInputs_Control.digitalInputs_Setting.inputFunction[input_no_u8] == SET_DEMAND_INPUT_1) // 4    
    {
      discrete_demand_input_nos_u8[count_u8] =  input_no_u8;
      count_u8++;
    }
    
    if(digitalInputs_Control.digitalInputs_Setting.inputFunction[input_no_u8] == SET_DEMAND_INPUT_2) // 5    
    {
      discrete_demand_input_nos_u8[count_u8] =  input_no_u8;
      count_u8++;
    }
    
    if(digitalInputs_Control.digitalInputs_Setting.inputFunction[input_no_u8] == SET_DEMAND_INPUT_3) // 6    
    {
      discrete_demand_input_nos_u8[count_u8] =  input_no_u8;
      count_u8++;
    }
  }
  discrete_demand_inputs_count_u8 = count_u8;  
}

/**
********************************************************************************************************************************
* @brief   Find the demand based on input state
* @details
* @param   None 
* @retval  None
********************************************************************************************************************************
*/
void ProcessDiscreteDemand(){
  
  uint8_t count_u8=0; // # of discrete demand inputs
  uint8_t discrete_demand_input_status_decimal_u8 = 0;
  // Find the decimal equivalent of the digital input state
  for (uint8_t input_no_u8 = 0; input_no_u8 < TOTAL_DIGITAL_INPUTS; input_no_u8++){
    if(input_no_u8 == discrete_demand_input_nos_u8[count_u8]){
      // Convert binary input value to decimal value
      discrete_demand_input_status_decimal_u8 |= (uint8_t)((((digitalInputs_Control.digitalInputs_Data.digitalInputStatusFiltered_u16) >> input_no_u8) & 1) << count_u8);
      count_u8++;  
    }
  }
  digitalInputs_Control.digitalInputs_Data.discreteDemandPercent = digitalInputs_Control.digitalInputs_Setting.demandPercent[discrete_demand_input_status_decimal_u8];
}

/**
********************************************************************************************************************************
* @brief   Debounce Digital Inputs
* @details Apply debounce to input. If the new value does not stay the same for at least debounceCountLimit_u16, revert to previous know good value
* @param   input_no_u8, Digial input no
* @param   current_value_u8, Current value of digital input status
* @retval  debounced_value_u8
********************************************************************************************************************************
*/
uint8_t GetDebouncedValue(uint8_t input_no_u8, uint8_t current_value_u8){
  uint8_t debounced_value_u8 = previous_input_state_u8[input_no_u8];
  if(previous_input_state_u8[input_no_u8] != current_value_u8) // Input state changed.
  {
    debouce_count_u8[input_no_u8]++;
    if(debouce_count_u8[input_no_u8] >= digitalInputs_Control.digitalInputs_Setting.debounceCountLimit_u16)
    { // Debounce count satisified
      previous_input_state_u8[input_no_u8] = current_value_u8;
      debounced_value_u8 = current_value_u8;
    }
  } else{ // No change in input state.
    debouce_count_u8[input_no_u8]=0;
    debounced_value_u8 = previous_input_state_u8[input_no_u8];
  } 
  return(debounced_value_u8);
}