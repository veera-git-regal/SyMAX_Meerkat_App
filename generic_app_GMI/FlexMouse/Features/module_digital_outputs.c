/**
*******************************************************************************
* @file    module_digital_outputs.c 
* @author  Satya Akkina
* @brief   Module to handle all digital ouputs
* @details Module handles PWM out, fault out, LED out and relay output
*******************************************************************************
*/

// Includes -------------------------------------------------------------------
#include "module_digital_outputs.h"

#include "module_gpio.h"
#include "macros.h"
#include "module_analog_0_10v.h"
#include "module_motor_com.h"
//#include "module_motor_demand_multiplexer.h"

//#include <stdio.h> // for debug only

// Content --------------------------------------------------------------------
// Function Prototypes
void AssignModuleMemDigitalOutputs(void);
void InitDigitalOutputsSetting(void);
void InitDigitalOutputsLocalVariables(void);
void ExecuteFaultLedOutBlinking(uint8_t output_num_u8);
void ProcessFaultLedOut(uint8_t output_num_u8);
void ProcessRelayOut(uint8_t output_num_u8);
void ProcessPwmDutyCycleOut(uint8_t output_num_u8);
void ProcessPwmFrequencyOutput(uint8_t output_num_u8);

// - Constants
#define RELAY_ON_TH_SPEED 500
// -- Module States
enum {
  MEMORY_INIT_MODULE,
  INIT_MODULE,
  RUN_MODULE,
  // Additional states to be added here as necessary.
  IRQ_MODULE = DEFAULT_IRQ_STATE,
  KILL_MODULE = KILL_APP
};
// State for Fault/LED output sequence
enum
{
  SEQ_LOW = 0,
  SEQ_HI,        // 1
  SEQ_DELAY,     // 2
  SEQ_NO_CHANGE, // 3
};

// - External Variables
extern ProcessInfo processInfoTable[];

// - Global variables specific to this module
static Ram_Buf_Handle digital_Outputs_StructMem_u32;
Digital_Outputs_Control digital_Outputs_Control;
uint8_t onBoardLedExternalSequences_u8 = 0; // Sequences that need to be executed next. Other modules can added to this
uint8_t current_external_sequence_u8 = 0;   // External sequence that is currently executed
uint8_t current_external_sequence_count_u8 = 0; // Count of sequence that finihsed executing
uint8_t is_external_led_sequence_executing = FALSE; // Flag that identifies that external sequence is being executed
uint8_t is_processed_led_fault_outputs[TOTAL_DIGITAL_OUTPUTS]; // Once LED/Fault output sequence execution is compelte update the corresponding flag.

uint8_t current_sequence_u8 = 0;        // Current sequence that is being executed
uint8_t fault_led_outputs_count_u8 = 0; // Total number of fault/led outputs
uint8_t processed_fault_led_outputs_count_u8 = 0;     // Number of fault/led ouputs that completed executing sequence
uint8_t new_sequence_u8 = 0; // First finish current sequence and then execute this. Executes only when is_sequence_executing = FALSE.
uint8_t is_sequence_executing[TOTAL_DIGITAL_OUTPUTS]; // Flags identifying what ouptuts are already executing sequence 
uint64_t sequence_time_u64 =0;         // Placed holder for current systick time
uint64_t seq_change_time_u64[TOTAL_DIGITAL_OUTPUTS];  // Time when output state is changed
uint8_t output_seq_count_u8[TOTAL_DIGITAL_OUTPUTS];   // Placed holder for current sequence step

uint64_t module_digital_outputs_poll_time_u64 =0;     // Poll time for the module
uint8_t pwm_out_init_error_state = 0;  // Error state that identifies if wrong pin it used for PWM out. Only Dout1 can be used for PWM out

float pwm_freq_output_slope_f = 0;     // Slope used for PWM frequency output
float pwm_freq_output_intercept_f = 0; // Intercept used for PWM frequency output
float pwm_duty_output_slope_f = 0;     // Slope used for PWM duty cycle output
float pwm_duty_output_intercept_f = 0; // Intercept used for PWM duty cycle output

uint32_t faultStatus_u32 = 0;   // fault status flags
uint16_t measuredSpeed_u16 = 0; // Speed in RPM
uint16_t demand_percent_u16 =0; // demand percent
uint16_t Realy_On_Check_Counter=0,Realy_Off_Check_Counter=0;
// -- Define Pointers that will be used as References to other Modules, where applicable
//MotorDemandMux_Control* digitalOutputs_MotorDemandMux_Control;
Motor_Com_Control* digitalOutputs_MotorCom_Control_ptr;

/**
********************************************************************************
* @brief   State machine for digital outputs module
* @details
* @param   drv_identifier_u8, previous_state_u8, next_stat_u8, irq_identfier_u8
* @retval  return_state_u8
********************************************************************************
*/
uint8_t module_Digital_Outputs_u32(uint8_t drv_identifier_u8, uint8_t previous_state_u8, uint8_t next_state_u8,
                                   uint8_t irq_identifier_u8)
{
  uint8_t return_state_u8 = MEMORY_INIT_MODULE;
  switch (next_state_u8)
  {
  case MEMORY_INIT_MODULE:
    {
      AssignModuleMemDigitalOutputs(); // Assign structured memory
      return_state_u8 = INIT_MODULE;
      break;
    }
  case INIT_MODULE:
    {     
      AssignModuleMemDigitalOutputs(); // Assign structured memory for module   
      InitDigitalOutputsSetting();     // Init settings
      InitDigitalOutputsLocalVariables(); // Init local variables
      module_digital_outputs_poll_time_u64 = getSysCount() + DIGITAL_OUTPUTS_POLL_TIME; // Store time tick value 
      return_state_u8 = RUN_MODULE;
      break;
      
    }   
  case RUN_MODULE:                                                             
    {
      // Process digital outputs every "DIGITAL_OUTPUTS_POLL_TIME" mSec
      if (getSysCount() >= module_digital_outputs_poll_time_u64) // Time above poll time
      {
        module_digital_outputs_poll_time_u64 = getSysCount() + DIGITAL_OUTPUTS_POLL_TIME; // Next poll time
        
        uint8_t output_function_u8;
        
        for(uint8_t output_num_u8=0; output_num_u8 < TOTAL_DIGITAL_OUTPUTS; output_num_u8++){
          output_function_u8 = digital_Outputs_Control.digital_outputs_Setting.outputFunction[output_num_u8];
          
          switch(output_function_u8){
            
          case DISABLE: //0
            break;
          case FAULT_OUTPUT:  //1
            {
              if ( ((digital_Outputs_Control.digital_outputs_Setting.outputEnable_u16 & (1 << output_num_u8)) >> output_num_u8) && BIT_HI)
              { // Output Enabled
                ProcessFaultLedOut(output_num_u8);
              }
              break;
            }
          case LED_OUTPUT:    // 2
          case ON_BOARD_LED_OUTPUT: // 3
            {
              if ( ((digital_Outputs_Control.digital_outputs_Setting.outputEnable_u16 & (1 << output_num_u8)) >> output_num_u8) && BIT_HI)
              { // Output Enabled
                ProcessFaultLedOut(output_num_u8);
              }
              break;
            }  
          case RELAY_OUTPUT:   // 5
            {
              if ( ((digital_Outputs_Control.digital_outputs_Setting.outputEnable_u16 & (1 << output_num_u8)) >> output_num_u8) && BIT_HI)
              { // Output Enabled
                ProcessRelayOut(output_num_u8);
              }
              break;
            }
          case PWM_DUTY_CYCLE_OUTPUT:     // 5
            {
              if ( ((digital_Outputs_Control.digital_outputs_Setting.outputEnable_u16 & (1 << output_num_u8)) >> output_num_u8) && BIT_HI)
              { // Output Enabled
                // No error in PWM out init
                ProcessPwmDutyCycleOut(output_num_u8);
              } else {
                Module_Gpio_StopPwmOut(output_num_u8);
              }
              break;
            }
          case PWM_FREQUENCY_OUTPUT:   // 6
            {
              if ( ((digital_Outputs_Control.digital_outputs_Setting.outputEnable_u16 & (1 << output_num_u8)) >> output_num_u8) && BIT_HI)
              { // Output Enabled
                // No error in PWM out init
                ProcessPwmFrequencyOutput(output_num_u8);
              } else {
                Module_Gpio_StopPwmOut(output_num_u8);
              }
              break;
            }
          default:
            break;            
          }          
        }
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
********************************************************************************
* @brief   Assign structured memory
* @details Assign structured memory for digital outputs module
* @param   None 
* @return  None
********************************************************************************
*/
void AssignModuleMemDigitalOutputs(void){   
  digital_Outputs_StructMem_u32 =  StructMem_CreateInstance(MODULE_DIGITAL_OUTPUTS, sizeof(Digital_Outputs_Control), ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);
  (*digital_Outputs_StructMem_u32).p_ramBuf_u8 = (uint8_t *)&digital_Outputs_Control ;    // Map the ADC1 memory into the structured memory
  uint8_t module_digital_outputs_index_u8 = getProcessInfoIndex(MODULE_DIGITAL_OUTPUTS);
  processInfoTable[module_digital_outputs_index_u8].Sched_ModuleData.p_masterSharedMem_u32 = (Ram_Buf_Handle)digital_Outputs_StructMem_u32;
}

/**
********************************************************************************
* @brief   Init digital outputs settings
* @details Init global variables from flash
* @param   None 
* @return  None
********************************************************************************
*/
void InitDigitalOutputsSetting(void){ 
  digital_Outputs_Control.digital_outputs_Setting.maxPwmOutFreq_u16 = 10000;
  digital_Outputs_Control.digital_outputs_Setting.minPwmOutFreq_u16 = 50;
  digital_Outputs_Control.digital_outputs_Setting.minPwmOutDuty_u16 = 100; // 1.00% // Maximum PWM output duty cycle corresponding to min demand (0.01%)
  digital_Outputs_Control.digital_outputs_Setting.maxPwmOutDuty_u16 = 9500; // 95.00% // Minimum PWM output duty cycle corresponding to max demand (100%)
  digital_Outputs_Control.digital_outputs_Setting.pwmOutFreq_u16 = 100; // Output frequency for duty cycle mode
  digital_Outputs_Control.digital_outputs_Setting.outputEnable_u16 = 0xffff;
  digital_Outputs_Control.digital_outputs_Setting.outputFunction[0] = PWM_DUTY_CYCLE_OUTPUT;//PWM_FREQUENCY_OUTPUT;//PWM_DUTY_CYCLE_OUTPUT; //PWM_DUTY_CYCLE_OUTPUT or FAULT_OUTPUT;
  digital_Outputs_Control.digital_outputs_Setting.outputFunction[1] = LED_OUTPUT; //(uint8_t)LED_OUTPUT;
  digital_Outputs_Control.digital_outputs_Setting.outputFunction[2] = ON_BOARD_LED_OUTPUT; //(uint8_t)LED_OUTPUT;
  digital_Outputs_Control.digital_outputs_Setting.outputFunction[3] = RELAY_OUTPUT; //(uint8_t)RELAY_OUTPUT;
  digital_Outputs_Control.digital_outputs_Setting.outputFunctionMode[0] = MODE_1; // 
  digital_Outputs_Control.digital_outputs_Setting.outputFunctionMode[1] = MODE_1; // High during normal operation and blink when fault
  digital_Outputs_Control.digital_outputs_Setting.outputFunctionMode[2] = MODE_3; // High during normal operation and blink seq when faulted
  digital_Outputs_Control.digital_outputs_Setting.outputFunctionMode[3] = MODE_3; // NO when (low) during normal operation. NC (high) when faulted
  digital_Outputs_Control.digital_outputs_Setting.outputHighTime_u16 = 200; // On/High Time in mSec
  digital_Outputs_Control.digital_outputs_Setting.outputLowTime_u16 = 500; // Off/low time in mSec
  digital_Outputs_Control.digital_outputs_Setting.outputSequenceDelay_u16 = 2000; // delay between sequence in mSec
  digital_Outputs_Control.digital_outputs_Setting.outputPolarity_u16 = 0x0005; // Active Low  
  digital_Outputs_Control.digital_outputs_Setting.pwmFreqOutputScaleFactor_u16 = 1; // value * speed = Output frequency
  digital_Outputs_Control.digital_outputs_Setting.relayEnableDemandThreshold_u16 = 5000; //50%
  digital_Outputs_Control.digital_outputs_Setting.ledExternalSequenceHighTime_u16 = 100; // High time for sequences initiated by external modules
  digital_Outputs_Control.digital_outputs_Setting.ledExternalSequenceLowTime_u16 = 200;
  
}  

/**
********************************************************************************
* @brief   Init module variables
* @details Init local variables
* @param   None 
* @return  None
********************************************************************************
*/
void InitDigitalOutputsLocalVariables(void)
{
  // Local variable init
  for(uint8_t output_num_u8= 0; output_num_u8 < TOTAL_DIGITAL_OUTPUTS; output_num_u8++){
    output_seq_count_u8[output_num_u8] = 0;
    is_sequence_executing[output_num_u8] = FALSE;
    seq_change_time_u64[output_num_u8] = 0;
    is_processed_led_fault_outputs[output_num_u8] = FALSE;
    if( (digital_Outputs_Control.digital_outputs_Setting.outputFunction[output_num_u8] == FAULT_OUTPUT) || (digital_Outputs_Control.digital_outputs_Setting.outputFunction[output_num_u8] == LED_OUTPUT) || (digital_Outputs_Control.digital_outputs_Setting.outputFunction[output_num_u8] == ON_BOARD_LED_OUTPUT))
    {
      {  
        if(digital_Outputs_Control.digital_outputs_Setting.outputFunctionMode[output_num_u8] == MODE_3)
        {
          // Only count ouputs that need to execute blink sequence
          fault_led_outputs_count_u8++; 
        }
      }      
      // Init output state
      if(((digital_Outputs_Control.digital_outputs_Setting.outputPolarity_u16 & (1 << output_num_u8)) >> output_num_u8) && BIT_HI) // Invert Polarity
      {  // Inverted output
        Module_Gpio_WriteGpioState(output_num_u8, ACTIVE_HI);
      } else{
        Module_Gpio_WriteGpioState(output_num_u8, ACTIVE_LOW);
      }
    } 
    if( (digital_Outputs_Control.digital_outputs_Setting.outputFunction[output_num_u8] == PWM_DUTY_CYCLE_OUTPUT) || (digital_Outputs_Control.digital_outputs_Setting.outputFunction[output_num_u8] == PWM_FREQUENCY_OUTPUT) )
    {
      
      Module_Gpio_StartPwmOut(output_num_u8);
      
      if(pwm_out_init_error_state == 0) 
      { // Dout1 pin is used for PWM out 
        if( digital_Outputs_Control.digital_outputs_Setting.outputFunction[output_num_u8] == PWM_FREQUENCY_OUTPUT)
        {
          Module_Gpio_SetPwmOutPeriod(output_num_u8, digital_Outputs_Control.digital_outputs_Setting.minPwmOutFreq_u16 ); 
          Module_Gpio_SetPwmOutDutyCycle(output_num_u8, DEFAULT_PERCENT_DUTY_CYCLE);
          
          pwm_freq_output_slope_f = (digital_Outputs_Control.digital_outputs_Setting.maxPwmOutFreq_u16  - digital_Outputs_Control.digital_outputs_Setting.minPwmOutFreq_u16 );
          pwm_freq_output_slope_f = pwm_freq_output_slope_f/ ((float)(MAX_PERCENT_DEMAND - MIN_PERCENT_DEMAND));       
          pwm_freq_output_intercept_f = digital_Outputs_Control.digital_outputs_Setting.minPwmOutFreq_u16 - pwm_freq_output_slope_f * 1; // Min freq at 0.01%
          
        }
        else if (digital_Outputs_Control.digital_outputs_Setting.outputFunction[output_num_u8] == PWM_DUTY_CYCLE_OUTPUT)
        {
          Module_Gpio_SetPwmOutPeriod(output_num_u8, digital_Outputs_Control.digital_outputs_Setting.pwmOutFreq_u16 ); 
          
          pwm_duty_output_slope_f = digital_Outputs_Control.digital_outputs_Setting.maxPwmOutDuty_u16 - digital_Outputs_Control.digital_outputs_Setting.minPwmOutDuty_u16;
          pwm_duty_output_slope_f = pwm_duty_output_slope_f / ((float)(MAX_PERCENT_DEMAND - MIN_PERCENT_DEMAND));
          pwm_duty_output_intercept_f = digital_Outputs_Control.digital_outputs_Setting.minPwmOutDuty_u16 - pwm_duty_output_slope_f * 1; // @ 0.01% demand = minPwmOutDuty_u16
        }
        
        uint8_t polarity_value = ((digital_Outputs_Control.digital_outputs_Setting.outputPolarity_u16 >> output_num_u8) & (1u));
        
        Module_Gpio_SetPwmOutPolarity(output_num_u8, polarity_value); // Module_Gpio_StartPwm() resets polarity. So, set polarity here. 
      }
    } else //if(digital_Outputs_Control.digital_outputs_Setting.outputFunction[output_num_u8] == FAULT_OUTPUT)
    {
      Module_Gpio_Dout1_Init(output_num_u8); // Initilize Dout1 as a digital output
    }
  }  
  
  /*Attach motor demand mux module structured memory into this App*/
  //uint8_t module_motor_demand_mux_index  = getProcessInfoIndex(MODULE_MOTOR_DEMAND_MUX);   //return Process index from processInfo array
  //digitalOutputs_MotorDemandMux_Control = (MotorDemandMux_Control*)((*(processInfoTable[module_motor_demand_mux_index].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8);    //Get structured memory for Analog 0-10V input
  uint8_t module_motor_com_index  = getProcessInfoIndex(MODULE_MOTOR_COM);   //return Process index from processInfo array
  digitalOutputs_MotorCom_Control_ptr = (Motor_Com_Control*)((*(processInfoTable[module_motor_com_index].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8);    //Get structured memory 
}

/**
********************************************************************************
* @brief   Add data to LED output sequence
* @details Sets the on board/debug LED output to a sequence
* @param   output_num_u8 
* @return  None
********************************************************************************
*/
void DigitalOutputs_AddLedSequence(uint8_t sequence_value_u8){
  onBoardLedExternalSequences_u8 = sequence_value_u8;
}

/**
********************************************************************************
* @brief   Set the LED/Fault output sequence
* @details Sets the digital output to a sequence. Every state transition from 
*          SEQ_HI to SEQ_LOW state is counted as a sequence count. 
* @param   output_num_u8 (digital output number)
* @return  None
********************************************************************************
*/
void ExecuteFaultLedOutBlinking(uint8_t output_num_u8){
  uint8_t sequence_u8 =0;
  
  if( ((is_external_led_sequence_executing == TRUE) && (current_external_sequence_u8 != 0)) && (digital_Outputs_Control.digital_outputs_Setting.outputFunction[output_num_u8] == ON_BOARD_LED_OUTPUT))
  { // User external sequence if its present
    sequence_u8 = current_external_sequence_u8;
  } else{
    // Fault sequence
    sequence_u8 = current_sequence_u8;
  }
  
  // Check if output sequence is complete
  // First start a sequnce when output_seq_count_u8 = 0. Once a sequnce is started finish it before moving to next one. 
  // A sequence can be in one of the following states
  // SEQ_LOW: Output is low
  // SEQ_HI: Ouptut is high
  // SEQ_DELAY: Delay state. Delay between two sequences
  // SEQ_NO_CHANGE: Delay between low.hi of a current sequence
  if(output_seq_count_u8[output_num_u8] <= sequence_u8)
  {
    uint8_t state_u8=0;
    sequence_time_u64 = getSysCount(); // Store time tick value    
    
    // Check if new sequence has to start
    if(output_seq_count_u8[output_num_u8] == 0 ) 
    { 
      // Each sequence start after outputSequenceDelay_u16 dealy is satisfied
      if((sequence_time_u64 - seq_change_time_u64[output_num_u8]) > digital_Outputs_Control.digital_outputs_Setting.outputSequenceDelay_u16) // Check the delay once previous sequence ended
      { // Start new sequence
        output_seq_count_u8[output_num_u8]++;
        state_u8 = SEQ_HI; // Next state        
        seq_change_time_u64[output_num_u8] = sequence_time_u64; // Time when output is changed
      }
      else{ // Wait until outputSequenceDelay_u16 delay 
        state_u8 = SEQ_DELAY;
      }
    } else{ // Finish current sequence
      // Check if output is hi or low
      // Then check for the correspodning delay
      // If delay is satisfied, change state and increment sequence count (output_seq_count_u8) if the state changes from HI to LOW
      
      if(((((digital_Outputs_Control.digital_outputs_Data.digitalOutputsStatus_u16)& (uint16_t)((1 << output_num_u8))) >> output_num_u8) && BIT_HI)) // Check if current output is hi
      { 
        // Check if on board LED and if an external sequence is executing
        if( (is_external_led_sequence_executing == TRUE)&& (digital_Outputs_Control.digital_outputs_Setting.outputFunction[output_num_u8] == ON_BOARD_LED_OUTPUT) )
        {
          if( (sequence_time_u64 - seq_change_time_u64[output_num_u8]) > digital_Outputs_Control.digital_outputs_Setting.ledExternalSequenceHighTime_u16)
          {            
            state_u8 = SEQ_LOW; // Next state
            seq_change_time_u64[output_num_u8] = sequence_time_u64; // Time when output is changed
            output_seq_count_u8[output_num_u8]++;
          } else{ // Wait until outputHighTime_u16 delay 
            state_u8 = SEQ_NO_CHANGE;
          }          
        } else if( (is_sequence_executing[output_num_u8] == TRUE) && ((sequence_time_u64 - seq_change_time_u64[output_num_u8]) > digital_Outputs_Control.digital_outputs_Setting.outputHighTime_u16) )
        { // Check if the output high delay time is above outputHighTime_u16
          output_seq_count_u8[output_num_u8]++;
          state_u8 = SEQ_LOW; // Next state
          seq_change_time_u64[output_num_u8] = sequence_time_u64; // Time when output is changed
        } else{ // Wait until outputHighTime_u16 delay 
          state_u8 = SEQ_NO_CHANGE;
        }
      } else{ // Current output is low
        
        if( (is_external_led_sequence_executing == TRUE)&& (digital_Outputs_Control.digital_outputs_Setting.outputFunction[output_num_u8] == ON_BOARD_LED_OUTPUT) ){
          // Check if on board LED and if an external sequence is executing
          if( (sequence_time_u64 - seq_change_time_u64[output_num_u8]) > digital_Outputs_Control.digital_outputs_Setting.ledExternalSequenceLowTime_u16)
          {
            state_u8 = SEQ_HI; // Next state
            seq_change_time_u64[output_num_u8] = sequence_time_u64;
            
          } else{ // Wait until outputLowTime_u16 delay
            state_u8 = SEQ_NO_CHANGE;
          }
        } else if( (is_sequence_executing[output_num_u8] == TRUE) && ((sequence_time_u64 - seq_change_time_u64[output_num_u8]) > digital_Outputs_Control.digital_outputs_Setting.outputLowTime_u16) ) 
        { // Check if the output low delay time is above outputLowTime_u16
          state_u8 = SEQ_HI; // Next state
          seq_change_time_u64[output_num_u8] = sequence_time_u64;
          
        } else{ // Wait until outputLowTime_u16 delay
          state_u8 = SEQ_NO_CHANGE;
        }
      }
      
      // Output sequence is complete when state_u8 is "SEQ_LOW" and output is "low"
      if((state_u8 == SEQ_LOW) && ((((digital_Outputs_Control.digital_outputs_Data.digitalOutputsStatus_u16) & ((uint16_t)(~(1 << output_num_u8)))) >> output_num_u8) && BIT_LOW) ) // Sequence dealy
      {   
        state_u8 = SEQ_DELAY;
      }      
    }
    
    // State machine for the output state
    // Apply output polarity as necessary
    switch(state_u8)
    {
    case SEQ_LOW: // Change output to low
      {
        if(((digital_Outputs_Control.digital_outputs_Setting.outputPolarity_u16 & (1 << output_num_u8)) >> output_num_u8) && BIT_HI) // Invert Polarity
        {  // Inverted output
          Module_Gpio_WriteGpioState(output_num_u8, ACTIVE_HI);
        } else{
          Module_Gpio_WriteGpioState(output_num_u8, ACTIVE_LOW);
        }
        digital_Outputs_Control.digital_outputs_Data.digitalOutputsStatus_u16 &= (uint16_t)(~(1 << output_num_u8));
        break;
      }
    case SEQ_HI: // Change output to high
      {
        if(((digital_Outputs_Control.digital_outputs_Setting.outputPolarity_u16 & (1 << output_num_u8))  >> output_num_u8) && BIT_HI) // Invert Polarity
        { // Inverted output
          Module_Gpio_WriteGpioState(output_num_u8, ACTIVE_LOW);
        } else{
          Module_Gpio_WriteGpioState(output_num_u8, ACTIVE_HI);
        }
        digital_Outputs_Control.digital_outputs_Data.digitalOutputsStatus_u16 |= (uint16_t)(1 << output_num_u8);
        break;
      }
    case SEQ_DELAY: // Wait for sequence delay
      break;
      
    case SEQ_NO_CHANGE: // Delay between ON/OFF for current sequence
      break;
      
    default:
      break;
    }
    // End of if(output_seq_count_u8[output_num_u8] <= sequence_u8)
  } else{ 
    // A sequence is complete
    // processed_fault_led_outputs_count_u8 indicates if all outputs have completed executing sequence
    // Each output sequence wont be synced once an external sequence is recieved due to the time stored in seq_change_time_u64.
    
    sequence_u8 = 0;
    
    if (is_processed_led_fault_outputs[output_num_u8] == FALSE) // Current output sequence complete.
    { // This assures that same output is not checked twice
      processed_fault_led_outputs_count_u8++; // Increment the ouputs that finished sequence
      is_processed_led_fault_outputs[output_num_u8] = TRUE;
    }    
    
    // Clear all local variables/flags once all outputs are processed.
    // Need to wait for all ouputs to complte since the "current_sequence_u8" is used by all outputs
    if (processed_fault_led_outputs_count_u8 >= fault_led_outputs_count_u8) // All Fault and LED outputs processed
    {
      for (uint8_t output_number_u8 = 0; output_number_u8 < TOTAL_DIGITAL_OUTPUTS; output_number_u8++)
      {
        if ((digital_Outputs_Control.digital_outputs_Setting.outputFunction[output_number_u8] == ON_BOARD_LED_OUTPUT))
        {
          if((is_external_led_sequence_executing == TRUE))
          {
            // Clear flags and variables
            is_external_led_sequence_executing = FALSE;
            current_external_sequence_u8 = 0;
          }
          else{
            // Clear flags and variables
            current_sequence_u8 = 0;
          }
        }
        is_sequence_executing[output_number_u8] = FALSE;
        output_seq_count_u8[output_number_u8] = 0;
        is_processed_led_fault_outputs[output_number_u8] = FALSE;
      }
      processed_fault_led_outputs_count_u8 = 0;
      current_sequence_u8 = 0;
    } else {
      // No fault codes and external led seq is complete. fault_led_outputs_count_u8 wont be satisifed when external seq only.
      if(current_sequence_u8 == 0)
      {
        is_external_led_sequence_executing = FALSE;
        current_external_sequence_u8 = 0;
        is_sequence_executing[output_num_u8] = FALSE;
        output_seq_count_u8[output_num_u8] = 0;
        is_processed_led_fault_outputs[output_num_u8] = FALSE;
        processed_fault_led_outputs_count_u8 = 0;
        current_sequence_u8 = 0;
      }
    }    
  }
}

/**
********************************************************************************
* @brief   Check for faults flags and process settings
* @details Process ouputs based on function mode
* @param   output_num_u8 
* @return  None
********************************************************************************
*/
void ProcessFaultLedOut(uint8_t output_num_u8){
  uint8_t mode_u8;
  mode_u8 = digital_Outputs_Control.digital_outputs_Setting.outputFunctionMode[output_num_u8];
  faultStatus_u32 = (*digitalOutputs_MotorCom_Control_ptr).motor_Metering_Data.motor_alarms_u32;
  switch(mode_u8){
    
  case MODE_0:  // Disabled
    // Do nothing
    break;
  case MODE_1: // Low when no fault, High when faulted
    {
      if(faultStatus_u32 != 0)
      {
        if(((digital_Outputs_Control.digital_outputs_Setting.outputPolarity_u16 & (1 << output_num_u8))  >> output_num_u8) && BIT_HI) // Invert Polarity
        {
          // Inverted output
          Module_Gpio_WriteGpioState(output_num_u8, ACTIVE_LOW);
          digital_Outputs_Control.digital_outputs_Data.digitalOutputsStatus_u16 &= (uint16_t)(~(1 << output_num_u8));
        } else {
          Module_Gpio_WriteGpioState(output_num_u8, ACTIVE_HI);
          digital_Outputs_Control.digital_outputs_Data.digitalOutputsStatus_u16 |= (uint16_t)(1 << output_num_u8);
        }
      } else {
        if(((digital_Outputs_Control.digital_outputs_Setting.outputPolarity_u16 & (1 << output_num_u8))  >> output_num_u8) && BIT_HI) // Invert Polarity
        {
          // Inverted output
          Module_Gpio_WriteGpioState(output_num_u8, ACTIVE_HI);
          digital_Outputs_Control.digital_outputs_Data.digitalOutputsStatus_u16 |= (uint16_t)(1 << output_num_u8);          
        } else {
          Module_Gpio_WriteGpioState(output_num_u8, ACTIVE_LOW);
          digital_Outputs_Control.digital_outputs_Data.digitalOutputsStatus_u16 &= (uint16_t)(~(1 << output_num_u8));
        }
      }
      break;
    }
  case MODE_2: // High when no fault, blinks when faulted
    {
      if(faultStatus_u32 != 0) // Faults are present
      {
        uint8_t state_u8;
        sequence_time_u64 = getSysCount(); // Store time tick value  
        state_u8 = Module_Gpio_ReadGpioOutputState(output_num_u8);
        if((state_u8 == ACTIVE_LOW))
        {
          if(((sequence_time_u64 - seq_change_time_u64[output_num_u8]) > digital_Outputs_Control.digital_outputs_Setting.outputLowTime_u16)){
            seq_change_time_u64[output_num_u8] = sequence_time_u64;
            Module_Gpio_TogglePin(output_num_u8);
          }
        } else{
          if((sequence_time_u64 - seq_change_time_u64[output_num_u8]) > digital_Outputs_Control.digital_outputs_Setting.outputHighTime_u16)
          {
            seq_change_time_u64[output_num_u8] = sequence_time_u64;
            Module_Gpio_TogglePin(output_num_u8);
          }
        }
      } else{ // No faults
        if(((digital_Outputs_Control.digital_outputs_Setting.outputPolarity_u16 & (1 << output_num_u8))  >> output_num_u8) && BIT_HI) // Invert Polarity
        {
          Module_Gpio_WriteGpioState(output_num_u8, ACTIVE_HI);
          digital_Outputs_Control.digital_outputs_Data.digitalOutputsStatus_u16 |= (uint16_t)(1 << output_num_u8); 
        }
        else{
          Module_Gpio_WriteGpioState(output_num_u8, ACTIVE_LOW);
          digital_Outputs_Control.digital_outputs_Data.digitalOutputsStatus_u16 &= (uint16_t)(~(1 << output_num_u8));
        }
      }
    }
    break;
    
  case MODE_3: // High when no fault, fault code blink sequenc when faulted   
    {
      if((is_sequence_executing[output_num_u8] == FALSE)) // Check if no sequence is in process. 
      { // No sequence is in process
        
        // Check if external LED sequence is present and if the ouptut is on board/debug LED.
        if( (digital_Outputs_Control.digital_outputs_Setting.outputFunction[output_num_u8] == ON_BOARD_LED_OUTPUT) && (onBoardLedExternalSequences_u8 != 0) ) 
        {
          current_external_sequence_u8 = onBoardLedExternalSequences_u8; // Execute the sequence
          onBoardLedExternalSequences_u8 = 0; // Ready to take net sequence
          is_external_led_sequence_executing = TRUE; // Indicate that external sequnce is running
          
          is_sequence_executing[output_num_u8] = TRUE;
          output_seq_count_u8[output_num_u8] = 0;
          if( (current_sequence_u8 != 0) || (current_external_sequence_u8 != 0) )
          {
            ExecuteFaultLedOutBlinking(output_num_u8); // process new_sequence_u8
          }
        } else {
          
          // Get new sequence
          //faultStatus_u32 = (*digitalOutputs_MotorCom_Control_ptr).motor_Metering_Data.motor_alarms_u32;
          if(faultStatus_u32 != 0) //FAULTS_NONE)
          {
            uint8_t bit_no_u8=0;
            while(bit_no_u8 < 32){
              if((faultStatus_u32 >> bit_no_u8) & BIT_HI){
                new_sequence_u8 =(uint8_t) ((bit_no_u8 + 1)); //*2); // Count for falling and rising edge 
                break; // lowest no fault gets the first priority.
              } else{
                new_sequence_u8 = 0;
                bit_no_u8++;
              }          
            }  
          }
          if(new_sequence_u8 != 0)
          {
            current_sequence_u8 = new_sequence_u8;
            new_sequence_u8 = 0;  
            is_sequence_executing[output_num_u8] = TRUE;
            output_seq_count_u8[output_num_u8] = 0;
            ExecuteFaultLedOutBlinking(output_num_u8); // process new_sequence_u8
          }
        }      
      } else { // current_sequence_u8 is under process
        // Even if fault is cleared, the current sequence has to finish before going to default state (reset)
        ExecuteFaultLedOutBlinking(output_num_u8);
      }    
      break;
    }
  default:    
    {
      current_sequence_u8 = 0;
      is_sequence_executing[output_num_u8] = FALSE;
      break;
    }
  }
}

/**
********************************************************************************
* @brief   Check for satus and set outputs
* @details Check fault falgs/user enable/speed threshold and set relay accordingly
* @param   output_num_u8 
* @return  None
********************************************************************************
*/
void ProcessRelayOut(uint8_t output_num_u8){
  static uint8_t mode_u8, state_u8;
  mode_u8 = digital_Outputs_Control.digital_outputs_Setting.outputFunctionMode[output_num_u8];
  
  
  switch(mode_u8){
  case MODE_1: // Fault Mode
    {
      if(faultStatus_u32 != 0)
      {
        state_u8 = ENABLE;
      } else{
        state_u8 = DISABLE;
      }
      break;
    }
  case MODE_2: // User Enable Mode
    {
      if(digital_Outputs_Control.digital_outputs_Data.discretes_u16.is_relayOutputEnable)
      {
        state_u8 = ENABLE;
      } else{
        state_u8 = DISABLE;
      }
      break;
    }
  case MODE_3: // Speed Threshold Mode
    {
      //demand_percent_u16 =Actual_Measured_Speed;// (*digitalOutputs_MotorCom_Control_ptr).motor_Metering_Data.demand_Reference_Percent_u16;
      
        if(Actual_Measured_Speed==0){
          state_u8 = DISABLE;
        }else if( Actual_Measured_Speed > RELAY_ON_TH_SPEED-5)//digital_Outputs_Control.digital_outputs_Setting.relayEnableDemandThreshold_u16)
        {
          Realy_Off_Check_Counter=0;
          if(Realy_On_Check_Counter>=100){
            //Realy_On_Check_Counter=0;
            state_u8 = ENABLE;
          }else{
            Realy_On_Check_Counter++;
          }
        } else if( Actual_Measured_Speed < RELAY_ON_TH_SPEED-30)//digital_Outputs_Control.digital_outputs_Setting.relayEnableDemandThreshold_u16)
        {
          Realy_On_Check_Counter=0;
          if(Realy_Off_Check_Counter>=100){
            state_u8 = DISABLE;
          }else{
            Realy_Off_Check_Counter++;
          }
        }
      
     // Actual_Measured_Speed=0;
      break;
    }
  default:
    break;
  }
  
  // Set output state
  if(state_u8 == ENABLE)
  {
    if(((digital_Outputs_Control.digital_outputs_Setting.outputPolarity_u16 & (1 << output_num_u8))  >> output_num_u8) && BIT_HI) // Invert Polarity
    { 
      Module_Gpio_WriteGpioState(output_num_u8, NORMAL_OPEN);
    } else{
      Module_Gpio_WriteGpioState(output_num_u8, NORMAL_CLOSED);
    }
  } else {
    if(((digital_Outputs_Control.digital_outputs_Setting.outputPolarity_u16 & (1 << output_num_u8))  >> output_num_u8) && BIT_HI) // Invert Polarity
    { 
      Module_Gpio_WriteGpioState(output_num_u8, NORMAL_CLOSED);
    } else{
      Module_Gpio_WriteGpioState(output_num_u8, NORMAL_OPEN);
    }
  }
  
}

/**
********************************************************************************
* @brief   Set PWM output duty cycle in PWM_DUTY_CYCLE_OUTPUT mode
* @details Used %demand (xxxyy = xxx.yy% format) to set duty cycle
* @param   output_num_u8 
* @return  None
********************************************************************************
*/
void ProcessPwmDutyCycleOut(uint8_t output_num_u8){   
  uint16_t pwm_duty_u16 =0;
  demand_percent_u16 = (*digitalOutputs_MotorCom_Control_ptr).motor_Metering_Data.demand_Reference_Percent_u16;
  if(demand_percent_u16 == 0)
  {
    pwm_duty_u16 = 0;
  } else {
    pwm_duty_u16 = (uint16_t)(pwm_duty_output_slope_f * demand_percent_u16 + pwm_duty_output_intercept_f);
  }
  digital_Outputs_Control.digital_outputs_Data.pwmOutputDutyCycle_u16 = pwm_duty_u16;  
   Module_Gpio_SetPwmOutDutyCycle(output_num_u8, pwm_duty_u16); 
}

/**
********************************************************************************************************************************
* @brief   Set PWM output duty cycle in PWM_FREQUENCY_OUTPUT mode
* @details Uses %demand (xxxyy = xxx.yy% format) or speed in RPM to set output frequency
* @param   output_num_u8 
* @return  None
********************************************************************************************************************************
*/
void ProcessPwmFrequencyOutput(uint8_t output_num_u8){  
  uint8_t mode_u8;
  mode_u8 = digital_Outputs_Control.digital_outputs_Setting.outputFunctionMode[output_num_u8];
  uint16_t pwm_freq_u16 = 0;
  switch(mode_u8){
  case MODE_1: // Measured speed output
    {
      measuredSpeed_u16 = (*digitalOutputs_MotorCom_Control_ptr).motor_Metering_Data.demand_Reference_Speed_u16;
      if(measuredSpeed_u16 == 0)
      {
        pwm_freq_u16 = 0;
      } else {
        pwm_freq_u16 = (uint16_t)(measuredSpeed_u16 * digital_Outputs_Control.digital_outputs_Setting.pwmFreqOutputScaleFactor_u16);
      }
      break;
    }
  case MODE_2: // Commanded demand output
    {
      demand_percent_u16 =(*digitalOutputs_MotorCom_Control_ptr).motor_Metering_Data.demand_Reference_Percent_u16;
      if(demand_percent_u16 == 0)
      {
        pwm_freq_u16 = 0;
      } else {         
        pwm_freq_u16 = (uint16_t)(pwm_freq_output_slope_f * demand_percent_u16 + pwm_freq_output_intercept_f); // No need to left shift
      }
      break;
    }
  default:
    {
      pwm_freq_u16 = 0;
      break;
    }
  }
  if(pwm_freq_u16 != 0){ // Set to "0" if Speed or demand are zeros
    if(pwm_freq_u16 > digital_Outputs_Control.digital_outputs_Setting.maxPwmOutFreq_u16)
    {
      pwm_freq_u16 = digital_Outputs_Control.digital_outputs_Setting.maxPwmOutFreq_u16;
    } 
  }
  digital_Outputs_Control.digital_outputs_Data.pwmOuputFrequency_u16 = pwm_freq_u16;
  Module_Gpio_SetPwmOutPeriod(output_num_u8, pwm_freq_u16); // Set frequency
  Module_Gpio_SetPwmOutDutyCycle(output_num_u8, DEFAULT_PERCENT_DUTY_CYCLE); // Set duty cycle. Need this since duty cycle calculation repends on PWM period value
  
}