/**
  *****************************************************************************
  * @file    module_digital_outputs.h 
  * @author  Satya Akkina
  * @brief   Header for module_digital_output.c
  * @details 
  *****************************************************************************
  */

// Define to prevent recursive inclusion --------------------------------------
#ifndef _MODULE_DIGTIAL_OUTPUTS_H_
#define _MODULE_DIGTIAL_OUTPUTS_H_

// Includes -------------------------------------------------------------------
#include "module_gpio.h"

#include "main.h"
#include "typedef.h"

#include "scheduler.h"
#include "sequential_memory.h"
#include "structured_memory.h"

// Content --------------------------------------------------------------------
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


#define DIGITAL_OUTPUTS_POLL_TIME (10) // Poll time for the module
#define MAX_ALLOWED_OUTPUT_FREQ (6400)  // in hertz
#define DEFAULT_PERCENT_DUTY_CYCLE (5000) //50.00%
  
#define MAX_PERCENT_DEMAND (10000) // 100.00%
#define MIN_PERCENT_DEMAND (1)  // 0.01%
  
#define MAX_EXTERNAL_STORED_LED_SEQUENCES (8)

//----------------- Digital Outputs Control (inside shared memory) ------------
// Digital Outputs settings
struct Digital_Outputs_Settings
{ 
  uint16_t outputPolarity_u16;    // Each bit indicates the polarity of the output. 0= Normally Open, 1 = Normally closed. Bit 0 = Digital output 0 polarity.
  uint16_t outputEnable_u16;      // Each bit enables an individual digital output. Bit 0 = Digital output 0 enable. // Used by customer
  uint16_t minPwmOutFreq_u16;     // Maximum PWM output frequency corresponding to min demand
  uint16_t maxPwmOutFreq_u16;     // Minimum PWM output frequency corresponding to max demand
  uint16_t minPwmOutDuty_u16;     // Maximum PWM output duty cycle corresponding to min demand (0.01%)
  uint16_t maxPwmOutDuty_u16;     // Minimum PWM output duty cycle corresponding to max demand (100%)
  uint16_t pwmOutFreq_u16;        // PWM output frequency
  uint16_t outputHighTime_u16;    // Time between high to low (ON time) for fault sequence
  uint16_t outputLowTime_u16;     // Delay between output toggle in mSec(from ON to OFF)
  uint16_t outputSequenceDelay_u16;    // Delay between two consicutive sequences in mSec(used for fault sequences)
  uint16_t ledExternalSequenceHighTime_u16; // High time for sequences initiated by external modules
  uint16_t ledExternalSequenceLowTime_u16;  // Low time for sequences initiated by external modules
  uint16_t relayEnableDemandThreshold_u16;  // Demand above this will change the state of the relay  
  uint16_t pwmFreqOutputScaleFactor_u16;      // Scale factor for tachOuputFrequency = speed * tachScaleFactor
  uint16_t outputFunction[TOTAL_DIGITAL_OUTPUTS];     // Function of each digital output
  uint16_t outputFunctionMode[TOTAL_DIGITAL_OUTPUTS]; // Function mode of each digital output. Fault output can be set in 3 different modes
};

typedef struct
{  
  uint16_t  is_relayOutputEnable:1;              // Set to "1" if loss of analog is detected
  uint16_t	empty01:1;   
  uint16_t	empty02:1; 
  uint16_t	empty03:1; 
  uint16_t	empty04:1; 
  uint16_t	empty05:1; 
  uint16_t	empty06:1;
  uint16_t	empty07:1; 
  uint16_t	empty08:1; 
  uint16_t	empty09:1;
  uint16_t	empty10:1;   
  uint16_t	empty11:1;   
  uint16_t	empty12:1;   
  uint16_t	empty13:1;   
  uint16_t	empty14:1;   
  uint16_t	empty15:1;     
} DigitalOutputs_Discretes;

// Live Digital Outputs Data
struct Digital_Outputs_Data
{ 
  uint16_t digitalOutputsStatus_u16;  // Each bit indicates status of digital output. Bit 0 = Digital output 0 status.
  uint16_t pwmOutputDutyCycle_u16;    // PWM output duty cycle
  uint16_t pwmDutyCycleOuputFrequency_u16;// PWM output when in PWM duty cycle mode frequency
  uint16_t pwmOuputFrequency_u16;     // PWM frequency output in pwm frequency output mode
  DigitalOutputs_Discretes discretes_u16;
};

typedef struct{
 struct Digital_Outputs_Settings digital_outputs_Setting;
 struct Digital_Outputs_Data digital_outputs_Data;
}Digital_Outputs_Control;
//-------------- End of Digital Outputs Control (inside shared memory) --------

// Gloabal functions for digital outputs  
/**
*******************************************************************************
* @brief   Add data to LED output sequence
* @details Sets the on board/debug LED output to a sequence
* @param   output_num_u8 
* @return  None
*******************************************************************************
*/
void DigitalOutputs_AddLedSequence(uint8_t sequence_value_u8);

// Function for digital outputs
enum{
  DIGITAL_OUTPUT_DISABLED = 0,  //0
  FAULT_OUTPUT,          // 1
  LED_OUTPUT,            // 2
  ON_BOARD_LED_OUTPUT,   // 3
  RELAY_OUTPUT,          // 4
  PWM_DUTY_CYCLE_OUTPUT, // 5
  PWM_FREQUENCY_OUTPUT,  // 6
};

// Function Modes for digital outputs
enum{
  MODE_0 = 0, // Disable
  MODE_1,     // 1
  MODE_2,     // 2 
  MODE_3,     // 3
  MODE_4,     // 4
};

// LED/ FAULT sequences // Not used in code
enum{
  SEQ_NONE = 0,
  SEQ_1, // 1
  SEQ_2, // 2
  SEQ_3, // 3
  SEQ_4, // 4
  SEQ_5, // 5
  SEQ_6, // 6
  SEQ_7, // 7
  SEQ_8, // 8
  SEQ_9, // 9
  SEQ_10, // 10
  SEQ_11, // 11
  SEQ_12, // 12
  SEQ_13, // 13
  SEQ_14, // 14
  SEQ_15, // 15
}; 

// LED sequences for on board/debug LED
enum{
  LED_STATE_0 = 0, // No sequence
  LED_MOTOR_START_STATE, // 1
  LED_UNIV_COM_STATE,    // 2
  LED_MODBUS_COM_STATE,  // 3  
}; 


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _MODULE_DIGTIAL_OUTPUTS_H_ */