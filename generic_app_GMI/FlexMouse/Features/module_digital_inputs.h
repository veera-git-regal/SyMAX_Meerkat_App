/**
  *****************************************************************************
  * @file    module_digital_inputs.h 
  * @author  Regal, Satya Akkina
  * @version V1.0
  * @date    10-Oct-2020
  * @brief   Header file for module_digital_inputs module
  * @note    
  *****************************************************************************
  */

// Define to prevent recursive inclusion 
#ifndef _MODULE_DIGITAL_INPUTS_H_
#define _MODULE_DIGITAL_INPUTS_H_

#ifdef __cplusplus
extern "C" {
#endif
  
// Includes 
#include "module_gpio.h"
  
#include "structured_memory.h"
#include "scheduler.h"
  
extern Ram_Buf sharedMemArray[TOTAL_NUM_OF_STRUCT_MEM_INSTANCES];
extern ProcessInfo processInfoTable[];

#define DIGITAL_INPUT_POLL_TIME    (50) // Digital inputs poll time in mSec
#define MAX_DISCRETE_DEMAND_INPUTS (3)   // Maximum number of allowed discrete demand digital inputs
#define MAX_DISCRETE_DEMANDS (8) // REVIEW: 2^MAX_DISCRETE_DEMAND_INPUTS led to an array size of 1

//******************* Digital Inputs Control (inside shared memory) ************  
// Digital Inputs settings
struct DigitalInputs_Settings
{
  uint16_t demandPercent[MAX_DISCRETE_DEMANDS]; // Demand for each possible input combination
  uint16_t inputPolarity_u16;        // Each bit indicates the polarity of the input. 0= Normally Open, 1 = Normally closed. Bit 0 = Digital input 0 polarity.
  uint16_t inputEnable_u16;          // Each bit enables an individual digital input. Bit 0 = Digital input 0 enable.  
  uint16_t inputFunction[MAX_DIGITAL_INPUTS]; // Function of each digital input
  uint16_t debounceCountLimit_u16; // # of counts digital input should be active high/low to consider it as active high/low
};

typedef struct
{  
  uint16_t  is_invertDirection:1;       // Set to "1" if loss of analog is detected
  uint16_t  is_motorEnabled:1;       	// Set to "1" if analog voltage is decreasing
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
} DigitalInputs_Discretes;

// Live Digital Inputs Data
struct DigitalInputs_Data
{ 
  uint32_t digitalInputsValue[MAX_DIGITAL_INPUTS]; // Value based on input functionality of digital input
  uint16_t digitalInputStatus_u16;    // Each bit indicates status of digital inputs. Bit 0 = Digital input 0 status.
  uint16_t digitalInputStatusFiltered_u16; // Each bit indicates filtered status of digital inputs. Bit 0 = Digital input 0 status.
  uint16_t discreteDemandPercent; // Demand when input mode is sepected as discrete demand input
  DigitalInputs_Discretes discretes_u16;	  // Discrete indicator bits  
};

typedef struct{
 struct DigitalInputs_Settings digitalInputs_Setting;
 struct DigitalInputs_Data digitalInputs_Data;
}DigitalInputs_Control;
//******************* End of Digital Inputs Control (inside shared memory) ************  

// Modes for digital inputs
enum{
  INPUT_DISABLED = 0,
  MOTOR_ENABLE,       // 1
  MOTOR_START,        // 2
  SET_DIRECTION,      // 3
  FIREMODE_ENABLE,    // 4
  ALARM_RESET,        // 5
  SET_DEMAND_INPUT_0, // 6
  SET_DEMAND_INPUT_1, // 7
  SET_DEMAND_INPUT_2, // 8
  SET_DEMAND_INPUT_3, // 9
};

// Decimal value used to compare digital inputs status for discrete demand mode.
enum{
  DEMAND0 = 0,  // demandPercent0. This can be set to "0" to disable demand. 
  DEMAND1, // 1 // demandPercent1
  DEMAND2, // 2 // demandPercent2
  DEMAND3, // 3 // demandPercent3
  DEMAND4, // 4 // demandPercent4
  DEMAND5, // 5 // demandPercent5
  DEMAND6, // 6 // demandPercent6
  DEMAND7, // 7 // demandPercent7
};

#ifdef __cplusplus
}
#endif

#endif /* _MODULE_DIGTIAL_INPUTS_H_ */
