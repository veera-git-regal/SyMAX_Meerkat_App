/**
  *****************************************************************************
  * @file    module_pwm.h 
  * @author  Regal, Logan Schaufler
  * @version V1.0
  * @date    23-Nov-2020
  * @brief   Header for Digital PWM input
  * @note    
  *****************************************************************************
  */

// Define to prevent recursive inclusion 
#ifndef _MODULE_PWM_INPUT_H_
#define _MODULE_PWM_INPUT_H_

#ifdef __cplusplus
extern "C" {
#endif
  
// Includes 
#include "driver_gpio.h"  // REVIEW: is this needed now that PWM in has been moved to it's own driver?
#include "driver_pwm_input.h"
    
#include "structured_memory.h"
#include "scheduler.h"
  
extern Ram_Buf sharedMemArray[STRUCT_MEM_ARRAY_SIZE];
extern ProcessInfo processInfoTable[];
extern Ram_Buf *pwm_input_Settings_StructMem_u32;
extern Ram_Buf *pwm_input_Data_StructMem_u32;

#define DigitalPeriod 50 //PWM digital Update time mSec

//******************* Digital PWM Control (inside shared memory) ************  
// Digital PWM settings
typedef struct
{  
  //uint16_t	bits_convert_u16;			 // Union structure of all 16 bits in the field				
  uint16_t  is_pwmFailSafeEnable:1;       		// If "1" enable analog input for demand
  uint16_t  is_invertDigitalPWM:1; 				// If "1", 0V = Max demand 10V= 0 Demand
  uint16_t  is_pwmLowEnable:1;  			// Enable loss of analog input  
  uint16_t  is_pwmMinDemandEnable:1;		// If "1" Switch to fail safe demand when loss of anlaog is detected
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
  uint16_t      empty12:1;
} PwmInput_Flags;

struct PWMInput_Settings
{  
  float          dutyCyleCalibratonFactor_f;// Calibration factor to correct duty cycle error
  uint16_t       maxPWMFrequency_u16;       // Max readable PWM frequency, ignores duty cycle for anything past this
  uint16_t       minPWMFrequency_u16;       // Min readable PWM frequency, ignores duty cycle for anything before this 
  uint16_t       maxDutyCycle_u16;          // Maximum acceptable duty cycle         
  uint16_t       minDutyCycle_u16;          // Minimum acceptable duty cycle    
  uint16_t       minDemand_u16;             // Min allowed demand             
  uint16_t       maxDemand_u16;             // Maximum allowed demand   
  uint16_t       calibratonFactor_u16;      // Add/Delete this form measured for calibration  
  uint16_t       pwmPeriod_u16;                 // Time until calculated demand is transmitted to the main app (in ms) 
  uint16_t       pwmLossFrequency_u16;      // Frequency below which the bPWM_Loss_Enable is set     
  uint16_t       failSafeDemand_u16;        // Fail safe demand when loss of pwm input is detected  
  uint16_t       minDemandHysteresis_u16;   // Lower end hysteresis if Min_Demand > Off_Volts demand
  uint16_t       minTurnOnDutyCycle_u16;        // Minimum duty cycle module takes to turn on and begin operation  
  uint16_t       pwmLowDutyCycle_u16;
  uint16_t       inputMode_u16;              // Enable alternate function of Digital Input
  uint16_t       dutyCycleHysteresis_u16;
  uint16_t       offDutyCycle_u16;
  uint16_t       onDutyCycle_u16;  
  uint16_t       pwmLowAlarmEnableCount_u16; // pwmPeriod*count delay before loss of analog low volts alarm is detected in mSec. 
  uint16_t       pwmInputDebounceCount_u16;  // Minimum counts required to verified valied duty cycle
  PwmInput_Flags flags_u16;
};

// Live Digital Frequency Data

typedef struct
{  
  uint16_t       is_decreasingPWM:1;             // Set to "1" if analog voltage is decreasing
  uint16_t       is_pwmDutyCycleLow:1;           // Set to "1" if loss of analog is detected
  uint16_t       is_pwmFrequencyLow:1;           // Set to "1" if loss of pwm input frequency is low
  uint16_t       is_demandOn:1;                  // Set to "1" if demand goes from 0 to above min demand.
  uint16_t       is_lowerHysteresisEnable:1;     // Set to "1" if lower end hysteresis need to be enabled
  uint16_t       is_upperHysteresisEnable:1;     // Set to "1" if upper end hysteresis need to be enabled  
  uint16_t       is_digitalInputON:1;            // Enable digital input
  uint16_t       is_TurnOnDutyCycleOccured:1;    // Determines when Turn On Duty Cycle has been achieved
  uint16_t       is_LossDutyCycleBeenExceeded:1; //Set to '1' if loss duty cycle has been exceeded and fail safe mode can now engage
  uint16_t	 empty01:1;   
  uint16_t	 empty02:1; 
  uint16_t	 empty03:1; 
  uint16_t	 empty04:1; 
  uint16_t	 empty05:1; 
  uint16_t	 empty06:1;
  uint16_t	 empty07:1;    
} PwmInput_Discretes;

struct PWMInput_Data
{  
  float          inheritedPwmInputDutyCycle_f;   // Calculated duty cycle demand (calculation done in driver) averaged over 4 values
  float          increasingIntercept_f;          // Intercept used to calculate demand when analog is increasing
  float          decreasingIntercept_f;          // Intercept used to calcualte demand when analog is decreasing
  float          decreasingSlope_f;              // Slope used to calculate demand when analog is decreasing //SPA REVIEW
  float          increasingSlope_f;              // Slope used to calculate demand when analog is increasing //SPA REVIEW
  uint16_t       pwmInputDigitalInputState_u16;  // '1' = high state;'2' = low state
  uint16_t       pwmInputDemand_u16;             // Calculated output demand from duty cycle
  uint16_t       inheritedPWMInputFrequency_u16; // Measured PWM input frequency
  uint16_t       pwmInputDemandPercentage_u16;   // Output demand percentage given by module
  uint16_t       pwmStableDutyCycle_u16;         // Previous duty cycle readings  
  uint16_t       pwmErrorCode_u16;               // Error code    
  PwmInput_Discretes discretes_u16;              // discrete bit indicators
  
//  uint8_t        is_decreasingPWM;               // Set to "1" if analog voltage is decreasing
//  uint8_t        is_pwmDutyCycleLow;             // Set to "1" if loss of analog is detected
//  uint8_t        is_pwmFrequencyLow;             // Set to "1" if loss of pwm input frequency is low
//  uint8_t        is_demandOn;                    // Set to "1" if demand goes from 0 to above min demand.
//  uint8_t        is_lowerHysteresisEnable;       // Set to "1" if lower end hysteresis need to be enabled
//  uint8_t        is_upperHysteresisEnable;       // Set to "1" if upper end hysteresis need to be enabled  
//  uint8_t        is_digitalInputON;              // Enable digital input
//  uint8_t        is_TurnOnDutyCycleOccured;      // Determines when Turn On Duty Cycle has been achieved
//  uint8_t        is_LossDutyCycleBeenExceeded;   //Set to '1' if loss duty cycle has been exceeded and fail safe mode can now engage
};

typedef struct{
 struct PWMInput_Settings pwmInput_Settings;
 struct PWMInput_Data pwmInput_Data;  
}PwmInput_Control;

enum { // TODO: This enum had conflicts with 0-10V module, why can it see that module?
  PWM_IN_DISABLE_INPUT = 0,
  PWM_IN_PWM_MODE = 1,
  PWM_IN_DIGITAL_MODE = 2
};

//******* end of Digital PWM Control (inside shared memory) ****************

#ifdef __cplusplus
}
#endif

#endif /* _MODULE_PWM_INPUT_H_ */

