/**
*************************************************************************************
* @file    module_pwm_input.c 
* @author  Regal, Logan Schaufler
* @version V1.0
* @date    23-Nov-2020
* @brief   module for PWM input
* @note    Provide structures to access PWM setting and duty cycle, demand
*************************************************************************************
*/

// Includes -------------------------------------------------------------------
#include "module_pwm_input.h"
#include "driver_pwm_input.h" // PWM Input
#include "module_gpio.h"
#include "macros.h"

// Content --------------------------------------------------------------------
// Function Prototypes
void AssignPWMInputModuleMem(void);
void Init_PWM_Input_Setting(void);
void Init_PWM_Input_Data(void);
void CheckDutyCycle(void);
uint16_t GetPWMDemand(void);
void SetDemandToZero(void);
void UpdateModuleRuntimeData(void);
void CheckDigitalInputDebounce(void);

// Constants
#define PWM_IN_NUM 3 // pwm input number defined under gpioInputPortsPins in drivers_gpio.c
#define MAX_HARDWARE_PWM_IN_DUTY_CYCLE 10000  // 100.00%

// Module States
enum {
  MEMORY_INIT_MODULE,
  INIT_MODULE,
  RUN_MODULE,
  // Ddditional states to be added here as necessary.
  IRQ_MODULE = DEFAULT_IRQ_STATE,
  KILL_MODULE = KILL_APP
};

// External Variables
extern Ram_Buf sharedMemArray[TOTAL_NUM_OF_STRUCT_MEM_INSTANCES];
extern ProcessInfo processInfoTable[];

// Global variables specific to this module
static  Ram_Buf_Handle pwm_input_StructMem_u32;
uint64_t pwm_input_time_u64; // Delay time between init and run module
uint16_t pwm_input_duty_cycle_percentage_tmp_u16 = 0;   // Temporary storage for duty cycle conversion value
uint16_t pwm_input_frequency_tmp_u16 = 0;     // Temporary storage for pwm frequency
uint16_t pwm_input_low_alarm_enable_count_u16 = 0;  // Time when analog module init is complete
uint16_t is_pwm_input_above_low_threshold = FALSE; // Time when analog module init is complete
//uint8_t digital_input_debounce_count_u8 = 4; // Digitial input stage change should be at least this count before updating the current state
uint64_t tt_PWMInputDelayTime; // Delay time between init and run module
uint16_t digital_input_current_bit_low_debounce_count_u16 = 0; // Current debounce count when input is low
uint16_t digital_input_current_bit_hi_debounce_count_u16 = 0;  // Current debounce count when input is hi
uint16_t pwm_input_duty_cycle_decreasing_debounce_count_u16 = 0;
uint16_t pwm_input_duty_cycle_increasing_debounce_count_u16 = 0;

PwmInput_Control pwmInput_Control;

// Define Pointers that will be used as References to other Modules, where applicable
Gpio_Control* gpio_PWMInputLocalControl;
TIM1_Control*  tim1_LocalControl;

/**
********************************************************************************************************************************
* @brief   State machine for Digital PWM Module
* @details
* @retval  return_state_u8
********************************************************************************************************************************
*/
uint8_t modulePWM_Input_u32(uint8_t drv_identifier_u8, uint8_t previous_state_u8, uint8_t next_state_u8,
                            uint8_t irq_identifier_u8)
{
  //local variables  
  uint8_t return_state_u8 = MEMORY_INIT_MODULE;
  
  switch (next_state_u8)
  {
  case MEMORY_INIT_MODULE:
    {
      AssignPWMInputModuleMem(); // Assign structured memory 
      return_state_u8 = INIT_MODULE;
      break;
    }
  case INIT_MODULE:
    {     
      //AssignPWMInputModuleMem(); // Assign structured memory to PWM setting and data      
      Init_PWM_Input_Setting();  // Initilize PWM Input settings        
      Init_PWM_Input_Data();     // Init PWM input live data
      
      // Get structured memory for TIM1 data
      uint8_t module_TIM1_Index = getProcessInfoIndex(MODULE_TIM1);
      tim1_LocalControl = (TIM1_Control*)((*(processInfoTable[module_TIM1_Index].Sched_DrvData.p_masterSharedMem_u32)).p_ramBuf_u8);
      
      // Get structured memory for GPIO module data
      uint8_t module_gpio_index_u8 = getProcessInfoIndex(MODULE_GPIO);
      gpio_PWMInputLocalControl = (Gpio_Control*)((*(processInfoTable[module_gpio_index_u8].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8);
      
      pwm_input_low_alarm_enable_count_u16 = pwmInput_Control.pwmInput_Settings.pwmLowAlarmEnableCount_u16; // delay before low duty cycle alarm can be triggered
      
      pwm_input_time_u64 = getSysCount() + pwmInput_Control.pwmInput_Settings.pwmPeriod_u16;   // Store next poll time value for the module
      return_state_u8 = RUN_MODULE;
      break;
    }   
  case RUN_MODULE:                                                             
    {
      // Process analog input every "AnalogPeriod" mSec
      if (getSysCount() >= pwm_input_time_u64) 
      { 
        if(pwmInput_Control.pwmInput_Settings.inputMode_u16 == PWM_IN_DIGITAL_MODE)
        {
          // Process PWM input as Digital input
          CheckDigitalInputDebounce(); // Check for debounce and update is_digitalInputON state
          
          // Set PWM related parameters to defaults since the input is digital input
          SetDemandToZero();
        }else 
        {
          UpdateModuleRuntimeData(); // Update tick time data, module memory inherited from driver memory  
          CheckDutyCycle(); // Process the pwm duty cycle and frequency
          // Check if Loss of pwm. If enabled and loss of analog, demand is "0"
		  //if ((pwmInput_Control.pwmInput_Data.is_pwmDutyCycleLow == TRUE) && (pwmInput_Control.pwmInput_Data.is_pwmFrequencyLow == TRUE) && (pwmInput_Control.pwmInput_Settings.is_pwmFailSafeEnable == FALSE) && (pwmInput_Control.pwmInput_Settings.is_pwmLowEnable == TRUE))
          if (((pwmInput_Control.pwmInput_Data.discretes_u16.is_pwmDutyCycleLow == TRUE) || (pwmInput_Control.pwmInput_Data.discretes_u16.is_pwmFrequencyLow == TRUE)) && (pwmInput_Control.pwmInput_Settings.flags_u16.is_pwmFailSafeEnable == FALSE) && (pwmInput_Control.pwmInput_Settings.flags_u16.is_pwmLowEnable == TRUE))
          {
            SetDemandToZero();
          } else
          { // No loss of pwm input is detected or loss of pwm input is not enabled; normal operation
            pwmInput_Control.pwmInput_Data.pwmInputDemand_u16 = GetPWMDemand();       // Convert duty cycle into demand
            
            //float temporary_pwm_input_value = 0;
            //temporary_pwm_input_value = pwmInput_Control.pwmInput_Data.pwmInputDemand_u16;
            
            // Module Return Values when a demand exists
            pwmInput_Control.pwmInput_Data.pwmInputDemandPercentage_u16 = DECIMAL_RIGHT_SHIFT_U16(CALCULATE_PERCENT(pwmInput_Control.pwmInput_Data.pwmInputDemand_u16, 65536),2);
            //pwmInput_Control.pwmInput_Data.pwmInputDemandPercentage_u16 = pwmInput_Control.pwmInput_Data.pwmInputDemand_u16; //(uint16_t)(temporary_pwm_input_value * 100);  // Module's converted TIM1 duty cycle to PWM Demand, scheduler pings this; scaled 0 to 10000
          }
        }
      }     
      return_state_u8 = RUN_MODULE;
      break;
    }
  case IRQ_MODULE: 
    {
      // If there are more than one interrupts, from different drivers, you can identify each individually by:
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
* @details Assign structured memory for Digital PWM control
* @retval  None
********************************************************************************************************************************
*/
//
void AssignPWMInputModuleMem(void){   
  pwm_input_StructMem_u32 =  StructMem_CreateInstance(MODULE_PWM_INPUT, sizeof(PwmInput_Control), ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);
  (*pwm_input_StructMem_u32).p_ramBuf_u8 = (uint8_t *)&pwmInput_Control ;    // Map the TIM1 memory into the structured memory
  uint8_t Module_Digital_Pwm_Index = getProcessInfoIndex(MODULE_PWM_INPUT);
  processInfoTable[Module_Digital_Pwm_Index].Sched_ModuleData.p_masterSharedMem_u32 = (Ram_Buf_Handle)pwm_input_StructMem_u32;
}

/**
********************************************************************************************************************************
* @brief   Initilize all pwm settings and live data
* @details Read settings from the RAM and initilize the settings
* @retval  None
********************************************************************************************************************************
*/
void Init_PWM_Input_Setting(void){  
  
  // Initialize Module Settings  
  pwmInput_Control.pwmInput_Settings.onDutyCycle_u16 = 800;//3;      // Demand duty cycle to turn on and begin operation
  //pwmInput_Control.pwmInput_Settings.lowEndOffDutyCycle_u16 = 5;   // Demand duty cycle to turn off and stop operation; acts as Hysteresis
  pwmInput_Control.pwmInput_Settings.maxDutyCycle_u16 = 9500;        // Maximum acceptable duty cycle (in %) corresponding to max demand
  pwmInput_Control.pwmInput_Settings.minDutyCycle_u16 = 1000;         // Minimum acceptable duty cycle (in %)            spec 5
  pwmInput_Control.pwmInput_Settings.dutyCycleHysteresis_u16 = 50;//100;  // Hys duty cycle at the top end of demand
  pwmInput_Control.pwmInput_Settings.minDemand_u16 = 21845;//500;              // Min allowed demand                       spec 6
  pwmInput_Control.pwmInput_Settings.maxDemand_u16 = 0xFFFF;//10000;            // Maximum allowed demand                   spec 6
  //  pwmInput_Control.pwmInput_Settings.is_enableDigitalPWM = FALSE;  // If '1' enable pwm input for demand       spec 1
  pwmInput_Control.pwmInput_Settings.maxPWMFrequency_u16 = 1010;       // Max readable PWM frequency, ignores duty cycle for anything past this   spec 3
  pwmInput_Control.pwmInput_Settings.minPWMFrequency_u16 = 45;         // Max readable PWM frequency, ignores duty cycle for anything past this   spec 3
  pwmInput_Control.pwmInput_Settings.flags_u16.is_invertDigitalPWM =TRUE;//FALSE;      // If '1' and is_enableDigitalPWM is '1', 0% = Max demand and 100%= 0 Demand   spec 4
  pwmInput_Control.pwmInput_Settings.pwmPeriod_u16 = 50;  // Time until calculated demand is transmitted to the main app (in ms)   spec 9
  pwmInput_Control.pwmInput_Settings.flags_u16.is_pwmLowEnable = TRUE;           // Enable loss of digital input   spec 12
  pwmInput_Control.pwmInput_Settings.flags_u16.is_pwmFailSafeEnable = TRUE;//FALSE;     // If "1" Switch to fail safe demand when loss of pwm is detected  spec 12
  pwmInput_Control.pwmInput_Settings.flags_u16.is_pwmMinDemandEnable = FALSE;//TRUE;
  pwmInput_Control.pwmInput_Settings.pwmLowDutyCycle_u16 = 500;       // Frequency below which the bPWM_Loss_Enable is set     spec 13
  pwmInput_Control.pwmInput_Settings.failSafeDemand_u16 = 0x7FFF;// 50;          // Fail safe demand when loss of pwm input is detected (0x7FFF taken from 0-10V module) spec 14
  pwmInput_Control.pwmInput_Settings.pwmLowAlarmEnableCount_u16 = 100;  // pwmPeriod*count delay before loss of pwm input frequency alarm is detected in mSec.
  pwmInput_Control.pwmInput_Settings.inputMode_u16 = PWM_IN_PWM_MODE;   //PWM_IN_DIGITAL_MODE or PWM_IN_PWM_MODE
  pwmInput_Control.pwmInput_Settings.minDemandHysteresis_u16 = 10;
  pwmInput_Control.pwmInput_Settings.offDutyCycle_u16 = 600;
  pwmInput_Control.pwmInput_Settings.minTurnOnDutyCycle_u16 = 10;//800;         // Minimum duty cycle module takes to turn on and begin operation
  
  pwmInput_Control.pwmInput_Settings.dutyCyleCalibratonFactor_f = 0; //0.0055; // Calibration factor to correct duty cycle error
  pwmInput_Control.pwmInput_Settings.pwmInputDebounceCount_u16 = 4;     // Minimum counts required to verified valied duty cycle
}

void Init_PWM_Input_Data(void)
{
  // Initialize Live Module Data
  pwmInput_Control.pwmInput_Data.inheritedPwmInputDutyCycle_f = 0;    // Measured duty cycle(from driver) averaged over 4 values
  // pwmInput_Control.pwmInput_Data.dutyCycleScaled_f = 0;        // PWM scalled 0 to 100%
  pwmInput_Control.pwmInput_Data.inheritedPWMInputFrequency_u16 = 0; // PWM Demand scalled 0 to 100%
  pwmInput_Control.pwmInput_Data.increasingIntercept_f = 0;    // Intercept used to calculate demand when analog is increasing
  pwmInput_Control.pwmInput_Data.decreasingIntercept_f = 0;    // Intercept used to calcualte demand when analog is decreasing
  pwmInput_Control.pwmInput_Data.pwmStableDutyCycle_u16 = 0;   // Previous duty cycle sample
  pwmInput_Control.pwmInput_Data.discretes_u16.is_pwmDutyCycleLow = FALSE;         // Set to "1" if loss of analog is detected
  pwmInput_Control.pwmInput_Data.discretes_u16.is_pwmFrequencyLow = FALSE;         // Set to "1" if loss of pwm input frequency is low
  pwmInput_Control.pwmInput_Data.discretes_u16.is_demandOn = FALSE;                // Set to "1" if demand goes from 0 to above min demand.
  pwmInput_Control.pwmInput_Data.discretes_u16.is_lowerHysteresisEnable = FALSE;   // Set to "1" if lower end hysteresis need to be enabled
  pwmInput_Control.pwmInput_Data.discretes_u16.is_upperHysteresisEnable = FALSE;   // Set to "1" if upper end hysteresis need to be enabled
  pwmInput_Control.pwmInput_Data.discretes_u16.is_digitalInputON = FALSE;  
  pwmInput_Control.pwmInput_Data.discretes_u16.is_TurnOnDutyCycleOccured = FALSE;  // Detemines if Turn On Duty Cycle scenario has been satisfied
  // pwmInput_Control.pwmInput_Data.has_NormalPWMOperationBegan = FALSE;    // Determines when normal duty cycle has occurred; enables hysteresis
  pwmInput_Control.pwmInput_Data.discretes_u16.is_LossDutyCycleBeenExceeded = FALSE;
  pwmInput_Control.pwmInput_Data.pwmInputDemandPercentage_u16 = 0;
  
  pwm_input_low_alarm_enable_count_u16 = pwmInput_Control.pwmInput_Settings.pwmLowAlarmEnableCount_u16;
  
  if(pwmInput_Control.pwmInput_Settings.inputMode_u16 == PWM_IN_DIGITAL_MODE)
  {
    // Init input as a digital input
    PWMInput_GPIO_DigitalInput_Init();
  }else
  {
    // Init input as an PWM input
    PWMInput_GPIO_PwmInput_Init();
    PWMInputTimerInit();
  }
  
  // Local Variables for Slope and Intercept Calculations
  uint16_t demand_range = 0;
  uint16_t pwm_range = 0;
  
  // Decreasing Slope and Intercept Calculations
  demand_range = pwmInput_Control.pwmInput_Settings.maxDemand_u16 - pwmInput_Control.pwmInput_Settings.minDemand_u16;
  pwm_range = (pwmInput_Control.pwmInput_Settings.maxDutyCycle_u16 - pwmInput_Control.pwmInput_Settings.dutyCycleHysteresis_u16) - (pwmInput_Control.pwmInput_Settings.minDutyCycle_u16 - pwmInput_Control.pwmInput_Settings.dutyCycleHysteresis_u16);                                                                 
  pwmInput_Control.pwmInput_Data.decreasingSlope_f = (float)(demand_range / ((float)pwm_range));                                                                                                                                                                                                                           // Slope used to calculate demand when analog is decreasing
  pwmInput_Control.pwmInput_Data.decreasingIntercept_f = (float)(pwmInput_Control.pwmInput_Settings.minDemand_u16 - (float)((pwmInput_Control.pwmInput_Data.decreasingSlope_f) * (pwmInput_Control.pwmInput_Settings.minDutyCycle_u16 - pwmInput_Control.pwmInput_Settings.dutyCycleHysteresis_u16))); // Intercept used to calcualte demand when analog is decreasing
  
  // Increasing Slope and Intercept Calculations
  demand_range = pwmInput_Control.pwmInput_Settings.maxDemand_u16 - pwmInput_Control.pwmInput_Settings.minDemand_u16;
  pwm_range = pwmInput_Control.pwmInput_Settings.maxDutyCycle_u16 - pwmInput_Control.pwmInput_Settings.minDutyCycle_u16; 
  pwmInput_Control.pwmInput_Data.increasingSlope_f = (float)(demand_range / ((float)pwm_range));  // Slope used to calculate demand when analog is increasing
  pwmInput_Control.pwmInput_Data.increasingIntercept_f = (float)(pwmInput_Control.pwmInput_Settings.minDemand_u16 - (float)((pwmInput_Control.pwmInput_Data.increasingSlope_f) * (pwmInput_Control.pwmInput_Settings.minDutyCycle_u16))); // Interecept used to calculate demand when analog is increasing
}

/**
********************************************************************************************************************************
* @brief   Check if PWM is increasing or decresing
* @details Scenarios to update Duty Cycle. Check for Loss of Input.
* @retval  None
********************************************************************************************************************************
*/
/**
********************************************************************************************************************************
* @brief   Check if PWM is increasing or decresing
* @details Check for hysteresis, turn ON/OFF and calculate demand 
* @param   None 
* @return  None
********************************************************************************************************************************
*/
void CheckDutyCycle(void)
{
  //Check if Turn On Duty Cycle has been achieved, needs to occur for normal operation to begin; Only executed once
  if (pwmInput_Control.pwmInput_Data.discretes_u16.is_TurnOnDutyCycleOccured == FALSE)
  {
   /* if (pwmInput_Control.pwmInput_Settings.flags_u16.is_invertDigitalPWM == FALSE && pwm_input_duty_cycle_percentage_tmp_u16 >= pwmInput_Control.pwmInput_Settings.minTurnOnDutyCycle_u16)
    {
      pwmInput_Control.pwmInput_Data.discretes_u16.is_TurnOnDutyCycleOccured = TRUE;
    } else if(pwmInput_Control.pwmInput_Settings.flags_u16.is_invertDigitalPWM == TRUE)
    {
      pwmInput_Control.pwmInput_Data.discretes_u16.is_TurnOnDutyCycleOccured = TRUE;
    }*/
    pwmInput_Control.pwmInput_Data.discretes_u16.is_TurnOnDutyCycleOccured = TRUE;
  }
  
  //Check to see if loss duty cycle has been exceeded yet, needed to enable fail safe mode; Only executed once
  if (pwmInput_Control.pwmInput_Data.discretes_u16.is_LossDutyCycleBeenExceeded == FALSE)  
  {
    if (pwm_input_duty_cycle_percentage_tmp_u16 > pwmInput_Control.pwmInput_Settings.pwmLowDutyCycle_u16)
    {
      pwmInput_Control.pwmInput_Data.discretes_u16.is_LossDutyCycleBeenExceeded = TRUE;
    }
  }
  
  //Check if Duty cycle is decreasing/increasing
  //Duty cycle is decreasing
  if (pwmInput_Control.pwmInput_Data.pwmStableDutyCycle_u16 > pwm_input_duty_cycle_percentage_tmp_u16)
  { // Dutycycle is decreasing
    pwm_input_duty_cycle_decreasing_debounce_count_u16++;
    pwm_input_duty_cycle_increasing_debounce_count_u16 = 0;
  } else if(pwmInput_Control.pwmInput_Data.pwmStableDutyCycle_u16 <= pwm_input_duty_cycle_percentage_tmp_u16)
  { // Dutycycle is increasing
    pwm_input_duty_cycle_decreasing_debounce_count_u16 = 0;
    pwm_input_duty_cycle_increasing_debounce_count_u16++;    
  }
  
  if( (pwm_input_duty_cycle_decreasing_debounce_count_u16 > pwmInput_Control.pwmInput_Settings.pwmInputDebounceCount_u16) || (pwm_input_duty_cycle_increasing_debounce_count_u16 > pwmInput_Control.pwmInput_Settings.pwmInputDebounceCount_u16) )
  { // Debounce count satisfied
    pwmInput_Control.pwmInput_Data.pwmStableDutyCycle_u16 = pwm_input_duty_cycle_percentage_tmp_u16; //pwmInput_Control.pwmInput_Data.inheritedPwmInputDutyCycle_f; 
    //pwmInput_Control.pwmInput_Data.inheritedPwmInputDutyCycle_f = pwm_input_duty_cycle_percentage_tmp_u16; 
    if(pwm_input_duty_cycle_decreasing_debounce_count_u16 > pwmInput_Control.pwmInput_Settings.pwmInputDebounceCount_u16)
    {
      pwmInput_Control.pwmInput_Data.discretes_u16.is_decreasingPWM = TRUE;
    }else
    {
      pwmInput_Control.pwmInput_Data.discretes_u16.is_decreasingPWM = FALSE;
    }    
  }

  // Delay low pwm detection. Needed on power up 
  if (pwm_input_low_alarm_enable_count_u16 == 0)
  {
    // Check if loss of digital pwm
    //if (pwmInput_Control.pwmInput_Data.inheritedPwmInputDutyCycle_f < pwmInput_Control.pwmInput_Settings.pwmLowDutyCycle_u16)
    if (pwm_input_duty_cycle_percentage_tmp_u16 < pwmInput_Control.pwmInput_Settings.pwmLowDutyCycle_u16)  
    {
      pwmInput_Control.pwmInput_Data.discretes_u16.is_pwmDutyCycleLow = TRUE;
    } else 
    {
      is_pwm_input_above_low_threshold = TRUE;
      pwmInput_Control.pwmInput_Data.discretes_u16.is_pwmDutyCycleLow = FALSE;
    }
    
    //if (pwmInput_Control.pwmInput_Data.inheritedPWMInputFrequency_u16 < pwmInput_Control.pwmInput_Settings.minPWMFrequency_u16)
    if (pwm_input_frequency_tmp_u16 < pwmInput_Control.pwmInput_Settings.minPWMFrequency_u16)  
    {
      pwmInput_Control.pwmInput_Data.discretes_u16.is_pwmFrequencyLow = TRUE;
    } else
    {
      pwmInput_Control.pwmInput_Data.discretes_u16.is_pwmFrequencyLow = FALSE;
    }
  } else
  {
    is_pwm_input_above_low_threshold = TRUE;
    pwmInput_Control.pwmInput_Data.discretes_u16.is_pwmDutyCycleLow = FALSE;
  }
}

/**
********************************************************************************************************************************
* @brief   Convert PWM Duty Cycle to Demand ( min_Demand_u16 to max_Demand_u16)
* @details Check for hysteresis, turn ON/OFF and calculate demand 
* @param   None 
* @return  None
********************************************************************************************************************************
*/
uint16_t GetPWMDemand(void){
  //local variables for demand calculations
  uint32_t pwm_input_demand_u32 = 0;
  float slope_f;
  float intercept_u16;
  uint16_t pwm_input_duty_cycle_u16 = 0; 
  uint16_t pwm_input_frequency_stable_tmp_u16 = 0;
  
  pwm_input_duty_cycle_u16 = pwmInput_Control.pwmInput_Data.pwmStableDutyCycle_u16;
  pwm_input_frequency_stable_tmp_u16 = pwmInput_Control.pwmInput_Data.inheritedPWMInputFrequency_u16;
  
  //Mode 1 of 3: PWM Mode not selected
  if(pwmInput_Control.pwmInput_Settings.inputMode_u16 != PWM_IN_PWM_MODE)
  {
    pwm_input_demand_u32 = 0;
    pwmInput_Control.pwmInput_Data.discretes_u16.is_demandOn = FALSE;
  } else if ((pwmInput_Control.pwmInput_Settings.flags_u16.is_pwmLowEnable == TRUE) && (pwmInput_Control.pwmInput_Settings.flags_u16.is_pwmFailSafeEnable == TRUE ) && (pwmInput_Control.pwmInput_Data.discretes_u16.is_pwmDutyCycleLow == TRUE) && pwmInput_Control.pwmInput_Data.discretes_u16.is_LossDutyCycleBeenExceeded == TRUE)
  { //Mode 2 of 3: Fail Safe Mode Engaged
    // Check if Loss of PWM Input
    if((pwmInput_Control.pwmInput_Data.discretes_u16.is_demandOn == FALSE) && (is_pwm_input_above_low_threshold == FALSE)) //demand has never gone above min demand. 
    { // pwm never crossed turned ON duty cycle
      pwm_input_demand_u32 = 0;
    } else if ((is_pwm_input_above_low_threshold == TRUE) &&(pwmInput_Control.pwmInput_Data.discretes_u16.is_demandOn == TRUE))
    {
      // Use fail safe demand if loss of pwm detected
      pwm_input_demand_u32 = pwmInput_Control.pwmInput_Settings.failSafeDemand_u16; 
      pwmInput_Control.pwmInput_Data.discretes_u16.is_upperHysteresisEnable = FALSE;
    }else{
      pwm_input_demand_u32 = 0;
    }
  } else 
  { //Mode 3 of 3: Normal Operation Engaged
    // Check if Turn On Duty Cycle has occured
    if (pwmInput_Control.pwmInput_Data.discretes_u16.is_TurnOnDutyCycleOccured == TRUE)
    {
      // Check if inverted demand is enabled, adjusts calculations accordingly
     /* if(pwmInput_Control.pwmInput_Settings.flags_u16.is_invertDigitalPWM == TRUE) 
      {
        pwm_input_duty_cycle_u16 = MAX_HARDWARE_PWM_IN_DUTY_CYCLE - pwmInput_Control.pwmInput_Data.pwmStableDutyCycle_u16;
        if (pwm_input_duty_cycle_u16 < 0)
        {
          pwm_input_duty_cycle_u16 = 0;
        }
      }*/
      
      // Check if increasing or decreasing slope
      // Slope used when pwm duty cycle is increasing
      if(pwmInput_Control.pwmInput_Data.discretes_u16.is_decreasingPWM == FALSE)
      { 
        slope_f = pwmInput_Control.pwmInput_Data.increasingSlope_f;
        intercept_u16 = pwmInput_Control.pwmInput_Data.increasingIntercept_f;
      } else 
      {// Slope used when pwm duty cycle is decreasing
        slope_f = pwmInput_Control.pwmInput_Data.decreasingSlope_f;
        intercept_u16 = pwmInput_Control.pwmInput_Data.decreasingIntercept_f;
      }
      
      if( (pwm_input_frequency_stable_tmp_u16 >= pwmInput_Control.pwmInput_Settings.minPWMFrequency_u16) && (pwm_input_frequency_stable_tmp_u16 <= pwmInput_Control.pwmInput_Settings.maxPWMFrequency_u16) )
      {  
        //Duty Cycle is in normal operation zone
        pwmInput_Control.pwmInput_Data.discretes_u16.is_pwmFrequencyLow = FALSE;
        
        if ( (pwm_input_duty_cycle_u16 >= pwmInput_Control.pwmInput_Settings.offDutyCycle_u16) && (pwm_input_duty_cycle_u16 < (pwmInput_Control.pwmInput_Settings.maxDutyCycle_u16 - pwmInput_Control.pwmInput_Settings.dutyCycleHysteresis_u16)) )
        {         
          // Duty Cycle when above ON duty cycle for first time and meeting frequency criteria
          //pwmInput_Control.pwmInput_Data.discretes_u16.is_lowerHysteresisEnable = TRUE;
          pwmInput_Control.pwmInput_Data.discretes_u16.is_upperHysteresisEnable = FALSE;
          
          if (pwm_input_duty_cycle_u16 >= pwmInput_Control.pwmInput_Settings.onDutyCycle_u16)
          {
            if (pwm_input_duty_cycle_u16 >= pwmInput_Control.pwmInput_Settings.minDutyCycle_u16)
            {
              pwmInput_Control.pwmInput_Data.discretes_u16.is_lowerHysteresisEnable = TRUE;
              pwm_input_demand_u32 = (uint16_t)((float)(slope_f * pwm_input_duty_cycle_u16) + intercept_u16);
            } else if(pwm_input_duty_cycle_u16 < pwmInput_Control.pwmInput_Settings.minDutyCycle_u16)
            {
              // Lower end hysteresis handler with MinDemandEnable
              if ((pwmInput_Control.pwmInput_Settings.flags_u16.is_pwmMinDemandEnable == TRUE) )//{&& (pwmInput_Control.pwmInput_Data.discretes_u16.is_lowerHysteresisEnable == TRUE))
              {
                pwm_input_demand_u32 = pwmInput_Control.pwmInput_Settings.minDemand_u16;
              } else if (pwm_input_duty_cycle_u16 >= pwmInput_Control.pwmInput_Settings.minDutyCycle_u16 - pwmInput_Control.pwmInput_Settings.dutyCycleHysteresis_u16 && pwmInput_Control.pwmInput_Data.discretes_u16.is_lowerHysteresisEnable == TRUE)
              { // Lower end hysteresis handler without MinDemandEnable
                pwm_input_demand_u32 = (uint16_t)((float)(slope_f * pwm_input_duty_cycle_u16) + intercept_u16);
              } else
              {
                // Inverted Demand Handler without Hysteresis
                /*if ((pwmInput_Control.pwmInput_Settings.flags_u16.is_invertDigitalPWM == TRUE) && (pwmInput_Control.pwmInput_Settings.flags_u16.is_pwmMinDemandEnable == TRUE))
                {
                  pwm_input_demand_u32 = pwmInput_Control.pwmInput_Settings.maxDemand_u16;
                } else
                {*/
                  pwm_input_demand_u32 = 0;
                  pwmInput_Control.pwmInput_Data.discretes_u16.is_lowerHysteresisEnable = FALSE;
              //  }
                
              }
            }
          }else if (pwm_input_duty_cycle_u16 >= pwmInput_Control.pwmInput_Settings.offDutyCycle_u16) {
        
          if((pwmInput_Control.pwmInput_Settings.flags_u16.is_pwmMinDemandEnable) == TRUE && ( pwmInput_Control.pwmInput_Data.discretes_u16.is_lowerHysteresisEnable == TRUE)){
            
             pwm_input_demand_u32 = pwmInput_Control.pwmInput_Settings.minDemand_u16;
           
          }else{
            pwm_input_demand_u32 = 0;
            pwmInput_Control.pwmInput_Data.discretes_u16.is_demandOn = FALSE;
          }
        } else if (pwm_input_duty_cycle_u16 < pwmInput_Control.pwmInput_Settings.onDutyCycle_u16)
          { // Turn On Duty Cycle never satisfied
            pwm_input_demand_u32 = 0;
          } else
          {
            pwm_input_demand_u32 = 0;
          }
          //   pwmInput_Control.pwmInput_Data.has_NormalPWMOperationBegan = TRUE;
          //   pwmInput_Control.pwmInput_Data.discretes_u16.is_demandOn == TRUE;
          //   pwm_input_demand_u32 = (float)((float)(slope_f * pwm_input_duty_cycle_u16) + intercept_u16);
        } else if (pwm_input_duty_cycle_u16 >= (pwmInput_Control.pwmInput_Settings.maxDutyCycle_u16 - pwmInput_Control.pwmInput_Settings.dutyCycleHysteresis_u16)) 
        { // Upper End Hysteresis Operation
          if (pwmInput_Control.pwmInput_Data.discretes_u16.is_upperHysteresisEnable == TRUE)
          {
            // Default to Max Demand with Upper Hysteresis enabled
            if (pwmInput_Control.pwmInput_Data.discretes_u16.is_decreasingPWM == TRUE)
            {
              pwm_input_demand_u32 = pwmInput_Control.pwmInput_Settings.maxDemand_u16;
            } else
            { // Calculated demand with Upper Hysteresis enabled
              pwm_input_demand_u32 = (uint32_t)((float)(slope_f * pwm_input_duty_cycle_u16) + intercept_u16);
            }  
          } else if (pwm_input_duty_cycle_u16 <= pwmInput_Control.pwmInput_Settings.maxDutyCycle_u16)
          { // Demand is less than maxDutyCycle and is calculated
            pwmInput_Control.pwmInput_Data.discretes_u16.is_upperHysteresisEnable = TRUE;
            pwm_input_demand_u32 = (uint32_t)((float)(slope_f * pwm_input_duty_cycle_u16) + intercept_u16);
          } else if (pwm_input_duty_cycle_u16 > pwmInput_Control.pwmInput_Settings.maxDutyCycle_u16)
          { // Scale back demand to maxDutyCycle
            pwmInput_Control.pwmInput_Data.discretes_u16.is_upperHysteresisEnable = TRUE;
            pwm_input_demand_u32 = pwmInput_Control.pwmInput_Settings.maxDemand_u16;
          }
        }/*else if (pwm_input_demand_u32 < pwmInput_Control.pwmInput_Settings.minDemand_u16)
        { // Demand is less than minDemand
          if (pwm_input_demand_u32 < (pwmInput_Control.pwmInput_Settings.minDemand_u16 - pwmInput_Control.pwmInput_Settings.minDemandHysteresis_u16))
          {
            if (pwmInput_Control.pwmInput_Data.discretes_u16.is_decreasingPWM == TRUE && pwmInput_Control.pwmInput_Settings.flags_u16.is_invertDigitalPWM == FALSE)
            {
              // Scale up demand to minDemand with inverted demand
              if(pwmInput_Control.pwmInput_Settings.flags_u16.is_pwmMinDemandEnable == TRUE)
              {
                pwm_input_demand_u32 = pwmInput_Control.pwmInput_Settings.minDemand_u16;
              }
            } else if (pwmInput_Control.pwmInput_Data.discretes_u16.is_decreasingPWM == TRUE && pwmInput_Control.pwmInput_Settings.flags_u16.is_invertDigitalPWM == TRUE)
            {
              // Scale up demand to minDeman without inverted demand
              if(pwmInput_Control.pwmInput_Settings.flags_u16.is_pwmMinDemandEnable == TRUE)
              {
                pwm_input_demand_u32 = pwmInput_Control.pwmInput_Settings.minDemand_u16;
              } else
              {
                pwmInput_Control.pwmInput_Data.discretes_u16.is_lowerHysteresisEnable = FALSE;
                pwm_input_demand_u32 = 0;
              }
            } else if (pwmInput_Control.pwmInput_Data.discretes_u16.is_decreasingPWM == FALSE && pwmInput_Control.pwmInput_Settings.flags_u16.is_invertDigitalPWM == TRUE)
            {
              // Scale demand with inverted demand to minDemand
              if(pwmInput_Control.pwmInput_Settings.flags_u16.is_pwmMinDemandEnable == TRUE)
              {
                pwm_input_demand_u32 = pwmInput_Control.pwmInput_Settings.minDemand_u16;
              } else
              {
                pwmInput_Control.pwmInput_Data.discretes_u16.is_lowerHysteresisEnable = FALSE;
                pwm_input_demand_u32 = 0;
              }
            } else
            {
              pwm_input_demand_u32 = 0;
            }          
          }
        } */else
        {
          pwm_input_demand_u32 = 0;
          pwmInput_Control.pwmInput_Data.discretes_u16.is_demandOn = FALSE;
          pwmInput_Control.pwmInput_Data.discretes_u16.is_lowerHysteresisEnable = TRUE;
        }
        // Scale back demand to max demand; used in case of a bug in code occuring
        if (pwm_input_demand_u32 > pwmInput_Control.pwmInput_Settings.maxDemand_u16)
        {
          pwm_input_demand_u32 = pwmInput_Control.pwmInput_Settings.maxDemand_u16;
        }
      } else
      {
        pwmInput_Control.pwmInput_Data.discretes_u16.is_pwmFrequencyLow = TRUE;   
        pwm_input_demand_u32 = 0;
      }
    }
  } 
  // In case of negative demand calculated, demand is scaled to zero
  if (pwm_input_demand_u32 < 0)
  {
    pwm_input_demand_u32 = 0;
  }
  
  // Calculation for determining if Demand is on
 // if (pwm_input_demand_u32 != 0)
 // {
 //   pwmInput_Control.pwmInput_Data.discretes_u16.is_demandOn = TRUE;
 // } else
//  {
 //   pwmInput_Control.pwmInput_Data.discretes_u16.is_demandOn = FALSE;
//  }
  
  if (pwm_input_demand_u32 < pwmInput_Control.pwmInput_Settings.minDemand_u16)
    {
      // Check if low end demand hysteresis
      if (pwm_input_demand_u32 < (pwmInput_Control.pwmInput_Settings.minDemand_u16 - pwmInput_Control.pwmInput_Settings.minDemandHysteresis_u16))
      {
        pwm_input_demand_u32 = 0;
        pwmInput_Control.pwmInput_Data.discretes_u16.is_demandOn = FALSE;
        pwmInput_Control.pwmInput_Data.discretes_u16.is_lowerHysteresisEnable = FALSE;
      }
    } else {
      pwmInput_Control.pwmInput_Data.discretes_u16.is_demandOn = TRUE; // Demand greater than min demand.  
      pwmInput_Control.pwmInput_Data.discretes_u16.is_lowerHysteresisEnable = TRUE; // Enable Hysteresis
    }
  
  //pwmInput_Control.pwmInput_Data.pwmInputDemand_u16 = pwm_input_demand_u32;
  return (pwm_input_demand_u32);  
}

/**
********************************************************************************************************************************
* @brief   Set module demand to zero
* @details Clear all local live data
* @param   None 
* @return  None
********************************************************************************************************************************
*/
void SetDemandToZero(void)
{ 
  if (pwmInput_Control.pwmInput_Data.discretes_u16.is_pwmFrequencyLow == TRUE)
  {
     //pwmInput_Control.pwmInput_Data.inheritedPwmInputDutyCycle_f = 0;   // Measured duty cycle
     pwmInput_Control.pwmInput_Data.inheritedPWMInputFrequency_u16 = 0; // Measured frequency
     pwmInput_Control.pwmInput_Data.discretes_u16.is_demandOn = FALSE;                // Set to "1" if demand goes from 0 to above min demand.
     pwmInput_Control.pwmInput_Data.pwmInputDemandPercentage_u16 = 0;
     pwmInput_Control.pwmInput_Data.pwmInputDemand_u16 = 0;
     pwmInput_Control.pwmInput_Data.pwmStableDutyCycle_u16 = 0;  // Measured duty cycle
     pwmInput_Control.pwmInput_Data.discretes_u16.is_lowerHysteresisEnable = FALSE;
     pwmInput_Control.pwmInput_Data.discretes_u16.is_upperHysteresisEnable = FALSE;
  } else
  {
    pwmInput_Control.pwmInput_Data.discretes_u16.is_demandOn = FALSE;                 // Set to "1" if demand goes from 0 to above min demand.
    pwmInput_Control.pwmInput_Data.pwmInputDemandPercentage_u16 = 0;
    pwmInput_Control.pwmInput_Data.pwmInputDemand_u16 = 0;
    pwmInput_Control.pwmInput_Data.discretes_u16.is_lowerHysteresisEnable = FALSE;
    pwmInput_Control.pwmInput_Data.discretes_u16.is_upperHysteresisEnable = FALSE;
  }
}

/**             
********************************************************************************************************************************
* @brief   Get data from PWM input driver.
* @details Get duty cycle and frequency from tim1_Control
* @param   None 
* @return  None
********************************************************************************************************************************
*/
void UpdateModuleRuntimeData(void)
{   
  pwm_input_time_u64 = getSysCount() + pwmInput_Control.pwmInput_Settings.pwmPeriod_u16; // Update time tick value
  if(pwm_input_low_alarm_enable_count_u16 > 0)
  { // Wait time count to check pwm input low
    pwm_input_low_alarm_enable_count_u16--;
  }
  pwmInput_Control.pwmInput_Data.inheritedPwmInputDutyCycle_f  = ((*tim1_LocalControl).tim1_ResultAvg.PWMInputDutyCycleAverageCalc_f); // represented in xxxyy format which is xxx.yy%
  pwmInput_Control.pwmInput_Data.inheritedPWMInputFrequency_u16 = ((*tim1_LocalControl).tim1_ResultAvg.PWMInputFrequencyAverageCalc_u16);
  pwmInput_Control.pwmInput_Data.pwmErrorCode_u16 = (*tim1_LocalControl).tim1_ResultAvg.errorCode_u8; // Get any error codes
  
  pwmInput_Control.pwmInput_Data.inheritedPwmInputDutyCycle_f += (float)(pwmInput_Control.pwmInput_Settings.dutyCyleCalibratonFactor_f * pwmInput_Control.pwmInput_Data.inheritedPWMInputFrequency_u16); // Calibrate duty cycle
  
  if(pwmInput_Control.pwmInput_Data.inheritedPwmInputDutyCycle_f > 100)
  {
    pwmInput_Control.pwmInput_Data.inheritedPwmInputDutyCycle_f = 100;
  }
  pwm_input_duty_cycle_percentage_tmp_u16 = (uint16_t)(pwmInput_Control.pwmInput_Data.inheritedPwmInputDutyCycle_f * 100); //((*tim1_LocalControl).tim1_ResultAvg.PWMInputDutyCycleAverageCalc_f);

  if(pwmInput_Control.pwmInput_Settings.flags_u16.is_invertDigitalPWM == TRUE) 
  {
    pwm_input_duty_cycle_percentage_tmp_u16 = 10000-pwm_input_duty_cycle_percentage_tmp_u16;//(uint16_t)(pwmInput_Control.pwmInput_Data.inheritedPwmInputDutyCycle_f * 100); //((*tim1_LocalControl).tim1_ResultAvg.PWMInputDutyCycleAverageCalc_f);
  }
  pwm_input_frequency_tmp_u16 = ((*tim1_LocalControl).tim1_ResultAvg.PWMInputFrequencyAverageCalc_u16);  
}

/**
********************************************************************************************************************************
* @brief   Apply debounce to pwm input duty cycle.
* @details Uses pwmInputDebounceCount_u8 count for debounce
* @param   None 
* @return  None
********************************************************************************************************************************
*/
void CheckDigitalInputDebounce()
{
  uint16_t input_status_u16 = 0;
  input_status_u16 = (uint8_t)GET_BIT((*gpio_PWMInputLocalControl).gpio_Result.gpio_Status_u16, PWM_IN_NUM);
  if(input_status_u16 != pwmInput_Control.pwmInput_Data.discretes_u16.is_digitalInputON)
  {
    if(input_status_u16 == BIT_HI)
    {
      digital_input_current_bit_low_debounce_count_u16 = 0;
      digital_input_current_bit_hi_debounce_count_u16++;
      if(digital_input_current_bit_hi_debounce_count_u16 > pwmInput_Control.pwmInput_Settings.pwmInputDebounceCount_u16)
      {
        pwmInput_Control.pwmInput_Data.discretes_u16.is_digitalInputON = BIT_HI;
        digital_input_current_bit_hi_debounce_count_u16 = 0;
      }      
    }else { // input_status_u8 == BIT_LOW
      digital_input_current_bit_hi_debounce_count_u16 = 0;
      digital_input_current_bit_low_debounce_count_u16++;
      if(digital_input_current_bit_low_debounce_count_u16 > pwmInput_Control.pwmInput_Settings.pwmInputDebounceCount_u16)
      {
        pwmInput_Control.pwmInput_Data.discretes_u16.is_digitalInputON = BIT_LOW;
        digital_input_current_bit_low_debounce_count_u16 = 0;
      }       
    }
  }  
}