/**
*************************************************************************************
* @file    module_analog_0_10v.c 
* @author  Regal, Satya Akkina
* @version V1.0
* @date    17-Aug-2020
* @brief   module for Analog 0-10V input
* @note    Provide structures to access Analog setting and Analog volts, demand
*************************************************************************************
*/

// Includes -------------------------------------------------------------------
#include "module_analog_0_10v.h"

//#include <stdio.h>
#include <math.h>

#include "driver_usart2.h"
#include "module_usart2.h"

#include "macros.h"

// Content --------------------------------------------------------------------
// Function Prototypes
void AssignModuleMemAnalog_0_10V(void);
void Init_Analog_Setting(void);
void InitAnalogVoltsInputParameters(void);
void DebounceAnalogVolts(void);
uint16_t GetAnalogDemand(void);

// Constants ------------------------------------------------------------------
// Module States
enum 
{
  MEMORY_INIT_MODULE,
  INIT_MODULE,
  RUN_MODULE,
  // Ddditional states to be added here as necessary.
  IRQ_MODULE = DEFAULT_IRQ_STATE,
  KILL_MODULE = KILL_APP
};

// External Variables ---------------------------------------------------------
extern Ram_Buf sharedMemArray[TOTAL_NUM_OF_STRUCT_MEM_INSTANCES];
extern ProcessInfo processInfoTable[];
static  Ram_Buf_Handle analog_0_10v_StructMem_u32;

// Global variables specific to this module ---------------------------------
uint64_t module_analog_0_10v_poll_time_u64;      // Delay time between init and run module
uint16_t analog_increasing_debounce_count_u16 = 0; // Count to check if analog volts is above debounce value
uint16_t analog_decreasing_debounce_count_u16 = 0; // Count to check if analog volts is below debounce value
uint16_t analog_volts_counts_u16 = 0;            // Temporary storage for ADC value
uint16_t analog_low_alarm_enable_count_u16 = 0;    // Time when analog module init is complete
uint8_t is_volts_above_loss_analog = FALSE;      // Time when analog module init is complete

int32_t temperature_degc_s16=0;

// Define Pointers that will be used as References to other Modules, where applicable
ADC1_Control*  adc1_LocalControl;
AnalogVolts_Control analogVolts_Control;


/**
********************************************************************************************************************************
* @brief   State machine for Analog 0-10V module
* @details
* @param   drv_identifier_u8, previous_state_u8, next_stat_u8, irq_identfier_u8
* @retval  return_state_u8
********************************************************************************************************************************
*/
uint8_t moduleAnalog_0_10V(uint8_t drv_identifier_u8, uint8_t previous_state_u8, uint8_t next_state_u8,
                           uint8_t irq_identifier_u8)
{
  uint8_t return_state_u8 = MEMORY_INIT_MODULE;
  switch (next_state_u8)
  {
  case MEMORY_INIT_MODULE:
    {
      AssignModuleMemAnalog_0_10V(); // Assign structured memory to Analog 0-10V setting and data 
      return_state_u8 = INIT_MODULE;
      break;
    }
  case INIT_MODULE:
    {     
      //AssignModuleMemAnalog_0_10V(); // Assign structured memory to Analog 0-10V setting and data      
      Init_Analog_Setting(); // Initilize analog 0-10V settings   
      InitAnalogVoltsInputParameters(); // Init local parameters and metering data
      
      // Get structured memory for ADC1 data
      uint8_t module_ADC1_Index = getProcessInfoIndex(MODULE_ADC1);
      adc1_LocalControl = (ADC1_Control*)((*(processInfoTable[module_ADC1_Index].Sched_DrvData.p_masterSharedMem_u32)).p_ramBuf_u8);
      
      analog_low_alarm_enable_count_u16 = analogVolts_Control.analogVolts_Setting.analogLowAlarmEnableCount_u16; // delay before analog low voltage alarm can be triggered
      module_analog_0_10v_poll_time_u64 = getSysCount() + analogVolts_Control.analogVolts_Setting.analogPeriod_u16; //AnalogPeriod;    // Store time tick value
      
      //return_state_u8 = INIT_MODULE;
      return_state_u8 = RUN_MODULE;
      break;
      
    }   
  case RUN_MODULE:                                                             
    {      
      // Process analog input every "analogPeriod_u16" mSec
      if (getSysCount() >= module_analog_0_10v_poll_time_u64) 
      {
        module_analog_0_10v_poll_time_u64 = getSysCount() + analogVolts_Control.analogVolts_Setting.analogPeriod_u16; //AnalogPeriod;   // Update time tick value
        if(analog_low_alarm_enable_count_u16 > 0){
          analog_low_alarm_enable_count_u16--;
        } 
        if(analog_low_alarm_enable_count_u16 == 0) // On power up wait for few sec to detect correct analog input
          {
        // Convert ADC1 value into 100.00%. analogVoltsPercent_u16 value is stored in xxxyy format = xxx.yy% 
        analogVolts_Control.analogVolts_Data.analogVoltsPercent_u16 = DECIMAL_RIGHT_SHIFT_U16(CALCULATE_PERCENT((*adc1_LocalControl).adc1_ResultAvg.adc1_0_10V_Avg_u16, ADC_12_BIT_FULL_SCALE),2);
        analog_volts_counts_u16 = (*adc1_LocalControl).adc1_ResultAvg.adc1_0_10V_Avg_u16; // Get result from ADC module into analog 0-10V module
        analogVolts_Control.analogVolts_Data.analogErrorCode_u16 = (uint16_t) ((*adc1_LocalControl).adc1_ResultAvg.errorCode_u8); // Get any error codes
        DebounceAnalogVolts(); // Process the analog voltage
        
        // Check if analog is low. If enabled and analog is low, demand is "0"
        if( ((analogVolts_Control.analogVolts_Data.discretes_u16.is_analogLow == TRUE) && \
			 (analogVolts_Control.analogVolts_Setting.flags_u16.is_analogLowEnable == TRUE) && \
			 (analogVolts_Control.analogVolts_Setting.flags_u16.is_analogFailSafeEnable == FALSE)) || \
		     (analogVolts_Control.analogVolts_Setting.inputMode_u16 == DISABLE_INPUT)|| \
                       (analogVolts_Control.analogVolts_Setting.flags_u16.is_enableAnalog == FALSE))
        {
          analogVolts_Control.analogVolts_Data.analogDemand_u16 = 0;
          analogVolts_Control.analogVolts_Data.analogDemandPercent_u16 = 0;
          analogVolts_Control.analogVolts_Data.discretes_u16.is_demandOn = FALSE;
          analogVolts_Control.analogVolts_Data.discretes_u16.is_lowerHysteresisEnable = FALSE;
          analogVolts_Control.analogVolts_Data.discretes_u16.is_upperHysteresisEnable = FALSE;
        } else { // Analog low is not detected or analog low is not enabled          
            analogVolts_Control.analogVolts_Data.analogDemand_u16 = GetAnalogDemand(); // Convert analog voltage into demand 0 - 0xFFFF
          }
          // Convert demand value into 0 to 100.00%. Value is stored in xxxyy format = xxx.yy% 
          analogVolts_Control.analogVolts_Data.analogDemandPercent_u16 = DECIMAL_RIGHT_SHIFT_U16(CALCULATE_PERCENT(analogVolts_Control.analogVolts_Data.analogDemand_u16, 65536),2);
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
********************************************************************************************************************************
* @brief   Assign structured memory
* @details Assign structured memory for Analog 0-10V control
* @param   None
* @retval  None
********************************************************************************************************************************
*/
//
void AssignModuleMemAnalog_0_10V(void){   
  analog_0_10v_StructMem_u32 =  StructMem_CreateInstance(MODULE_ANALOG_0_10V, sizeof(AnalogVolts_Control), ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);
  (*analog_0_10v_StructMem_u32).p_ramBuf_u8 = (uint8_t *)&analogVolts_Control ;    // Map the ADC1 memory into the structured memory
  uint8_t Module_Analog_0_10v_Index = getProcessInfoIndex(MODULE_ANALOG_0_10V);
  processInfoTable[Module_Analog_0_10v_Index].Sched_ModuleData.p_masterSharedMem_u32 = (Ram_Buf_Handle)analog_0_10v_StructMem_u32;
}

/**
********************************************************************************************************************************
* @brief   Initilize all analog settings and live data
* @details Read settings from the RAM and initilize the settings
* @retval  None
********************************************************************************************************************************
*/
void Init_Analog_Setting(void){  
  analogVolts_Control.analogVolts_Setting.onVolts_u16 = 621;//205;               // Demand turn ON volts in counts of ADC
  analogVolts_Control.analogVolts_Setting.offVolts_u16 = 310;//164;              // Demand turn OFF volts in counts of ADC. Acts as Hysteresis
  analogVolts_Control.analogVolts_Setting.minVolts_u16 = 931;//300;              // Min volts correspodning to min demand
  analogVolts_Control.analogVolts_Setting.maxVolts_u16 = 2793;//1284;             // Max volts corresponding to max demand  
  analogVolts_Control.analogVolts_Setting.maxAdcCounts_u16 = 4095;         // Max output of ADC. 12 bit ADC left sifted by 4
  analogVolts_Control.analogVolts_Setting.debounceThresholdCounts_u16 = 10;//21;// Change demand if the change in analog is outside this threshold. In ADC counts
  analogVolts_Control.analogVolts_Setting.calibratonFactor_s16 = 0;        // Add/Delete this form measured for calibration. In ADC counts
  analogVolts_Control.analogVolts_Setting.voltsHysteresis_u16 = 10 ;       // Hys volts at the top end of demand. In ADC counts
  analogVolts_Control.analogVolts_Setting.minDemand_u16 = 10922;//21845;//13107;           // Min allowed demand. 20%
  analogVolts_Control.analogVolts_Setting.maxDemand_u16 = 0xFFFF;          // Maximum allowed demand
  analogVolts_Control.analogVolts_Setting.minDemandHysteresis_u16 = 21;   // Lower end hysteresis if Min_Demand > Off_Volts demand. Counts 
  analogVolts_Control.analogVolts_Setting.analogLowVolts_u16 = 125;//250;        // Volts below which the bAnalog_Loss_Enable is set. In ADC counts
  analogVolts_Control.analogVolts_Setting.failSafeDemand_u16  = 0x7FFF;    // If set to "1" use Fail safe demand when low analog input is detected //50%
  analogVolts_Control.analogVolts_Setting.analogPeriod_u16 = 50;           // Update rate in mSec for the analog voltage
  analogVolts_Control.analogVolts_Setting.analogLowAlarmEnableCount_u16 = 0;//5;//100;   // analogPeriod*count delay before analog low volts alarm is detected in mSec. 
  analogVolts_Control.analogVolts_Setting.debounceLoopCount_u16 = 4;             // Debounce count
  analogVolts_Control.analogVolts_Setting.analogGain_f = 0.08056640625;         // Convert ADC coutns to volts 100*3.3/4096
  
  analogVolts_Control.analogVolts_Setting.flags_u16.is_enableAnalog = TRUE;          // If set to "0", 0V = Max demand 10V= 0 Demand
  analogVolts_Control.analogVolts_Setting.flags_u16.is_invertAnalog = FALSE;         // If set to "0", 0V = Max demand 10V= 0 Demand
  analogVolts_Control.analogVolts_Setting.flags_u16.is_analogLowEnable = TRUE;//FALSE;       // Enable analog low detect if set to "1"
  analogVolts_Control.analogVolts_Setting.flags_u16.is_analogFailSafeEnable = FALSE; // If set to "1", switch to fail safe demand when anlaog low is detected  
  analogVolts_Control.analogVolts_Setting.flags_u16.is_analogMinDemandEnable = TRUE;//FALSE;// If "True", min demand is maintaned when volts is below minVolts and above onVolts
  
  analogVolts_Control.analogVolts_Setting.inputMode_u16 = ANALOG_MODE;//DISABLE_INPUT;//  // 0 = Analog input; 1 = digital input; 
  analogVolts_Control.analogVolts_Setting.digitalOnVolts_u16 = 1000;   // On volts above which for digitial input high
  analogVolts_Control.analogVolts_Setting.digtialOffVolts_u16 = 800;   // Off volts below which digital input is low  
}

void InitAnalogVoltsInputParameters(void)
{
  // Initilize live data                                                   // Flag indicates when the analog module is first initlized
  analogVolts_Control.analogVolts_Data.analogVoltsCounts_u16 = 0;        // Current Filtered Analog ADC Counts, 0 to 4096
  analogVolts_Control.analogVolts_Data.analogVolts_u16 = 0;              // Current Filtered Analog Volts, 0.00V
  analogVolts_Control.analogVolts_Data.analogVoltsCountsStable_u16 = 0;  // Previous measured analog volts used for debounce check
  analogVolts_Control.analogVolts_Data.analogDemand_u16 = 0;             // Calculated analog demand
  analogVolts_Control.analogVolts_Data.analogVoltsPercent_u16 = 0;       // Analog Volts scalled 0 to 100%
  analogVolts_Control.analogVolts_Data.analogDemandPercent_u16 = 0;      // Analog Demand scalled 0 to 100%  
  analogVolts_Control.analogVolts_Data.increasingIntercept_f = 0;    // Intercept used to calculate demand when analog is increasing
  analogVolts_Control.analogVolts_Data.decreasingIntercept_f = 0;    // Intercept used to calcualte demand when analog is decreasing
  
  analog_low_alarm_enable_count_u16 = analogVolts_Control.analogVolts_Setting.analogLowAlarmEnableCount_u16;
  uint16_t demand_range_u16 = 0;
  uint16_t analog_range_u16 = 0;
  
//  demand_range_u16 = analogVolts_Control.analogVolts_Setting.maxDemand_u16 - analogVolts_Control.analogVolts_Setting.minDemand_u16;
//  analog_range_u16 = (analogVolts_Control.analogVolts_Setting.maxVolts_u16 - analogVolts_Control.analogVolts_Setting.voltsHysteresis_u16) - (analogVolts_Control.analogVolts_Setting.minVolts_u16 - analogVolts_Control.analogVolts_Setting.voltsHysteresis_u16);                                                                 
//  analogVolts_Control.analogVolts_Data.decreasingSlope_f = (float)(demand_range_u16 / ((float)analog_range_u16));                                                                                                                                                                                                                           // Slope used to calculate demand when analog is decreasing
//  analogVolts_Control.analogVolts_Data.decreasingIntercept_u16 = (float)(analogVolts_Control.analogVolts_Setting.minDemand_u16 - (float)((analogVolts_Control.analogVolts_Data.decreasingSlope_f) * (analogVolts_Control.analogVolts_Setting.minVolts_u16 - analogVolts_Control.analogVolts_Setting.voltsHysteresis_u16))); // Intercept used to calcualte demand when analog is decreasing
//  
//  demand_range_u16 = analogVolts_Control.analogVolts_Setting.maxDemand_u16 - analogVolts_Control.analogVolts_Setting.minDemand_u16;
//  analog_range_u16 = analogVolts_Control.analogVolts_Setting.maxVolts_u16 - analogVolts_Control.analogVolts_Setting.minVolts_u16; 
//  analogVolts_Control.analogVolts_Data.increasingSlope_f = (float)(demand_range_u16 / ((float)analog_range_u16));                       // Slope used to calculate demand when analog is increasing
//  analogVolts_Control.analogVolts_Data.increasingIntercept_f = (float)(analogVolts_Control.analogVolts_Setting.minDemand_u16 - (float)((analogVolts_Control.analogVolts_Data.increasingSlope_f) * (analogVolts_Control.analogVolts_Setting.minVolts_u16))); // Interecept used to calculate demand when analog is increasing
  
  demand_range_u16 = analogVolts_Control.analogVolts_Setting.maxDemand_u16 - analogVolts_Control.analogVolts_Setting.minDemand_u16;
  analog_range_u16 = analogVolts_Control.analogVolts_Setting.maxVolts_u16 - analogVolts_Control.analogVolts_Setting.minVolts_u16; 
  analogVolts_Control.analogVolts_Data.increasingSlope_f = (float)(demand_range_u16 / ((float)analog_range_u16)); // Slope used to calculate demand when analog is increasing
  
  if(analogVolts_Control.analogVolts_Setting.flags_u16.is_invertAnalog == TRUE)
  {
    analogVolts_Control.analogVolts_Data.increasingSlope_f = analogVolts_Control.analogVolts_Data.increasingSlope_f * -1;
    analogVolts_Control.analogVolts_Data.increasingIntercept_f = (float)(analogVolts_Control.analogVolts_Setting.minDemand_u16 - (float)((analogVolts_Control.analogVolts_Data.increasingSlope_f) * (analogVolts_Control.analogVolts_Setting.maxVolts_u16))); // Interecept used to calculate demand when analog is increasing
  }else
  {
    analogVolts_Control.analogVolts_Data.increasingIntercept_f = (float)(analogVolts_Control.analogVolts_Setting.minDemand_u16 - (float)((analogVolts_Control.analogVolts_Data.increasingSlope_f) * (analogVolts_Control.analogVolts_Setting.minVolts_u16))); // Interecept used to calculate demand when analog is increasing
  }
  analogVolts_Control.analogVolts_Data.decreasingSlope_f = analogVolts_Control.analogVolts_Data.increasingSlope_f;
  analogVolts_Control.analogVolts_Data.decreasingIntercept_f = analogVolts_Control.analogVolts_Data.increasingIntercept_f; 
  
  
  // Assumes 0 V = 0 demand; Max volts = Max demand  
  analogVolts_Control.analogVolts_Data.discretes_u16.is_analogLow = FALSE;             // Set to "TRUE", if analog low is detected
  analogVolts_Control.analogVolts_Data.discretes_u16.is_decreasingAnalog = FALSE;       // Set to "TRUE", if analog voltage is decresing
  analogVolts_Control.analogVolts_Data.discretes_u16.is_lowerHysteresisEnable = FALSE;  // Set to "TRUE", if lower end hys is enabled
  analogVolts_Control.analogVolts_Data.discretes_u16.is_upperHysteresisEnable = FALSE;  // Set to "TRUE", if upper end hys is enabled 
  analogVolts_Control.analogVolts_Data.discretes_u16.is_demandOn = FALSE;               // Set to"TRUE", if demand goes from 0 to above min_Demand_u16.  
}

/**
********************************************************************************************************************************
* @brief   Check if Analog is increasing or decresing
* @details Check for debounce. Only update Analog if change is greater than debounce
debounce. Check for Low Analog Volts. 
* @param   None
* @retval  None
********************************************************************************************************************************
*/
void DebounceAnalogVolts(void){
  uint16_t abs_volts_diff_u16=0;
  abs_volts_diff_u16 = abs(analogVolts_Control.analogVolts_Data.analogVoltsCountsStable_u16  - analog_volts_counts_u16);
  
  // Check debounce threshold
  if(abs_volts_diff_u16 > analogVolts_Control.analogVolts_Setting.debounceThresholdCounts_u16)
  {  // Change is analog is above debounce
    if (analogVolts_Control.analogVolts_Data.analogVoltsCountsStable_u16  >  analog_volts_counts_u16)
    { // Analog is decreasing
      analog_decreasing_debounce_count_u16++;
      analog_increasing_debounce_count_u16 = 0;
    } else if (analogVolts_Control.analogVolts_Data.analogVoltsCountsStable_u16  <  analog_volts_counts_u16)
    { // Analog is increasing
      analog_increasing_debounce_count_u16++;
      analog_decreasing_debounce_count_u16 = 0;
    } 
    
    // Check debounce loop count
    if ( (analog_increasing_debounce_count_u16 > analogVolts_Control.analogVolts_Setting.debounceLoopCount_u16) || (analog_decreasing_debounce_count_u16 > analogVolts_Control.analogVolts_Setting.debounceLoopCount_u16)  )
    {
      analogVolts_Control.analogVolts_Data.analogVoltsCountsStable_u16 = analogVolts_Control.analogVolts_Data.analogVoltsCounts_u16 + analogVolts_Control.analogVolts_Setting.calibratonFactor_s16; // Update only when debounce count is satisfied and calibrate
      analogVolts_Control.analogVolts_Data.analogVolts_u16 = (uint16_t)(analogVolts_Control.analogVolts_Setting.analogGain_f* analogVolts_Control.analogVolts_Data.analogVoltsCountsStable_u16); // Convert ADC count to volts // No need to right shift since gain is adjusted by factor of 100
      
      analogVolts_Control.analogVolts_Data.analogVoltsCounts_u16 = analog_volts_counts_u16; // Update only when debounce count is satisfied  
      
      // Determine if volts are increasing or decreasing
      if(analog_decreasing_debounce_count_u16 > analogVolts_Control.analogVolts_Setting.debounceLoopCount_u16) 
      {
        analogVolts_Control.analogVolts_Data.discretes_u16.is_decreasingAnalog = TRUE;
      } else
      {
        analogVolts_Control.analogVolts_Data.discretes_u16.is_decreasingAnalog = FALSE;
      }
      analog_decreasing_debounce_count_u16 = 0;
      analog_increasing_debounce_count_u16 = 0;
    }
    
    // Set Digital input flag
    if(analogVolts_Control.analogVolts_Data.analogVoltsCountsStable_u16 < analogVolts_Control.analogVolts_Setting.digtialOffVolts_u16)
    {
      analogVolts_Control.analogVolts_Data.discretes_u16.is_digitalInputON = FALSE;
    } else{
      analogVolts_Control.analogVolts_Data.discretes_u16.is_digitalInputON = TRUE;
    }
    
    // Delay low analog detection. Need when power up
    if(analog_low_alarm_enable_count_u16 == 0) 
    { // Check if analog volts is low
      if (analogVolts_Control.analogVolts_Data.analogVoltsCounts_u16 < analogVolts_Control.analogVolts_Setting.analogLowVolts_u16)
      {
        analogVolts_Control.analogVolts_Data.discretes_u16.is_analogLow = TRUE;
      } else {
        analogVolts_Control.analogVolts_Data.discretes_u16.is_analogLow = FALSE;
      }
    } else{
      is_volts_above_loss_analog = TRUE;
      analogVolts_Control.analogVolts_Data.discretes_u16.is_analogLow = FALSE;
    }
    
  } else
  { // debounce threshold not satisified
    analog_increasing_debounce_count_u16 = 0;
    analog_decreasing_debounce_count_u16 = 0;
  }
}

/**
********************************************************************************************************************************
* @brief   Convert Analog volts to Demand ( min_Demand_u16 to max_Demand_u16)
* @details Check for hysteresis, turn ON/OFF and calculate demand 
* @retval  None
********************************************************************************************************************************
*/
uint16_t GetAnalogDemand(void){
  uint32_t analog_demand_u32 = 0;
  float slope_f;
  float intercept_f;
  uint16_t analog_volts_u16 = 0; 
  
  analog_volts_u16 = analogVolts_Control.analogVolts_Data.analogVoltsCountsStable_u16; // Used in case analog demand is inverted
  // Check if input mode is enabled (ANALOG_MODE)
  if (analogVolts_Control.analogVolts_Setting.inputMode_u16 != ANALOG_MODE)
  {
    analog_demand_u32 = 0;
    analogVolts_Control.analogVolts_Data.discretes_u16.is_demandOn = FALSE;
  } else if( (analogVolts_Control.analogVolts_Setting.flags_u16.is_analogLowEnable == TRUE) && (analogVolts_Control.analogVolts_Setting.flags_u16.is_analogFailSafeEnable == TRUE ) && (analogVolts_Control.analogVolts_Data.discretes_u16.is_analogLow == TRUE)) { 
    // Check if Loss of analog 
    if((analogVolts_Control.analogVolts_Data.discretes_u16.is_demandOn == FALSE))//{ && (is_volts_above_loss_analog == FALSE))//demand is never gone above min demand. 
    {  // Analog never crossed turned ON volts
      analog_demand_u32 = 0;
      //analogVolts_Control.analogVolts_Data.discretes_u16.is_demandOn = FALSE;
    } else{// if (is_volts_above_loss_analog == TRUE){ 
      // Use fail safe demand if low analog volts is detected  //If volatge suddenly drops from woking range to below failsafe set voltage
      analog_demand_u32 = analogVolts_Control.analogVolts_Setting.failSafeDemand_u16;
      analogVolts_Control.analogVolts_Data.discretes_u16.is_demandOn = TRUE;
      analogVolts_Control.analogVolts_Data.discretes_u16.is_upperHysteresisEnable = FALSE;
    }
  } else {
    // If invert is enabled invert measured value
    //if (analogVolts_Control.analogVolts_Setting.flags_u16.is_invertAnalog == TRUE) 
    //{
      //analog_volts_u16 = analogVolts_Control.analogVolts_Setting.maxAdcCounts_u16 - analogVolts_Control.analogVolts_Data.analogVoltsCountsStable_u16;
    //}
    
    if (analogVolts_Control.analogVolts_Data.discretes_u16.is_decreasingAnalog == FALSE)
    { // Slope used when analog value is increasing
      slope_f = analogVolts_Control.analogVolts_Data.increasingSlope_f;
      intercept_f = analogVolts_Control.analogVolts_Data.increasingIntercept_f;
    } else {
      // Slope used when analog value is decreasing
      slope_f = analogVolts_Control.analogVolts_Data.decreasingSlope_f;
      intercept_f = analogVolts_Control.analogVolts_Data.decreasingIntercept_f;
    }
    
    // Analog volts is divided up into three parts
    // Part 1: 0v to off volts
    // Part 2: off volts to upper hysteresis volts
    // Part 3: Upper hysteresis volts to max volts
    
    // Part 1, 2 : Analog input between OFF volts and upper hysteresis
    if ((analog_volts_u16 >= analogVolts_Control.analogVolts_Setting.offVolts_u16) && (analog_volts_u16 < (analogVolts_Control.analogVolts_Setting.maxVolts_u16 - analogVolts_Control.analogVolts_Setting.voltsHysteresis_u16)))
    {
      // If analog is between ON and OFF volts. Hysteresis part
      analogVolts_Control.analogVolts_Data.discretes_u16.is_upperHysteresisEnable = FALSE;
      
      // Part 2
      // onVolts till upperHysteresis
      if (analog_volts_u16 >= analogVolts_Control.analogVolts_Setting.onVolts_u16) {
        if (analog_volts_u16 >= analogVolts_Control.analogVolts_Setting.minVolts_u16)
        {          
          // Analog voltage is above ON volts for first time
          analogVolts_Control.analogVolts_Data.discretes_u16.is_lowerHysteresisEnable = TRUE;
          analog_demand_u32 = (uint32_t)((float)(slope_f * analog_volts_u16) + intercept_f);
        } else if (analog_volts_u16 < analogVolts_Control.analogVolts_Setting.minVolts_u16){
          // Analog voltage below minVolts limit
          if((analogVolts_Control.analogVolts_Setting.flags_u16.is_analogMinDemandEnable) == TRUE ){//&& ( analogVolts_Control.analogVolts_Data.discretes_u16.is_lowerHysteresisEnable == TRUE)){
            // Default to min demand when is_analogMinDemandEn is TRUE
            //analog_demand_u32 = analogVolts_Control.analogVolts_Setting.minDemand_u16;
            if(analogVolts_Control.analogVolts_Setting.flags_u16.is_invertAnalog == FALSE)
            {
              analog_demand_u32 = analogVolts_Control.analogVolts_Setting.minDemand_u16;
            } else
            {
              analog_demand_u32 = analogVolts_Control.analogVolts_Setting.maxDemand_u16;
            }
          } else if( (analog_volts_u16 >= (analogVolts_Control.analogVolts_Setting.minVolts_u16 - analogVolts_Control.analogVolts_Setting.voltsHysteresis_u16)) && ( analogVolts_Control.analogVolts_Data.discretes_u16.is_lowerHysteresisEnable == TRUE)){
            // Apply lower end hysteresis
            analog_demand_u32 = (uint32_t)((float)(slope_f * analog_volts_u16) + intercept_f);
          } else{
            if ((analogVolts_Control.analogVolts_Setting.flags_u16.is_invertAnalog == TRUE) && (analogVolts_Control.analogVolts_Setting.flags_u16.is_analogMinDemandEnable == TRUE))
            {
              analog_demand_u32 = analogVolts_Control.analogVolts_Setting.minDemand_u16;
            } else{
              // Analog volts below hysteresis value
              analog_demand_u32 = 0;
              analogVolts_Control.analogVolts_Data.discretes_u16.is_lowerHysteresisEnable = FALSE;
              analogVolts_Control.analogVolts_Data.discretes_u16.is_demandOn = FALSE;  
            }
          }          
        }
      }else if (analog_volts_u16 >= analogVolts_Control.analogVolts_Setting.offVolts_u16) {
        // Part 1
        // upper hysteresisdown to off volatge
        
        // Demand is minimum until analog reaches below off volatge
        if((analogVolts_Control.analogVolts_Setting.flags_u16.is_analogMinDemandEnable) == TRUE && ( analogVolts_Control.analogVolts_Data.discretes_u16.is_lowerHysteresisEnable == TRUE)){
            // Default to min demand when is_analogMinDemandEn is TRUE
            //analog_demand_u32 = analogVolts_Control.analogVolts_Setting.minDemand_u16;
            if(analogVolts_Control.analogVolts_Setting.flags_u16.is_invertAnalog == FALSE)
            {
              analog_demand_u32 = analogVolts_Control.analogVolts_Setting.minDemand_u16;
            } else
            {
              analog_demand_u32 = analogVolts_Control.analogVolts_Setting.maxDemand_u16;
            }
        }else{
          analog_demand_u32 = 0;
          analogVolts_Control.analogVolts_Data.discretes_u16.is_demandOn = FALSE;
        }
      } else if (analog_volts_u16 < analogVolts_Control.analogVolts_Setting.onVolts_u16) {
        // Part 1
        // 0V till onVolts
        
        // Demand is zero when analog never crossed ON volts
        analog_demand_u32 = 0;
        analogVolts_Control.analogVolts_Data.discretes_u16.is_demandOn = FALSE;
      } else {
        analog_demand_u32 = 0;
        analogVolts_Control.analogVolts_Data.discretes_u16.is_demandOn = FALSE;
      }
    } 
	else if ( (analog_volts_u16 >= (analogVolts_Control.analogVolts_Setting.maxVolts_u16 - analogVolts_Control.analogVolts_Setting.voltsHysteresis_u16)) && (analog_volts_u16 < analogVolts_Control.analogVolts_Setting.maxVolts_u16) ){
     // Part 3: 
      // upperHysteresis till maxVolts
      
      // Analog voltage above upper hysteresis voltage      
      if (analogVolts_Control.analogVolts_Data.discretes_u16.is_upperHysteresisEnable  == TRUE)
      { // Analog already crossed hysteresis point
        if (analogVolts_Control.analogVolts_Data.discretes_u16.is_decreasingAnalog == TRUE)
        { // Output demand = max demand till below hysteresis value
          if(analogVolts_Control.analogVolts_Setting.flags_u16.is_invertAnalog == FALSE)
          {
            analog_demand_u32 = analogVolts_Control.analogVolts_Setting.maxDemand_u16;
          } else
          {
            analog_demand_u32 = analogVolts_Control.analogVolts_Setting.minDemand_u16;
          }
        } else {
          // Analog volts never when above hysteresis voltage
          analog_demand_u32 = (uint32_t)((float)(slope_f * analog_volts_u16) + intercept_f);
        }
      } else if (analog_volts_u16 <= analogVolts_Control.analogVolts_Setting.maxVolts_u16) {
        // Analog went above Upper Hysteresis value for the first time
        analogVolts_Control.analogVolts_Data.discretes_u16.is_upperHysteresisEnable  = TRUE;
        analog_demand_u32 = (uint32_t)((float)(slope_f * analog_volts_u16) + intercept_f);
      } 
	  //else if (analog_volts_u16 > analogVolts_Control.analogVolts_Setting.maxVolts_u16) {
//        if(analogVolts_Control.analogVolts_Setting.is_invertAnalog == FALSE)
//        {
//          // Saturate demand value
//          analogVolts_Control.analogVolts_Data.is_upperHysteresisEnable  = TRUE;
//          analog_demand_u32 = analogVolts_Control.analogVolts_Setting.maxDemand_u16;
//        } else
//        {
//          // Saturate demand value for inverted input
//          //if( analog_volts_u16 > (analogVolts_Control.analogVolts_Setting.maxVolts_u16) )
//          analogVolts_Control.analogVolts_Data.is_upperHysteresisEnable  = FALSE;
//          analog_demand_u32 = 0;
//        }
//        
//      }
	} else if(analog_volts_u16 >= analogVolts_Control.analogVolts_Setting.maxVolts_u16) 
    {
      if(analogVolts_Control.analogVolts_Setting.flags_u16.is_invertAnalog == FALSE)
      {
        // Saturate demand value
        analogVolts_Control.analogVolts_Data.discretes_u16.is_upperHysteresisEnable  = TRUE;
        analog_demand_u32 = analogVolts_Control.analogVolts_Setting.maxDemand_u16;
      } else
      {
        // Saturate demand value for inverted input
        //if( analog_volts_u16 > (analogVolts_Control.analogVolts_Setting.maxVolts_u16) )
       // analogVolts_Control.analogVolts_Data.discretes_u16.is_upperHysteresisEnable  = FALSE;
        //analog_demand_u32 = 0;
        analogVolts_Control.analogVolts_Data.discretes_u16.is_upperHysteresisEnable  = TRUE;
        analog_demand_u32 = analogVolts_Control.analogVolts_Setting.minDemand_u16;
      }      
    }else{// lessthan OFF volatge
       analog_demand_u32 = 0;
       analogVolts_Control.analogVolts_Data.discretes_u16.is_demandOn = FALSE;
    }
    
    
    // Min demand is min_Demand_u16.
    if (analog_demand_u32 < analogVolts_Control.analogVolts_Setting.minDemand_u16)
    {
      // Check if low end demand hysteresis
      if (analog_demand_u32 < (analogVolts_Control.analogVolts_Setting.minDemand_u16 - analogVolts_Control.analogVolts_Setting.minDemandHysteresis_u16))
      {
        analog_demand_u32 = 0;
        analogVolts_Control.analogVolts_Data.discretes_u16.is_demandOn = FALSE;
        analogVolts_Control.analogVolts_Data.discretes_u16.is_lowerHysteresisEnable = FALSE;
      }
    } else {
      analogVolts_Control.analogVolts_Data.discretes_u16.is_demandOn = TRUE; // Demand greater than min demand.  
      analogVolts_Control.analogVolts_Data.discretes_u16.is_lowerHysteresisEnable = TRUE; // Enable Hysteresis
    }
    
    // Demand can't exceed max_Demand_u16
    if (analog_demand_u32 > analogVolts_Control.analogVolts_Setting.maxDemand_u16)
    {
      analog_demand_u32 = (uint16_t)analogVolts_Control.analogVolts_Setting.maxDemand_u16;
    }      
  }
  analogVolts_Control.analogVolts_Data.analogDemand_u16 = analog_demand_u32;
  return ((uint16_t)analog_demand_u32);  
}