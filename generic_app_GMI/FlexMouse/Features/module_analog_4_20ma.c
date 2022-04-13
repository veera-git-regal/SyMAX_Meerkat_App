/**
*************************************************************************************
* @file    module_analog_4_20ma.c 
* @author  Regal, Satya Akkina
* @version V1.0
* @date    04-Dec-2020
* @brief   module for Analog 4-20mA input
* @note    Provide structures to access Analog setting and Analog volts, demand
*************************************************************************************
*/

// Includes -------------------------------------------------------------------
#include "module_analog_4_20ma.h"

//#include <stdio.h>
#include <math.h>
#include "macros.h"

// Content --------------------------------------------------------------------
// Function Prototypes
void AssignModuleMemAnalog_4_20mA(void);
void Init_Analog_4_20mA_Setting(void);
void InitAnalogAmpsInputParameters(void);
void DebounceAnalogAmps(void);
uint16_t GetAnalogAmpsDemand(void);

// Constants ------------------------------------------------------------------
// Module States
enum {
  MEMORY_INIT_MODULE,
  INIT_MODULE,
  RUN_MODULE,
  // Ddditional states to be added here as necessary.
  IRQ_MODULE = DEFAULT_IRQ_STATE,
  KILL_MODULE = KILL_APP
};

enum {
  DISABLE_INPUT = 0,
  ANALOG_MODE = 1,
  DIGITAL_MODE = 2
};

// External Variables ---------------------------------------------------------
extern Ram_Buf sharedMemArray[TOTAL_NUM_OF_STRUCT_MEM_INSTANCES];
extern ProcessInfo processInfoTable[];
static  Ram_Buf_Handle analog_4_20ma_StructMem_u32;

// Global variables specific to this module ---------------------------------
uint64_t module_analog_4_20ma_poll_time_u64; // Delay time between init and run module
uint8_t analog_amps_increasing_debounce_count_u16 = 0; // Count to check if analog volts is above debounce value
uint8_t analog_amps_decreasing_debounce_count_u16 = 0; // Count to check if analog volts is below debounce value
uint16_t analog_amps_counts_u16 = 0;         // Temporary storage for ADC value
uint8_t analog_amps_low_alarm_enable_count_u16 = 0;    // Time when analog module init is complete
uint8_t is_amps_above_loss_analog = FALSE;   // Time when analog module init is complete

// Define Pointers that will be used as References to other Modules, where applicable
ADC1_Control*  adc1_LocalAnalogAmpsControl_ptr;
Analog_4_20ma_Control analog_4_20ma_Control_ptr;

/**
********************************************************************************************************************************
* @brief   State machine for Analog 4-20mA module
* @details
* @param   drv_identifier_u8, previous_state_u8, next_stat_u8, irq_identfier_u8
* @retval  return_state_u8
********************************************************************************************************************************
*/
uint8_t moduleAnalog_4_20mA(uint8_t drv_identifier_u8, uint8_t previous_state_u8, uint8_t next_state_u8,
                            uint8_t irq_identifier_u8)
{
  uint8_t return_state_u8 = MEMORY_INIT_MODULE;
  switch (next_state_u8)
  {
  case MEMORY_INIT_MODULE:
    {
      AssignModuleMemAnalog_4_20mA(); // Assign structured memory to Analog 0-10V setting and data 
      return_state_u8 = INIT_MODULE;
      break;
    }
  case INIT_MODULE:
    {     
      //AssignModuleMemAnalog_4_20mA(); // Assign structured memory to Analog 4-20mA setting and data      
      Init_Analog_4_20mA_Setting(); // Initilize analog 4-20mA settings      
      InitAnalogAmpsInputParameters(); // Init local parameters and metering data
      
      // Get structured memory for ADC1 data
      uint8_t module_ADC1_Index = getProcessInfoIndex(MODULE_ADC1);
      adc1_LocalAnalogAmpsControl_ptr = (ADC1_Control*)((*(processInfoTable[module_ADC1_Index].Sched_DrvData.p_masterSharedMem_u32)).p_ramBuf_u8);
      
      analog_amps_low_alarm_enable_count_u16 = analog_4_20ma_Control_ptr.analog_4_20mA_Setting.analogLowAlarmEnableCount_u16; // delay before analog low voltage alarm can be triggered
      module_analog_4_20ma_poll_time_u64 = getSysCount() + analog_4_20ma_Control_ptr.analog_4_20mA_Setting.analogPeriod_u16; //AnalogPeriod;    // Store time tick value
      
      return_state_u8 = RUN_MODULE;
      break;
      
    }   
  case RUN_MODULE:                                                             
    {      
      // Process analog input every "analogPeriod_u16" mSec
      if (getSysCount() >= module_analog_4_20ma_poll_time_u64) 
      {
        module_analog_4_20ma_poll_time_u64 = getSysCount() + analog_4_20ma_Control_ptr.analog_4_20mA_Setting.analogPeriod_u16; //AnalogPeriod;   // Update time tick value
        if(analog_amps_low_alarm_enable_count_u16 > 0){
          analog_amps_low_alarm_enable_count_u16--;
        }      
        // Convert ADC1 value into 100.00%. analogVoltsPercent_u16 value is stored in xxxyy format = xxx.yy% 
        analog_4_20ma_Control_ptr.analog_4_20mA_Data.analogVoltsPercent_u16 = DECIMAL_RIGHT_SHIFT_U16(CALCULATE_PERCENT((*adc1_LocalAnalogAmpsControl_ptr).adc1_ResultAvg.adc1_4_20mA_Avg_u16, ADC_12_BIT_FULL_SCALE),2);
        analog_amps_counts_u16 = (*adc1_LocalAnalogAmpsControl_ptr).adc1_ResultAvg.adc1_4_20mA_Avg_u16; // Get result from ADC module into analog 4-20mA module
        analog_4_20ma_Control_ptr.analog_4_20mA_Data.analogErrorCode_u16 = (uint16_t) ((*adc1_LocalAnalogAmpsControl_ptr).adc1_ResultAvg.errorCode_u8); // Get any error codes
        DebounceAnalogAmps(); // Process the analog voltage
        //analog_4_20ma_Control_ptr.analog_4_20mA_Data analogVoltsToAmpsGain_f
        
        // Check if analog is low. If enabled and analog is low, demand is "0"
        if (((analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_analogLow == TRUE) && (analog_4_20ma_Control_ptr.analog_4_20mA_Setting.flags_u16.is_analogLowEnable == TRUE) && (analog_4_20ma_Control_ptr.analog_4_20mA_Setting.flags_u16.is_analogFailSafeEnable == FALSE))
          ||(analog_4_20ma_Control_ptr.analog_4_20mA_Setting.inputMode_u16!=ANALOG_MODE))
        {
          analog_4_20ma_Control_ptr.analog_4_20mA_Data.analogDemand_u16 = 0;
          analog_4_20ma_Control_ptr.analog_4_20mA_Data.analogDemandPercent_u16 = 0;
          analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_demandOn = FALSE;
          analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_lowerHysteresisEnable = FALSE;
          analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_upperHysteresisEnable = FALSE;
        } else { // Analog low is not detected or analog low is not enabled
          
          analog_4_20ma_Control_ptr.analog_4_20mA_Data.analogDemand_u16 = GetAnalogAmpsDemand(); // Convert analog voltage into demand 0 - 0xFFFF
          
          // Convert demand value into 0 to 100.00%. Value is stored in xxxyy format = xxx.yy% 
          analog_4_20ma_Control_ptr.analog_4_20mA_Data.analogDemandPercent_u16 = DECIMAL_RIGHT_SHIFT_U16(CALCULATE_PERCENT(analog_4_20ma_Control_ptr.analog_4_20mA_Data.analogDemand_u16, 65536),2);
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
* @retval  None
********************************************************************************************************************************
*/
void AssignModuleMemAnalog_4_20mA(void){   
  analog_4_20ma_StructMem_u32 =  StructMem_CreateInstance(MODULE_ANALOG_4_20MA, sizeof(Analog_4_20ma_Control), ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);
  (*analog_4_20ma_StructMem_u32).p_ramBuf_u8 = (uint8_t *)&analog_4_20ma_Control_ptr ;    // Map the ADC1 memory into the structured memory
  uint8_t Module_Analog_4_20mA_Index = getProcessInfoIndex(MODULE_ANALOG_4_20MA);
  processInfoTable[Module_Analog_4_20mA_Index].Sched_ModuleData.p_masterSharedMem_u32 = (Ram_Buf_Handle)analog_4_20ma_StructMem_u32;
}

/**
********************************************************************************************************************************
* @brief   Initilize all analog settings and live data
* @details Read settings from the RAM and initilize the settings
* @param   None
* @retval  None
********************************************************************************************************************************
*/
void Init_Analog_4_20mA_Setting(void){  
  analog_4_20ma_Control_ptr.analog_4_20mA_Setting.onVolts_u16 = 621;//205;             // Demand turn ON volts in counts of ADC
  analog_4_20ma_Control_ptr.analog_4_20mA_Setting.offVolts_u16 = 310;//164;            // Demand turn OFF volts in counts of ADC. Acts as Hysteresis
  analog_4_20ma_Control_ptr.analog_4_20mA_Setting.minVolts_u16 = 931;//500;            // Min volts correspodning to min demand
  analog_4_20ma_Control_ptr.analog_4_20mA_Setting.maxVolts_u16 = 2793;//3800;           // Max volts corresponding to max demand  
  analog_4_20ma_Control_ptr.analog_4_20mA_Setting.maxAdcCounts_u16 = 4095;       // Max output of ADC. 12 bit ADC left sifted by 4
  analog_4_20ma_Control_ptr.analog_4_20mA_Setting.debounceThresholdCounts_u16 = 10;//41;    // Change demand if the change in analog is outside this threshold. In ADC counts
  analog_4_20ma_Control_ptr.analog_4_20mA_Setting.calibratonFactor_s16 = 0;      // Add/Delete this form measured for calibration. In ADC counts
  analog_4_20ma_Control_ptr.analog_4_20mA_Setting.voltsHysteresis_u16 = 10;//41 ;     // Hys volts at the top end of demand. In ADC counts
  analog_4_20ma_Control_ptr.analog_4_20mA_Setting.minDemand_u16 = 21845;//13107;         // Min allowed demand. 20%
  analog_4_20ma_Control_ptr.analog_4_20mA_Setting.maxDemand_u16 = 0xFFFF;        // Maximum allowed demand
  analog_4_20ma_Control_ptr.analog_4_20mA_Setting.minDemandHysteresis_u16 = 21;//500; // Lower end hysteresis if Min_Demand > Off_Volts demand. Counts 
  analog_4_20ma_Control_ptr.analog_4_20mA_Setting.analogLowVolts_u16 = 125;//820;     // Volts below which the bAnalog_Loss_Enable is set. In ADC counts
  analog_4_20ma_Control_ptr.analog_4_20mA_Setting.failSafeDemand_u16  = 0x7FFF;  // If set to "1" use Fail safe demand when low analog input is detected //50%
  analog_4_20ma_Control_ptr.analog_4_20mA_Setting.analogPeriod_u16 = 50;               // Update rate in mSec for the analog voltage
  analog_4_20ma_Control_ptr.analog_4_20mA_Setting.analogLowAlarmEnableCount_u16 = 0;//100;  // analogPeriod*count delay before analog low volts alarm is detected in mSec. 
  analog_4_20ma_Control_ptr.analog_4_20mA_Setting.debounceLoopCount_u16 = 4;            // Debounce count
  analog_4_20ma_Control_ptr.analog_4_20mA_Setting.analogGain_f = 0.08056640625;            // Convert ADC coutns to volts 3.3/4096*100
  analog_4_20ma_Control_ptr.analog_4_20mA_Setting.analogVoltsToAmpsGain_f = 7.976470588;//1.994117647;//6.329114; // Convert analog volts to mAmps 1000*1/158ohms
  
  analog_4_20ma_Control_ptr.analog_4_20mA_Setting.flags_u16.is_enableAnalog = TRUE;          // If set to "0", 0V = Max demand 10V= 0 Demand
  analog_4_20ma_Control_ptr.analog_4_20mA_Setting.flags_u16.is_invertAnalog = FALSE;         // If set to "0", 0V = Max demand 10V= 0 Demand
  analog_4_20ma_Control_ptr.analog_4_20mA_Setting.flags_u16.is_analogLowEnable = TRUE;       // Enable analog low detect if set to "1"
  analog_4_20ma_Control_ptr.analog_4_20mA_Setting.flags_u16.is_analogFailSafeEnable = FALSE; // If set to "1", switch to fail safe demand when anlaog low is detected  
  analog_4_20ma_Control_ptr.analog_4_20mA_Setting.flags_u16.is_analogMinDemandEnable = FALSE;// If "True", min demand is maintaned when volts is below minVolts and above onVolts
  
  analog_4_20ma_Control_ptr.analog_4_20mA_Setting.inputMode_u16 = ANALOG_MODE;    // 0 = Analog input; 1 = digital input; 
  analog_4_20ma_Control_ptr.analog_4_20mA_Setting.digitalOnVolts_u16 = 1000;     // On volts above which for digitial input high
  analog_4_20ma_Control_ptr.analog_4_20mA_Setting.digtialOffVolts_u16 = 800;     // Off volts below which digital input is low
  
}

void InitAnalogAmpsInputParameters(void) // Init local parameters and metering data
{
  // Initilize live data                                                     // Flag indicates when the analog module is first initlized
  analog_4_20ma_Control_ptr.analog_4_20mA_Data.analogVoltsCounts_u16 = 0;        // Current Filtered Analog ADC Counts, 
  analog_4_20ma_Control_ptr.analog_4_20mA_Data.analogAmps_u16 = 0;              // Current Filtered Analog amps, 0.00V
  analog_4_20ma_Control_ptr.analog_4_20mA_Data.analogVolts_u16 = 0;             // Current Filtered Analog volts
  analog_4_20ma_Control_ptr.analog_4_20mA_Data.analogVoltsCountsStable_u16 = 0;  // Previous measured analog volts used for debounce check
  analog_4_20ma_Control_ptr.analog_4_20mA_Data.analogDemand_u16 = 0;             // Calculated analog demand
  analog_4_20ma_Control_ptr.analog_4_20mA_Data.analogVoltsPercent_u16 = 0;       // Analog Volts scalled 0 to 100%
  analog_4_20ma_Control_ptr.analog_4_20mA_Data.analogDemandPercent_u16 = 0;      // Analog Demand scalled 0 to 100%  
  analog_4_20ma_Control_ptr.analog_4_20mA_Data.increasingIntercept_f = 0;      // Intercept used to calculate demand when analog is increasing
  analog_4_20ma_Control_ptr.analog_4_20mA_Data.decreasingIntercept_f = 0;      // Intercept used to calcualte demand when analog is decreasing
  
  analog_amps_low_alarm_enable_count_u16 = analog_4_20ma_Control_ptr.analog_4_20mA_Setting.analogLowAlarmEnableCount_u16;
  uint16_t demand_range_u16 = 0;
  uint16_t analog_range_u16 = 0;
  
//  demand_range_u16 = analog_4_20ma_Control_ptr.analog_4_20mA_Setting.maxDemand_u16 - analog_4_20ma_Control_ptr.analog_4_20mA_Setting.minDemand_u16;
//  analog_range_u16 = (analog_4_20ma_Control_ptr.analog_4_20mA_Setting.maxVolts_u16 - analog_4_20ma_Control_ptr.analog_4_20mA_Setting.voltsHysteresis_u16) - (analog_4_20ma_Control_ptr.analog_4_20mA_Setting.minVolts_u16 - analog_4_20ma_Control_ptr.analog_4_20mA_Setting.voltsHysteresis_u16);                                                                 
//  analog_4_20ma_Control_ptr.analog_4_20mA_Data.decreasingSlope_f = (float)(demand_range_u16 / ((float)analog_range_u16));                                                                                                                                                                                                                           // Slope used to calculate demand when analog is decreasing
//  analog_4_20ma_Control_ptr.analog_4_20mA_Data.decreasingIntercept_f = (float)(analog_4_20ma_Control_ptr.analog_4_20mA_Setting.minDemand_u16 - (float)((analog_4_20ma_Control_ptr.analog_4_20mA_Data.decreasingSlope_f) * (analog_4_20ma_Control_ptr.analog_4_20mA_Setting.minVolts_u16 - analog_4_20ma_Control_ptr.analog_4_20mA_Setting.voltsHysteresis_u16))); // Intercept used to calcualte demand when analog is decreasing
//  
//  demand_range_u16 = analog_4_20ma_Control_ptr.analog_4_20mA_Setting.maxDemand_u16 - analog_4_20ma_Control_ptr.analog_4_20mA_Setting.minDemand_u16;
//  analog_range_u16 = analog_4_20ma_Control_ptr.analog_4_20mA_Setting.maxVolts_u16 - analog_4_20ma_Control_ptr.analog_4_20mA_Setting.minVolts_u16; 
//  analog_4_20ma_Control_ptr.analog_4_20mA_Data.increasingSlope_f = (float)(demand_range_u16 / ((float)analog_range_u16));                       // Slope used to calculate demand when analog is increasing
//  analog_4_20ma_Control_ptr.analog_4_20mA_Data.increasingIntercept_f = (float)(analog_4_20ma_Control_ptr.analog_4_20mA_Setting.minDemand_u16 - (float)((analog_4_20ma_Control_ptr.analog_4_20mA_Data.increasingSlope_f) * (analog_4_20ma_Control_ptr.analog_4_20mA_Setting.minVolts_u16))); // Interecept used to calculate demand when analog is increasing
  
  demand_range_u16 = analog_4_20ma_Control_ptr.analog_4_20mA_Setting.maxDemand_u16 - analog_4_20ma_Control_ptr.analog_4_20mA_Setting.minDemand_u16;
  analog_range_u16 = analog_4_20ma_Control_ptr.analog_4_20mA_Setting.maxVolts_u16 - analog_4_20ma_Control_ptr.analog_4_20mA_Setting.voltsHysteresis_u16; 
  analog_4_20ma_Control_ptr.analog_4_20mA_Data.increasingSlope_f = (float)(demand_range_u16 / ((float)analog_range_u16)); // Slope used to calculate demand when analog is increasing
  
  if(analog_4_20ma_Control_ptr.analog_4_20mA_Setting.flags_u16.is_invertAnalog == TRUE)
  {
    analog_4_20ma_Control_ptr.analog_4_20mA_Data.increasingSlope_f = analog_4_20ma_Control_ptr.analog_4_20mA_Data.increasingSlope_f * -1;
    analog_4_20ma_Control_ptr.analog_4_20mA_Data.increasingIntercept_f = (float)(analog_4_20ma_Control_ptr.analog_4_20mA_Setting.minDemand_u16 - (float)((analog_4_20ma_Control_ptr.analog_4_20mA_Data.increasingSlope_f) * (analog_4_20ma_Control_ptr.analog_4_20mA_Setting.maxVolts_u16))); // Interecept used to calculate demand when analog is increasing
  }else
  {
    analog_4_20ma_Control_ptr.analog_4_20mA_Data.increasingIntercept_f = (float)(analog_4_20ma_Control_ptr.analog_4_20mA_Setting.minDemand_u16 - (float)((analog_4_20ma_Control_ptr.analog_4_20mA_Data.increasingSlope_f) * (analog_4_20ma_Control_ptr.analog_4_20mA_Setting.minVolts_u16))); // Interecept used to calculate demand when analog is increasing
  }
  analog_4_20ma_Control_ptr.analog_4_20mA_Data.decreasingSlope_f = analog_4_20ma_Control_ptr.analog_4_20mA_Data.increasingSlope_f;
  analog_4_20ma_Control_ptr.analog_4_20mA_Data.decreasingIntercept_f = analog_4_20ma_Control_ptr.analog_4_20mA_Data.increasingIntercept_f; 
  
  
  // Assumes 0 V = 0 demand; Max volts = Max demand  
  analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_analogLow = FALSE;             // Set to "TRUE", if analog low is detected
  analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_decreasingAnalog = FALSE;       // Set to "TRUE", if analog voltage is decresing
  analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_lowerHysteresisEnable = FALSE;  // Set to "TRUE", if lower end hys is enabled
  analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_upperHysteresisEnable = FALSE;  // Set to "TRUE", if upper end hys is enabled 
  analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_demandOn = FALSE;               // Set to"TRUE", if demand goes from 0 to above min_Demand_u16.
  
}
/**
********************************************************************************************************************************
* @brief   Check if Analog is increasing or decresing
* @details Check for debounce. Only update Analog if change is greater than debounce
debounce. Check for Low Analog Volts. 
* @retval  None
********************************************************************************************************************************
*/
void DebounceAnalogAmps(void){
  uint16_t abs_volts_diff_u16=0;
  abs_volts_diff_u16 = abs(analog_4_20ma_Control_ptr.analog_4_20mA_Data.analogVoltsCountsStable_u16  - analog_amps_counts_u16);
  
  // Check debounce threshold
  if(abs_volts_diff_u16 > analog_4_20ma_Control_ptr.analog_4_20mA_Setting.debounceThresholdCounts_u16)
  {  // Change is analog is above debounce
    if (analog_4_20ma_Control_ptr.analog_4_20mA_Data.analogVoltsCountsStable_u16  >  analog_amps_counts_u16)
    { // Analog is decreasing
      analog_amps_decreasing_debounce_count_u16++;
      analog_amps_increasing_debounce_count_u16 = 0;
    } else if (analog_4_20ma_Control_ptr.analog_4_20mA_Data.analogVoltsCountsStable_u16  <  analog_amps_counts_u16)
    { // Analog is increasing
      analog_amps_increasing_debounce_count_u16++;
      analog_amps_decreasing_debounce_count_u16 = 0;
    } 
    
    // Check debounce loop count
    if ( (analog_amps_increasing_debounce_count_u16 > analog_4_20ma_Control_ptr.analog_4_20mA_Setting.debounceLoopCount_u16) || (analog_amps_decreasing_debounce_count_u16 > analog_4_20ma_Control_ptr.analog_4_20mA_Setting.debounceLoopCount_u16)  )
    {
      analog_4_20ma_Control_ptr.analog_4_20mA_Data.analogVoltsCountsStable_u16 = analog_4_20ma_Control_ptr.analog_4_20mA_Data.analogVoltsCounts_u16  + analog_4_20ma_Control_ptr.analog_4_20mA_Setting.calibratonFactor_s16 ; // Update only when debounce count is satisfied and calibrate
      analog_4_20ma_Control_ptr.analog_4_20mA_Data.analogVolts_u16 = (uint16_t)(analog_4_20ma_Control_ptr.analog_4_20mA_Setting.analogGain_f * analog_4_20ma_Control_ptr.analog_4_20mA_Data.analogVoltsCountsStable_u16); // No need to rightshift since analogGain_f is adjusted by a factor of 100 // Convert ADC count to volts 
      analog_4_20ma_Control_ptr.analog_4_20mA_Data.analogAmps_u16 = (uint16_t)(analog_4_20ma_Control_ptr.analog_4_20mA_Setting.analogVoltsToAmpsGain_f * analog_4_20ma_Control_ptr.analog_4_20mA_Data.analogVolts_u16); // No need to rightshift since analogVolts_u16 is already right shifted // Convert volts to amps 
      analog_4_20ma_Control_ptr.analog_4_20mA_Data.analogVoltsCounts_u16 = analog_amps_counts_u16; // Update only when debounce count is satisfied  
      
      // Determine if volts are increasing or decreasing
      if(analog_amps_decreasing_debounce_count_u16 > analog_4_20ma_Control_ptr.analog_4_20mA_Setting.debounceLoopCount_u16) 
      {
        analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_decreasingAnalog = TRUE;
      } else
      {
        analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_decreasingAnalog = FALSE;
      }
      analog_amps_decreasing_debounce_count_u16 = 0;
      analog_amps_increasing_debounce_count_u16 = 0;
    }
    
    // Set Digital input flag
    if(analog_4_20ma_Control_ptr.analog_4_20mA_Data.analogVoltsCountsStable_u16 < analog_4_20ma_Control_ptr.analog_4_20mA_Setting.digtialOffVolts_u16)
    {
      analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_digitalInputON = FALSE;
    } else{
      analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_digitalInputON = TRUE;
    }
    
    // Delay low analog detection. Need when power up
    if(analog_amps_low_alarm_enable_count_u16 == 0) 
    { // Check if analog volts is low
      if (analog_4_20ma_Control_ptr.analog_4_20mA_Data.analogVoltsCounts_u16 < analog_4_20ma_Control_ptr.analog_4_20mA_Setting.analogLowVolts_u16)
      {
        analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_analogLow = TRUE;
      } else {
        analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_analogLow = FALSE;
      }
    } else{
      //is_amps_above_loss_analog = TRUE;
      analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_analogLow = FALSE;
    }
    
  } else
  { // debounce threshold not satisified
    analog_amps_increasing_debounce_count_u16 = 0;
    analog_amps_decreasing_debounce_count_u16 = 0;
  }
}

/**
********************************************************************************************************************************
* @brief   Convert Analog volts to Demand ( min_Demand_u16 to max_Demand_u16)
* @details Check for hysteresis, turn ON/OFF and calculate demand 
* @retval  None
********************************************************************************************************************************
*/
uint16_t GetAnalogAmpsDemand(void){
  uint32_t analog_demand_u32 = 0;
  float slope_f;
  float intercept_f;
  uint16_t analog_volts_u16 = 0; 
  
  analog_volts_u16 = analog_4_20ma_Control_ptr.analog_4_20mA_Data.analogVoltsCountsStable_u16; // Used in case analog demand is inverted
  // Check if input mode is enabled (ANALOG_MODE)
  if (analog_4_20ma_Control_ptr.analog_4_20mA_Setting.inputMode_u16 != ANALOG_MODE)
  {
    analog_demand_u32 = 0;
    analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_demandOn = FALSE;
  } else if( (analog_4_20ma_Control_ptr.analog_4_20mA_Setting.flags_u16.is_analogLowEnable == TRUE) && (analog_4_20ma_Control_ptr.analog_4_20mA_Setting.flags_u16.is_analogFailSafeEnable == TRUE ) && (analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_analogLow == TRUE)) { 
    // Check if Loss of analog
    if((analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_demandOn == FALSE))// && (is_amps_above_loss_analog == FALSE))//demand is never gone above min demand. 
    {  // Analog never crossed turned ON volts
      analog_demand_u32 = 0;
      //analog_4_20ma_Control_ptr.analog_4_20mA_Data.is_demandOn = FALSE;
    } else{// if (is_amps_above_loss_analog == TRUE){
      // Use fail safe demand if low analog volts is detected
      analog_demand_u32 = analog_4_20ma_Control_ptr.analog_4_20mA_Setting.failSafeDemand_u16;
      analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_demandOn = TRUE;
      analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_upperHysteresisEnable = FALSE;
    }
  } else {
    // If invert is enabled invert measured value
    //if (analog_4_20ma_Control_ptr.analog_4_20mA_Setting.flags_u16.is_invertAnalog == TRUE) 
    //{
      //analog_volts_u16 = analog_4_20ma_Control_ptr.analog_4_20mA_Setting.maxAdcCounts_u16 - analog_4_20ma_Control_ptr.analog_4_20mA_Data.analogVoltsCountsStable_u16;
    //}
    
    if (analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_decreasingAnalog == FALSE)
    { // Slope used when analog value is increasing
      slope_f = analog_4_20ma_Control_ptr.analog_4_20mA_Data.increasingSlope_f;
      intercept_f = analog_4_20ma_Control_ptr.analog_4_20mA_Data.increasingIntercept_f;
    } else {
      // Slope used when analog value is decreasing
      slope_f = analog_4_20ma_Control_ptr.analog_4_20mA_Data.decreasingSlope_f;
      intercept_f = analog_4_20ma_Control_ptr.analog_4_20mA_Data.decreasingIntercept_f;
    }
    
    // Analog volts is divided up into three parts
    // Part 1: 0v to off volts
    // Part 2: off volts to upper hysteresis volts
    // Part 3: Upper hysteresis volts to max volts
    
    // Part 1, 2 : Analog input between OFF volts and upper hysteresis
    if ((analog_volts_u16 >= analog_4_20ma_Control_ptr.analog_4_20mA_Setting.offVolts_u16) && (analog_volts_u16 < (analog_4_20ma_Control_ptr.analog_4_20mA_Setting.maxVolts_u16 - analog_4_20ma_Control_ptr.analog_4_20mA_Setting.voltsHysteresis_u16)))
    {
      // If analog is between ON and OFF volts. Hysteresis part
      analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_upperHysteresisEnable = FALSE;
      
      // Part 2
      // onVolts till upperHysteresis
      if (analog_volts_u16 >= analog_4_20ma_Control_ptr.analog_4_20mA_Setting.onVolts_u16) {
        if (analog_volts_u16 >= analog_4_20ma_Control_ptr.analog_4_20mA_Setting.minVolts_u16)
        {          
          // Analog voltage is above ON volts for first time
          analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_lowerHysteresisEnable = TRUE;
          analog_demand_u32 = (uint32_t)((float)(slope_f * analog_volts_u16) + intercept_f);
        } else if (analog_volts_u16 < analog_4_20ma_Control_ptr.analog_4_20mA_Setting.minVolts_u16){
          // Analog voltage below minVolts limit
          if((analog_4_20ma_Control_ptr.analog_4_20mA_Setting.flags_u16.is_analogMinDemandEnable) == TRUE ){//&& ( analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_lowerHysteresisEnable == TRUE)){
            // Default to min demand when is_analogMinDemandEn is TRUE
            //analog_demand_u32 = analog_4_20ma_Control_ptr.analog_4_20mA_Setting.minDemand_u16;
            if(analog_4_20ma_Control_ptr.analog_4_20mA_Setting.flags_u16.is_invertAnalog == FALSE)
            {
              analog_demand_u32 = analog_4_20ma_Control_ptr.analog_4_20mA_Setting.minDemand_u16;
            } else
            {
              analog_demand_u32 = analog_4_20ma_Control_ptr.analog_4_20mA_Setting.maxDemand_u16;
            }
          } else if( (analog_volts_u16 >= (analog_4_20ma_Control_ptr.analog_4_20mA_Setting.minVolts_u16 - analog_4_20ma_Control_ptr.analog_4_20mA_Setting.minDemandHysteresis_u16)) && ( analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_lowerHysteresisEnable == TRUE)){
            // Apply lower end hysteresis
            analog_demand_u32 = (uint32_t)((float)(slope_f * analog_volts_u16) + intercept_f);
          } else{
            if ((analog_4_20ma_Control_ptr.analog_4_20mA_Setting.flags_u16.is_invertAnalog == TRUE) && (analog_4_20ma_Control_ptr.analog_4_20mA_Setting.flags_u16.is_analogMinDemandEnable == TRUE))
            {
              analog_demand_u32 = analog_4_20ma_Control_ptr.analog_4_20mA_Setting.minDemand_u16;
            } else{
              // Analog volts below hysteresis value
              analog_demand_u32 = 0;
              analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_lowerHysteresisEnable = FALSE;
              analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_demandOn = FALSE;  
            }
          }          
        }
      }else if (analog_volts_u16 >= analog_4_20ma_Control_ptr.analog_4_20mA_Setting.offVolts_u16) {
        // Part 1
        // upper hysteresisdown to off volatge
        
        // Demand is minimum until analog reaches below off volatge
        if((analog_4_20ma_Control_ptr.analog_4_20mA_Setting.flags_u16.is_analogMinDemandEnable) == TRUE && ( analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_lowerHysteresisEnable == TRUE)){
            // Default to min demand when is_analogMinDemandEn is TRUE
            //analog_demand_u32 = analogVolts_Control.analogVolts_Setting.minDemand_u16;
            if(analog_4_20ma_Control_ptr.analog_4_20mA_Setting.flags_u16.is_invertAnalog == FALSE)
            {
              analog_demand_u32 = analog_4_20ma_Control_ptr.analog_4_20mA_Setting.minDemand_u16;
            } else
            {
              analog_demand_u32 = analog_4_20ma_Control_ptr.analog_4_20mA_Setting.maxDemand_u16;
            }
        }else{
          analog_demand_u32 = 0;
          analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_demandOn = FALSE;
        }
      } else if (analog_volts_u16 < analog_4_20ma_Control_ptr.analog_4_20mA_Setting.onVolts_u16) {
        // Part 1
        // 0V till onVolts
        
        // Demand is zero when analog never crossed ON volts
        analog_demand_u32 = 0;
        analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_demandOn = FALSE;
      } else {
        analog_demand_u32 = 0;
        analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_demandOn = FALSE;
      }
    } else if ((analog_volts_u16 >= (analog_4_20ma_Control_ptr.analog_4_20mA_Setting.maxVolts_u16 - analog_4_20ma_Control_ptr.analog_4_20mA_Setting.voltsHysteresis_u16)) && (analog_volts_u16 < analog_4_20ma_Control_ptr.analog_4_20mA_Setting.maxVolts_u16) ){
      // Part 3: 
      // upperHysteresis till maxVolts
      
      // Analog voltage above upper hysteresis voltage      
      if (analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_upperHysteresisEnable  == TRUE)
      { // Analog already crossed hysteresis point
        if (analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_decreasingAnalog == TRUE)
        { // Output demand = max demand till below hysteresis value          
		  if(analog_4_20ma_Control_ptr.analog_4_20mA_Setting.flags_u16.is_invertAnalog == FALSE)
          {
            analog_demand_u32 = analog_4_20ma_Control_ptr.analog_4_20mA_Setting.maxDemand_u16;
          } else
          {
            analog_demand_u32 = analog_4_20ma_Control_ptr.analog_4_20mA_Setting.minDemand_u16;
          }
        } else {
          // Analog volts never when above hysteresis voltage
          analog_demand_u32 = (uint32_t)((float)(slope_f * analog_volts_u16) + intercept_f);
        }
      } else if (analog_volts_u16 <= analog_4_20ma_Control_ptr.analog_4_20mA_Setting.maxVolts_u16) {
        // Analog went above Upper Hysteresis value for the first time
        analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_upperHysteresisEnable  = TRUE;
        analog_demand_u32 = (uint32_t)((float)(slope_f * analog_volts_u16) + intercept_f);
      }
    } else if (analog_volts_u16 > analog_4_20ma_Control_ptr.analog_4_20mA_Setting.maxVolts_u16) {
	if(analog_4_20ma_Control_ptr.analog_4_20mA_Setting.flags_u16.is_invertAnalog == FALSE)
      {
        // Saturate demand value
        analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_upperHysteresisEnable  = TRUE;
        analog_demand_u32 = analog_4_20ma_Control_ptr.analog_4_20mA_Setting.maxDemand_u16;
      } else
      {
        // Saturate demand value for inverted input
        //if( analog_volts_u16 > (analog_4_20ma_Control_ptr.analog_4_20mA_Setting.maxVolts_u16) )
       // analogVolts_Control.analogVolts_Data.discretes_u16.is_upperHysteresisEnable  = FALSE;
        //analog_demand_u32 = 0;
        analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_upperHysteresisEnable  = TRUE;
        analog_demand_u32 = analog_4_20ma_Control_ptr.analog_4_20mA_Setting.minDemand_u16;
      }      
    }else{// lessthan OFF volatge
       analog_demand_u32 = 0;
       analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_demandOn = FALSE;
    }
    
    
    // Min demand is min_Demand_u16.
    if (analog_demand_u32 < analog_4_20ma_Control_ptr.analog_4_20mA_Setting.minDemand_u16)
    {
      // Check if low end demand hysteresis
      if (analog_demand_u32 < (analog_4_20ma_Control_ptr.analog_4_20mA_Setting.minDemand_u16 - analog_4_20ma_Control_ptr.analog_4_20mA_Setting.minDemandHysteresis_u16))
      {
        analog_demand_u32 = 0;
        analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_demandOn = FALSE;
        analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_lowerHysteresisEnable = FALSE;
      }
    } else {
      analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_demandOn = TRUE; // Demand greater than min demand.  
      analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16.is_lowerHysteresisEnable = TRUE; // Enable Hysteresis
    }
    
    // Demand can't exceed max_Demand_u16
    if (analog_demand_u32 > analog_4_20ma_Control_ptr.analog_4_20mA_Setting.maxDemand_u16)
    {
      analog_demand_u32 = (uint16_t)analog_4_20ma_Control_ptr.analog_4_20mA_Setting.maxDemand_u16;
    }      
  }
  analog_4_20ma_Control_ptr.analog_4_20mA_Data.analogDemand_u16 = analog_demand_u32;
  return ((uint16_t)analog_demand_u32);  
}