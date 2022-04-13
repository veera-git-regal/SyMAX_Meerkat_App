/**
*************************************************************************************
* @file    module_adc1.c
* @author  Satya Akkina
* @version V1.0
* @date    18-Jun-2020
* @brief   Module for ADC1 hardware
* @note    
*************************************************************************************
*/
#include "module_adc1.h"

/* Content ---------------------------------------------------------------------------------------------------------------------*/
// Function prototypes
void AssignModuleMemAdc1(void);

// -- Module States
enum AppStates {
  MEMORY_INIT_MODULE,
  INIT_MODULE,
  RUN_MODULE,
  // additional states to be added here as necessary.
  IRQ_MODULE = DEFAULT_IRQ_STATE,
  KILL_MODULE = KILL_APP
};

// - External Variables
extern ProcessInfo processInfoTable[];
extern ADC1_Control adc1_Control; 

// - Global variables specific to this module
static Ram_Buf_Handle adc1_Control_StructMem_u32;

/**
********************************************************************************************************************************
* @brief   State machine for adc1 module
* @details
* @param   drv_identifier_u8, previous_state_u8, next_stat_u8, irq_identfier_u8
* @retval  return_state_u8
********************************************************************************************************************************
*/
uint8_t moduleADC1_u32(uint8_t drv_id_u8, uint8_t prev_state_u8, uint8_t next_state_u8,
                       uint8_t irq_id_u8)
{
  uint8_t return_state_u8 = MEMORY_INIT_MODULE;
  switch (next_state_u8)
  {
  case MEMORY_INIT_MODULE:
    {
      AssignModuleMemAdc1(); // Assign structured memory
      return_state_u8 = INIT_MODULE;
      break;
    }
  case INIT_MODULE:
    {
      //AssignModuleMemAdc1(); //Assign RAM memory for ADC structures      
      adc1_Init();
      configDma();
      
      // Activate ADC 
      // Perform ADC activation procedure to make it ready to convert. 
      activate_ADC();
      start_ADC1_Conversion();
      
      return_state_u8 = RUN_MODULE;
      break;
      
    }   
  case RUN_MODULE:                                                             
    {
      //This module need only execute once then kill this process, in case want to run it again programmer can enable this process again by 
      //set the processStatus parameter to 0x00 and set the nextStage parameter to stage 0 or any stage 
      return_state_u8 = KILL_MODULE;
      break;
    }
  case KILL_MODULE: 
    {
      // The adc1 module must only be executed once.
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
* @details Assign structured memory for adc1_Control and init strcture variables
* @param   None
* @retval  None
********************************************************************************************************************************
*/
void AssignModuleMemAdc1(){  
  adc1_Control_StructMem_u32 =  StructMem_CreateInstance(MODULE_ADC1, sizeof(ADC1_Control), ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);
  (*adc1_Control_StructMem_u32).p_ramBuf_u8 = (uint8_t *)&adc1_Control ;    //map the ADC1 memory into the structured memory
  uint8_t Drv_ADC1Index = getProcessInfoIndex(MODULE_ADC1);
  processInfoTable[Drv_ADC1Index].Sched_DrvData.p_masterSharedMem_u32 = adc1_Control_StructMem_u32;  
  
  adc1_Control.adc1_Result.adc1_0_10V_Result_u16 = 0;   //clear the ADC1 result
  adc1_Control.adc1_Result.adc1_4_20mA_Result_u16 = 0;  //clear the ADC1 result
  adc1_Control.adc1_Result.adc1_Temp_Result_u16 = 0;    //clear the ADC1 result
  
  adc1_Control.adc1_ResultAvg.adc1_0_10V_Avg_u16 = 0;   //clear the ADC1 Avg result
  adc1_Control.adc1_ResultAvg.adc1_4_20mA_Avg_u16 = 0;  //clear the ADC1 Avg result
  adc1_Control.adc1_ResultAvg.adc1_Temp_Avg_u16 = 0;    //clear the ADC1 Avg result
  adc1_Control.adc1_ResultAvg.errorCode_u8 = 0;
}