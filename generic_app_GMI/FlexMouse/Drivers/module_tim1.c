/**
*************************************************************************************
* @file    module_tim1.c 
* @author  Satya Akkina
* @version V1.0
* @date    Jan 11 2021
* @brief   Module for Tim1
* @note    
*****************************************************************************
*/

#include "module_tim1.h"

// Content ---------------------------------------------------------------------
// Function prototypes
void assign_Tim1DrvMem(void);

// Module States
enum AppStates {
  MEMORY_INIT_MODULE,
  INIT_MODULE,
  RUN_MODULE,
  // additional states to be added here as necessary.
  IRQ_MODULE = DEFAULT_IRQ_STATE,
  KILL_MODULE = KILL_APP
};

// External Variables
extern TIM1_Control tim1_Control;
extern ProcessInfo processInfoTable[];

// Global variables specific to this module
static Ram_Buf_Handle tim1_Control_StructMem_u32;

/**
*******************************************************************************
* @brief   State machine for Digital Tim1 Module
* @details
* @param   drv_id_u8, prev_state_u8, next_state_u8, irq_id_u8
* @retval  return_state_u8
*******************************************************************************
*/
uint8_t moduleTim1_u32(uint8_t drv_id_u8, uint8_t prev_state_u8, uint8_t next_state_u8,
                       uint8_t irq_id_u8)
{
  uint8_t return_state_u8 = MEMORY_INIT_MODULE;
  switch (next_state_u8)
  {
  case MEMORY_INIT_MODULE:
    {
      assign_Tim1DrvMem(); // Assign structured memory
      return_state_u8 = INIT_MODULE;
      break;
    }
  case INIT_MODULE:                                                             
    {
      //assign_Tim1DrvMem(); //Assign RAM memory for ADC structures
      
      return_state_u8 = RUN_MODULE;
      break;      
    }   
  case RUN_MODULE:                                                             
    {
      //This Deamon need only execute once than kill this process, in case want to run it again programmer can enable this process again by 
      //set the processStatus parameter to 0x00 and set the nextStage parameter to stage 0 or any stage 
      return_state_u8 = KILL_MODULE; 
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
* @details Assign structured memory for Tim1_Control
* @retval  None
********************************************************************************************************************************
*/
void assign_Tim1DrvMem(){  
  tim1_Control_StructMem_u32 =  StructMem_CreateInstance(MODULE_TIM1, sizeof(TIM1_Control), ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);
  (*tim1_Control_StructMem_u32).p_ramBuf_u8 = (uint8_t *)&tim1_Control ;    //map the ADC1 memory into the structured memory
  uint8_t Drv_Tim1Index = getProcessInfoIndex(MODULE_TIM1);
  processInfoTable[Drv_Tim1Index].Sched_DrvData.p_masterSharedMem_u32 = tim1_Control_StructMem_u32;  
  
  tim1_Control.tim1_ResultAvg.PWMInputDutyCycleAverageCalc_f = 0;
  tim1_Control.tim1_ResultAvg.PWMInputFrequencyAverageCalc_u16 = 0;
}




