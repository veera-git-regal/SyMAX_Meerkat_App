/**
********************************************************************************************************************************
* @file    module_app.c 
* @author  Pamela Lee
* @brief   This is a template non-driver app.
* @details This app does nothing.
********************************************************************************************************************************
*/

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "module_app.h"

//#include "driver_gpio.h"

/* Content ---------------------------------------------------------------------------------------------------------------------*/
extern ProcessInfo processInfoTable[];
void AssignModuleMemAppModule(void);

// Global variables specific to this module
uint64_t module_app_poll_time_u64 =0; // Poll time for the module

enum {
  MEMORY_INIT_MODULE,
  INIT_MODULE,
  RUN_MODULE,
  // Ddditional states to be added here as necessary.
  IRQ_MODULE = DEFAULT_IRQ_STATE,
  KILL_MODULE = KILL_APP
};

/**
********************************************************************************************************************************
* @brief   State machine for app module
* @details
* @param   drv_identifier_u8, previous_state_u8, next_stat_u8, irq_identfier_u8
* @retval  return_state_u8
********************************************************************************************************************************
*/
uint8_t moduleApp_u32(uint8_t drv_identifier_u8, uint8_t previous_state_u8, uint8_t next_state_u8,
                      uint8_t irq_identifier_u8)
{
  uint8_t return_state_u8 = MEMORY_INIT_MODULE;
  switch (next_state_u8)
  {
  case MEMORY_INIT_MODULE:
    {
      AssignModuleMemAppModule(); // Assign structured memory to Analog 0-10V setting and data 
      return_state_u8 = INIT_MODULE;
      break;
    }
  case INIT_MODULE:
    {     
      // Assign structured memory for module
      //AssignModuleMemAppModule();
      
      module_app_poll_time_u64 = getSysCount() + POLL_TIME; // Store time tick value     
      
      // Get structured memory for GPIO module data
      //uint8_t module_drv_Gpio_Index = getProcessInfoIndex(MODULE_GPIO);
      //gpio_LocalControl = (Gpio_Control*)((*(processInfoTable[module_drv_Gpio_Index].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8);
      
      return_state_u8 = RUN_MODULE;
      break;
      
    }   
  case RUN_MODULE:                                                             
    {
      // Process analog input every "AnalogPeriod" mSec
      if (getSysCount() >= module_app_poll_time_u64) // Time above poll time
      {
        module_app_poll_time_u64 = getSysCount() + POLL_TIME; // Next poll time
        // User Code        
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
* @details Assign structured memory for App module
* @param   None 
* @return  None
********************************************************************************************************************************
*/
void AssignModuleMemModuleApp(void){   
  //gpio_StructMem_u32 =  StructMem_CreateInstance(MODULE_GPIO, sizeof(Gpio_Control), ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);
  //(*gpio_StructMem_u32).p_ramBuf_u8 = (uint8_t *)&gpio_Control ;    // Map the ADC1 memory into the structured memory
  //uint8_t module_gpio_index_u8 = getProcessInfoIndex(MODULE_GPIO);
  //processInfoTable[module_gpio_index_u8].Sched_ModuleData.p_masterSharedMem_u32 = (Ram_Buf_Handle)gpio_StructMem_u32;
}
