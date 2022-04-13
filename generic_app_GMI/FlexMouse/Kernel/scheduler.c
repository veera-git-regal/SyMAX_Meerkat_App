/**
  ********************************************************************************************************************************
  * @file    scheduler.c
  * @author  Pamela Lee
  * @brief   Implementation of c++ function/s for the kernel scheduler. 
  * @details Modules and drivers are collectively known as processes. This file contains the implementation of function related to
  *             scheduling processes and handling their respective driver interrupts, if any. The function scheduler_run is the
  *             'main loop' of the FlexMouse architecture.
  ********************************************************************************************************************************
  */

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "scheduler.h"

#include "main.h"
//#include "stm32f3xx_ll_iwdg.h"
//volatile uint64_t tickCounter = 0;
__root const uint32_t App_CRC @ "app_crc32_rom" = 0x00000000; // This is a placeholder for App Firmware Checksum (CRC32)

/*Software interrupt variable in bit oriented
  * SoftwareIrqBitPt[0] of binary IRQ process (from 0 to 63), 
  * SoftwareIrqBitPt[1] of binary IRQ process (from 64 to 127)
  * SoftwareIrqBitPt[2] of binary IRQ process (from 128 to 191)
  * SoftwareIrqBitPt[3] of binary IRQ process (from 192 to 254)
*/

static uint64_t SoftwareIrqBitPt[] = { 0,0,0,0 };        
static uint8_t  IrqTrigProcessID = 255;        //current module/process (ID) trigged IRQ 


extern volatile uint64_t tickCounter; //SPA
// REVIEW: Placement

/* Content ---------------------------------------------------------------------------------------------------------------------*/
ProcessInfo processInfoTable[TOTAL_NUM_OF_PROCESSES] = {
    // Application Modules
    // - Driver Extensions
    {MODULE_FLASH_ID, MODULE_FLASH_FUNCTION_POINTER, MODULE_FLASH_TOTAL_SEQ, MODULE_FLASH_TOTAL_STRUCT, MODULE_FLASH_PREV_STATE,
     MODULE_FLASH_NEXT_STATE, MODULE_FLASH_IRQ_STATUS, MODULE_FLASH_PROCESS_STATUS, MODULE_FLASH_MASTER_SHARED_MEM},
     
    {MODULE_GPIO_ID, MODULE_GPIO_FUNCTION_POINTER, MODULE_GPIO_TOTAL_SEQ, MODULE_GPIO_TOTAL_STRUCT, MODULE_GPIO_PREV_STATE,
     MODULE_GPIO_NEXT_STATE, MODULE_GPIO_IRQ_STATUS, MODULE_GPIO_PROCESS_STATUS, MODULE_GPIO_MASTER_SHARED_MEM},
     
    {MODULE_TIM1_ID, MODULE_TIM1_FUNCTION_POINTER, MODULE_TIM1_TOTAL_SEQ, MODULE_TIM1_TOTAL_STRUCT, MODULE_TIM1_PREV_STATE,
     MODULE_TIM1_NEXT_STATE, MODULE_TIM1_IRQ_STATUS, MODULE_TIM1_PROCESS_STATUS, MODULE_TIM1_MASTER_SHARED_MEM},
     
    {MODULE_USART2_ID, MODULE_USART2_FUNCTION_POINTER, MODULE_USART2_TOTAL_SEQ, MODULE_USART2_TOTAL_STRUCT, MODULE_USART2_PREV_STATE,
     MODULE_USART2_NEXT_STATE, MODULE_USART2_IRQ_STATUS, MODULE_USART2_PROCESS_STATUS, MODULE_USART2_MASTER_SHARED_MEM},
     
    {MODULE_ADC1_ID, MODULE_ADC1_FUNCTION_POINTER, MODULE_ADC1_TOTAL_SEQ, MODULE_ADC1_TOTAL_STRUCT, MODULE_ADC1_PREV_STATE,
     MODULE_ADC1_NEXT_STATE, MODULE_ADC1_IRQ_STATUS, MODULE_ADC1_PROCESS_STATUS, MODULE_ADC1_MASTER_SHARED_MEM},
    // - Input Sources
    {MODULE_ANALOG_0_10V_ID, MODULE_ANALOG_0_10V_FUNCTION_POINTER, MODULE_ANALOG_0_10V_TOTAL_SEQ, MODULE_ANALOG_0_10V_TOTAL_STRUCT, MODULE_ANALOG_0_10V_PREV_STATE,
     MODULE_ANALOG_0_10V_NEXT_STATE, MODULE_ANALOG_0_10V_IRQ_STATUS, MODULE_ANALOG_0_10V_PROCESS_STATUS, MODULE_ANALOG_0_10V_MASTER_SHARED_MEM},
     // 4-20mA
    {MODULE_ANALOG_4_20MA_ID, MODULE_ANALOG_4_20MA_FUNCTION_POINTER, MODULE_ANALOG_4_20MA_TOTAL_SEQ, MODULE_ANALOG_4_20MA_TOTAL_STRUCT, MODULE_ANALOG_4_20MA_PREV_STATE,
     MODULE_ANALOG_4_20MA_NEXT_STATE, MODULE_ANALOG_4_20MA_IRQ_STATUS, MODULE_ANALOG_4_20MA_PROCESS_STATUS, MODULE_ANALOG_4_20MA_MASTER_SHARED_MEM},
    // Digital Inputs 
    {MODULE_DIGITAL_INPUTS_ID, MODULE_DIGITAL_INPUTS_FUNCTION_POINTER, MODULE_DIGITAL_INPUTS_TOTAL_SEQ, MODULE_DIGITAL_INPUTS_TOTAL_STRUCT, MODULE_DIGITAL_INPUTS_PREV_STATE,     
     MODULE_DIGITAL_INPUTS_NEXT_STATE, MODULE_DIGITAL_INPUTS_IRQ_STATUS, MODULE_DIGITAL_INPUTS_PROCESS_STATUS, MODULE_DIGITAL_INPUTS_MASTER_SHARED_MEM},
    // PWM Input
    {MODULE_PWM_INPUT_ID, MODULE_PWM_INPUT_FUNCTION_POINTER, MODULE_PWM_INPUT_TOTAL_SEQ, MODULE_PWM_INPUT_TOTAL_STRUCT, MODULE_PWM_INPUT_PREV_STATE,     
     MODULE_PWM_INPUT_NEXT_STATE, MODULE_PWM_INPUT_IRQ_STATUS, MODULE_PWM_INPUT_PROCESS_STATUS, MODULE_PWM_INPUT_MASTER_SHARED_MEM},
        
    // - Modbus
    {MODULE_USART1_ID, MODULE_USART1_FUNCTION_POINTER, MODULE_USART1_TOTAL_SEQ, MODULE_USART1_TOTAL_STRUCT, MODULE_USART1_PREV_STATE,
    MODULE_USART1_NEXT_STATE, MODULE_USART1_IRQ_STATUS, MODULE_USART1_PROCESS_STATUS, MODULE_USART1_MASTER_SHARED_MEM},

    // SPI
    {MODULE_SPI1_ID, MODULE_SPI1_FUNCTION_POINTER, MODULE_SPI1_TOTAL_SEQ, MODULE_SPI1_TOTAL_STRUCT, MODULE_SPI1_PREV_STATE,
     MODULE_SPI1_NEXT_STATE, MODULE_SPI1_IRQ_STATUS, MODULE_SPI1_PROCESS_STATUS, MODULE_SPI1_MASTER_SHARED_MEM},
    
    {MODULE_MODBUS_ID, MODULE_MODBUS_FUNCTION_POINTER, MODULE_MODBUS_TOTAL_SEQ, MODULE_MODBUS_TOTAL_STRUCT, MODULE_MODBUS_PREV_STATE,
    MODULE_MODBUS_NEXT_STATE, MODULE_MODBUS_IRQ_STATUS, MODULE_MODBUS_PROCESS_STATUS, MODULE_MODBUS_MASTER_SHARED_MEM},
    
    // - Input Source Selector
    // MODULE_MOTOR_DEMAND_MUX must stay below all input sources, in order to accurately grab a pointer to each input sources structure.
    {MODULE_MOTOR_DEMAND_MUX_ID, MODULE_MOTOR_DEMAND_MUX_FUNCTION_POINTER, MODULE_MOTOR_DEMAND_MUX_TOTAL_SEQ, MODULE_MOTOR_DEMAND_MUX_TOTAL_STRUCT, MODULE_MOTOR_DEMAND_MUX_PREV_STATE,     
     MODULE_MOTOR_DEMAND_MUX_NEXT_STATE, MODULE_MOTOR_DEMAND_MUX_IRQ_STATUS, MODULE_MOTOR_DEMAND_MUX_PROCESS_STATUS, MODULE_MOTOR_DEMAND_MUX_MASTER_SHARED_MEM},
     
    // MODULE_MOTOR_COM must stay below MODULE_MOTOR_DEMAND_MUX, in order to accurately grab a pointer to the MOTOR_DEMAND_MUX structure.
    {MODULE_MOTOR_COM_ID, MODULE_MOTOR_COM_FUNCTION_POINTER, MODULE_MOTOR_COM_TOTAL_SEQ, MODULE_MOTOR_COM_TOTAL_STRUCT, MODULE_MOTOR_COM_PREV_STATE,     
     MODULE_MOTOR_COM_NEXT_STATE, MODULE_MOTOR_COM_IRQ_STATUS, MODULE_MOTOR_COM_PROCESS_STATUS, MODULE_MOTOR_COM_MASTER_SHARED_MEM},
    
    // - Digital Outputs
    {MODULE_DIGITAL_OUTPUTS_ID, MODULE_DIGITAL_OUTPUTS_FUNCTION_POINTER, MODULE_DIGITAL_OUTPUTS_TOTAL_SEQ, MODULE_DIGITAL_OUTPUTS_TOTAL_STRUCT, MODULE_DIGITAL_OUTPUTS_PREV_STATE,     
    MODULE_DIGITAL_OUTPUTS_NEXT_STATE, MODULE_DIGITAL_OUTPUTS_IRQ_STATUS, MODULE_DIGITAL_OUTPUTS_PROCESS_STATUS, MODULE_DIGITAL_OUTPUTS_MASTER_SHARED_MEM},
    
    {MODULE_ERR_LOGHANDLE_ID, MODULE_ERR_LOGHANDLE_FUNCTION_POINTER, MODULE_ERR_LOGHANDLE_TOTAL_SEQ, MODULE_ERR_LOGHANDLE_TOTAL_STRUCT, MODULE_ERR_LOGHANDLE_PREV_STATE,
     MODULE_ERR_LOGHANDLE_NEXT_STATE, MODULE_ERR_LOGHANDLE_IRQ_STATUS, MODULE_ERR_LOGHANDLE_PROCESS_STATUS, MODULE_ERR_LOGHANDLE_MASTER_SHARED_MEM},

   // Test Module
    {MODULE_TEST_ID, MODULE_TEST_FUNCTION_POINTER, MODULE_TEST_TOTAL_SEQ, MODULE_TEST_TOTAL_STRUCT, MODULE_TEST_PREV_STATE,
     MODULE_TEST_NEXT_STATE, MODULE_TEST_IRQ_STATUS, MODULE_TEST_PROCESS_STATUS, MODULE_TEST_MASTER_SHARED_MEM},

//    {MODULE_FLASH_BLK_SETTING_ID, MODULE_FLASH_BLK_SETTING_FUNCTION_POINTER, MODULE_FLASH_BLK_SETTING_TOTAL_SEQ, MODULE_FLASH_BLK_SETTING_TOTAL_STRUCT, MODULE_FLASH_BLK_SETTING_PREV_STATE,     
//     MODULE_FLASH_BLK_SETTING_NEXT_STATE, MODULE_FLASH_BLK_SETTING_IRQ_STATUS, MODULE_FLASH_BLK_SETTING_PROCESS_STATUS, MODULE_FLASH_BLK_SETTING_MASTER_SHARED_MEM},   
     
//    {MODULE_MOTOR_FW_UPDATE_ID, MODULE_MOTOR_FW_UPDATE_FUNCTION_POINTER, MODULE_MOTOR_FW_UPDATE_TOTAL_SEQ, MODULE_MOTOR_FW_UPDATE_TOTAL_STRUCT, MODULE_MOTOR_FW_UPDATE_PREV_STATE,     
//     MODULE_MOTOR_FW_UPDATE_NEXT_STATE, MODULE_MOTOR_FW_UPDATE_IRQ_STATUS, MODULE_MOTOR_FW_UPDATE_PROCESS_STATUS, MODULE_MOTOR_FW_UPDATE_MASTER_SHARED_MEM},   
     
};


uint8_t Sched_Initialize() {
    StructMem_InitBufs();
    SeqMem_InitBufs();


    // Watchdog_Initialize(NUM_OF_625_MS_INC);

    return TRUE;
}

void Sched_Run() {
    while (TRUE) {
        // Run each process with nextState_u8, save nextState_u8 as prevState_u8, and update nextState_u8 using return value.
        for (uint8_t table_index_u8 = 0; table_index_u8 < TOTAL_NUM_OF_PROCESSES; table_index_u8++) 
        {
            if (processInfoTable[table_index_u8].Sched_ModuleData.processStatus_u8 == PROCESS_STATUS_RUNNING) 
            {
                uint8_t current_state_u8 = processInfoTable[table_index_u8].Sched_ModuleData.nextState_u8;
                processInfoTable[table_index_u8].Sched_ModuleData.nextState_u8 =
                    (*processInfoTable[table_index_u8].Sched_ModuleData.p_module_u32)(
                        processInfoTable[table_index_u8].Sched_ModuleData.moduleId_u8,
                        processInfoTable[table_index_u8].Sched_ModuleData.prevState_u8,
                        processInfoTable[table_index_u8].Sched_ModuleData.nextState_u8,
                        processInfoTable[table_index_u8].Sched_ModuleData.irqState_u8);
                processInfoTable[table_index_u8].Sched_ModuleData.prevState_u8 = current_state_u8;
            }
            // Iterate through the interrupt register when an interrupt is present.
            // Handle each event sequentially by calling the interrupt handler of the driver's respective associated module.
            // Clear each event from the register after being handled.
            // After the event is handled, the module's previousStage_u8 remains unchanged, so normal operation can resume.
            while (SoftwareIrqBitPt[0] || SoftwareIrqBitPt[1] || SoftwareIrqBitPt[2] || SoftwareIrqBitPt[3]) {    
                uint64_t shifter_u8 = SHIFTER; 
                uint8_t IrqGroupIndx_u8 = 0;
                for (uint8_t drv_id_u8 = MIN_IRQ_ID; drv_id_u8 <= MAX_IRQ_ID; drv_id_u8++) 
                {
                    if (SoftwareIrqBitPt[IrqGroupIndx_u8] & shifter_u8) 
                    {
                        if (getProcessInfoIndex(drv_id_u8) != INDEX_NOT_FOUND) 
                        {
                            (*processInfoTable[drv_id_u8].Sched_ModuleData.p_module_u32)(
                                 processInfoTable[drv_id_u8].Sched_ModuleData.moduleId_u8,      //which software isr module ID
                                 1,                                                             //this meaningless for prevState in interrupt
                                 processInfoTable[drv_id_u8].Sched_ModuleData.irqState_u8,      //entry point/state for interrupt call back
                                 IrqTrigProcessID);                                             //the interrupt triggered module 
                            SoftwareIrqBitPt[IrqGroupIndx_u8] &= ~shifter_u8;
                        }
                    }
                    shifter_u8 <<= 1;
                    IrqGroupIndx_u8 = (drv_id_u8 & 0xC0) >> 6; //get the current interrupted module groups number 
                }              
            }
        }
        // Watchdog_Reload(); // Reload IWDG counter. TODO: Only call when MEERKAT is disabled by compiler directive
        // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8); // Toggle GPIO PC8
        // Insert delay 100 ms
        // HAL_Delay(10);
    }
}


uint8_t getProcessInfoIndex(uint8_t moduleId_u8)        //return Process index from processInfo array with the appID
{
    uint8_t idValue= 255;
    for(int i =0; i < TOTAL_NUM_OF_PROCESSES ; i++)
    {
       if(processInfoTable[i].Sched_ModuleData.moduleId_u8 == moduleId_u8)                               //find system appInfo of this driver
       {
          idValue = i;
       }
    }
    return idValue;                                                               //not found
}

/**
  * @brief  This function performs CRC calculation on BufSize bytes from input data buffer aDataBuf.
  * @param  BufSize Nb of bytes to be processed for CRC calculation
  * @retval 16-bit CRC value computed on input data buffer
  */
uint16_t Calculate_CRC(uint8_t BufSize, unsigned char* aDataBuf)
{
  register uint8_t index = 0;
  LL_CRC_ResetCRCCalculationUnit(CRC);
  /* Compute the CRC of Data Buffer array*/
  for (index = 0; index < BufSize ; index++)
  {
    LL_CRC_FeedData8(CRC,aDataBuf[index] );
  }
  /* Return computed CRC value */
  return (LL_CRC_ReadData16(CRC));
}


uint32_t Calculate_CRC32(uint8_t BufSize, unsigned char* aDataBuf)
{
  register uint8_t index = 0;
  LL_CRC_ResetCRCCalculationUnit(CRC);
  /* Compute the CRC of Data Buffer array*/
  for (index = 0; index < BufSize ; index++)
  {
    LL_CRC_FeedData8(CRC,aDataBuf[index] );
  }
  /* Return computed CRC value */
  return (LL_CRC_ReadData32(CRC));
}

// void Watchdog_Initialize(uint32_t timeout_u8) {
//     IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); // Enable write access to IWDG registers. (IWDG_PR and IWDG_RLR)
//     IWDG_SetPrescaler(IWDG_Prescaler_32);         // IWDG timer clock will be (LSI / 32).
//     IWDG_SetReload(timeout_u8 * LsiFreq / 32);   // Set counter reload value to obtain 250ms IWDG TimeOut.
//     IWDG_ReloadCounter();                         // Reload the IWDG counter (kick the dog for once!).
//     IWDG_Enable();                                // Enable IWDG (the LSI oscillator will be enabled by hardware).
// }

//void HAL_SYSTICK_Callback(void) { //Using SysTick_Handler() instead //SPA
    //tickCounter++; //SPA
    ////Meerkat_SafetyCore_SysTickCallback(); //SPA
//}

uint64_t getSysCount(void)
{
  return tickCounter; 
}
// void Watchdog_Initialize(uint8_t timeout_u8) {
//     LL_IWDG_Enable(IWDG);                             // Start the Independent Watchdog.
//     LL_IWDG_EnableWriteAccess(IWDG);                  // Enable write access to IWDG registers.
//     LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_4);  // IWDG timer clock will be (LSI / 32).
//     LL_IWDG_SetReloadCounter(IWDG, timeout_u8 * 625); // (timeout_s * 625) must be between Min_Data=0 and Max_Data=0x0FFF
//     while (LL_IWDG_IsReady(IWDG) != TRUE)             // Wait for the registers to be updated
//     {
//     }
//     LL_IWDG_ReloadCounter(IWDG); // Reload the IWDG counter (kick the dog for once!).
// }

// void Watchdog_Reload(void) {
//     LL_IWDG_ReloadCounter(IWDG);
// }

/**
  *************************************************************************************************************************************************************
  * @brief   Setup a software interrupt 
  * @details find out and set the correct bit in the software interrupt bit table SoftwareIrqBitPt[IrqGroupIndx_u8] 
  *             parameters:     SENDER_MODULE_ID        the module ID for the source of this interrupt
  *                             RECIVER_MODULE_ID       the module ID for the responding this interrupt
  *                             _irqType_u8             interrupt category of this interrupt
  *                             _irqDat_u8              data pass to the responding module
  *                             _irqDat1_len_u8         if _irqDatPt_u8 not equal to NULL this is the second byte of data pass to the responding module
  *                             _irqDatPt_u8            if data more than 2 byte can wrap it as pointer and use _irqDat1_len_u8 as the length of this data set
  * @return  
  *************************************************************************************************************************************************************
  */
void setupSoftwareIRQ(uint8_t SENDER_MODULE_ID, uint8_t RECIVER_MODULE_ID, uint8_t _irqType_u8, uint8_t _irqDat_u8, uint8_t _irqDat1_len_u8, uint8_t * _irqDatPt_u8) 
{ /**prepare software interrupt for the Ack timeout module**/                  
  uint8_t SoftwareIrqBitPtIndx = RECIVER_MODULE_ID / 64;     // get the interrupt pointer group of software IRQ point index
  uint64_t IrqBitTempry = 0x01;
  SoftwareIrqBitPt[SoftwareIrqBitPtIndx] |= IrqBitTempry << (RECIVER_MODULE_ID - (SoftwareIrqBitPtIndx * 64)); //set software interrupt trigger bit
  IrqTrigProcessID = SENDER_MODULE_ID;                                     /**set current module ID to let the IRQ response module know who triggered this interrupt  **/          
  //find out the ISR module and enter all the parameter for it to respone the interrupt, "this ACK time out error"
  uint8_t table_index_u8 = getProcessInfoIndex(RECIVER_MODULE_ID);  
  if (table_index_u8 != INDEX_NOT_FOUND) {
    processInfoTable[table_index_u8].Sched_DrvData.irqType_u8 = _irqType_u8;                     /**inform the interrupt response module this is an error message**/
    processInfoTable[table_index_u8].Sched_DrvData.irqDat_u8 = _irqDat_u8;                
    processInfoTable[table_index_u8].Sched_DrvData.irqDat1_len_u8 = _irqDat1_len_u8;      
    processInfoTable[table_index_u8].Sched_DrvData.irqDatPt_u8 = _irqDatPt_u8;                          //if no extend data so point to NULL                  
  }
}
