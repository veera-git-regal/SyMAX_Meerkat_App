/**
********************************************************************************************************************************
* @file    module_usart1.c 
* @author  Justin Moon/ Myron Mychal
* @brief   Main driver module for USART1 Communication.
* @details This module initializes the USART1 port and attaches the pre-selected fixed memory allocation to the module.
To Transmitt data in the RUN_MODULE case: put data into seqMemTX, and call this function:
*             USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
********************************************************************************************************************************
*/

// Includes --------------------------------------------------------------------
#include "module_usart1.h"
#include "driver_usart1.h"

// Content ---------------------------------------------------------------------

// - Function Prototypes
extern void Delay(__IO uint32_t nTime);
void assignModuleMem_USART1(void);

// -- Module States
enum {
  MEMORY_INIT_MODULE,
  INIT_MODULE,
  RUN_MODULE,
  // additional states to be added here as necessary.
  IRQ_MODULE = DEFAULT_IRQ_STATE,
  KILL_MODULE = KILL_APP
};

// - External Variables
extern Ram_Buf sharedMemArray[STRUCT_MEM_ARRAY_SIZE];
extern ProcessInfo processInfoTable[];

extern uint8_t usart1_CaptureLen;
extern uint8_t ModbusProtocolState;
extern __IO uint8_t indexTx_Usart1;

// - Global variables specific to this module
// -- Define Pointers that will be used as References to other Modules, where applicable
extern Usart1_Control usart1Control;
extern Ram_Buf_Handle usart1StructMem;

uint8_t usart1_RawRxBuf_u8[TX_RX_BUF_SIZE];
uint8_t usart1_ModbusRxBuf_u8[TX_RX_BUF_SIZE];
uint8_t usart1_TxBuf_u8[TX_RX_BUF_SIZE+7];
/**
********************************************************************************************************************************
* @brief   State machine for USART1 module
* @details
* @param   drv_identifier_u8, previous_state_u8, next_stat_u8, irq_identfier_u8
* @retval  return_state_u8
********************************************************************************************************************************
*/
uint32_t test_timeout_counter = 0;
uint32_t test_timeout_counter2 = 0;
uint8_t moduleUsart1(uint8_t drv_id_u8, uint8_t prev_state_u8, uint8_t next_state_u8, uint8_t irq_id_u8) {
  uint8_t return_state_u8 = MEMORY_INIT_MODULE;
  switch (next_state_u8) {
  case MEMORY_INIT_MODULE:
    {
      assignModuleMem_USART1(); // Assign structured memory
      return_state_u8 = INIT_MODULE;
      break;
    }
  case INIT_MODULE: {
    
    //assignModuleMem_USART1();
    
    /*
    // Find the structured memory for the UART2 driver module, by searching for the UART2 onwer id.
    Ram_Buf_Handle this_ram_buf_u32;
    for (uint8_t struct_mem_index_u8 = 0; struct_mem_index_u8 < TOTAL_NUM_OF_STRUCT_MEM_INSTANCES;
    struct_mem_index_u8++) {
      this_ram_buf_u32 = &sharedMemArray[struct_mem_index_u8];
      if (RamBuf_GetOwner(this_ram_buf_u32) == drv_id_u8) {
        usart1StructMem = &sharedMemArray[struct_mem_index_u8];
      }
    }
    
    // Attach the structured memory to the process's master shared memory.
    uint8_t table_index_u8 = getProcessInfoIndex(drv_id_u8);
    if (table_index_u8 != INDEX_NOT_FOUND) {
      processInfoTable[table_index_u8].Sched_DrvData.irqState_u8 = DEFAULT_IRQ_STATE;
      processInfoTable[table_index_u8].Sched_DrvData.p_masterSharedMem_u32 =
        usart1StructMem;
    }
    
    //Get structured memory for ADC1 data
    usart1Control = (Usart1_Control*)((*(processInfoTable[table_index_u8].Sched_DrvData.p_masterSharedMem_u32)).p_ramBuf_u8);
    */
    
    usart1_CaptureLen = ModbusHeaderLen;                                 //pam bug without this
    //usart1_CaptureLen = 1;           // TODO: Why init to header len?
    
    Usart1_Init(); // The interrupts need seq memeory. So init seq memory first.
    
    return_state_u8 = RUN_MODULE;
    break;
  }
  case RUN_MODULE: {
    // if (1) { // !test: Uart transmit every 1 second
    //   static uint64_t last_send_time = 0;
    //   const uint8_t test_buf[8] = {1,2,3,4,5,6,7,8}; 
    //   static uint8_t test_index = 0;
    //   uint64_t current_time = getSysCount();
    //   if (current_time - last_send_time > 1000) {
    //       LL_USART_TransmitData8(USART1, test_buf[test_index]);
    //       test_index = test_index + 1 & 0x07;
    //       last_send_time = current_time;
    //   }
    // }
    
    // If items in buffer, parse the buffer
    if((RingBuf_GetUsedNumOfElements((usart1Control).seqMem_RawRx) >= 1 ))
    {
      LL_USART_DisableIT_RXNE(USART1); // disable interrupts so new data doesn't come in while we are editing receive buffer
      // Idle line detection: Only interpret messages when transmission is complete
      //uint32_t idle_line = LL_USART_IsActiveFlag_IDLE(USART1); // TODO: LL calls should be in Driver only, Audit this file for those!
      uint32_t idle_line = LL_USART_IsActiveFlag_RTO(USART1);
      if (idle_line) {
        protocolHeaderFetch_Usart1();
        // LL_USART_ClearFlag_IDLE(USART1); // Clear the idle line flag.
        WRITE_REG(USART1->ICR, USART_ICR_RTOCF); // Clear the timeout flag. Note: Commented out - USING Idle Line detection instead
      }
      LL_USART_EnableIT_RXNE(USART1); // re-enable the receive interrupt after parsing the received data
    } 
    
    uint8_t TxLen = ModbusHeaderLen;
    //if((RingBuf_GetUsedNumOfElements((Ring_Buf_Handle)((*(usart1Control).seqMemTX).p_ringBuf_u8)) >= TxLen) && !indexTx_Usart1)
    if(((RingBuf_GetUsedNumOfElements((usart1Control).seqMemTX) >= TxLen) && !indexTx_Usart1) && (LL_USART_IsActiveFlag_TXE(USART1)))
    {
      TxProcess_Usart1();
    }  
    return_state_u8 = RUN_MODULE;
    break;
  }
  case KILL_MODULE: {
    // The USART1 driver module must only be executed once.
    // Setting processStatus_u8 to PROCESS_STATUS_KILLED prevents the scheduler main loop from calling this module again.
    uint8_t table_index_u8 = getProcessInfoIndex(drv_id_u8);
    if (table_index_u8 != INDEX_NOT_FOUND) {
      processInfoTable[table_index_u8].Sched_DrvData.processStatus_u8 = PROCESS_STATUS_KILLED;
    }
    return_state_u8 = KILL_MODULE;
    break;
  }
  default: {
    return_state_u8 = KILL_MODULE;
    break;
  }
  }
  return return_state_u8;
}

/**
********************************************************************************************************************************
* @brief   Assign structured/sequential memory
* @details Assign structured/sequential memory for USART1 module
* @param   None 
* @return  None
********************************************************************************************************************************
*/
void assignModuleMem_USART1(){  
  usart1SeqMem_RawRx = SeqMem_CreateInstance(MODULE_USART1, TX_RX_BUF_SIZE, 
                                             ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);//System call create a buffer for this driver need to be bigger than 1 complete frame 
(*usart1SeqMem_RawRx).p_ringBuf_u8 = (uint8_t *)usart1_RawRxBuf_u8;
  usart1SeqMem_ModbusRx = SeqMem_CreateInstance(MODULE_USART1, TX_RX_BUF_SIZE, 
                                                ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);//System call create a buffer for final packet receiver buffer 
 (*usart1SeqMem_ModbusRx).p_ringBuf_u8 = (uint8_t *)usart1_ModbusRxBuf_u8;
  usart1SeqMem_Tx = SeqMem_CreateInstance(MODULE_USART1, TX_RX_BUF_SIZE +7 , 
                                          ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);//System call create a buffer for Tx data 
(*usart1SeqMem_Tx).p_ringBuf_u8 = (uint8_t *)usart1_TxBuf_u8;
  //usart1StructMem =  StructMem_CreateInstance(MODULE_USART1, sizeof(Usart1_Control), ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);//System call create a structured memory for this driver [should map it back to this driver local struct]
  
  usart1StructMem =  StructMem_CreateInstance(MODULE_USART1, sizeof(Usart1_Control), ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);//System call create a structured memory for this driver [should map it back to this driver local struct]
  (*usart1StructMem).p_ramBuf_u8 = (uint8_t *)&usart1Control ;    // Map the usart1Control memory into the structured memory
  uint8_t usart1_index_u8 = getProcessInfoIndex(MODULE_USART1);
  processInfoTable[usart1_index_u8].Sched_DrvData.irqState_u8 = DEFAULT_IRQ_STATE;
  processInfoTable[usart1_index_u8].Sched_DrvData.p_masterSharedMem_u32 = (Ram_Buf_Handle)usart1StructMem;
  
  
  //usart1Control = (Usart1_Control*)(*usart1StructMem).p_ramBuf_u8;
  
  /** assign all the new generated sequential-memory of USART1 to the structured-memory **/
  usart1Control.seqMemTX = usart1SeqMem_Tx;
  usart1Control.seqMemTX->is_OverwrittingAllowed_u8 = TRUE; //FALSE;
  usart1Control.seqMem_ModbusRx = usart1SeqMem_ModbusRx;
  usart1Control.seqMem_RawRx = usart1SeqMem_RawRx;
  usart1Control.errorCode_u8 = 0;
  
}
