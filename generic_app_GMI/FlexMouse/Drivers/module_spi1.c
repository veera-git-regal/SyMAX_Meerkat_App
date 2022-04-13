/**
********************************************************************************************************************************
* @file    module_spi1.c 
* @author  Logan Schaufler
* @brief   Main driver module for SPI1 Communication.
* @details This module initializes the SPI1 port and attaches the pre-selected fixed memory allocation to the module.
To Transmitt data in the RUN_MODULE case: put data into seqMemTX, and call this function:
*             SPI_ITConfig(spi1, SPI_IT_TXE, ENABLE);
********************************************************************************************************************************
*/

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "module_spi1.h"
//#include "typedef.h"
//#include "stm32g0xx_ll_spi.h"
//#include "stm32g0xx_it.h"
//#include "macros.h"

#include <stdio.h>

/* scheduler handle declaration */
#include "scheduler.h"
#include "sequential_memory.h"
#include "structured_memory.h"

/* Content ---------------------------------------------------------------------------------------------------------------------*/
/* Uarts handle declaration */
//extern void Delay(__IO uint32_t nTime);

//void SPI1_TransferError_Callback(void);
//void WaitAndCheckEndOfTransfer(void);
void assignModuleMem_SPI1(void);
void SPI1_Init(void);
void TransferHandler(void);
void Send_Properly_Decoded_Response(void);
void Send_Improperly_Decoded_Response(void);

extern Ram_Buf sharedMemArray[STRUCT_MEM_ARRAY_SIZE];
extern ProcessInfo processInfoTable[];

SPI1_Control spi1_Control;

static Ram_Buf_Handle spi1_StructMem_u32;

//spi1_Control* spi1_Module_Control;
// unsigned char* RxCMD_Usart2 ;
extern uint8_t spi1_CaptureLen;
extern uint8_t SPIProtocolState;
//extern __IO uint8_t indexTx_SPI1;

//extern SPI1_Control *SPI1Control;

enum {
  MEMORY_INIT_MODULE,
  INIT_MODULE,
  RUN_MODULE,
  // additional states to be added here as necessary.
  IRQ_MODULE = DEFAULT_IRQ_STATE,
  KILL_MODULE = KILL_APP
};

uint8_t  Buffercmp8(uint8_t* pBuffer1, uint8_t* pBuffer2, uint8_t BufferLength);

//SPI_HandleTypeDef hspi1;

/* Buffer used for reception */
//uint8_t aRxBuffer[sizeof(aTxBuffer)];
//uint8_t ubNbDataToReceive = sizeof(aTxBuffer);
//__IO uint8_t ubReceiveIndex = 0;



uint8_t moduleSPI1_u32(uint8_t drv_id_u8, uint8_t prev_state_u8, uint8_t next_state_u8, uint8_t irq_id_u8) 
{
  uint8_t return_state_u8 = MEMORY_INIT_MODULE;
  switch (next_state_u8) {
  case MEMORY_INIT_MODULE:
    {
      assignModuleMem_SPI1(); // Assign structured memory
      return_state_u8 = INIT_MODULE;
      break;
    }
  case INIT_MODULE: 
    {
      //assignModuleMem_SPI1();
      SPI1_Init();
      LL_SPI_Enable(SPI1);
      
      // Find the structured memory for the UART2 driver module, by searching for the UART2 onwer id.
      //Ram_Buf_Handle this_ram_buf_u32;
      //uint8_t module_SPI1_Index = getProcessInfoIndex(MODULE_SPI);
      //spi1_LocalSPIControl = (SPI1_Control*)((*(processInfoTable[module_SPI1_Index].Sched_DrvData.p_masterSharedMem_u32)).p_ramBuf_u8);
      
      //analog_amps_low_alarm_enable_count_u8 = analog_4_20ma_Control.analog_4_20mA_Setting.analogLowAlarmEnableCount_u8; // delay before analog low voltage alarm can be triggered
      //module_spi1_poll_time_u64 = getSysCount() + analog_4_20ma_Control.analog_4_20mA_Setting.analogPeriod_u16; //AnalogPeriod;    // Store time tick value
      //module_spi1_poll_time_u64 = getSysCount();  
      
      // for (uint8_t struct_mem_index_u8 = 0; struct_mem_index_u8 < TOTAL_NUM_OF_STRUCT_MEM_INSTANCES;
      // struct_mem_index_u8++) {
      //   this_ram_buf_u32 = &sharedMemArray[struct_mem_index_u8];
      //   if (RamBuf_GetOwner(this_ram_buf_u32) == drv_id_u8) {
      //     spi1StructMem = &sharedMemArray[struct_mem_index_u8];
      //   }
      // }
      
      // Attach the structured memory to the process's master shared memory.
      // uint8_t table_index_u8 = getProcessInfoIndex(drv_id_u8);
      // if (table_index_u8 != INDEX_NOT_FOUND) {
      //   processInfoTable[table_index_u8].Sched_DrvData.irqState_u8 = DEFAULT_IRQ_STATE;
      //   processInfoTable[table_index_u8].Sched_DrvData.p_masterSharedMem_u32 =
      //     spi1StructMem;
      // }
      
      //Get structured memory for SPI1 data
      // spi1Control = (SPI1_Control*)((*(processInfoTable[table_index_u8].Sched_DrvData.p_masterSharedMem_u32)).p_ramBuf_u8);
      // spi1_CaptureLen = SPIHeaderLen;                                 //pam bug without this
      return_state_u8 = RUN_MODULE;
      //test_index++;
      break;
    }
  case RUN_MODULE: 
    {
      if (0) 
      { // !errorTest: Uart transmit every 1 second
      }
      
      
      static uint64_t last_send_time = 0;
      uint8_t test_buf[] = {0x55,0x00,0x00,0x55,0x00,0x01}; 
      static uint8_t test_index = 0;
      uint64_t current_time = getSysCount();
      //if (spi_mode == MASTER_MODE) 
      //{
      if (current_time - last_send_time > 10) {
        //LL_SPI_Enable(SPI1);
        //for(uint8_t i = 0; i < (sizeof(test_buf)); i++)
        //{
        LL_SPI_TransmitData8(SPI1, test_buf[test_index]);
        //LL_SPI_DisableIT_RXNE(SPI1);
        //if(LL_SPI_IsActiveFlag_TXE(SPI1))
        //{
        
        //  LL_SPI_TransmitData16(SPI1, );
        //}
        test_index++;
        if (test_index > 6) test_index = 0;
        last_send_time = current_time;
        
        //aRxBuffer[receive_index++] = LL_SPI_ReceiveData8(SPI1);
        //receive_index = (receive_index + 1) & 0x07;
        //}
        //LL_SPI_Disable(SPI1);
        //last_send_time = current_time;
      }
      //}
      
      
      //WaitAndCheckEndOfTransfer();
      
      /* FUTURE
      // //(*spi1Control).seqMem_RawRx->systemInstanceIndex_u8)
      // //if(RingBuf_GetUsedNumOfElements((Ring_Buf_Handle)((*(*spi1Control).seqMem_RawRx).p_ringBuf_u8)) >= spi1_CaptureLen )
      // if(RingBuf_GetUsedNumOfElements((*spi1Control).seqMem_RawRx) >= spi1_CaptureLen )
      // {
      //   protocolHeaderFetch_spi1();
      // }
      // uint8_t TxLen = SPIHeaderLen;
      // //if((RingBuf_GetUsedNumOfElements((Ring_Buf_Handle)((*(*spi1Control).seqMemTX).p_ringBuf_u8)) >= TxLen) && !indexTx_spi1)
      // if(((RingBuf_GetUsedNumOfElements((*spi1Control).seqMemTX) >= TxLen) && !indexTx_spi1) && (LL_SPI_IsActiveFlag_TXE(spi1)))
      // {
      //   TxProcess_spi1();
      // } 
      */
      //spi1_Control.spi1_result.Rx_buffer = {0,0,0,0,0,0,0,0};
      
      return_state_u8 = RUN_MODULE;
      break;
    }
  case KILL_MODULE: 
    {
      // The spi1 driver module must only be executed once.
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
      return_state_u8 = KILL_MODULE;
      break;
    }
  }
  return return_state_u8;
}

void assignModuleMem_SPI1(void)
{  
  // spi1SeqMem_RawRx = SeqMem_CreateInstance(MODULE_SPI1, TX_RX_BUF_SIZE, 
  //                             ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);//System call create a buffer for this driver need to be bigger than 1 complete frame 
  // spi1SeqMem_SPIRx = SeqMem_CreateInstance(MODULE_SPI1, TX_RX_BUF_SIZE, 
  //                             ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);//System call create a buffer for final packet receiver buffer 
  // // spi1SeqMemRXG3_u32 = SeqMem_CreateInstance(MODULE_spi1, TX_RX_BUF_SIZE, 
  // //                             ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);//System call create a buffer for final packet receiver buffer 
  // // spi1SeqMemRXG4_u32 = SeqMem_CreateInstance(MODULE_spi1, TX_RX_BUF_SIZE, 
  // //                             ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);//System call create a buffer for final packet receiver buffer 
  // spi1SeqMem_Tx = SeqMem_CreateInstance(MODULE_SPI1, TX_RX_BUF_SIZE +7 , 
  //                             ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);//System call create a buffer for Tx data 
  // spi1StructMem =  StructMem_CreateInstance(MODULE_SPI1, sizeof(spi1_Control), ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);//System call create a structured memory for this driver [should map it back to this driver local struct]
  
  // spi1Control = (spi1_Control*)(*spi1StructMem).p_ramBuf_u8;
  
  // /** assign all the new generated sequential-memory of spi1 to the structured-memory **/
  // spi1Control->seqMemTX = spi1SeqMem_Tx;
  // spi1Control->seqMemTX->is_OverwrittingAllowed_u8 = TRUE; //FALSE;
  // spi1Control->seqMem_SPIRx = spi1SeqMem_SPIRx;
  // // spi1Control->seqMemRXG3_u32 = spi1SeqMemRXG3_u32;
  // // spi1Control->seqMemRXG4_u32 = spi1SeqMemRXG4_u32;
  // spi1Control->seqMem_RawRx = spi1SeqMem_RawRx;
  // spi1Control->errorCode_u8 = 0;
  spi1_StructMem_u32 =  StructMem_CreateInstance(MODULE_SPI, sizeof(SPI1_Control), ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);
  (*spi1_StructMem_u32).p_ramBuf_u8 = (uint8_t *)&spi1_Control ;    // Map the SPI1 memory into the structured memory
  uint8_t module_spi1_index_u8 = getProcessInfoIndex(MODULE_SPI);
  processInfoTable[module_spi1_index_u8].Sched_ModuleData.p_masterSharedMem_u32 = (Ram_Buf_Handle)spi1_StructMem_u32;

}

/**
* @brief  Function called when Rx buffer is full; interprets received messages
* @param  None
* @retval None
*/
void Decode_SPI_Rx_Buffer(uint8_t *aRxBuffer)
{
  //Code to determine first byte of message
  uint8_t spi_msg_sync = aRxBuffer[0];
  uint8_t spi_msg_len = aRxBuffer[1];
  uint8_t spi_msg_cmd = aRxBuffer[2];
  
  //!FUTURE: IMPLEMENT SCHEMING FOR MESSAGES THAT ARE LONGER THAN 3 BYTES
  
  uint8_t SPI_SlaveTxBuffer[8] = {0,0,0,0,0,0,0,0};  //Initialize null Tx Buffer
  
  if (spi_msg_sync == 0x55)  //INDICATES BOARD IS IN SLAVE MODE
  {
    //Switch statement for all possible messages that can be received
    switch(spi_msg_cmd)
    {
    case 0x00:  //Start Motor
      if(spi_msg_len == 0x00)
      {
        
        //DO WHATEVER HAPPENS HERE TO MAKE THE MOTOR START
        
        SPI_SlaveTxBuffer[0] = 0xAA;
        SPI_SlaveTxBuffer[1] = 0x00;
        SPI_SlaveTxBuffer[2] = 0x00;
        //Send_Properly_Decoded_Response(SPI_SlaveTxBuffer);
        Send_Properly_Decoded_Response();
      }
      else
      {
        Send_Improperly_Decoded_Response();
      }
      
      break;
      
    case 0x01:  //Stop Motor
      if(spi_msg_len == 0x00)
      {
        
        //DO WHATEVER HAPPENS HERE TO MAKE THE MOTOR STOP
        
        SPI_SlaveTxBuffer[0] = 0xAA;
        SPI_SlaveTxBuffer[1] = 0x00;
        SPI_SlaveTxBuffer[2] = 0x01;
        //Send_Properly_Decoded_Response(SPI_SlaveTxBuffer);
        Send_Properly_Decoded_Response();
      }
      else
      {
        Send_Improperly_Decoded_Response();
      }
      
      break;
      
    default:  //Command is not recognized
      Send_Improperly_Decoded_Response();
    }
  }
}

//Logic, function that states if message has been received and decoded properly

//void Send_Properly_Decoded_Response(uint64_t SPI_SlaveTxBuffer[]) 
void Send_Properly_Decoded_Response()
{
  //uint8_t SPI_Message_Length = 0x07 & SPI_SlaveTxBuffer[1];    //Determines length of additional bytes in full message to send to master
  
  // for (uint8_t transfer_index = 0; transfer_index < 3 + SPI_Message_Length; transfer_index++)
  // {
  // LL_SPI_TransmitData8(SPI_SlaveTxBuffer[transfer_index]);
  // }
  
  LL_SPI_TransmitData8(SPI1,0xAA); //Temp ACK command
  
  //clear Tx buffer
  // for (uint8_t buffer_clear_index = 0; buffer_clear_index < 3 + SPI_Message_Length; buffer_clear_index++)
  // {
  //   SPI_SlaveTxBuffer[buffer_clear_index] = 0x00;
  // }
}

//Logic, function that states if message has been received decoded improperly (NAK response, etc.)
void Send_Improperly_Decoded_Response(void) 
{
  //LL_SPI_TransmitData8();
}