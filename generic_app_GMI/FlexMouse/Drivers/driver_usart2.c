/**
  ********************************************************************************************************************************
  * @file    drv_usart2.c 
  * @author  Pamela Lee
  * @brief   Main Driver function/s for serial protocol with Usart2 hardware
  * @details Protocol Usart2, after decode a whole valid frame from serial port2,
  *          trigger the system control to execute the relative APP in the int stage
  *          the Rx data is in usart2SeqMemRX_u32.
  *          To Transmitt data : put data into usart2SeqMemTX_u32, and call this function
  *                              USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
  ********************************************************************************************************************************
  */

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "driver_usart2.h"

#include "main.h"

/* Private variables ---------------------------------------------------------*/
//UART_HandleTypeDef huart2; //SPA

Usart2_Control usart2Control;
//extern uint64_t irqRequestReg_u64;            //superseded by SoftwareIrqBitPt[0 to 3] for all 254 module ID bit pointer                  
extern volatile uint64_t tickCounter; //SPA

__IO ITStatus UartReady = RESET;
uint8_t usart2CaptureLen;                     //default Universal Protocol header length
//uint8_t counter = 0;
uint16_t uwCRCValue = 0;
uint64_t lastRxTime_u64 = 0;

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
  LL_IOP_GRP1_EnableClock(USART_IOP_GRP1_PERIPH_PORT);
  /**USART2 GPIO Configuration  
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX 
  */
  GPIO_InitStruct.Pin = USART_TX_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = USART_GPIO_AF;
  LL_GPIO_Init(USART_TX_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = USART_RX_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = USART_GPIO_AF;
  LL_GPIO_Init(USART_RX_GPIO_Port, &GPIO_InitStruct);

  /* USART2 interrupt Init */
  NVIC_SetPriority(USART2_IRQn, 0);
  NVIC_EnableIRQ(USART2_IRQn);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  
  LL_USART_DisableIT_CTS(USART2);
  LL_USART_DisableIT_ERROR(USART2); // REVIEW: This should disable the ORE Interrupt, but doesn't appear to
  // LL_USART_DisableDMADeactOnRxErr(USART2);
  LL_USART_ConfigAsyncMode(USART2);
  //LL_USART_EnableIT_IDLE(USART2); // Idle line detection, interrupt is not required for use of Flag outside of ISRs
  /* USER CODE BEGIN WKUPType USART2 */

  /* USER CODE END WKUPType USART2 */

  LL_USART_Enable(USART2);
  // LL_USART_ClearFlag_ORE(USART2);               //reset all usart error bit

  /* Polling USART2 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART2))) || (!(LL_USART_IsActiveFlag_REACK(USART2))))
  {
  }
  LL_USART_EnableIT_RXNE(USART2);
  /* USER CODE BEGIN USART2_Init 2 */


  /* USER CODE END USART2_Init 2 */

}


//void assign_UART2_DrvMem(){  
//  usart2InternalSeqMem_u32 = SeqMem_CreateInstance(DRV_USART2, TX_RX_BUF_SIZE, 
//                              ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);//System call create a buffer for this driver need to be bigger than 1 complete frame 
//  usart2SeqMemRX_u32 = SeqMem_CreateInstance(DRV_USART2, TX_RX_BUF_SIZE, 
//                              ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);//System call create a buffer for final packet receiver buffer 
//  usart2SeqMemTX_u32 = SeqMem_CreateInstance(DRV_USART2, TX_RX_BUF_SIZE, 
//                              ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);//System call create a buffer for Tx data 
//  usart2StructMem_u32 =  StructMem_CreateInstance(DRV_USART2, sizeof(Usart2_Control), ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);//System call create a structured memory for this driver [should map it back to this driver local struct]
//  
//  usart2Control = (Usart2_Control*)(*usart2StructMem_u32).p_ramBuf_u8;
//  
//  usart2Control->seqMemTX_u32 = usart2SeqMemTX_u32;
//  usart2Control->seqMemRX_u32 = usart2SeqMemRX_u32;
//  usart2Control->seqMem_InternalPipe_u32 = usart2InternalSeqMem_u32;
//  usart2Control->errorCode_u8 = 0;
//  
//  
// // Usart2InternalPipe =GetPipe(DM_Usart2, 80, 2, NULL, 0);                       //System call create a pipe for this driver  need to be bigger than 1 complete frame
//  //Usart2RxPipe = GetPipe(DM_Usart2, 80, 2, NULL, 0);                           //System call create a pipe for final packet receiver buffer 
//  //Usart2TxPipe = GetPipe(DM_Usart2, 80, 2, NULL, 0);                           //System call create a pipe for Tx data    
//  //Usart2ShMem = GetShMem(DM_Usart2, sizeof(Usart2Control) , 2, NULL, 0);        //System call create a Shared memory for this driver [should map it back to this driver local struct]
//  //usart2Control = (Usart2Control*)(*Usart2ShMem).ShMemBuf;                      //map the Uart2Control memory into the shared memory
//  //usart2Control->TxPipe = Usart2TxPipe;                                         //put Tx pipe pipe pointer into shared memory
//  //usart2Control->RxPipe = Usart2RxPipe;                                         //put Rx pipe pipe pointer into shared memory
//  //usart2Control->InternalPipe = Usart2InternalPipe;                             //put internal pipe pipe pointer into shared memory
//  //usart2Control->ErrorCode = 0;                                                 //clear the error code//
//}


void usart2_Init(){
  MX_USART2_UART_Init();
}



/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
//void MX_USART2_UART_Init(void)
//{
//
//  /* USER CODE BEGIN USART2_Init 0 */
//
//  /* USER CODE END USART2_Init 0 */
//
//  LL_USART_InitTypeDef USART_InitStruct = {0};
//
//  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//  /* Peripheral clock enable */
//  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
//  
//  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
//  /**USART2 GPIO Configuration  
//  PA2   ------> USART2_TX
//  PA3   ------> USART2_RX 
//  */
//  GPIO_InitStruct.Pin = USART_TX_Pin;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
//  LL_GPIO_Init(USART_TX_GPIO_Port, &GPIO_InitStruct);
//
//  GPIO_InitStruct.Pin = USART_RX_Pin;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
//  LL_GPIO_Init(USART_RX_GPIO_Port, &GPIO_InitStruct);
//
//  /* USART2 interrupt Init */
//  NVIC_SetPriority(USART2_IRQn, 0);
//  NVIC_EnableIRQ(USART2_IRQn);
//
//  /* USER CODE BEGIN USART2_Init 1 */
//
//  /* USER CODE END USART2_Init 1 */
//  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
//  USART_InitStruct.BaudRate = 115200;
//  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
//  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
//  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
//  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
//  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
//  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
//  LL_USART_Init(USART2, &USART_InitStruct);
//  LL_USART_ConfigAsyncMode(USART2);
//
//  /* USER CODE BEGIN WKUPType USART2 */
//
//  /* USER CODE END WKUPType USART2 */
//
//  LL_USART_Enable(USART2);
//
//  /* Polling USART2 initialisation */
//  while((!(LL_USART_IsActiveFlag_TEACK(USART2))) || (!(LL_USART_IsActiveFlag_REACK(USART2))))
//  {
//  }
//  /* USER CODE BEGIN USART2_Init 2 */
//
//  /* USER CODE END USART2_Init 2 */
//
//}

/*
uint8_t Usart2_IRQCallback(void) {
    uint8_t rxDat2;
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) { //  Received characters added to fifo
        rxDat2 = (uint8_t) USART_RxData(USART2);
        (*usart2InternalSeqMem_u32).Write(&rxDat2);
#ifdef M_TALK
        if (MtalkGetDat(usart2InternalSeqMem_u32, usart2SeqMemRX_u32, usart2SeqMemTX_u32)) {
            irqRequestReg_u64 |=
                0x2; // Setup driver interrupt request for Uart2 driver2, note bit oriented setting, can call more than one apps
        }
#endif
#ifdef EASY_LINK
        if (EasyLinkGetDat(usart2InternalSeqMem_u32, usart2SeqMemRX_u32, usart2SeqMemTX_u32)) {
            irqRequestReg_u64 |=
                0x2; // Setup driver interrupt request for Uart2 driver2, note bit oriented setting, can call more than one apps;
        }
#endif
#ifdef UNIVERSAL
        if (UniversalGetDat(usart2InternalSeqMem_u32, usart2SeqMemRX_u32, usart2SeqMemTX_u32)) {
            irqRequestReg_u64 |=
                0x2; // Setup driver interrupt request for Uart2 driver2, note bit oriented setting, can call more than one apps;
        }
#endif
    }

    if (USART_GetITStatus(USART2, USART_IT_TXE) != RESET) {
        uint8_t txDat2;
        uint32_t length = 1;
        if (usart2SeqMemTX_u32->IsEmpty()) {
            USART_ITConfig(USART2, USART_IT_TXE, DISABLE); //  Suppress interrupt when empty
        } else {
            (*usart2SeqMemTX_u32).Read(&txDat2, &length);
            USART_SendData(USART2, (uint16_t) txDat2); //  Transmit the character
        }
    }
    return 1;
}
*/

/**
  * @brief  Function called from USART IRQ Handler when RXNE flag is set
  *         Function is in charge of reading character received on USART RX line.
  * @param  None
  * @retval None
  */
void USART2_CharReception_Callback(void)
{
  /*
  __IO uint32_t received_char;

  // Read Received character. RXNE flag is cleared by reading of RDR register //
  received_char = LL_USART_ReceiveData8(USART2);
  */
 
  unsigned char rxDat2;
  rxDat2 = LL_USART_ReceiveData8(USART2);
  if(usart2Control.UsartMode_u8 == 1)          //check this is universal protocol mode or transparent mode
  {
    RingBuf_WriteCharacter((usart2Control).seqMemRX_transparentMode_u32,&rxDat2);      //In transparent mode will directly push in the Usart transparent pipe
  }
  else
  {
    RingBuf_WriteCharacter((usart2Control).seqMem_InternalPipe_u32,&rxDat2);
  }
  lastRxTime_u64 = tickCounter;
  //RBWrite(usart2InternalSeqMem_u32->systemInstanceIndex_u8,&rxDat2); 
  
  /* Echo received character on TX */
 // LL_USART_TransmitData8(USART2, received_char);
}

// USART2 does not appear to support IDLE line detection
// - this function is a patch to emulate the behavior by comparing 
// -- Systick to the time the last data byte was received.
#define USART2_IDLE_PATCH_RX_TIMEOUT_MS 10 // @115200 Baud: Byte Transmission time is 0.07ms.
uint8_t Usart2IdlePatch_RxIsIdle(void) { 
  return (tickCounter - lastRxTime_u64 > USART2_IDLE_PATCH_RX_TIMEOUT_MS ? 1 : 0);
}
/******************* variable for Usart2 TX interrupt **************************/
__IO uint8_t indexTx = 0;
uint8_t ubSizeToSend = 0;


#define ENABLE_WHOLEFRAME_USART2_BUF_FIXED_LEN 1
#if ENABLE_WHOLEFRAME_USART2_BUF_FIXED_LEN >= 1
  // This is a one-shot buffer, that is written to and read from in single calls.
  // - it does not currently need to be tracked for current index because of this.
  #define FIXED_WHOLEFRAME_USART2_MAX_LENGTH TX_RX_BUF_SIZE // Inclusive (this value is accepted) 
  unsigned char fixedwholeFrameBuf_Usart2_Length = 0;
  unsigned char fixedwholeFrameBuf_Usart2[FIXED_WHOLEFRAME_USART2_MAX_LENGTH];
  unsigned char* wholeFrameBuf_Usart2 = fixedwholeFrameBuf_Usart2;
#else // if ENABLE_WHOLEFRAME_USART2_BUF_FIXED_LEN <= 0
  unsigned char* wholeFrameBuf_Usart2;
#endif




/*******************************************************************************/
/**
  * @brief  Function called for achieving next TX Byte sending
  * @param  None
  * @retval None
  */
void USART2_TXEmpty_Callback(void)
{
  if((usart2Control).UsartMode_u8 == 1)
  {
    /* Disable TXE interrupt */
      LL_USART_DisableIT_TXE(USART2);    
  }
  else
  {
    /* Fill TDR with a new char */
    LL_USART_TransmitData8(USART2, wholeFrameBuf_Usart2[indexTx++]);
    
    if (indexTx == (ubSizeToSend ))
    {
      /* Disable TXE interrupt */
      LL_USART_DisableIT_TXE(USART2);
      indexTx = 0;
    } 
  }
}

/**
  * @brief  Function called at completion of last byte transmission
  * @param  None
  * @retval None
  */
void USART2_CharTransmitComplete_Callback(void)
{
//  if (indexTx == sizeof(ubSizeToSend))
//  {
//    indexTx = 0;

    /* Disable TC interrupt */
//    LL_USART_DisableIT_TC(USART2);
//  }
}


static uint8_t dataLen = 0;                                         //length of data byte/s plus CRC expected
uint8_t UniProtocolState = protocolstart;
#define ENABLE_PROTOCOLBUF_USART2_FIXED_LEN 1
#if ENABLE_PROTOCOLBUF_USART2_FIXED_LEN >= 1
  // This is a one-shot buffer, that is written to and read from in single calls.
  // - it does not currently need to be tracked for current index because of this.
  #define FIXED_PROTOCOLBUF_USART2_MAX_LENGTH USART2_SINGLE_MESSAGE_RX_BUF_SIZE // Inclusive (this value is accepted) 
  unsigned char fixedProtocolBuf_Usart2_Length = 0;
  unsigned char fixedProtocolBuf_Usart2[FIXED_PROTOCOLBUF_USART2_MAX_LENGTH];
  unsigned char* protocolBuf_Usart2 = fixedProtocolBuf_Usart2; // !start here: Test Me!
  unsigned char protocolBuf_Usart2_OverflowCounter = 0;
#else
  unsigned char* protocolBuf_Usart2;
#endif

/**
  *@brief   Function for decode a valid frame of universal protocol
  *         the Rx data will put into an internal pipe and process after 
  *         the first 5 byte contain sync byte and then wait for the whole header 
  *         bytes in side the internal pipe, then perform the header veification.
  *         If the the header bytes are valid, then prepare for getting the whole data fields 
  *         and perform CRC check.
  * @param  None
  * @retval None
  */
void protocolHeaderfetch(void)
{  
  // TODO: 20210303a - Universal Protocol Misses Messages Frequently by checking for full messages when transmission is not complete.
  // - The longer the message, the more likely it will be discarded for a missing byte. Easy to duplicate at message size 32.
  // -- REVIEW: Is this an issue on Motor Side? Or has this been patched there?
  
//  usart2SeqMemRX_u32 = (usart2Control).seqMemRX_u32;   //pam bug without
  switch(UniProtocolState)
  {
    case  protocolstart:
     Reentryprotocalstart: 
      { // assume this is very beginning of universal-frame, check and clear any garble before sync byte
        usart2CaptureLen = UniHeaderlen; 
        dataLen = 0;
        unsigned int SyncPosition = RingBuf_Search((usart2Control).seqMem_InternalPipe_u32, RxSyncChr, 0);              //search sync byte within internal pipe,[return position or 0xf000 == not found 0xffff == error]
        if (SyncPosition != 0) 
        {
          if(SyncPosition & 0xf000)                                                            
          { //sync byte not found or error, then clear the first 5 bytes
            SyncPosition = UniHeaderlen;                                                                // this will clear all 5 byte                                       
          }
          #if ENABLE_PROTOCOLBUF_USART2_FIXED_LEN >= 1
              // Clear Pre-Sync Bytes from the buffer by 'Read'ing them from the RingBuf
              fixedProtocolBuf_Usart2_Length = SyncPosition;
              RingBuf_ReadBlock((usart2Control).seqMem_InternalPipe_u32, protocolBuf_Usart2, &SyncPosition);                          //truncate any char before SYNC char 
              // Clear Message Parsing Data and Exit Routine
              usart2CaptureLen = UniHeaderlen; 
              dataLen = 0;
              UniProtocolState = protocolstart;
              break;
          #else // ENABLE_PROTOCOLBUF_FIXED_LEN == 0
            if((protocolBuf_Usart2 = (unsigned char*) realloc(protocolBuf_Usart2,SyncPosition)) == NULL) reallocError++;
            RingBuf_ReadBlock((usart2Control).seqMem_InternalPipe_u32, protocolBuf_Usart2, &SyncPosition);                          //truncate any char before SYNC char 
            if((protocolBuf_Usart2 = (unsigned char*)realloc(protocolBuf_Usart2,1)) == NULL) reallocError++;
            usart2CaptureLen = UniHeaderlen; 
            UniProtocolState = protocolstart;                                                             // still to back here next stage for complete header frame 
            break;
          #endif // ENABLE_PROTOCOLBUF_FIXED_LEN == 0
          break;
        }
        // otherwise mean the the first byte in buffer is sync byte, then can directly process to header Validate state !!!!!!!!!
        UniProtocolState = headerValidate;
      }
    case headerValidate:
      { //check this is a valid header frame in internal pipe
        unsigned char headerBuf[]={ 0, 0, 0, 0, 0, 0, 0, 0, 0};                 //9 byte array  for header including advanced CMD 
        unsigned int headerLen = 9;     
        RingBuf_Observe((usart2Control).seqMem_InternalPipe_u32, headerBuf, 0, &headerLen);      //get the pipe data without clear internal pipe, headerLen will return number of bytes in pipe if less than 9 byte

        uint8_t dataA ;
        uint8_t headerCRCLen = 7;                                               //default normal frame header + CRC bytes = 7 byte
        uint8_t dataC = (uint8_t) headerBuf[1] & 0xC0;
        if (dataC == 0)                                                         //check this is advanced frame [00]+[data length]
        {
          dataA = (uint8_t)headerBuf[4] & 0xf0 ;                                //this is normal frame, store dataA as the expected Header-end in byte 5
        }
        else
        {  //this is advanced frame [11]+[data length]   or  garbage 
          if (dataC == 0xC0)
          { //this is advanced frame 
            dataA = (uint8_t)headerBuf[6] & 0xf0 ;                              //this is Advanced frame,  store dataA as the expected Header-end in byte 7 
            headerCRCLen = 9;                                                   //header + CRC bytes = 9 byte
          }
          else
          { //this is garbage, because the frame type pointer not valid
            unsigned char tmpbuf3;
            unsigned int truncateLen = 1;   
            RingBuf_ReadBlock((usart2Control).seqMem_InternalPipe_u32, &tmpbuf3, &truncateLen);                             //truncate the first byte incase the garbage is the same of sync byte
            usart2CaptureLen = UniHeaderlen; 
            UniProtocolState = protocolstart;    
            if( *RingBuf_GetPointerToRingBuf((usart2Control).seqMem_InternalPipe_u32) >= usart2CaptureLen)                     //after truncase the carbage byte still contain more then header length then goto back to do again
            {
              goto Reentryprotocalstart;                                                                 //this line needed to use goto for finish the frame check in one go
            }
            break;   
          }
        }
       if(dataA != (~((uint8_t)RxSyncChr) & 0xf0))                                  //check header-end byte is valid
        {// this is a valid header        
         // if the header frame is not valid, then delete the first byte only
          unsigned char tmpbuf3;
          unsigned int truncateLen = 1;   
          RingBuf_ReadBlock((usart2Control).seqMem_InternalPipe_u32, &tmpbuf3, &truncateLen);                             //truncate the first byte incase the garbage is the same of sync byte
          usart2CaptureLen = UniHeaderlen; 
          UniProtocolState = protocolstart;    
          if( *RingBuf_GetPointerToRingBuf((usart2Control).seqMem_InternalPipe_u32) >= usart2CaptureLen)                     //after truncase the carbage byte still contain more then header length then goto back to do again
          {
            goto Reentryprotocalstart;                  //this line needed to use goto for finish the frame check in one go
          }
          break;
        }
        else
        {
          dataLen = (uint8_t) headerBuf[1] & 0x3F;     
          usart2CaptureLen = (uint8_t)(dataLen + headerCRCLen) ; // header+ CRC +data length = number of bytes for next capture
          UniProtocolState = frameCRC;
          //valid header so fall through to checksum
        }
      }
    case frameCRC:
      {
        #if ENABLE_PROTOCOLBUF_USART2_FIXED_LEN >= 1
          // Clear Pre-Sync Bytes from the buffer by 'Read'ing them from the RingBuf
          if (usart2CaptureLen <= FIXED_PROTOCOLBUF_USART2_MAX_LENGTH) { // Normal Case
            fixedProtocolBuf_Usart2_Length = usart2CaptureLen; // TODO: Make this segment use DataLen2 (assigned below), just like the others.
            // REVIEW: usart2CaptureLen is calculated from Data Length (and not where the CRC was)
          } else { // Overflow Case
            protocolBuf_Usart2_OverflowCounter++; // !errorRemove: Testing
            // Read All Data (Clear the Buffer)
            while (usart2CaptureLen > 0) {
              if (usart2CaptureLen > FIXED_PROTOCOLBUF_USART2_MAX_LENGTH) {
                // REVIEW: Replace with RingBuf_ClearContents? Much less processing
                unsigned int read_length = FIXED_PROTOCOLBUF_USART2_MAX_LENGTH;
                RingBuf_ReadBlock((usart2Control).seqMem_InternalPipe_u32, protocolBuf_Usart2, &read_length); //extract the whole frame
                // RingBuf_ReadBlock((usart2Control).seqMemTX_u32, headerFramebuf, &read_length);             //copy the complete frame into buffer
                usart2CaptureLen -= FIXED_PROTOCOLBUF_USART2_MAX_LENGTH;
              } else {
                unsigned int read_length = usart2CaptureLen;
                RingBuf_ReadBlock((usart2Control).seqMem_InternalPipe_u32, protocolBuf_Usart2, &read_length); //extract the whole frame
                // RingBuf_ReadBlock((usart2Control).seqMemTX_u32, headerFramebuf, &DataLen2);             //copy the complete frame into buffer
                usart2CaptureLen = 0;
              }
            }
            // Clear Message Parsing Data and Exit Routine
            usart2CaptureLen = UniHeaderlen; 
            dataLen = 0;
            UniProtocolState = protocolstart;
            break;
          }
        #else // ENABLE_PROTOCOLBUF_FIXED_LEN == 0
          if((protocolBuf_Usart2 = (unsigned char*) realloc(protocolBuf_Usart2,usart2CaptureLen)) == NULL) reallocError++;        
        #endif

        unsigned int DataLen2 = (unsigned int)usart2CaptureLen;
        RingBuf_Observe((usart2Control).seqMem_InternalPipe_u32, protocolBuf_Usart2, 0, &DataLen2);                             //copy the whole frame into buffer
        uint16_t frameCRC = (((uint16_t)protocolBuf_Usart2[DataLen2 - 2]) << 8) + ((uint16_t)protocolBuf_Usart2[DataLen2 - 1]) ;
        
        uwCRCValue = Calculate_CRC((DataLen2 - 2) , protocolBuf_Usart2);                                       //Get calculated CRC of this frame
        if(uwCRCValue == frameCRC)
        { //CRC match put whole frame into RX pipe      
          RingBuf_WriteBlock((usart2Control).seqMemRX_u32, protocolBuf_Usart2, &DataLen2);  //put fully checked (Valid) Rx frame into Rx Pipe
    /*      if((protocolBuf_Usart2[3]) && (protocolBuf_Usart2[2] != 0x3f))                                                                            //if auto ACK
          {
            unsigned char ackTx[] = {0x55, 0x00, 0x3F, protocolBuf_Usart2[3], 0x00, 0xCC, 0xCC};
            unsigned int TxLen = sizeof(ackTx);
            RingBuf_WriteBlock((usart2Control).seqMemTX_u32, ackTx, &TxLen); 
          }     
 */         
          
          /*********** this part only for testing the Tx message *************/
          /*
          //Echo back the full valid frame to sender
          protocolBuf_Usart2[DataLen2 - 2] = 0xCC;                                                             //Tx frame always use 0xCC as CRC byte, the Tx sending routine should process it with the final CRC value
          protocolBuf_Usart2[DataLen2 - 1] = 0xCC;                                                             //Tx frame always use 0xCC as CRC byte, the Tx sending routine should process it with the final CRC value
          protocolBuf_Usart2[0] = 0x55;                                                                           //Tx frame always use 0x55 as Sync, the Tx sending routine should process it for Leader or Follower
          if(protocolBuf_Usart2[1] & 0xC0)
          {//Advanced frame
            protocolBuf_Usart2[5] = 0x00;                                                                         //Tx frame always use 0xFF as Source Frame ID for ACK require, and 0x00 as non-ACK frame, the Tx sending routine should process with the value for ACK frame        
            protocolBuf_Usart2[6] = 0x01;                                                                         //Tx frame slways put motor address to lower nibble, and leave upper nibble as 0 for Tx routine to put the Header-end value to this field
          }
          else
          {//Normal frame
            protocolBuf_Usart2[3] = 0x00;                                                                         //Tx frame always use 0xFF as Source Frame ID for ACK require, and 0x00 as non-ACK frame, the Tx sending routine should process with the value for ACK frame        
            protocolBuf_Usart2[4] = 0x01;                                                                         //Tx frame slways put motor address to lower nibble, and leave upper nibble as 0 for Tx routine to put the Header-end value to this field
          }
          RingBuf_WriteBlock(usart2SeqMemTX_u32->systemInstanceIndex_u8, protocolBuf_Usart2, &DataLen2);        
*/
          /*********** this part only for testing the Tx message End End End *************/
        }                                
        else
        { 
          DataLen2 = UniHeaderlen;                                                                   //only truncase the header         
        }                                                                  

        RingBuf_ReadBlock((usart2Control).seqMem_InternalPipe_u32, protocolBuf_Usart2, &DataLen2);                              //extract the whole frame         
        #if ENABLE_PROTOCOLBUF_USART2_FIXED_LEN <= 0
        protocolBuf_Usart2 = (unsigned char*) realloc(protocolBuf_Usart2,1);                                          //free heap only leave 1 byte 
        #endif // ENABLE_PROTOCOLBUF_USART2_FIXED_LEN <= 0
        usart2CaptureLen = UniHeaderlen; 
        UniProtocolState = protocolstart;    
        break;
      }

    default:
      {
        usart2CaptureLen = UniHeaderlen; 
        dataLen = 0;
        UniProtocolState = protocolstart;
        break;
      } 
    
  }

}

unsigned char* headerFrameBuf_Usart2;
#define ENABLE_HEADERFRAME_BUF_USART2_FIXED_LEN 1
#if ENABLE_HEADERFRAME_BUF_USART2_FIXED_LEN >= 1
  // This is a one-shot buffer, that is written to and read from in single calls.
  // - it does not currently need to be tracked for current index because of this.
  // #define FIXED_HEADERFRAMEBUF_MAX_LENGTH 21 // Inclusive (this value is accepted) 
  #define FIXED_HEADERFRAMEBUF_USART2_MAX_LENGTH UniHeaderlen // Inclusive (only used to read universal protocol headers)
  unsigned char fixedHeaderFrameBuf_Usart2_Length = 0;
  unsigned char fixedHeaderFrameBuf_Usart2[FIXED_HEADERFRAMEBUF_USART2_MAX_LENGTH];
  unsigned char* headerFrameBuf_Usart2 = fixedHeaderFrameBuf_Usart2;
#else
  unsigned char* headerFrameBuf_Usart2;
#endif


uint8_t TxProcess(void)
{ //process Tx frame in Tx pipe
  int txHeaderSize = UniHeaderlen;

  #if ENABLE_HEADERFRAME_BUF_USART2_FIXED_LEN >= 1
    // length checking not necessary. Since only used to look at universal protocol headers
    fixedHeaderFrameBuf_Usart2_Length = txHeaderSize;
  #else // if ENABLE_HEADERFRAME_BUF_USART2_FIXED_LEN <= 0
    if((headerFrameBuf_Usart2 = (unsigned char*) realloc(headerFrameBuf_Usart2, txHeaderSize)) == NULL) reallocError++;
  #endif
  unsigned int DataLenTx = (unsigned int)txHeaderSize;
  if(indexTx == 0)
  {
    if(RingBuf_GetUsedNumOfElements((usart2Control).seqMemTX_u32) >= txHeaderSize)
    {
      RingBuf_Observe((usart2Control).seqMemTX_u32, headerFrameBuf_Usart2, 0, &DataLenTx);   //pam bug!!!!!!!!!!!!!!!!!!!!                          //copy the whole header frame into buffer
      if(headerFrameBuf_Usart2[1] & 0xc0) 
      {
        DataLenTx = 9; //Advanced frame +CRC
      }
      else
      {
        DataLenTx = 7; //normal frame +CRC
      }
    }
    DataLenTx += ((unsigned int)(headerFrameBuf_Usart2[1] & 0x3F)); 


    #if ENABLE_HEADERFRAME_BUF_USART2_FIXED_LEN <= 0
      headerFrameBuf_Usart2= (unsigned char*) realloc(headerFrameBuf_Usart2, 1);                                   //free heap only leave 1 byte 
    #endif //ENABLE_HEADERFRAME_BUF_USART2_FIXED_LEN <= 0


    #if ENABLE_WHOLEFRAME_USART2_BUF_FIXED_LEN >= 1
      if (DataLenTx <= FIXED_WHOLEFRAME_USART2_MAX_LENGTH) { // Normal Case: Prepare Message for Send
        RingBuf_ReadBlock((usart2Control).seqMemTX_u32, wholeFrameBuf_Usart2, &DataLenTx);                                //copy the complete frame into buffer
      } else { // Overflow Case: Discard All Data
        // Overflow leads to unknown data state, don't send any of it
        // - Clear All Data
        // -- Read into the Header Buffer, just in case the wholeFrameBuf is currently active.
        while (DataLenTx > 0) {
          if (DataLenTx > FIXED_WHOLEFRAME_USART2_MAX_LENGTH) {
            // REVIEW: Replace with RingBuf_ClearContents? Much less processing, but clears whole buffer, and not just one message
            unsigned int read_length = FIXED_WHOLEFRAME_USART2_MAX_LENGTH;
            RingBuf_ReadBlock((usart2Control).seqMemTX_u32, wholeFrameBuf_Usart2, &read_length); //extract the whole frame
            DataLenTx -= FIXED_WHOLEFRAME_USART2_MAX_LENGTH;
          } else {
            RingBuf_ReadBlock((usart2Control).seqMemTX_u32, wholeFrameBuf_Usart2, &DataLenTx); //extract the whole frame
            DataLenTx = 0;
          }        
        }
        // - Exit Gracefully
        return 1;
      }
    #else // if ENABLE_WHOLEFRAME_USART2_BUF_FIXED_LEN <= 0
      wholeFrameBuf_Usart2= (unsigned char*) realloc(wholeFrameBuf_Usart2, DataLenTx);       
      RingBuf_ReadBlock((usart2Control).seqMemTX_u32, wholeFrameBuf_Usart2, &DataLenTx);                                //copy the complete frame into buffer
    #endif

    if(wholeFrameBuf_Usart2[1] &0xC0)
    { //Advanced frame
      wholeFrameBuf_Usart2[6] = (~TxSyncChr & 0xf0) + (wholeFrameBuf_Usart2[6] & 0xf);
      if (wholeFrameBuf_Usart2[5] == 0xFF)                                                                   //process Source Frame ID
      {
          wholeFrameBuf_Usart2[5] = frameID++;
          if (frameID == 0) frameID++;        //preserve 0x00 as non-ACK frame
          ///To Do  setup ACK receive ringbuffer data to fulfil the whole ACK process
          ///end of To Do  setup ACK receive ringbuffer data to fulfil the whole ACK process
      }
    }  
    else
    { //Normal frame
      wholeFrameBuf_Usart2[4] = (~TxSyncChr & 0xf0) + (wholeFrameBuf_Usart2[4] & 0xf);   
      if (wholeFrameBuf_Usart2[3] == 0xFF)                                                                   //process Source Frame ID
      {
          wholeFrameBuf_Usart2[3] = frameID++;
          if (frameID == 0) frameID++;        //preserve 0x00 as non-ACK frame
          ///To Do  setup ACK receive ringbuffer data to fulfil the whole ACK process
          
          ///end of To Do  setup ACK receive ringbuffer data to fulfil the whole ACK process
      } 
    }
    wholeFrameBuf_Usart2[0] = TxSyncChr;
    uwCRCValue = Calculate_CRC((DataLenTx - 2) , wholeFrameBuf_Usart2);    
    wholeFrameBuf_Usart2[DataLenTx - 2] = (unsigned char)((uwCRCValue & 0xff00) >> 8);                  //put calculated CRC back into Tx frame
    wholeFrameBuf_Usart2[DataLenTx - 1] = (unsigned char)(uwCRCValue & 0xff) ;                          //put calculated CRC back into Tx frame
  }    
  
  if((!indexTx) && (LL_USART_IsActiveFlag_TXE(USART2)))
  {
    ubSizeToSend = DataLenTx;      //set TX length                                                      
    /* Start USART transmission : Will initiate TXE interrupt after TDR register is empty */
    LL_USART_TransmitData8(USART2, wholeFrameBuf_Usart2[indexTx++]);                                       //put buffer in
    /* Enable TXE interrupt */
    LL_USART_EnableIT_TXE(USART2);
  }
  /**********************for TX interrupt still using this variable , so don't free it!!!!!!!*******/
  //wholeFrameBuf_Usart2= (unsigned char*) malloc(1);   //for 
  return 0;
}

/**
  * @brief  This function performs CRC calculation on BufSize bytes from input data buffer aDataBuf.
  * @param  BufSize Nb of bytes to be processed for CRC calculation
  * @retval 16-bit CRC value computed on input data buffer
  */
/*uint16_t Calculate_CRC(uint8_t BufSize, unsigned char* aDataBuf)
{
  register uint8_t index = 0;
  LL_CRC_ResetCRCCalculationUnit(CRC);
  // Compute the CRC of Data Buffer array
  for (index = 0; index < BufSize ; index++)
  {
    LL_CRC_FeedData8(CRC,aDataBuf[index] );
  }
  // Return computed CRC value 
  return (LL_CRC_ReadData16(CRC));
}*/


/**
  * @brief  Function called in case of error detected in USART IT Handler
  * @param  None
  * @retval None
  */
void Error_Callback(void)
{
  __IO uint32_t isr_reg;

  /* Disable USARTx_IRQn */
  NVIC_DisableIRQ(USART2_IRQn);

  /* Error handling example :
    - Read USART ISR register to identify flag that leads to IT raising
    - Perform corresponding error handling treatment according to flag
  */
  isr_reg = LL_USART_ReadReg(USART2, ISR);
  if (isr_reg & LL_USART_ISR_NE)
  {
    /* case Noise Error flag is raised : ... */
//    LED_Blinking(LED_BLINK_FAST);
  }
  else
  {
    /* Unexpected IT source : Set LED to Blinking mode to indicate error occurs */
//    LED_Blinking(LED_BLINK_ERROR);
  }
}

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
  /* Check RXNE flag value in ISR register */
  if (LL_USART_IsActiveFlag_RXNE(USART2) && LL_USART_IsEnabledIT_RXNE(USART2))
  {
    /* RXNE flag will be cleared by reading of RDR register (done in call) */
    /* Call function in charge of handling Character reception */
    USART2_CharReception_Callback();
  }
  
  if (LL_USART_IsEnabledIT_TXE(USART2) && LL_USART_IsActiveFlag_TXE(USART2))
  {
    /* TXE flag will be automatically cleared when writing new data in TDR register */

    /* Call function in charge of handling empty DR => will lead to transmission of next character */
    USART2_TXEmpty_Callback();
  }

  if (LL_USART_IsEnabledIT_TC(USART2) && LL_USART_IsActiveFlag_TC(USART2))
  {
    /* Clear TC flag */
    LL_USART_ClearFlag_TC(USART2);
    /* Call function in charge of handling end of transmission of sent character
       and prepare next charcater transmission */
    USART2_CharTransmitComplete_Callback();
  }
  
  if (LL_USART_IsEnabledIT_ERROR(USART2) && LL_USART_IsActiveFlag_NE(USART2))
  {
    /* Call Error function */
    Error_Callback();
  }

  LL_USART_ClearFlag_ORE(USART2); // TODO: Upgrade this temporary patch that keeps us from getting stuck in this ISR

  /* USER CODE END USART2_IRQn 0 */
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}


