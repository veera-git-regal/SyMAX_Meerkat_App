/**
  ********************************************************************************************************************************
  * @file    drv_usart1.c 
  * @author  Justin Moon
  * @brief   Main Driver function/s for serial protocol with Usart1 hardware
  * @details Protocol Modbus
  *          To Transmitt data : put data into usart1SeqMem_Tx, and call this function
  *                              USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
  ********************************************************************************************************************************
  */

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "driver_usart1.h"
// #include "module_AutoAck.h"
#include "stm32g0xx_ll_crc.h"
#include "main.h"
// #include "stm32f3xx_ll_iwdg.h" // REVIEW only for init, TODO: meerkat "should" handle watchdog, how to deal with init delays... 

/* Private variables ---------------------------------------------------------*/
//UART_HandleTypeDef huart2; //SPA

Usart1_Control usart1Control;

// AutoAck_Control *autoAckControl_u32;             

__IO ITStatus usart1_UsartReady = RESET;
uint8_t usart1_CaptureLen = 0;                     //default Universal Protocol header length
//uint8_t counter = 0;
uint16_t usart1_workingCrcValue = 0;

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN Usart1_Init 0 */

  /* USER CODE END Usart1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
  LL_IOP_GRP1_EnableClock(MODBUS_IOP_GRP1_PERIPH_Port);

  /**USART1 GPIO Configuration  
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX 
  */
  GPIO_InitStruct.Pin = MODBUS_TX_Pin|MODBUS_RX_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = MODBUS_LL_GPIO_AF;
  LL_GPIO_Init(MODBUS_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = MODBUS_EN_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(MODBUS_EN_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN Usart1_Init 1 */

  /* USER CODE END Usart1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);

  LL_USART_DisableIT_CTS(USART1);
  LL_USART_DisableIT_ERROR(USART1); // REVIEW: This should disable the ORE Interrupt, but doesn't appear to
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_SetRxTimeout(USART1, 1152); // (approximately 10 ms) Not using in favor of LL_USART_IsActiveFlag_IDLE
//  LL_USART_SetRxTimeout(USART1, 1728); // (approximately 15 ms) Not using in favor of LL_USART_IsActiveFlag_IDLE
//  LL_USART_SetRxTimeout(USART1, 864); // (approximately 7.5 ms) Not using in favor of LL_USART_IsActiveFlag_IDLE
  // LL_USART_EnableIT_IDLE(USART1); // Idle line detection, interrupt is not required for use of Flag outside of ISRs

  NVIC_SetPriority(USART1_IRQn, 0); // !error Move to main.c, lower priority
  NVIC_EnableIRQ(USART1_IRQn); // !error Move to main.c
  LL_USART_Enable(USART1);
  Set_DE_Mode(MODBUS_FOLLOWER_RX);
  LL_USART_EnableRxTimeout(USART1); 
  // - Note: RxTimeout method is proven working, but using the LL_USART_IsActiveFlag_IDLE instead, since it auto-adjusts to buadrate.
  // -- If we later have problems with customers using 'Lazy Senders' adjust to this method to customize the timeout.
  /* USER CODE END Usart1_Init 1 */
 
 /* USER CODE BEGIN WKUPType USART1 */
  /* USER CODE END WKUPType USART1 */

  // LL_USART_ClearFlag_ORE(USART1);               //reset all usart error bit
  /* Polling USART1 initialisation */
 while((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))))
 {
   //LL_IWDG_ReloadCounter(IWDG); // Kick the watchdog TODO: safety core should ber handling watchdog, but we reset during this...
 } 
  LL_USART_EnableIT_RXNE(USART1);
  /* USER CODE BEGIN Usart1_Init 2 */


  /* USER CODE END Usart1_Init 2 */

}

void Set_DE_Mode(modbus_direction_t direction) {
   if(direction == MODBUS_FOLLOWER_TX) 
  {	
	 LL_GPIO_SetOutputPin(MODBUS_EN_GPIO_Port, MODBUS_EN_Pin); 		// Follower in TX mode
	
  }
  else {	
	  LL_GPIO_ResetOutputPin(MODBUS_EN_GPIO_Port, MODBUS_EN_Pin);  	// Follower in TX mode
  }
}

void Usart1_Init(){
  MX_USART1_UART_Init();
}

//uint8_t count_u8=0;
/**
  * @brief  Function called from USART IRQ Handler when RXNE flag is set
  *         Function is in charge of reading character received on USART RX line.
  * @param  None
  * @retval None
  */
void USART1_CharReception_Callback(void)
{
  /*
  __IO uint32_t received_char;

  // Read Received character. RXNE flag is cleared by reading of RDR register //
  received_char = LL_USART_ReceiveData8(USART1);
  */
 
  unsigned char rxDat2;
  rxDat2 = LL_USART_ReceiveData8(USART1);

  RingBuf_WriteCharacter((usart1Control).seqMem_RawRx,&rxDat2);
  //CLEAR_BIT(USART1->ISR, USART_ISR_RTOF);	// received a good character and thus should be cleared
 
  //RBWrite(usart1SeqMem_RawRx->systemInstanceIndex_u8,&rxDat2); 
  
  /* Echo received character on TX */
 // LL_USART_TransmitData8(USART1, received_char);
  //count_u8++;

}

/******************* variable for Usart1 TX interrupt **************************/
__IO uint8_t indexTx_Usart1 = 0;
uint8_t ubSizeToSend_Uart1 = 0;


#define ENABLE_WHOLEFRAME_USART1_BUF_FIXED_LEN 1
#if ENABLE_WHOLEFRAME_USART1_BUF_FIXED_LEN >= 1
  // This is a one-shot buffer, that is written to and read from in single calls.
  // - it does not currently need to be tracked for current index because of this.
  // #define FIXED_HEADERFRAMEBUF_MAX_LENGTH 21 // Inclusive (this value is accepted) 
  #define FIXED_WHOLEFRAME_USART1_MAX_LENGTH TX_RX_BUF_SIZE // Inclusive (this value is accepted) 
  unsigned char fixedwholeFrameBuf_Usart1_Length = 0;
  unsigned char fixedwholeFrameBuf_Usart1[FIXED_WHOLEFRAME_USART1_MAX_LENGTH];
  unsigned char* wholeFrameBuf_Usart1 = fixedwholeFrameBuf_Usart1;
#else // if ENABLE_WHOLEFRAME_USART1_BUF_FIXED_LEN <= 0
  unsigned char* wholeFrameBuf_Usart1;
#endif


/*******************************************************************************/
/**
  * @brief  Function called for achieving next TX Byte sending
  * @param  None
  * @retval None
  */
void USART1_TXEmpty_Callback(void)
{
  /* Fill TDR with a new char */
  LL_USART_TransmitData8(USART1, wholeFrameBuf_Usart1[indexTx_Usart1++]);
  
  if (indexTx_Usart1 == (ubSizeToSend_Uart1 ))
  {
    /* Disable TXE interrupt */
    LL_USART_DisableIT_TXE(USART1);
    indexTx_Usart1 = 0;
  } 
}

/**
  * @brief  Function called at completion of last byte transmission
  * @param  None
  * @retval None
  */
void USART1_CharTransmitComplete_Callback(void) {
}


//static uint8_t dataLen = 0;                                         //length of data byte/s plus CRC expected
uint8_t ModbusProtocolState = usart1_protocolstart;


#define ENABLE_PROTOCOLBUF_USART1_FIXED_LEN 1
#if ENABLE_PROTOCOLBUF_USART1_FIXED_LEN >= 1
  // This is a one-shot buffer, that is written to and read from in single calls.
  // - it does not currently need to be tracked for current index because of this.
  #define FIXED_PROTOCOLBUF_USART1_MAX_LENGTH USART1_SINGLE_MESSAGE_RX_BUF_SIZE // Inclusive (this value is accepted) 
  unsigned char fixedProtocolBuf_Usart1_Length = 0;
  unsigned char fixedProtocolBuf_Usart1[FIXED_PROTOCOLBUF_USART1_MAX_LENGTH];
  unsigned char* protocolBuf_Usart1 = fixedProtocolBuf_Usart1;
#else
  unsigned char* protocolBuf_Usart1;
#endif

uint16_t usart1IsIdle(void) {
  uint32_t register_value = LL_USART_IsActiveFlag_IDLE(USART1);
  if (register_value) {
    LL_USART_ClearFlag_IDLE(USART1);
    return 1;
  } else {
    return 0;
  }
}

// uint32_t not_idle_counter1 = 0;
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
void protocolHeaderFetch_Usart1(void)
{
  // Copy all data to modbus buffer
  usart1_CaptureLen = RingBuf_GetUsedNumOfElements((usart1Control).seqMem_RawRx);
  Set_DE_Mode(MODBUS_FOLLOWER_TX);	// turn MODBUS to transmission for response
  unsigned int DataLen2 = (unsigned int)usart1_CaptureLen;

  #if ENABLE_PROTOCOLBUF_USART1_FIXED_LEN >= 1
    // Clear Pre-Sync Bytes from the buffer by 'Read'ing them from the RingBuf
    if (usart1_CaptureLen <= FIXED_PROTOCOLBUF_USART1_MAX_LENGTH) { // Normal Case
      fixedProtocolBuf_Usart1_Length = DataLen2;
      // REVIEW: usart1_CaptureLen is calculated from Data Length (and not where the CRC was)
    } else { // Overflow Case
      // Read All Data (Clear the Buffer)
      while (DataLen2 > 0) {
        if (DataLen2 > FIXED_PROTOCOLBUF_USART1_MAX_LENGTH) {
          // REVIEW: Replace with RingBuf_ClearContents? Much less processing
          unsigned int read_length = FIXED_PROTOCOLBUF_USART1_MAX_LENGTH;
          RingBuf_ReadBlock((usart1Control).seqMem_RawRx, protocolBuf_Usart1, &read_length); //extract the whole frame
          DataLen2 -= FIXED_PROTOCOLBUF_USART1_MAX_LENGTH;
        } else {
          unsigned int read_length = DataLen2;
          RingBuf_ReadBlock((usart1Control).seqMem_RawRx, protocolBuf_Usart1, &read_length); //extract the whole frame
          DataLen2 = 0;
        }
      }
      // Exit Gracefully
      return; // Do not copy data for parsing
    }
  #else // ENABLE_PROTOCOLBUF_USART1_FIXED_LEN == 0
    if((protocolBuf_Usart1 = (unsigned char*) realloc(protocolBuf_Usart1,DataLen2)) == NULL) reallocError++;        
  #endif

  // REVIEW: Not length checking here presumes RawRx is not bigger than protocolBuf_Usart1, is that alright?
  RingBuf_ReadBlock((usart1Control).seqMem_RawRx, protocolBuf_Usart1, &DataLen2);   //extract the whole frame
  RingBuf_WriteBlock((usart1Control).seqMem_ModbusRx, protocolBuf_Usart1, &DataLen2); // put fully checked (Valid) Rx frame into Rx Pipe

  #if ENABLE_PROTOCOLBUF_USART1_FIXED_LEN == 0
  if((protocolBuf_Usart1 = (unsigned char*) realloc(protocolBuf_Usart1,1)) == NULL) reallocError++;   // free heap only leave 1 byte 
  #endif // ENABLE_PROTOCOLBUF_USART1_FIXED_LEN == 0

  return;
}


/**
  *@brief   Universal protocol transmit
  * @ usage  To send a frame out user should setup a frame buffer
  *          always start with a 0x55 (no matter Leader or Follower) 
  *            |    count how many data length
  *            |     |   Command of this frame                 
  *            |     |     |   FrameID as 0x00 = non-ack frame or 0xff = auto ack frame
  *            |     |     |     |    put the motor addres for the low 4bits and always 0x0 for the upper 4 bits 
  *            |     |     |     |     |   data for this frame or omit these two byte if no data
  *            |     |     |     |     |     |     |   if this is Auto ack frame put the module ID here, otherwise always leave it as 0xCC
  *            |     |     |     |     |     |     |     |    Always leave as 0xCC
  *            |     |     |     |     |     |     |     |     |
  *          0x55, 0x02, 0x40, 0x00, 0x00, 0xff, 0xff, 0xCC, 0xCC
  *          Then put them into Tx sequential-memory of Usart1, the system will add all the correct sync byte and CRC.
  *          User can put as many frame as the Tx sequential-memory is not full, so please check the Tx sequential-memory status before append new frame.
  * @param  None
  * @retval None
  */
unsigned char* headerFrameBuf_Usart1;

uint8_t TxProcess_Usart1(void)
{ //process Tx frame in Tx pipe
  unsigned int data_size;

  if(indexTx_Usart1 == 0)
  {
    data_size = RingBuf_GetUsedNumOfElements((usart1Control).seqMemTX);
    if (data_size > 0) {
      #if ENABLE_WHOLEFRAME_USART1_BUF_FIXED_LEN >= 1
        if (data_size <= FIXED_WHOLEFRAME_USART1_MAX_LENGTH) { // Normal Case: Prepare Message for Send
          RingBuf_ReadBlock((usart1Control).seqMemTX, wholeFrameBuf_Usart1, &data_size);             //copy the complete frame into buffer
        } else { // Overflow Case: Discard All Data
          // Overflow leads to unknown data state, don't send any of it
          // - Read all data into the temporary header frame buf, so that it is clearly discaraded.
          while (data_size > 0) {
            if (data_size > FIXED_WHOLEFRAME_USART1_MAX_LENGTH) {
              unsigned int read_length = FIXED_WHOLEFRAME_USART1_MAX_LENGTH;
              RingBuf_ReadBlock((usart1Control).seqMemTX, wholeFrameBuf_Usart1, &read_length);             //copy the complete frame into buffer
              data_size -= FIXED_WHOLEFRAME_USART1_MAX_LENGTH;
            } else {
              RingBuf_ReadBlock((usart1Control).seqMemTX, wholeFrameBuf_Usart1, &data_size);             //copy the complete frame into buffer
              data_size = 0;
            }
          }
          // Exit Gracefully
          return 1;  // TODO: 1=ERROR_BUFFER_OVERFLOW, or remove status return
        }
      #else // if ENABLE_WHOLEFRAME_USART1_BUF_FIXED_LEN <= 0
        if((wholeFrameBuf_Usart1 = (unsigned char*) realloc(wholeFrameBuf_Usart1, data_size)) == NULL) reallocError++;       
        RingBuf_ReadBlock((usart1Control).seqMemTX, wholeFrameBuf_Usart1, &data_size);             //copy the complete frame into buffer
      #endif
    }
  }    
  
  if((!indexTx_Usart1) && (LL_USART_IsActiveFlag_TXE(USART1)))
  {
    ubSizeToSend_Uart1 = data_size;      //set TX length                                                      
    /* Start USART transmission : Will initiate TXE interrupt after TDR register is empty */
    LL_USART_TransmitData8(USART1, wholeFrameBuf_Usart1[indexTx_Usart1++]);                                       //put buffer in
    /* Enable TXE interrupt */
    LL_USART_EnableIT_TXE(USART1);
  }
  /**********************for TX interrupt still using this variable , so don't free it!!!!!!!*******/
  //wholeFrameBuf_Usart1 = (unsigned char*) malloc(1);   //for 
  return 0; // TODO: 0=NO_ERROR, or remove status return
}

/**
  * @brief  This function performs CRC calculation on BufSize bytes from input data buffer aDataBuf.
  * @param  BufSize Nb of bytes to be processed for CRC calculation
  * @retval 16-bit CRC value computed on input data buffer
  */
uint16_t Modbus_CalculateCrc(uint16_t BufSize, unsigned char* aDataBuf) {
  register uint16_t index = 0;
  LL_CRC_ResetCRCCalculationUnit(CRC);
  /* Compute the CRC of Data Buffer array*/
  for (index = 0; index < BufSize ; index++)
  {
    LL_CRC_FeedData8(CRC,aDataBuf[index] );
  }
  /* Return computed CRC value */
  return (LL_CRC_ReadData16(CRC));
}


/**
  * @brief  Function called in case of error detected in USART IT Handler
  * @param  None
  * @retval None
  */
void Usart1_ErrorCallback(void)
{
  __IO uint32_t isr_reg;

  /* Disable USARTx_IRQn */
  NVIC_DisableIRQ(USART1_IRQn);

  /* Error handling example :
    - Read USART ISR register to identify flag that leads to IT raising
    - Perform corresponding error handling treatment according to flag
  */
  isr_reg = LL_USART_ReadReg(USART1, ISR);
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
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 26.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  /* Check RXNE flag value in ISR register */
  if (LL_USART_IsActiveFlag_RXNE(USART1) && LL_USART_IsEnabledIT_RXNE(USART1))
  {
    /* RXNE flag will be cleared by reading of RDR register (done in call) */
    /* Call function in charge of handling Character reception */
    USART1_CharReception_Callback();
  }
  
  if (LL_USART_IsEnabledIT_TXE(USART1) && LL_USART_IsActiveFlag_TXE(USART1))
  {
    /* Enable Transmission Complete Interrupt flag */
    LL_USART_EnableIT_TC(USART1); 
    /* TXE flag will be automatically cleared when writing new data in TDR register */

    /* Call function in charge of handling empty DR => will lead to transmission of next character */
    
    USART1_TXEmpty_Callback();
  }

  if (LL_USART_IsEnabledIT_TC(USART1) && LL_USART_IsActiveFlag_TC(USART1))
  {
    Set_DE_Mode(MODBUS_FOLLOWER_RX); // Set MODBUS_EN pin to enable reciveing data from leader
    /* Clear TC flag */
    LL_USART_ClearFlag_TC(USART1);
    /* Call function in charge of handling end of transmission of sent character
       and prepare next charcater transmission */
    USART1_CharTransmitComplete_Callback();
  }
  
  if (LL_USART_IsEnabledIT_ERROR(USART1) && LL_USART_IsActiveFlag_NE(USART1))
  {
    /* Call Error function */
    Usart1_ErrorCallback();
  }
  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */
  LL_USART_ClearFlag_ORE(USART1); // TODO: Upgrade this temporary patch that keeps us from getting stuck in this ISR

  /* USER CODE END USART1_IRQn 1 */
}

void DriverUsart1_SetBaudRate(uint16_t baud_rate_u16)
{
   
}

