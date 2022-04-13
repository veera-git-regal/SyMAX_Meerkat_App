/**
  ********************************************************************************************************************************
  * @file    driver_usart2.h 
  * @author  Pamela Lee
  * @brief   Header of Driver function/s for serial protocol with Usart2 hardware
  * @details Protocol Usart2, after decode a whole valid frame from serial port2,
  *          trigger the system control to execute the relative APP in the int stage the Rx data is in usart2SeqMemRX_u32.
  *          To Transmitt data : put data into Usart1TxPipe, and call this function USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
  ********************************************************************************************************************************
  */

/* Define to prevent recursive inclusion ---------------------------------------------------------------------------------------*/
#ifndef _DRV_USART1_H_
#define _DRV_USART1_H_

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "typedef.h"

#include "sequential_memory.h"
#include "structured_memory.h"
#include "scheduler.h"

/* Content ---------------------------------------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

typedef enum           // Direction of communication in half duplex mode
{ 
  MODBUS_FOLLOWER_RX = 0,  // Follower is receiving
  MODBUS_FOLLOWER_TX = 1,  // Follower is transmitting
} modbus_direction_t;  
  
/* Prototypes ------------------------------------------------------------------------------------------------------------------*/
void Set_DE_Mode(modbus_direction_t direction);

/* User parameters -------------------------------------------------------------------------------------------------------------*/
// - Buffer Sizes
#define TX_RX_BUF_SIZE 80 // REVIEW: This define is duplicated across multiple drivers, rename so they are individual?
#define USART1_SINGLE_MESSAGE_RX_BUF_SIZE 80 // REVIEW: What size do we need for flash maps?
#define USART1_SINGLE_MESSAGE_TX_BUF_SIZE 80 // REVIEW: What size do we need for flash maps?

// - Communication Properties
#define	ACTIVE_HIGH			1
#define	ACTIVE_LOW 			0  
#define	MODBUS_DE_FOLLOWER_TX	ACTIVE_HIGH  // state of pin to enable follower TX mode for half duplex
#define MODBUS_DE_FOLLOWER_RX 	ACTIVE_LOW   // state of pin to enable follower RX mode for half duplex
// #define	MODBUS_DE_PIN_NUM	12
  

/* Setup -----------------------------------------------------------------------------------------------------------------------*/
static Ring_Buf_Handle usart1SeqMem_RawRx;
static Ring_Buf_Handle usart1SeqMem_ModbusRx;
static Ring_Buf_Handle usart1SeqMem_Tx;
static Ram_Buf_Handle usart1StructMem;

/**
  ********************************************************************************************************************************
  * @brief   Uart2 Control (inside shared memory)
  * @details 
  * @param   seqMemRX_u32 
  * @param   seqMemTX 
  * @param   errorCode_u8           Error code of this USART2 module.
  ********************************************************************************************************************************
  */
//******************* Uart2 Control (inside structured memory) *******************************************************************************************************************************  
typedef struct {
    Ring_Buf_Handle seqMem_ModbusRx;
    Ring_Buf_Handle seqMemTX;
    Ring_Buf_Handle seqMem_RawRx;
    int16_t motorSpeed_s16;
    uint16_t motorStatus_u16;
    uint8_t errorCode_u8;
} Usart1_Control;
//******************* end of Uart1 Control (inside structured memory) ******************************************************************************************************************************* 

enum           //Universal Protocol state define
{ 
  usart1_protocolstart,   // 1 of 3 Rx frame process
  usart1_headerValidate,  // 2 of 3 Rx frame process
  usart1_frameCRC         // 3 of 3 Rx frame process
};

/* Function Declarations -------------------------------------------------------------------------------------------------------*/
/**
  ********************************************************************************************************************************
  * @brief   USART2 initialization function.
  * @details Configure peripheral clocks. Configure hardware TX/RX pins. Configure USART2 interrupt.
  *             PA2 -> USART2_TX, PA3 -> USART2_RX 
  ********************************************************************************************************************************
  */
void MX_USART1_UART_Init(void);

/**
  ********************************************************************************************************************************
  * @brief   Returns 
  * @details
  * @return  Returns
  ********************************************************************************************************************************
  */
// uint8_t Usart1_IRQCallback(void);

// void USART_CharReception_Callback(void);
// void USART_TXEmpty_Callback(void);
// void USART_CharTransmitComplete_Callback(void);
void protocolHeaderFetch_Usart1(void);
uint8_t TxProcess_Usart1(void);
uint16_t Modbus_CalculateCrc(uint16_t BufSize, unsigned char* aDataBuf);
void Usart1_ErrorCallback(void);
// void assign_UART1_DrvMem(void);
void Usart1_Init(void);
void USART1_IRQHandler(void);



#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _DRV_USART2_H_ */