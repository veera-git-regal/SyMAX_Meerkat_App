/**
  ********************************************************************************************************************************
  * @file    driver_spi1.h 
  * @author  Logan Schaufler
  * @brief   Header of Driver function/s for serial protocol with SPI1 hardware
  * @details Protocol SPI1, after decode a whole valid frame from serial port2,
  *          trigger the system control to execute the relative APP in the int stage the Rx data is in spi1SeqMemRX_u32.
  *          To Transmitt data : put data into SPI1TxPipe, and call this function SPI_ITConfig(SPI2, SPI_IT_TXE, ENABLE);
  ********************************************************************************************************************************
  */

/* Define to prevent recursive inclusion ---------------------------------------------------------------------------------------*/
#ifndef _DRIVER_SPI1_H_
#define _DRIVER_SPI1_H_


/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include <stdint.h>

#include "typedef.h"

#include "sequential_memory.h"
#include "structured_memory.h"
#include "scheduler.h"

#include "stm32g0xx_ll_spi.h"
#include "stm32g0xx_it.h"

/* Content ---------------------------------------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* User parameters -------------------------------------------------------------------------------------------------------------*/
#define TX_RX_BUF_SIZE 80

static void MX_SPI1_Init(void);
void SPI1_TransferError_Callback(void);

/* Setup -----------------------------------------------------------------------------------------------------------------------*/
static Ring_Buf_Handle spi1SeqMem_RawRx;
static Ring_Buf_Handle spi1SeqMem_SPIRx;
static Ring_Buf_Handle spi1SeqMem_Tx;
static Ram_Buf_Handle spi1StructMem;

/*
********************************************************************************************************************************
* @brief   SPI1 Control (inside shared memory)
* @details 
* @param   seqMemRX_u32 
* @param   seqMemTX 
* @param   errorCode_u8           Error code of this SPI1 module.
********************************************************************************************************************************
*/
//******************* SPI1 Control (inside structured memory) *******************************************************************************************************************************  
// typedef struct {
//    // Ring_Buf_Handle seqMemRX_u32;
//     Ring_Buf_Handle seqMem_SPIRx;
//     // Ring_Buf_Handle seqMemRXG3_u32;
//     // Ring_Buf_Handle seqMemRXG4_u32;
//     Ring_Buf_Handle seqMemTX;
//     Ring_Buf_Handle seqMem_RawRx;
//     int16_t motorSpeed_s16;
//     uint16_t motorStatus_u16;
//     uint8_t errorCode_u8;
// } SPI1_Control;

// enum           //Universal Protocol state define
// { 
//   protocolstart,                //1 of 3 Rx frame process
//   headerValidate,               //2 of 3 Rx frame process
//   frameCRC                     //3 of 3 Rx frame process
// };

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _DRV_SPI1_H_ */