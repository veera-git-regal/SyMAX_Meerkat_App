/**
  ********************************************************************************************************************************
  * @file    module_spi.h 
  * @author  Logan Schaufler
  * @brief   Main driver module for SPI1 Communication.
  * @details This module initializes the SPI1 port and attaches the pre-selected fixed memory allocation to the module.
  ********************************************************************************************************************************
  */

/* Define to prevent recursive inclusion ---------------------------------------------------------------------------------------*/
#ifndef _MODULE_SPI1_H_
#define _MODULE_SPI1_H_

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "main.h"
#include "typedef.h"

#include "driver_spi1.h"

#include "stm32g0xx_it.h"
//#include "stdint.h"
#include "stm32g0xx_ll_spi.h"
#include "stm32g0xx_ll_exti.h"

#include "scheduler.h"
#include "sequential_memory.h"
#include "structured_memory.h"

//******************* SPI1 Control (inside shared memory) *******************************************************************************************************************************  
// SPI packets are stored here
struct SPI1_Result
{
  uint16_t Rx_buffer[8];
  //uint8_t  errorCode_u8; 
};

//Main structure used by other modules
typedef struct{
 struct SPI1_Result spi1_result; 
}SPI1_Control;



/* Content ---------------------------------------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _MODULE_SPI1_H_ */

