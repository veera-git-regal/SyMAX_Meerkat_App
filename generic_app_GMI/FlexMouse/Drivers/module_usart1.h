/**
  ********************************************************************************************************************************
  * @file    module_usart1.h 
  * @author  Pamela Lee
  * @brief   Main driver module for USART2 Communication.
  * @details This module initializes the USART2 port and attaches the pre-selected fixed memory allocation to the module.
  ********************************************************************************************************************************
  */

/* Define to prevent recursive inclusion ---------------------------------------------------------------------------------------*/
#ifndef _MODULE_USART1_H_
#define _MODULE_USART1_H_

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "main.h"
#include "typedef.h"

#include "scheduler.h"
#include "sequential_memory.h"
#include "structured_memory.h"

/* Content ---------------------------------------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

// Constants
#define ModbusHeaderLen 3 //8


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _MODULE_USART1_H_ */