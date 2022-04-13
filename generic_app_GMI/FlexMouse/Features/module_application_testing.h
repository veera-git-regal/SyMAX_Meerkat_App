/**
  ********************************************************************************************************************************
  * @file    module_application_testing.h 
  * @author  Myron Mychal
  * @brief   This is a template non-driver app.
  * @details This app does nothing.
  ********************************************************************************************************************************
  */

/* Define to prevent recursive inclusion ---------------------------------------------------------------------------------------*/
#ifndef _MODULE_APPLICATON_TESTING_H_
#define _MODULE_APPLICATON_TESTING_H_

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

/* Setup -----------------------------------------------------------------------------------------------------------------------*/
static Ring_Buf_Handle applicationTestingSeqMem0_u32;
static Ring_Buf_Handle applicationTestingSeqMem1_u32;
static Ram_Buf_Handle applicationTestingStructMem_u32;

#define POLL_TIME (200) // Poll time for the module

/**
  ********************************************************************************************************************************
  * @brief   Module Application Testing control (inside shared memory)
  * @details 
  * @param   internalPipe_u32 
  * @param   errorCode_u8       Error code of this app module.
  ********************************************************************************************************************************
  */

typedef struct
{  
  uint16_t  is_burn_in_enabled:1;       	// If "1" enable analog input for demand
  uint16_t	empty01:1;   
  uint16_t	empty02:1; 
  uint16_t	empty03:1; 
  uint16_t	empty04:1; 
  uint16_t	empty05:1; 
  uint16_t	empty06:1;
  uint16_t	empty07:1; 
  uint16_t	empty08:1; 
  uint16_t	empty09:1;
  uint16_t	empty10:1;   
  uint16_t	empty11:1;
  uint16_t	empty12:1; 
  uint16_t	empty13:1;
  uint16_t	empty14:1;   
  uint16_t	empty15:1;
} Application_Testing_Flags;

struct Application_Testing_Settings
{
  uint16_t	burn_in_current_amplitude_u16;		// amplitude for burn-in test mode current 
  uint16_t	burn_in_current_frequency_u16;		// 
  Application_Testing_Flags flags_u16;
};

typedef struct
{  
  uint16_t	empty01:1;   
  uint16_t	empty02:1; 
  uint16_t	empty03:1; 
  uint16_t	empty04:1; 
  uint16_t	empty05:1; 
  uint16_t	empty06:1;
  uint16_t	empty07:1; 
  uint16_t	empty08:1; 
  uint16_t	empty09:1;
  uint16_t	empty10:1;   
  uint16_t	empty11:1;
  uint16_t	empty12:1;
  uint16_t	empty13:1;
  uint16_t	empty14:1;
  uint16_t	empty15:1;
  uint16_t	empty16:1;  
} Application_Testing_Discretes;

// Live Analog Data
struct Application_Testing_Data
{  
  Application_Testing_Discretes discretes_u16;// discrete bit indicators
};

typedef struct {
    Ring_Buf_Handle internalPipe_u32;
    uint16_t errorCode_u16;
	struct Application_Testing_Settings applicationTesting_Setting;
    struct Application_Testing_Data applicationTesting_Data;
} Application_Testing_Control;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _MODULE_APP_H_ */