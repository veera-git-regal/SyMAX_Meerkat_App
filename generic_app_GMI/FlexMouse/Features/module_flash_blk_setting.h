/**
  ***************************************************************************************************
  * @file    module_flash_blk_setting.h 
  * @author  Regal Pamela Lee
  * @version V1.0
  * @date    29-Dec-2020
  * @brief   Header file of Flash setting in block mode
  * @note    
  ***************************************************************************************************
  */
//^** Tips: APPs/Drivers adding process example step6  [refer to user manual ) 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MODULE_FLASH_BLK_SETTING_H_
#define _MODULE_FLASH_BLK_SETTING_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "structured_memory.h"
#include "scheduler.h"
#include "ring_buffer.h"

#ifdef __cplusplus
extern "C" {
#endif

extern Ram_Buf sharedMemArray[STRUCT_MEM_ARRAY_SIZE];
extern ProcessInfo processInfoTable[];

//******************* Module Control (inside shared memory) *******************************************************************************************************************************  
typedef struct{
  uint16_t errorCode_u16;
}Flash_Blk_Setting_Control;

//******************* end of Modulee Control (inside shared memory) ******************************************************************************************************************************* 
  
uint8_t burnSettings(void);                                     //block setting burnning statemachine
uint8_t flashTxBlk(uint16_t _flashBlkNum, unsigned char* _buf);

#ifdef __cplusplus
}
#endif

#endif /* _MODULE_FLASH_BLK_SETTING_H_ */

