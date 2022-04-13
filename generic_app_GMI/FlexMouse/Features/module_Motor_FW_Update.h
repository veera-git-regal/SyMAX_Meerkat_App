/**
  ***************************************************************************************************
  * @file    module_Motor_FW_Update.h 
  * @author  Regal Pamela Lee
  * @version V1.0
  * @date    04-Jan-2021
  * @brief   Header file of Motor firmware update
  * @note    
  ***************************************************************************************************
  */
//^** Tips: APPs/Drivers adding process example step6  [refer to user manual ) 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MODULE_MOTOR_FW_UPDATE_H_
#define _MODULE_MOTOR_FW_UPDATE_H_

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

static Ram_Buf_Handle Motor_FW_UpdateStructMem_u32;

//******************* Module Control (inside shared memory) *******************************************************************************************************************************  
typedef struct{
  uint16_t FWCMD_u16;
  
  //Firmware block number(256byte per block) in current buffer, Bootloader can handle max 256byte per write. 
  //Firmware write will end when this paramter contain a negative value(WrBufBlkNO < -1)
  int16_t WrBufBlkNO_s16;  
  //When Rd_Wr_busy = 1, extrnal user should not alter the content of the read/write buffer, value of WrBufBlkNO
  int16_t RdBufBlkNO_s16;
  uint16_t Rd_Wr_busy_u16;
  uint16_t errorCode_u16;
}Motor_FW_Update_Control;

//******************* end of Modulee Control (inside shared memory) ******************************************************************************************************************************* 
uint8_t FWAutoBaudStateMachine(void);                 //startup target MPU bootloader firmware update 
uint8_t FWGetVerNRdProtection(uint8_t* _FW_RdVerNProtectBuf) ;              //Get version and read protection status
uint8_t FWGetID(uint8_t* _FW_GetIDBuf) ;              //Get Target chip ID
uint8_t FWBlkRead(uint8_t* _FW_RdDatBuf);             //read a block (256byte) of firmware from motor-side and return the read buffer
uint8_t FWGo(uint32_t StartingAddr);                  //Target system start execute at Starting Address
uint8_t FWWrStateMachine(void);                       //write block data into target MPU with or without whole flash erase first


//      local functions
uint8_t XorBuf(uint8_t* _buf, uint16_t _len);
uint8_t FW_Tx(void);
int16_t waitForACK(void);
int8_t waitForDatRd(uint8_t* _FW_RxBuf, uint8_t Len);

#ifdef __cplusplus
}
#endif

#endif /* _MODULE_MOTOR_FW_UPDATE_H_ */

