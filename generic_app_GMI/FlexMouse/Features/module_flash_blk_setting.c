/**
***************************************************************************************************
* @file    module_flash_blk_setting.c 
* @author  Regal Pamela Lee
* @version V1.0
* @date    29-Dec-2020
* @brief   Main function/s of Flash setting in block mode
* @note   
***************************************************************************************************
*/

#include "module_flash_blk_setting.h"
#include "driver_usart2.h"


extern Ram_Buf sharedMemArray[STRUCT_MEM_ARRAY_SIZE];
extern ProcessInfo processInfoTable[];


Usart2_Control* usart2Control_FlashBlkSetting;
Flash_Blk_Setting_Control *flash_blk_setting_Control;

enum                                                                            //Default APPs/Driver stage template
{ 
  MEMORY_INIT_MODULE,
  AppInit,
  AppStart,
  //any other stage in here !!!
  blk_setting_burnning,                                 //burnning flash setting in block mode
  blk_setting_Read,
  flash_format_request,                                  //Flash setting block read

  //above 200 will be all interrupt for this APP
  AppIrq = 200,
  killApp = 255
};

/** Setting update to Motor side parameters **/
unsigned char SettingBlk[] = {0x0f,  0x00,  0xff,  0xff,  0xff,  0xff,  0x64,  0x15,  0xd0,  0x07,  0x00,  0x00,  0x3c,  0x06,  0x71,  0x00, 0x10,  0x27,  0x61,  0x08,  0xa9,  0x07,  0x61,  0x08,  0xa9,  0x07,  0x4c,  0x1d,  0xf4,  0x01,  0x64,  0x15,\
                              0x00,  0x00,  0x26,  0x02,  0x23,  0x00,  0x5a,  0x00,  0xff,  0xff,  0xe8,  0x03,  0x00,  0x00,  0x8e,  0x08, 0x88,  0x13,  0xa6,  0x00,  0x8e,  0x08,  0x00,  0x00,  0xa6,  0x00,  0x8e,  0x08,  0x00,  0x00,  0xa6,  0x00,\
                              0x8e,  0x08,  0x00,  0x00,  0xa6,  0x00,  0x8e,  0x08,  0x19,  0x00,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff, 0x18,  0xa1,  0xb3,  0x6a,  0x2c,  0x01,  0xc4,  0x09,  0x4b,  0x00,  0xff,  0xff,  0xff,  0xff,  0x04,  0x00,\
                              0xe8,  0x03,  0x06,  0x00,  0xd0,  0x07,  0x10,  0x27,  0xe8,  0x03,  0x0a,  0x00,  0xc8,  0x00,  0xb8,  0x0b, 0x0a,  0x00,  0xc8,  0x00,  0x21,  0x00,  0x0a,  0x00,  0x30,  0x75,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff};
unsigned char SettingBlk63[]= {0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff, 0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0x90,  0x08};
              
typedef enum
{ //block setting burnning stateMachine
  start_blk,                                            //start with the last block(block63)
  start_blk_ack,
  current_blk,                                          //loop through all blocks
  end_blk                                               //finalize the last block (block0)
}settingBlkWrite;

#define flashBlockSize 32
//#define FLASH_PAGE_SIZE 0x800
uint16_t blkIndex = FLASH_PAGE_SIZE/flashBlockSize;
settingBlkWrite settingBlkWriteStateMachine = start_blk;
uint64_t blkAcked = 0;
/** Setting update to Motor side parameters end**/

//^**Tips: APPs/Drivers adding process example step7 (Add the Additional funtion itself)
//uint8_t _App_template(uint8_t appID, uint8_t previousStage, uint8_t nextStage, uint8_t interruptIdentfer)  
uint8_t module_Flash_Blk_Setting_u32(uint8_t drv_id_u8, uint8_t prev_state_u8, uint8_t next_state_u8,
                                     uint8_t irq_id_u8)
{ 
  uint8_t     return_state_u8 = MEMORY_INIT_MODULE; 
  switch (next_state_u8)
  {
  case MEMORY_INIT_MODULE:
    return_state_u8 = AppInit;
    break;
  case AppInit:                                                              //initial stage
    {             
      /*Attach Uart2 structured memory into this App*/
      //uint8_t Usart2index  = getProcessInfoIndex(DRV_USART2); //return Process index from processInfo array          
      uint8_t Usart2index  = getProcessInfoIndex(MODULE_USART2); //return Process index from processInfo array
      usart2Control_FlashBlkSetting = (Usart2_Control*)((*(processInfoTable[Usart2index].Sched_DrvData.p_masterSharedMem_u32)).p_ramBuf_u8);    //Get structured memory for USART2
      
      //       tt_DemandTime = getSysCount() + DemandPollPeriod;                          //store time tick value 
      return_state_u8 = AppStart;
      //      return_state_u8 = blk_setting_Read; 
      break;
    }       
  case AppStart:
    { 
      
      
      return_state_u8 = AppStart ;
      break;
    }
  case blk_setting_burnning:
    {
      if(burnSettings()){ 
        return_state_u8 = AppStart ;
      } else {   
        return_state_u8 = blk_setting_burnning ;
      }
      break;
    }
  case blk_setting_Read:
    {
      uint16_t blkNum = 3;
      unsigned char FlashBlkRequest[] = {0x55, 0x02, 0x79, 0x00, 0x00, blkNum >> 8, blkNum, 0xcc, 0xcc};
      unsigned int TxLen = sizeof(FlashBlkRequest);
      RingBuf_WriteBlock((*usart2Control_FlashBlkSetting).seqMemTX_u32, FlashBlkRequest, &TxLen);  
      return_state_u8 = AppStart ;
      break;
    }            
  case flash_format_request:
    { //request flash data format, total number of sector for flash setting and total number of blocks      
      unsigned char blkResetAck[] = {0x55, 0x01, 0x7A, 0x00, 0x00, 0x07, 0xCC, 0xCC}; //
      unsigned int blkSettingLen = sizeof(blkResetAck);  //blkSettingTx);
      RingBuf_WriteBlock((*usart2Control_FlashBlkSetting).seqMemTX_u32, blkResetAck, &blkSettingLen);    
      return_state_u8 = AppStart ;
      break;
    }
  case AppIrq:
    {
      break;
    }               
  case killApp:
    {
      return_state_u8 = AppInit;
      break;
    }
  default:
    {
      return_state_u8 = killApp;   
      break;
    }
  }
  return return_state_u8;
}

uint8_t burnSettings(void)
{
  switch(settingBlkWriteStateMachine)
  {
  case start_blk:
    { //         |- if block number larger than 0x8000 means this is the erase block with CRC, so the normal block of all 0xff can be omitted
      //         V
      flashTxBlk(0x803f, SettingBlk63);                      //send the last block of data will first erase the flash and fill up the last block
      blkIndex = 63;                                                 // put the last block number for not 0xFF of the whole 32 bytes
      settingBlkWriteStateMachine = start_blk_ack;
      break;
    }    
  case start_blk_ack:
    {
      if ( (blkAcked >> blkIndex) &0x01)
      {
        blkIndex = 4; 
        blkAcked |= 0x10;
        settingBlkWriteStateMachine = current_blk;
      }
      break;
    }
  case current_blk:
    {
      if ( (blkAcked >> blkIndex) &0x01)    //check the block relate bit bit0= block0, bit1 = block1 ...... bit 63- block63 acknowledge
      {
        if(blkIndex--){
          flashTxBlk(blkIndex,  SettingBlk);
        } else {        
          settingBlkWriteStateMachine = end_blk;          
        }  
      }
      break;
    }
  case end_blk:
    {
      unsigned char blkResetAck[] = {0x55, 0x01, 0x7A, 0x00, 0x00, 0x07, 0xCC, 0xCC}; //block flash reset acknowledgement
      unsigned int blkSettingLen = sizeof(blkResetAck);  //blkSettingTx);
      RingBuf_WriteBlock((*usart2Control_FlashBlkSetting).seqMemTX_u32, blkResetAck, &blkSettingLen);     
      
      settingBlkWriteStateMachine = start_blk; 
      return 1;
      break;
    }   
  default:
    settingBlkWriteStateMachine = start_blk; 
  }
  return 0;
}

uint8_t flashTxBlk(uint16_t _flashBlkNum, unsigned char* _buf)
{
  uint16_t blkStartAddr = _flashBlkNum*flashBlockSize;
  if ( _flashBlkNum > 0x8000) //if flashBlkNum > 0x8000 mean this is an absolute block number 
  {
    blkStartAddr = 0;
    _flashBlkNum &= 0xfff;
  }
  uint16_t uwCRCValue = Calculate_CRC(32, _buf + blkStartAddr);          //only flash data CRC, no block pointer.
  unsigned char blkSettingTx[] = {0x55, 0x24, 0x78, 0x00, 0x00, (_flashBlkNum & 0xff00) >> 8, _flashBlkNum & 0xff,\
                                  _buf[ 0 + blkStartAddr],  _buf[1 + blkStartAddr],  _buf[2 + blkStartAddr],  _buf[3 + blkStartAddr],  _buf[4 + blkStartAddr],  _buf[5 + blkStartAddr],  _buf[6 + blkStartAddr],  _buf[7 + blkStartAddr],  _buf[8 + blkStartAddr],  _buf[9 + blkStartAddr],\
                                  _buf[10 + blkStartAddr], _buf[11 + blkStartAddr], _buf[12 + blkStartAddr], _buf[13 + blkStartAddr], _buf[14 + blkStartAddr], _buf[15 + blkStartAddr], _buf[16 + blkStartAddr], _buf[17 + blkStartAddr], _buf[18 + blkStartAddr], _buf[19 + blkStartAddr],\
                                  _buf[20 + blkStartAddr], _buf[21 + blkStartAddr], _buf[22 + blkStartAddr], _buf[23 + blkStartAddr], _buf[24 + blkStartAddr], _buf[25 + blkStartAddr], _buf[26 + blkStartAddr], _buf[27 + blkStartAddr], _buf[28 + blkStartAddr], _buf[29 + blkStartAddr],\
                                  _buf[30 + blkStartAddr], _buf[31 + blkStartAddr],\
                                  (uwCRCValue & 0xff00) >> 8, uwCRCValue & 0xff, 0xCC, 0xCC};
  
  unsigned int blkSettingLen = sizeof(blkSettingTx);
  RingBuf_WriteBlock((*usart2Control_FlashBlkSetting).seqMemTX_u32, blkSettingTx, &blkSettingLen);
  
  return 0;
}