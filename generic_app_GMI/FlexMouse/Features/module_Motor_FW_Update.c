/**
***************************************************************************************************
* @file    module_Motor_FW_Update.c 
* @author  Regal Pamela Lee
* @version V1.0
* @date    04-Jan-2021
* @brief   Main function/s of Motor firmware update
* @note    
***************************************************************************************************
*/

#include "module_Motor_FW_Update.h"
#include "driver_usart2.h"

#define ADDR_FLASH_PAGE_0     ((uint32_t)0x08000000) // Base address of Page 0, 2 Kbytes //
#define NumOfFWTimeOutLoop 100 

extern Ram_Buf sharedMemArray[STRUCT_MEM_ARRAY_SIZE];
extern ProcessInfo processInfoTable[];

Usart2_Control* usart2Control_MotorFW_Update;
Motor_FW_Update_Control motor_FW_Update_Control;

uint64_t tt_FW_TxDelayTime;
#define DemandFWDelayPeriod 2;                //tx delay wait period 2ms

uint64_t tt_FW_WaitTime;
#define DemandFWWaitPeriod 100;              //Motor reset waiting period after reset 500ms

int16_t previousWrittenBlkNO = -1;           //for 

/** Firmware update to Target data  8x32 **/
uint8_t FW_DatBuf[] = {0xff,  0x0f,  0x00,  0xff,  0xff,  0xff,  0xff,  0x64,  0x15,  0xd0,  0x07,  0x00,  0x00,  0x3c,  0x06,  0x71,  0x00, 0x10,  0x27,  0x61,  0x08,  0xa9,  0x07,  0x61,  0x08,  0xa9,  0x07,  0x4c,  0x1d,  0xf4,  0x01,  0x64,  0x15,\
                       0x00,  0x00,  0x26,  0x02,  0x23,  0x00,  0x5a,  0x00,  0xff,  0xff,  0xe8,  0x03,  0x00,  0x00,  0x8e,  0x08, 0x88,  0x13,  0xa6,  0x00,  0x8e,  0x08,  0x00,  0x00,  0xa6,  0x00,  0x8e,  0x08,  0x00,  0x00,  0xa6,  0x00,\
                       0x8e,  0x08,  0x00,  0x00,  0xa6,  0x00,  0x8e,  0x08,  0x19,  0x00,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff, 0x18,  0xa1,  0xb3,  0x6a,  0x2c,  0x01,  0xc4,  0x09,  0x4b,  0x00,  0xff,  0xff,  0xff,  0xff,  0x04,  0x00,\
                       0xe8,  0x03,  0x06,  0x00,  0xd0,  0x07,  0x10,  0x27,  0xe8,  0x03,  0x0a,  0x00,  0xc8,  0x00,  0xb8,  0x0b, 0x0a,  0x00,  0xc8,  0x00,  0x21,  0x00,  0x0a,  0x00,  0x30,  0x75,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,\
                       0x0f,  0x00,  0xff,  0xff,  0xff,  0xff,  0x64,  0x15,  0xd0,  0x07,  0x00,  0x00,  0x3c,  0x06,  0x71,  0x00, 0x10,  0x27,  0x61,  0x08,  0xa9,  0x07,  0x61,  0x08,  0xa9,  0x07,  0x4c,  0x1d,  0xf4,  0x01,  0x64,  0x15,\
                       0x00,  0x00,  0x26,  0x02,  0x23,  0x00,  0x5a,  0x00,  0xff,  0xff,  0xe8,  0x03,  0x00,  0x00,  0x8e,  0x08, 0x88,  0x13,  0xa6,  0x00,  0x8e,  0x08,  0x00,  0x00,  0xa6,  0x00,  0x8e,  0x08,  0x00,  0x00,  0xa6,  0x00,\
                       0x8e,  0x08,  0x00,  0x00,  0xa6,  0x00,  0x8e,  0x08,  0x19,  0x00,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff, 0x18,  0xa1,  0xb3,  0x6a,  0x2c,  0x01,  0xc4,  0x09,  0x4b,  0x00,  0xff,  0xff,  0xff,  0xff,  0x04,  0x00,\
                       0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff, 0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0x90,  0x08};
         

void AssignModuleMemMotorFWUpdate(void);

enum                                                                            //Default APPs/Driver stage template
{ 
  MEMORY_INIT_MODULE,
  AppInit,
  AppStart,
  //any other stage in here !!!
  CMDGetVersionNRdProtect,
  exeGetVersionNRdProtect,
  
  CMDGetID,
  exeGetID,
  
  CMDBlkRead,
  exeBlkRead,
  
  CMDGo,
  exeGo,
  
  CMDBlkWrite,          //block write cmd group
  exeBlkwrite,          //block write cmd group
  
  //above 200 will be all interrupt for this APP
  AppIrq = 200,
  killApp = 255
};

uint16_t TXFWDatLen = 0;
uint8_t* _txFWdata;
uint16_t TXFWDatIndx = 0;

typedef enum                    //ST bootloader startup stateMachine                                                     
{ 
  PreFwStartUp,
  AutoBaudFmBoot,
  ABaudWaitAckAfterReset,
  
  ABaudSuccess,
  ABaudFail
}FWAutoBaudState;
FWAutoBaudState fwAutoBaudState;

typedef enum                    //ST bootloader version and read protection status stateMachine                                                     
{ 
  PreFwRdVerNProtect,
  waitAckRdVerNProtect,
  FwRdVerNProtectDat,
  RdVerNProtectSuccess,
  RdVerNProtectFail
}FWRdVerNProtectState;
FWRdVerNProtectState fwRdVerNProtectState;

typedef enum                    //ST bootloader version and read protection status stateMachine                                                     
{ 
  PreFwGetID,
  waitAckGetID,
  FwGetIDDat,
  GetIDSuccess,
  GetIDFail
}FWGetIDState;
FWGetIDState fwGetIDState;

typedef enum                    //ST bootloader data read stateMachine                                                     
{ 
  PreFwRd,
  waitAckRdFw,
  FwRdAddr,
  waitAddrAckRdFw,
  FwRdLen,
  waitLenAckRdFw,
  FwRdData,
  RdSuccess,
  RdFail
}FWRdState;
FWRdState fwRdState;

typedef enum                    //ST bootloader go execute target stateMachine                                                     
{ 
  PreFwGo,
  waitAckGoFw,
  FwGoAddr,
  waitAddrAckGoFw,
  GoSuccess,
  GoFail
}FWGoState;
FWGoState fwGoState;

typedef enum                    //Write flash stateMachine                                           
{ 
  WrFw_WithErase,  
  WaitAckErase,
  WholeFlashErase,
  WaitAckEraseFinal,
  
  WrFw_WithOutErase,
  waitAckWrFw,
  WrFwAddr,
  waitAddrAckWrFw,
  WrFwData,
  waitAckWrFwFinal,
  
  WrProtect,
  JumpStart,
  FWWrSuccess,
  FWWrFail
}FWWrState;
FWWrState fwWrState;



uint8_t* FWRddataBuf; 
uint8_t* FWGetIDBuf; 
uint8_t* FW_RdVerNProtectBuf;
//^**Tips: APPs/Drivers adding process example step7 (Add the Additional funtion itself)
uint8_t module_Motor_FW_Update_u32(uint8_t module_id_u8, uint8_t prev_state_u8, uint8_t next_state_u8,
                                   uint8_t irq_id_u8)
{ 
  uint8_t return_state_u8 = MEMORY_INIT_MODULE; 
  if((TXFWDatIndx) && (_txFWdata !=NULL)) FW_Tx();                   //Tx routine will send out all data,
  
  switch (next_state_u8)
  {
  case MEMORY_INIT_MODULE:
    {
      AssignModuleMemMotorFWUpdate(); // Assign structured memory to Analog 0-10V setting and data 
      return_state_u8 = AppInit;
      break;
    }
  case AppInit:                                                              //initial stage
    {     
      //init this module 
      //AssignModuleMemMotorFWUpdate();
      ////Motor_FW_UpdateStructMem_u32 =  StructMem_CreateInstance(MODULE_MOTOR_FW_UPDATE, sizeof(Motor_FW_Update_Control), ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST); //System call create a structured memory for this driver [should map it back to this driver local struct]
      ////(*Motor_FW_UpdateStructMem_u32).p_ramBuf_u8 = (uint8_t *)&motor_FW_Update_Control ;    //map the generated module's control memory into the structured memory
      ////uint8_t motor_FW_Update_Index = getProcessInfoIndex(MODULE_MOTOR_FW_UPDATE);
      ////processInfoTable[motor_FW_Update_Index].Sched_ModuleData.p_masterSharedMem_u32 = Motor_FW_UpdateStructMem_u32;     //also map it back to module_paramters under kernel    
      
      
      /*Attach Uart2 structured memory into this App*/ 
      uint8_t Usart2index  = getProcessInfoIndex(MODULE_USART2); //return Process index from processInfo array
      usart2Control_MotorFW_Update = (Usart2_Control*)((*(processInfoTable[Usart2index].Sched_DrvData.p_masterSharedMem_u32)).p_ramBuf_u8);    //Get structured memory for USART2   
      
      
      motor_FW_Update_Control.WrBufBlkNO_s16 = 0;
      
      /** serial handling
      //rx part
      unsigned char* FW_Updatebuf;
      unsigned int DataLen2 = (unsigned int)UniHeaderlen;
      RingBuf_Observe((*usart2Control_MotorFW_Update).seqMemRX_transparentMode_u32, FW_Updatebuf, 0, &DataLen2);
      RingBuf_ReadBlock((*usart2Control_MotorFW_Update).seqMemRX_transparentMode_u32, FW_Updatebuf, &DataLen2); //extract the whole frame
      
      usart2Control_MotorFW_Update->UsartMode_u8 = 1; 
      static uint8_t TestDat[] = {0x08, 0x00, 0xff, 0xee};
      _txFWdata = TestDat;
      //  _txFWdata= (uint8_t*) realloc(_txFWdata, sizeof(TestDat));
      
      TXFWDatLen = 4;
      TXFWDatIndx = TXFWDatLen;
      tt_FW_TxDelayTime = getSysCount();                          //store time tick value 
      FW_Tx(); 
      
      **/     
      
      
      
      //       tt_FW_TxDelayTime = getSysCount() + DemandFWDelayPeriod;                          //store time tick value 
      return_state_u8 = AppStart;
      break;
    }       
  case AppStart:
    { 
      // just set return_state_u8 to the command to perform(like CMDBlkRead, CMDBlkWrite,CMDGetVersionNRdProtect ..... ,CMDGo)  , then the related stateMachine will be executed.
      //   return_state_u8 = CMDBlkWrite; //direct test for the write with erase command
      return_state_u8 = AppStart;
      break;
    }  
    
  case CMDBlkRead: /*****************************************************block Read cmd group****************************************************/
    { 
      fwRdState = PreFwRd;
      fwAutoBaudState = PreFwStartUp;
      FWRddataBuf = (uint8_t*) malloc(255);        //setup receive data buffer
    }  
  case exeBlkRead:
    {         
      if(FWAutoBaudStateMachine() == (FWAutoBaudState)ABaudSuccess)
      {
        FWBlkRead(FWRddataBuf);
      }
      return_state_u8 = CMDBlkRead;
      break;
    }   /********************************************************** End of block Read cmd group****************************************************/
    
  case CMDBlkWrite: /*****************************************************block write cmd group****************************************************/
    {
      fwAutoBaudState = PreFwStartUp;              
      fwWrState = WrFw_WithErase;
    }
  case exeBlkwrite:                                 
    {
      if(FWAutoBaudStateMachine() == (FWAutoBaudState)ABaudSuccess)
      {
        FWWrStateMachine();
      }
      return_state_u8 = exeBlkwrite ;
      break;
    }   /************************************************************ End of block write cmd group***************************************************/                                            
    
  case CMDGetVersionNRdProtect: /****************************Get Target boolder version and read protection status ID group**************************/
    {
      fwGetIDState = PreFwGetID;
      FW_RdVerNProtectBuf = (uint8_t*) malloc(3);        //setup Target boolder version and read protection status data buffer
    }
  case exeGetVersionNRdProtect:
    {
      if(FWAutoBaudStateMachine() == (FWAutoBaudState)ABaudSuccess)
      {
        FWGetVerNRdProtection(FW_RdVerNProtectBuf) ;
      }
      return_state_u8 = exeGetVersionNRdProtect;
      break;
    }       /***************************************** End of Get Target boolder version and read protection status ID group**************************/
    
  case CMDGetID:    /*****************************************************Get Target ID group********************************************************/
    {
      fwAutoBaudState = PreFwStartUp;
      fwGetIDState = PreFwGetID;
      FWGetIDBuf = (uint8_t*) malloc(3);        //setup Target ID data buffer
    }
  case exeGetID:
    {
      if(FWAutoBaudStateMachine() == (FWAutoBaudState)ABaudSuccess)
      {
        FWGetID(FWGetIDBuf);
      }
      return_state_u8 = exeGetID ;
      break;
    }       /***************************************************** End of Get Target ID group********************************************************/
    
  case CMDGo:       /***************************************************** Go execute in Target group************************************************/
    {
      fwAutoBaudState = PreFwStartUp;
      fwGoState = PreFwGo;
    }
  case exeGo:
    {
      if(FWAutoBaudStateMachine() == (FWAutoBaudState)ABaudSuccess)
      {
        // FWGo(StartingAddr); //need to define the starting address
      }
      return_state_u8 = exeGo;
    }       /***************************************************** End of Go execute in Target group************************************************/
    
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
    return_state_u8 = killApp;   
  }
  return return_state_u8;
}

void AssignModuleMemMotorFWUpdate(void)
{
  Motor_FW_UpdateStructMem_u32 =  StructMem_CreateInstance(MODULE_MOTOR_FW_UPDATE, sizeof(Motor_FW_Update_Control), ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST); //System call create a structured memory for this driver [should map it back to this driver local struct]
  (*Motor_FW_UpdateStructMem_u32).p_ramBuf_u8 = (uint8_t *)&motor_FW_Update_Control ;    //map the generated module's control memory into the structured memory
  uint8_t motor_FW_Update_Index = getProcessInfoIndex(MODULE_MOTOR_FW_UPDATE);
  processInfoTable[motor_FW_Update_Index].Sched_ModuleData.p_masterSharedMem_u32 = Motor_FW_UpdateStructMem_u32;     //also map it back to module_paramters under kernel    
}

/** @section below -------------------------------------------------- All Local stateMachine functions (below this line)------------------------------------------------------------------
* @author  Pamela Lee
* @date    9 Jan 2021
* @version 1.0
* @brief   Contain all the stateMachine functions for each of the ST-bootloader commands 
*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
**/
int16_t FW_TimeOutCounter = 0;
/**
********************************************************************************************************************************
* @brief   Function of AutoBaud setup for firmware into target flash.
* @author  Pamela Lee
* @details - flash data from input buffer into target flash
*          - The finction will run only once to either ABaudSuccess or ABaudFail stage
- if re-execute this function again, user need to reload the PreFwWr into fwAutoBaudState
- otherwise even keep running this function will not perform any further action!!!!!!
* @param   fwAutoBaudState  -- user should enter the startup state(PreFwWr) before execute this function
* @return  fwAutoBaudState  -- finially will end with either (ABaudSuccess or ABaudFail)
********************************************************************************************************************************
*/
uint8_t FWAutoBaudStateMachine(void)
{
  switch(fwAutoBaudState)
  {
  case PreFwStartUp :                  
    {
      /** todo1 setup boot0 pin **/
      /** todo2 send universal protocol for reboot motor **/      
      tt_FW_WaitTime = getSysCount() + 1000;                          //store time tick value for target reset time
      fwAutoBaudState = AutoBaudFmBoot;
      break;
    }
  case AutoBaudFmBoot:
    {
      if (getSysCount() >= tt_FW_WaitTime){
        usart2Control_MotorFW_Update->UsartMode_u8 = 1;                                                     //SET App-side Usart2 as transparent mode. All data will directly push into seqMemRX_transparentMode_u32 pipe  
        RingBuf_ClearContents((*usart2Control_MotorFW_Update).seqMemRX_transparentMode_u32);                //flush transparent mode ringbuffer
        // prepare data for tx
        _txFWdata= (uint8_t*) malloc(1);
        _txFWdata[0] = 0x7F;                                        //setup 115200 baud for motor bootloader with autobaud char 0x7F
        TXFWDatLen = 1;
        TXFWDatIndx = TXFWDatLen;
        tt_FW_TxDelayTime = getSysCount();                          //store time tick value  
        FW_Tx(); 
        // end of prepare data for tx
        tt_FW_WaitTime = getSysCount() + DemandFWWaitPeriod;                          //store time tick value 
        FW_TimeOutCounter = NumOfFWTimeOutLoop;                                     //setup timeout value for ACK wait timeout
        fwAutoBaudState = ABaudWaitAckAfterReset;
      }
      break;
    }
  case ABaudWaitAckAfterReset:
    {
      switch(waitForACK())
      {
      case 0x79:    //ACK
        { //ACK
          fwAutoBaudState = ABaudSuccess;
          break;
        }  
      case 0x1F:    //NACK        
      case -1:      //Timeout
        { //NACK and Timeout
          fwAutoBaudState = ABaudFail;         //fail will stay in fail state
        }
      case 0:
      default:
        break;
      }
      break;
    }
  case ABaudSuccess:          
    {
      break;
    }
  case ABaudFail:
    {
      break;
    }
  default:
    break;
  }
  return((uint8_t)fwAutoBaudState);
}

/**
********************************************************************************************************************************
* @brief   Function of Read version and read protection from target flash.
* @author  Pamela Lee
* @details Read bootloader version and read protection status
*                
* @param   _FW_RdVerNProtectBuf read 3 bytes, byte0 = version number, byte1 = Read option byte0, byte2 = Read option byte1
* @return  - the current state of this stateMachine fwRdVerNProtectState 
*          - flash data should stored into buffer (_FW_RdVerNProtectBuf) from target flash
********************************************************************************************************************************
*/
uint8_t FWGetVerNRdProtection(uint8_t* _FW_RdVerNProtectBuf)                     //Get version and read protection status
{
  switch(fwRdVerNProtectState)
  {
  case PreFwRdVerNProtect:                   
    { 
      _txFWdata= (uint8_t*) malloc(2);
      _txFWdata[0] = 0x01;                                        //setup cmd as Read Version snd read protection status cmd
      _txFWdata[1] = 0xFE;
      TXFWDatLen = 2;
      TXFWDatIndx = TXFWDatLen;
      tt_FW_TxDelayTime = getSysCount();                          //store time tick value  
      FW_Tx(); 
      // end of prepare data for tx
      tt_FW_WaitTime = getSysCount() + DemandFWWaitPeriod;                          //store time tick value 
      FW_TimeOutCounter = NumOfFWTimeOutLoop;                                     //setup timeout value for ACK wait timeout
      fwRdVerNProtectState = waitAckRdVerNProtect;       
      break;
    }
  case waitAckRdVerNProtect:
    {
      switch(waitForACK())
      {
      case 0x79:    //ACK
        { //ACK
          fwRdVerNProtectState = FwRdVerNProtectDat;
          break;
        }  
      case 0x1F:    //NACK        
      case -1:      //Timeout
        { //NACK and Timeout
          fwRdVerNProtectState = RdVerNProtectFail;         //fail will stay in fail state
        }
      case 0:
      default:
        break;
      }
      break;
    }
  case FwRdVerNProtectDat:
    {
      switch(waitForDatRd(_FW_RdVerNProtectBuf, 3))
      {
      case 1:
        {
          fwRdVerNProtectState = RdVerNProtectSuccess;
          break;
        }
      case -1:
        {
          fwRdVerNProtectState = RdVerNProtectFail;         //fail will stay in fail state
        }
      case 0:
      default:
        break;   
      }
    }      
  case RdVerNProtectSuccess:
    {
      break;
    }
  case RdVerNProtectFail:
    {
      break;
    }
  default:
    break;   
  }   
  return((uint8_t)fwRdVerNProtectState);
}

/**
********************************************************************************************************************************
* @brief   Function of Get Target ID from target flash.
* @author  Pamela Lee
* @details Get Target chip ID
*                
* @param   _FW_GetIDBuf read 3 bytes, byte0 = number of data, byte1 = PID =>0x04, byte2 = 0xXX
* @return  - the current state of this stateMachine fwGetIDState
*          - flash data should stored into buffer (_FW_GetIDBuf) from target flash
********************************************************************************************************************************
*/
uint8_t FWGetID(uint8_t* _FW_GetIDBuf)             //Get Target chip ID
{
  switch(fwGetIDState)
  {
  case PreFwGetID:                   
    { 
      _txFWdata= (uint8_t*) malloc(2);
      _txFWdata[0] = 0x02;                                        //setup cmd as Read Version snd read protection status cmd
      _txFWdata[1] = 0xFD;
      TXFWDatLen = 2;
      TXFWDatIndx = TXFWDatLen;
      tt_FW_TxDelayTime = getSysCount();                          //store time tick value  
      FW_Tx(); 
      // end of prepare data for tx
      tt_FW_WaitTime = getSysCount() + DemandFWWaitPeriod;                          //store time tick value 
      FW_TimeOutCounter = NumOfFWTimeOutLoop;                                     //setup timeout value for ACK wait timeout
      fwGetIDState = waitAckGetID;       
      break;
    }
  case waitAckGetID:
    {
      switch(waitForACK())
      {
      case 0x79:    //ACK
        { //ACK
          fwGetIDState = FwGetIDDat;
          break;
        }  
      case 0x1F:    //NACK        
      case -1:      //Timeout
        { //NACK and Timeout
          fwGetIDState = GetIDFail;         //fail will stay in fail state
        }
      case 0:
      default:
        break;
      }
      break;
    }
  case FwGetIDDat:
    {
      switch(waitForDatRd(_FW_GetIDBuf, 3))
      {
      case 1:
        {
          fwGetIDState = GetIDSuccess;
          break;
        }
      case -1:
        {
          fwGetIDState = GetIDFail;         //fail will stay in fail state
        }
      case 0:
      default:
        break;   
      }
    }      
  case GetIDSuccess:
    {
      break;
    }
  case GetIDFail:
    {
      break;
    }
  default:
    break;   
  }   
  return((uint8_t)fwGetIDState);
}

/**
********************************************************************************************************************************
* @brief   Function of go execute target flash.
* @author  Pamela Lee
* @details 
* @param   StartingAddr starting address of Target system
* @return  - the current state of this stateMachine fwGoState 
********************************************************************************************************************************
*/
uint8_t FWGo(uint32_t StartingAddr)                     //Target system start execute at Starting Address
{
  switch(fwGoState)
  {
  case PreFwGo:                   
    { 
      _txFWdata= (uint8_t*) malloc(2);
      _txFWdata[0] = 0x21;                                        //setup cmd as Go command
      _txFWdata[1] = 0xDE;
      TXFWDatLen = 2;
      TXFWDatIndx = TXFWDatLen;
      tt_FW_TxDelayTime = getSysCount();                          //store time tick value  
      FW_Tx(); 
      // end of prepare data for tx
      tt_FW_WaitTime = getSysCount() + DemandFWWaitPeriod;                          //store time tick value 
      FW_TimeOutCounter = NumOfFWTimeOutLoop;                                     //setup timeout value for ACK wait timeout
      fwGoState = waitAckGoFw;       
      break;
    }
  case waitAckGoFw:
    {
      switch(waitForACK())
      {
      case 0x79:    //ACK
        { //ACK
          fwGoState = FwGoAddr;
          break;
        }  
      case 0x1F:    //NACK        
      case -1:      //Timeout
        { //NACK and Timeout
          fwGoState = GoFail;         //fail will stay in fail state
        }
      case 0:
      default:
        break;
      }
      break;
    }
  case FwGoAddr:
    { //ADDR_FLASH_PAGE_0
      _txFWdata= (uint8_t*) malloc(5);
      _txFWdata[0] = (uint8_t)(StartingAddr >> 24);                 
      _txFWdata[1] = (uint8_t)(StartingAddr >> 16); 
      _txFWdata[2] = (uint8_t)(StartingAddr >> 8); 
      _txFWdata[3] = (uint8_t)StartingAddr;
      _txFWdata[4] = XorBuf(_txFWdata, 4);
      TXFWDatLen = 5;
      TXFWDatIndx = TXFWDatLen;
      tt_FW_TxDelayTime = getSysCount();                          //store time tick value  
      FW_Tx(); 
      // end of prepare data for tx
      tt_FW_WaitTime = getSysCount() + DemandFWWaitPeriod;                          //store time tick value 
      FW_TimeOutCounter = NumOfFWTimeOutLoop;                                     //setup timeout value for ACK wait timeout
      fwGoState = waitAddrAckGoFw;  
      break;
    }
  case waitAddrAckGoFw:
    {
      switch(waitForACK())
      {
      case 0x79:    //ACK
        { //ACK
          fwGoState = GoSuccess;
          break;
        }  
      case 0x1F:    //NACK        
      case -1:      //Timeout
        { //NACK 
          fwGoState = GoFail;         //fail will stay in fail state
        }
      case 0:
      default:
        break;
      }
      break;
    }
  case GoSuccess:                       
    {
      break;
    }
  case GoFail:
    {
      break;
    }
  default:
    break;
  }
  return((uint8_t)fwGoState);
}

/**
********************************************************************************************************************************
* @brief   Function of burnning firmware into target flash.
* @author  Pamela Lee
* @details - flash data should stored in input buffer (FW_DatBuf) into target flash
*          - the starting address is define in "ADDR_FLASH_PAGE_0"
*          - Block number store in  (motor_FW_Update_Control.WrBufBlkNO) ; each block is 256byte per block for example if 128kbyte flash MPU with be block number 0 to 511
*          - usage: first put data into "FW_DatBuf" and block number(data going to burn in target flash location), then choose fill in the starting-state (either WrFw_WithErase or  WrFw_WithOutErase).
*                   Call this function will burn the buffer data into target MPU. This function will return WrFw_WithOutErase (for successfully burnning data), the next burnning block can fill into 
*                   "FW_DatBuf" with the new block number, then the function will know this is next block of data to burn, until exhaust the whole firmware.
*                   The process can be finshed by putting a negative number into (motor_FW_Update_Control.WrBufBlkNO) = -1, the stateMachine will enter into the FWUpdateSuccess forever(endless loop)
*          - remark1: if any stage receive a negative acknowledgement(NAK), the stateMachine will enter into the FWUpdateSFail forever(endless loop)
*          - remark2: when stateMachine enter into a wait for external signal state( for example after finished the last block and jump to WrFw_WithOutErase )
*                     , stateMachine will keep running in this state without any further action!!!!!! until a new block of data or -1 in motor_FW_Update_Control.WrBufBlkNO.
*                
* @param   fwWrState  -- before execute contain startup state (either WrFw_WithErase or  WrFw_WithOutErase) for burning with or without Erase the whole flash
* @return  fwWrState  -- the current state of this stateMachine
********************************************************************************************************************************
*/
uint8_t FWWrStateMachine(void)
{
  switch(fwWrState)
  { /** ============================================================  Firmware download part to target ================================================================= **/   
  case WrFw_WithErase:               /*******************************  Erase target whole flash stage ****************************************/
    { //Erase the whole flash
      // prepare data for tx
      _txFWdata= (uint8_t*) malloc(2);
      _txFWdata[0] = 0x44;                                        //setup cmd as extended erase
      _txFWdata[1] = 0xBB;
      TXFWDatLen = 2;
      TXFWDatIndx = TXFWDatLen;
      tt_FW_TxDelayTime = getSysCount();                          //store time tick value  
      FW_Tx(); 
      // end of prepare data for tx
      tt_FW_WaitTime = getSysCount() + DemandFWWaitPeriod;                          //store time tick value 
      FW_TimeOutCounter = NumOfFWTimeOutLoop;                                     //setup timeout value for ACK wait timeout
      fwWrState = WaitAckErase;     
      break;
    }
  case WaitAckErase:  
    {
      switch(waitForACK())
      {
      case 0x79:    //ACK
        { //ACK
          fwWrState = WholeFlashErase;
          break;
        }  
      case 0x1F:    //NACK        
      case -1:      //Timeout
        { //NACK and Timeout
          FW_TimeOutCounter = 0;
          fwWrState = FWWrFail;         //fail will stay in fail state
        }
      case 0:
      default:
        break;
      }
      break;
    }
  case WholeFlashErase:
    {    // prepare data for tx
      _txFWdata= (uint8_t*) malloc(3);
      _txFWdata[0] = 0xFF;                                        //global erase for target flash
      _txFWdata[1] = 0xFF;
      _txFWdata[2] = 0x00;
      TXFWDatLen = 3;
      TXFWDatIndx = TXFWDatLen;
      tt_FW_TxDelayTime = getSysCount();                          //store time tick value  
      FW_Tx(); 
      // end of prepare data for tx
      tt_FW_WaitTime = getSysCount() + DemandFWWaitPeriod;                          //store time tick value 
      FW_TimeOutCounter = NumOfFWTimeOutLoop;                                     //setup timeout value for ACK wait timeout
      fwWrState = WaitAckEraseFinal;     
      break;        
    }
  case WaitAckEraseFinal:  
    {
      switch(waitForACK())
      {
      case 0x79:    //ACK
        { //ACK
          fwWrState = WrFw_WithOutErase;
          break;
        }  
      case 0x1F:    //NACK        
      case -1:      //Timeout
        { //NACK and Timeout
          FW_TimeOutCounter = 0;
          fwWrState = FWWrFail;         //fail will stay in fail state
        }
      case 0:
      default:
        break;
      }
      break;
    } 
  case WrFw_WithOutErase:                   /*******************************  Write target whole firmware stage ****************************************/
    { 
      if(motor_FW_Update_Control.WrBufBlkNO_s16 != previousWrittenBlkNO)
      {
        if(motor_FW_Update_Control.WrBufBlkNO_s16 >= 0 )
        {//start a new block write
          motor_FW_Update_Control.Rd_Wr_busy_u16 = TRUE;
          previousWrittenBlkNO = motor_FW_Update_Control.WrBufBlkNO_s16;
          _txFWdata= (uint8_t*) malloc(2);
          _txFWdata[0] = 0x31;                                        //setup cmd as extended erase
          _txFWdata[1] = 0xCE;
          TXFWDatLen = 2;
          TXFWDatIndx = TXFWDatLen;
          tt_FW_TxDelayTime = getSysCount();                          //store time tick value  
          FW_Tx(); 
          // end of prepare data for tx
          tt_FW_WaitTime = getSysCount() + DemandFWWaitPeriod;                          //store time tick value 
          FW_TimeOutCounter = NumOfFWTimeOutLoop;                                     //setup timeout value for ACK wait timeout
          fwWrState = waitAckWrFw;
        }
        else
        {//the whole write process have been done
          motor_FW_Update_Control.Rd_Wr_busy_u16 = FALSE;
          fwWrState = FWWrSuccess;         //all blk finish
        }         
      }
      break;
    }
    
  case waitAckWrFw:
    {
      switch(waitForACK())
      {
      case 0x79:    //ACK
        { //ACK
          fwWrState = WrFwAddr;
          break;
        }  
      case 0x1F:    //NACK        
      case -1:      //Timeout
        { //NACK and Timeout
          FW_TimeOutCounter = 0;
          fwWrState = FWWrFail;         //fail will stay in fail state
        }
      case 0:
      default:
        break;
      }
      break;
    }
  case WrFwAddr:
    { //ADDR_FLASH_PAGE_0
      _txFWdata= (uint8_t*) malloc(5);
      uint32_t blkStartAddr = ADDR_FLASH_PAGE_0 + (motor_FW_Update_Control.WrBufBlkNO_s16 * 256);
      _txFWdata[0] = (uint8_t)(blkStartAddr >> 24);                 
      _txFWdata[1] = (uint8_t)(blkStartAddr >> 16); 
      _txFWdata[2] = (uint8_t)(blkStartAddr >> 8); 
      _txFWdata[3] = (uint8_t)blkStartAddr;
      _txFWdata[4] = XorBuf(_txFWdata, 4);
      TXFWDatLen = 5;
      TXFWDatIndx = TXFWDatLen;
      tt_FW_TxDelayTime = getSysCount();                          //store time tick value  
      FW_Tx(); 
      // end of prepare data for tx
      tt_FW_WaitTime = getSysCount() + DemandFWWaitPeriod;                          //store time tick value 
      FW_TimeOutCounter = NumOfFWTimeOutLoop;                                     //setup timeout value for ACK wait timeout
      fwWrState = waitAddrAckWrFw;  
      break;
    }
  case waitAddrAckWrFw:
    {
      switch(waitForACK())
      {
      case 0x79:    //ACK
        { //ACK
          fwWrState = WrFwData;
          break;
        }  
      case 0x1F:    //NACK        
      case -1:      //Timeout
        { //NACK and Timeout
          FW_TimeOutCounter = 0;
          fwWrState = FWWrFail;         //fail will stay in fail state
        }
      case 0:
      default:
        break;
      }
      break;
    }
  case WrFwData:
    { //Len + 256 byte data + Checksum byte
      _txFWdata = (uint8_t*) malloc(258);
      _txFWdata = FW_DatBuf;                                      //start of data address                               
      _txFWdata[257] = XorBuf(_txFWdata, 257);                    //send 256 byte
      TXFWDatLen = 258;
      TXFWDatIndx = TXFWDatLen;
      tt_FW_TxDelayTime = getSysCount();                           //store time tick value  
      FW_Tx(); 
      // end of prepare data for tx
      tt_FW_WaitTime = getSysCount() + DemandFWWaitPeriod;         //store time tick value 
      FW_TimeOutCounter = NumOfFWTimeOutLoop;                                    //setup timeout value for ACK wait timeout
      fwWrState = waitAckWrFwFinal;  
      break;
    }
  case waitAckWrFwFinal:     
    {
      switch(waitForACK())
      {
      case 0x79:    //ACK
        { //ACK
          motor_FW_Update_Control.Rd_Wr_busy_u16 = FALSE;
          fwWrState = WrFw_WithOutErase;
          break;
        }  
      case 0x1F:    //NACK        
      case -1:      //Timeout
        { //NACK and Timeout
          FW_TimeOutCounter = 0;
          fwWrState = FWWrFail;         //fail will stay in fail state
        }
      case 0:
      default:
        break;
      }
      break;
    }
    /** ============================================================ end of Firmware download part to target ================================================================= **/          
    
  case FWWrSuccess:                       /*******************************  Write protection for target firmware stage ****************************************/
    {
      motor_FW_Update_Control.Rd_Wr_busy_u16 = FALSE;
      break;
    }
  case FWWrFail:
    {
      motor_FW_Update_Control.Rd_Wr_busy_u16 = FALSE;
      break;
    }
  default:
    break;
  }
  return((uint8_t)fwWrState);
}

/**
********************************************************************************************************************************
* @brief   Function of Read block of firmware from target flash.
* @author  Pamela Lee
* @warning - The read size cannot larger than the Usart buffers(internal and transparent) 
*          - Current read data size will be 64byte(for both internal and transparent buffers are 80byte).
*             Size can be increase to 256 but need to also increase both internal and transparent buffers, with
*             the RingBuf_GetUsedNumOfElements() function return type increase from uint8_t to uint16_t.
* @details - flash data should stored in input buffer (FW_DatBuf) into target flash
*          - the starting address is define in "ADDR_FLASH_PAGE_0"
*          - Block number store in  (motor_FW_Update_Control.RdBufBlkNO) ; each block is 64byte per block for example if 128kbyte flash MPU with be block number 0 to 2048
*          - usage: Enter the block number(data going to read from target flash location), then just call this function.
*                
* @param   
* @return  - the current state of this stateMachine fwRdState 
*          - flash data should stored into buffer (_FW_RdDatBuf) from target flash
********************************************************************************************************************************
*/
uint8_t FWBlkRead(uint8_t* _FW_RdDatBuf)
{
  switch(fwRdState)
  {
  case PreFwRd:                   
    { 
      _txFWdata= (uint8_t*) malloc(2);
      _txFWdata[0] = 0x11;                                        //setup cmd as Read Memory
      _txFWdata[1] = 0xEE;
      TXFWDatLen = 2;
      TXFWDatIndx = TXFWDatLen;
      tt_FW_TxDelayTime = getSysCount();                          //store time tick value  
      FW_Tx(); 
      // end of prepare data for tx
      tt_FW_WaitTime = getSysCount() + DemandFWWaitPeriod;                          //store time tick value 
      FW_TimeOutCounter = NumOfFWTimeOutLoop;                                     //setup timeout value for ACK wait timeout
      fwRdState = waitAckRdFw;       
      break;
    }
  case waitAckRdFw:
    {
      switch(waitForACK())
      {
      case 0x79:    //ACK
        { //ACK
          fwRdState = FwRdAddr;
          break;
        }  
      case 0x1F:    //NACK        
      case -1:      //Timeout
        { //NACK and Timeout
          fwRdState = RdFail;         //fail will stay in fail state
        }
      case 0:
      default:
        break;
      }
      break;
    }
  case FwRdAddr:
    { //ADDR_FLASH_PAGE_0
      _txFWdata= (uint8_t*) malloc(5);
      uint32_t blkStartAddr = ADDR_FLASH_PAGE_0 + (motor_FW_Update_Control.RdBufBlkNO_s16 * 256);
      _txFWdata[0] = (uint8_t)(blkStartAddr >> 24);                 
      _txFWdata[1] = (uint8_t)(blkStartAddr >> 16); 
      _txFWdata[2] = (uint8_t)(blkStartAddr >> 8); 
      _txFWdata[3] = (uint8_t)blkStartAddr;
      _txFWdata[4] = XorBuf(_txFWdata, 4);
      TXFWDatLen = 5;
      TXFWDatIndx = TXFWDatLen;
      tt_FW_TxDelayTime = getSysCount();                          //store time tick value  
      FW_Tx(); 
      // end of prepare data for tx
      tt_FW_WaitTime = getSysCount() + DemandFWWaitPeriod;                          //store time tick value 
      FW_TimeOutCounter = NumOfFWTimeOutLoop;                                     //setup timeout value for ACK wait timeout
      fwRdState = waitAddrAckRdFw;  
      break;
    }
  case waitAddrAckRdFw:
    {
      switch(waitForACK())
      {
      case 0x79:    //ACK
        { //ACK
          fwRdState = FwRdLen;
          break;
        }  
      case 0x1F:    //NACK        
      case -1:      //Timeout
        { //NACK 
          fwRdState = RdFail;         //fail will stay in fail state
        }
      case 0:
      default:
        break;
      }
      break;
    }
  case FwRdLen: 
    {
      _txFWdata= (uint8_t*) malloc(2);
      _txFWdata[0] = 0x3f;                                        //data length = 0x3f =>63
      _txFWdata[1] = _txFWdata[0] ^ 0xFF;                          //Checksum: XOR byte 8 (complement of byte 8)
      TXFWDatLen = 2;
      TXFWDatIndx = TXFWDatLen;
      tt_FW_TxDelayTime = getSysCount();                          //store time tick value  
      FW_Tx(); 
      // end of prepare data for tx
      tt_FW_WaitTime = getSysCount() + DemandFWWaitPeriod;                          //store time tick value 
      FW_TimeOutCounter = NumOfFWTimeOutLoop;                                     //setup timeout value for ACK wait timeout
      fwRdState = waitLenAckRdFw;       
      break;
    }
  case waitLenAckRdFw:
    {
      switch(waitForACK())
      {
      case 0x79:    //ACK
        { //ACK
          
          fwRdState = FwRdData;
          break;
        }  
      case 0x1F:    //NACK        
      case -1:      //Timeout
        { //NACK 
          fwRdState = RdFail;         //fail will stay in fail state
        }
      case 0:
      default:
        break;
      }
      break;
    }   
  case FwRdData:
    {
      switch(waitForDatRd(_FW_RdDatBuf, 64))
      {
      case 1:
        {
          fwRdState = RdSuccess;
          break;
        }
      case -1:
        {
          fwRdState = RdFail;         //fail will stay in fail state
        }
      case 0:
      default:
        break;   
      }
    }      
  case RdSuccess:                       
    {
      break;
    }
  case RdFail:
    {
      break;
    }
  default:
    break;
  }
  return((uint8_t)fwRdState);
}

/*************************************************** Local Functions **************************************************************/
uint8_t XorBuf(uint8_t* _buf, uint16_t _len)
{
  uint8_t Ans = 0;
  if(_len == 1) return(*_buf ^ 0xff);
  for( uint16_t Indx = 0; Indx < _len; Indx++) {
    Ans ^= _buf[Indx];
  }  
  return Ans;
}

/** @caution this function need to run as background function and control 
by TxIndx, so please set the data length before call this function 
**/
uint8_t FW_Tx(void)                 
{
  if(TXFWDatIndx) 
  {
    if (getSysCount() >= tt_FW_TxDelayTime) {
      //Tx part   
      LL_USART_TransmitData8(USART2,  _txFWdata[TXFWDatLen - TXFWDatIndx-- ]);       //put buffer in
      // Enable TXE interrupt //
      LL_USART_EnableIT_TXE(USART2);
      /* Disable TXE interrupt */
      LL_USART_DisableIT_TXE(USART2);
      tt_FW_TxDelayTime = getSysCount() + DemandFWDelayPeriod;                          //store time tick value 
      return 1;
    }
  }
  else    free(_txFWdata);
  return 0;
}

int16_t waitForACK(void)
{
  if (getSysCount() >= tt_FW_WaitTime)  
  {
    if(RingBuf_GetUsedNumOfElements((*usart2Control_MotorFW_Update).seqMemRX_transparentMode_u32) >= 1 )
    {
      uint32_t DataLen2 = 1;
      uint8_t FW_RxBuf = 0x00; 
      RingBuf_ReadBlock((*usart2Control_MotorFW_Update).seqMemRX_transparentMode_u32, &FW_RxBuf, &DataLen2); //extract data
      return(FW_RxBuf);
    }
    if(FW_TimeOutCounter-- <= 0) return -1;                             //TimeOut will stay in fail state   
    tt_FW_WaitTime = getSysCount() + DemandFWWaitPeriod;             //store time tick value 
  }
  return 0;
}

int8_t waitForDatRd(uint8_t* _FW_RxBuf, uint8_t Len)
{
  if (getSysCount() >= tt_FW_WaitTime)  
  {
    if(RingBuf_GetUsedNumOfElements((*usart2Control_MotorFW_Update).seqMemRX_transparentMode_u32) >= Len )
    {
      uint32_t DataLen2 = Len;
      RingBuf_ReadBlock((*usart2Control_MotorFW_Update).seqMemRX_transparentMode_u32, _FW_RxBuf, &DataLen2); //extract data
      return 1;
    }
    if(FW_TimeOutCounter-- <= 0) return -1;                             //TimeOut will stay in fail state   
    tt_FW_WaitTime = getSysCount() + DemandFWWaitPeriod;             //store time tick value 
  }
  return 0;
}