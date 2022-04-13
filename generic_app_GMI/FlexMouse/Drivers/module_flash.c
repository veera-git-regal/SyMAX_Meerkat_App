/**
********************************************************************************************************************************
* @file    module_flash.c 
* @author  Pamela Lee
* @brief   Main driver module for flash.
* @details This module initializes the flash
*          The ST motor libraries parameters will be mapped from the top of FLASH_USER_START_ADDR in 16bit data,
*          the data can be updated by using the function of uint8_t FlashDatSet(uint16_t _offset, uint16_t _flashDat), then 
*          data and offset will store in internal buffer as temporary data, user can either store all the temporary data into flash 
*          by flashPageUpdate(), or if the internal buffer is full will also update the temporary into flash.
********************************************************************************************************************************
*/

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "zz_module_flash.h"

#include "module_analog_0_10V.h"

/* Content ---------------------------------------------------------------------------------------------------------------------*/

//extern Ram_Buf sharedMemArray[STRUCT_MEM_ARRAY_SIZE];
extern ProcessInfo processInfoTable[];
uint32_t Address = 0;                                     //, PageError = 0;
__IO uint8_t readData = 0 , MemoryProgramStatus = 0;

/*Variable used for Erase procedure*/
static FLASH_EraseInitTypeDef EraseInitStruct;


//*****************************************************************************************
typedef  struct 
{
  uint16_t      offset;
  uint64_t      flashDat;
}FlashBufInfo;
#define FlashBufSize 20
FlashBufInfo flashBuf[FlashBufSize];
uint8_t Flash_BufHead = 0;
uint8_t Flash_BufTail = 0;
uint8_t is_flashInitComplete = TRUE;
uint8_t is_flashUpdate = FALSE;
uint8_t flash_status_u8 = 0;

uint32_t address_value_ptr = 0;
#include "module_analog_4_20ma.h"
Analog_4_20ma_Control* flash_AnalogAmpsControl_ptr;
//********************************************************************************************************************************************************

enum {
  MEMORY_INIT_MODULE,
  INIT_MODULE,
  RUN_MODULE,
  // additional states to be added here as necessary.
  IRQ_MODULE = DEFAULT_IRQ_STATE,
  KILL_MODULE = KILL_APP
};

void Get_Flash_Index(void);
void Flash_Buf_Init(void);
//void Get_Flash_Index(void);
void Update_Flash(uint8_t module_id_u8, uint8_t flash_state_u8);
uint8_t update_Flash_Settings(uint8_t);
uint8_t init_Flash_Settings(void);
void copy_Data_To_Buffer(uint32_t index_u32, uint8_t* buff);
uint8_t update_Flash_Settings(uint8_t module_id_u8);
uint8_t flashBlockProgram8bytes(uint8_t drv_id_u8, uint32_t _TopageAddress, uint32_t _length);
//uint32_t flashGetSettingAddress(uint8_t drv_id_u8, uint8_t *module_address_ptr, uint8_t *setting_address_ptr);
uint8_t check_flash_empty(uint32_t page_start_addres_u32, uint16_t page_size_u16);
uint8_t flashWriteCRC32Version(uint8_t module_id_u8, uint32_t _TopageAddress);

uint8_t moduleFlash_u32(uint8_t drv_id_u8, uint8_t prev_state_u8, uint8_t next_state_u8, uint8_t irq_id_u8) {
  uint8_t return_state_u8 = MEMORY_INIT_MODULE;
  switch (next_state_u8) 
  {
  case MEMORY_INIT_MODULE:
    {
      //AssignModuleMemFlash(); // Assign structured memory
      return_state_u8 = INIT_MODULE;
      break;
    }
  case INIT_MODULE: 
    {
      //FlashBufInit();
      
      /** for pam testing the buffer system **/
      
      //FlashDataSet(0, 0xaaaabbbb);
      //FlashDataSet(16, 0xccccdddd);
      //FlashDataSet(0x1ff, 0x5555);
      //FlashDataSet(0x122, 0x5555);   //going to delete before write inflash
      
      //FlashDataSet(0x3fe, 0xaaaa);
      //FlashDataSet(0x200, 0x5555);
      //FlashDataSet(0x3fe, 0xaa55);   //going to replace the former same address
      //FlashDataSet(0x0ff, 0xaaaa);
      //FlashDataSet(0x320, 0xaaaa);
      //FlashDataSet(0x200, 0xaaaa);
      
      ////flashPageErase(drv_id_u8, FLASH_USER_START_ADDR);       //elase all two page for update
      //flashPageErase(drv_id_u8, MIRROR_FLASH_BLOCK_SETTING_PAGE);       //elase all two page for update 
      
      ////flashPageCopy(_module_id_u8, MIRROR_FLASH_BLOCK_SETTING_PAGE, FLASH_BLOCK_SETTING_PAGE);
      //flashPageUpdate(drv_id_u8, (FLASH_USER_START_ADDR), MIRROR_FLASH_BLOCK_SETTING_PAGE); //copy the lower page to upper page with ram data in buffer
      
      //flashPageCopy(drv_id_u8, FLASH_USER_START_ADDR, (FLASH_USER_START_ADDR + FLASH_PAGE_SIZE)); //copy the whole page from one to other
      
      //  flashPageErase(drv_id_u8, ADDR_FLASH_PAGE_31);
      //   flashPageCopy(drv_id_u8, ADDR_FLASH_PAGE_30, ADDR_FLASH_PAGE_31);
      
      return_state_u8 = RUN_MODULE;
      break;
    }
  case RUN_MODULE: 
    {
      /** Program the user Flash area word by word(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) **/
      
      if( (is_flashInitComplete == FALSE) || (is_flashUpdate == TRUE) )
      {
        Get_Flash_Index();
        flash_status_u8 = check_flash_empty(FLASH_USER_START_ADDR, PAGE_SIZE);
        flashPageErase(drv_id_u8, FLASH_USER_START_ADDR);           //elase all two page for update
        flashPageErase(drv_id_u8, MIRROR_FLASH_BLOCK_SETTING_PAGE); //elase all two page for update 
        flash_status_u8 = check_flash_empty(FLASH_USER_START_ADDR, PAGE_SIZE);
        init_Flash_Settings();
        flashWriteCRC32Version(drv_id_u8, FLASH_USER_START_ADDR); // write flash version and CRC
        is_flashInitComplete = TRUE;
        is_flashUpdate = FALSE;
      }
      //address_value_ptr =  flashGetSettingAddress(6,(uint8_t *)(flash_AnalogAmpsControl_ptr) , (uint8_t *)&((*flash_AnalogAmpsControl_ptr).analog_4_20mA_Setting.analogVoltsToAmpsGain_f));
      address_value_ptr =  flashGetSettingAddress(6,(uint8_t *)(flash_AnalogAmpsControl_ptr) , (uint8_t *)&((*flash_AnalogAmpsControl_ptr).analog_4_20mA_Setting.analogGain_f));
      //getSettingsFromFalsh(6); // Get flash setting for 4-20mA module and copy them to RAM
      return_state_u8 = RUN_MODULE;
      break;
    }
  case KILL_MODULE: 
    {
      // Setting processStatus_u8 to PROCESS_STATUS_KILLED prevents the scheduler main loop from calling this module again.
      uint8_t table_index_u8 = getProcessInfoIndex(drv_id_u8);
      if (table_index_u8 != INDEX_NOT_FOUND) {
        processInfoTable[table_index_u8].Sched_DrvData.processStatus_u8 = PROCESS_STATUS_KILLED;
      }
      return_state_u8 = KILL_MODULE;
      break;
    }
  default: 
    {
      return_state_u8 = KILL_MODULE;
      break;
    }
  }
  return return_state_u8;
}

/** =========================== private functions =========================================== **/
/**
* @brief  Erase page/s of flash data
* @param  drv_id_u8   The function caller module ID in case error occur within this function
*         pageAddress The starting address of the flash page
* @retval successful
*/
uint8_t flashPageErase(uint8_t drv_id_u8, uint32_t pageAddress)                 //pam tested
{
  uint8_t returnValue = true;
  uint32_t PageError = 0;
  volatile uint32_t page_number_u32 = 0;
  
  /** Unlock the Flash to enable the flash control register access **/
  HAL_FLASH_Unlock();
  /* Fill EraseInit structure*/
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  page_number_u32 = (uint32_t)( (pageAddress - FLASH_START_ADDR) / (float)PAGE_SIZE) ;
  EraseInitStruct.Page = page_number_u32;  //pageAddress;
  EraseInitStruct.NbPages     = (uint32_t)NUMBER_OF_FLASH_PAGES;
  /**                                                Flash erase                                             **/
  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)   //22ms
  {  // Error occurred while page erase.                                                                           
    setupSoftwareIRQ(drv_id_u8, MODULE_ERR_LOGHANDLE, 0xE4, 0x00, sizeof(uint32_t), (uint8_t*)(&PageError));     //report the error address 
    returnValue = false;
  }
  /** Lock the Flash to disable the flash control register access (recommended to protect the FLASH memory against possible unwanted operation) **/
  HAL_FLASH_Lock();    
  return returnValue;
}

/**
* @brief  Copy page/s of flash data to another sector with all the temporary storage in internal buffer
* @param  drv_id_u8        The function caller module ID in case error occur within this function
*         _FrompageAddress The starting address of the source flash page/s
*         _TopageAddress   The starting address of the sink flash page/s
* @retval successful
*/
uint8_t flashPageUpdate(uint8_t drv_id_u8, uint32_t _FrompageAddress, uint32_t _TopageAddress)
{
  /** Unlock the Flash to enable the flash control register access **/
  HAL_FLASH_Unlock();
  uint16_t indx = 0;
  volatile uint64_t currentDat;
  uint64_t returnBuf = 0;
  uint8_t returnValue = TRUE;
  for(  ;  indx < ((FLASH_PAGE_SIZE * NUMBER_OF_FLASH_PAGES) - FLASH_COPY_BYTE_SIZE) ; indx += FLASH_COPY_BYTE_SIZE)
  {
    if(FlashBufDeRegistered(indx, &returnBuf))  //check this offset address is changed 
    {
      currentDat = returnBuf;         
    }
    else
    {
      currentDat = FlashRead64Bits((unsigned char*) _FrompageAddress, indx); //no updated data then just read the old data
    } 
    // You can only write to flash when flash contains "0XFFFF FFFF" data or else you get HAL_ERROR (PROGERR)
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (_TopageAddress + indx), (uint64_t)currentDat) == HAL_OK) //write 64bit data
    {
      Address += FLASH_COPY_BYTE_SIZE;
    }
    else
    {
      // Error occurred while writing data in Flash memory.                                                                 
      setupSoftwareIRQ(drv_id_u8, MODULE_ERR_LOGHANDLE, 0xE4, 0x00, 0x00, NULL);     //report the error address 
    }  
  }
  // put CRC to the last 2byte
  uint16_t uwCRCValue = Calculate_CRC((uint16_t)((FLASH_PAGE_SIZE * NUMBER_OF_FLASH_PAGES) - 2) , (unsigned char*)_TopageAddress);    
  //put calculated CRC back to the last word of the page
  if (!HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (_TopageAddress + indx), (uint64_t)uwCRCValue) == HAL_OK) //write 16bit data
  {
    // Error occurred while writing data in Flash memory.                                                                 
    setupSoftwareIRQ(drv_id_u8, MODULE_ERR_LOGHANDLE, 0xE4, 0x00, 0x00, NULL);     //report the error address
    returnValue = FALSE;
  }  
  /** Lock the Flash to disable the flash control register access (recommended to protect the FLASH memory against possible unwanted operation) **/
  HAL_FLASH_Lock();
  return returnValue;
}

/**
* @brief  Program a block of data into flash with input buffer
* @param  drv_id_u8       The function caller module ID in case error occur within this function
*         _TopageAddress  The starting address of the sink flash page/s
*         _buf            Buffer of data to burn into flash
*          _length        The starting address of the source flash page/s
* @retval successful
*/
uint8_t flashBlockProgram(uint8_t drv_id_u8, uint32_t _TopageAddress, uint8_t* _buf, uint32_t _length)                  //Pam Tested
{
  uint8_t returnValue = TRUE;
  HAL_FLASH_Unlock();
  
  for(uint16_t index = 0  ;  index < _length ; index += FLASH_COPY_BYTE_SIZE)
  {
    uint64_t data_u64 = (((uint64_t)_buf[index]) << 56)     + (((uint64_t)_buf[index + 1]) << 48) + 
      (((uint64_t)_buf[index + 2]) << 40) + (((uint64_t)_buf[index + 3]) << 32) + 
        (((uint64_t)_buf[index + 4]) << 24) + (((uint64_t)_buf[index + 5]) << 16) + 
          (((uint64_t)_buf[index + 6]) << 8)  +  _buf[index +7];
    if (!HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (_TopageAddress + index), data_u64) == HAL_OK) //write 16bit data
    {
      // Error occurred while writing data in Flash memory.                                                                 
      setupSoftwareIRQ(drv_id_u8, MODULE_ERR_LOGHANDLE, 0xE4, 0x00, 0x00, NULL);     //report the error address 
      returnValue = FALSE;
    }  
  }
  /** Lock the Flash to disable the flash control register access (recommended to protect the FLASH memory against possible unwanted operation) **/
  HAL_FLASH_Lock();
  return returnValue;
}


/**
* @brief  Check the active flash content with the last word CRC is valid
* @param  _FLASH_START_ADDR      CRC sector starting address
*         NUMBER_OF_FLASH_PAGES  Number of sector for CRC check ( @Caution each page will contains two bytes of CRC and is accumulated                                             
*                                                                 for example> page0 CRC= page0, if also has page1 CRC in the last two bytes of page1 will be from page0 to page1 )
* @retval successful
*/
uint8_t isFlashCRCValid(uint32_t _FLASH_START_ADDR, uint16_t _NumOfPage){               //pam tested
  unsigned char* _pageAddress = (unsigned char*) _FLASH_START_ADDR;
  uint16_t uwCRCValue = Calculate_CRC((uint16_t)((FLASH_PAGE_SIZE * _NumOfPage) - 2) , _pageAddress);   
  uint16_t FlashCRCValue = FlashRead64Bits((unsigned char*)_pageAddress, (uint16_t)((FLASH_PAGE_SIZE * NUMBER_OF_FLASH_PAGES) - 2));
  if(uwCRCValue == FlashCRCValue) 
    return(true);
  else
    return (false);
}


/**
* @brief  Copy page/s of flash memory to another sector
* @param  drv_id_u8          The function caller module ID in case error occur within this function
*         _FrompageAddress   The starting address of the source flash page/s
*         _TopageAddress     The starting address of the sink flash page/s
* @retval successful
*/
//copy the whole page to another page only
uint8_t flashPageCopy(uint8_t drv_id_u8, uint32_t _FrompageAddress, uint32_t _TopageAddress)
{
  /** Unlock the Flash to enable the flash control register access **/
  HAL_FLASH_Unlock();
  uint16_t indx = 0;
  uint16_t currentDat;
  for(  ;  indx < (FLASH_PAGE_SIZE * NUMBER_OF_FLASH_PAGES) ; indx += FLASH_COPY_BYTE_SIZE)
  {
    currentDat = FlashRead64Bits((unsigned char*)_FrompageAddress, indx); 
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (_TopageAddress + indx), (uint64_t)currentDat) == HAL_OK) //write 16bit data
    {
      Address += FLASH_COPY_BYTE_SIZE;
    }
    else
    {
      // Error occurred while writing data in Flash memory.                                                                 
      setupSoftwareIRQ(drv_id_u8, MODULE_ERR_LOGHANDLE, 0xE4, 0x00, 0x00, NULL);     //report the error address 
    }  
  }     
  /** Lock the Flash to disable the flash control register access (recommended to protect the FLASH memory against possible unwanted operation) **/
  HAL_FLASH_Lock();
  return 0;
}


/**
* @brief  Read data from flash
* @param  pageAddress flash page starting address
*         offsetByte  offset of the physical flash address (as the offset in byte [caution !!! this is absolute offset address in flash !!!])
* @retval data (16bit)
*/
uint16_t FlashRead(unsigned char* aDataBuf, uint16_t offsetByte)       //this read a word
{
  return((((uint16_t)aDataBuf[offsetByte]) << 8) +  aDataBuf[offsetByte +1]);
}

/**
* @brief  Read data from flash
* @param  pageAddress flash page starting address
*         offsetByte  offset of the physical flash address (as the offset in byte [caution !!! this is absolute offset address in flash !!!])
* @retval data (16bit)
*/
uint64_t FlashRead64Bits(unsigned char* aDataBuf, uint16_t offsetByte) //this read a word
{
  return( (((uint64_t)aDataBuf[offsetByte]) << 56) + (((uint64_t)aDataBuf[offsetByte]) << 48) + 
         (((uint64_t)aDataBuf[offsetByte]) << 40) + (((uint64_t)aDataBuf[offsetByte]) << 32) + 
           (((uint64_t)aDataBuf[offsetByte]) << 24) + (((uint64_t)aDataBuf[offsetByte]) << 16) + 
             (((uint64_t)aDataBuf[offsetByte]) << 8)  +  aDataBuf[offsetByte +1] );
}

/** ----------------------------------------- Flash internal ram storage system ----------------------------------------------------------------- **/

/**
* @brief  initialize the internal buffer and head, tail pointer
* @param  None
* @retval None
*/
void FlashBufInit(void)
{
  for(uint8_t index = 0; index < FlashBufSize ; index++)
  {
    flashBuf[index].offset = 0xffff;
    flashBuf[index].flashDat = 0;
  }  
  Flash_BufHead = 0;
  Flash_BufTail = 0;
}

/**
* @brief  Empty the setting data according to offset of the setting data
* @param  _offset     offset of the setting data
*         _returnBuf  data found 
* @retval data found successfully = true/false, the data found will store in  _returnBuf
*/
uint16_t FlashBufDeRegistered(uint16_t _offset, uint64_t* _returnBuf)  //Got the ack from the receiver and de-registered the record 
{
  for(uint8_t index = 0; index < FlashBufSize ; index++)
  {
    if(_offset == flashBuf[index].offset)        //search the received FrameAckID in buffer
    {  //clear all this record when found
      flashBuf[index].offset = 0xffff;
      *_returnBuf = flashBuf[index].flashDat;   //return the found value 
      flashBuf[index].flashDat = 0;             //clear data
      FlashFlushBuf();
      return true;
    }
  }  
  return false;
}

/**
* @brief  Flush the internal buffer and packed the leading or trailing record is/are empty
* @param  None
* @retval None
*/
void FlashFlushBuf(void)                                                           
{ //push back all the trailing empty record, and move the tail back to the last trailing record
  while((Flash_BufHead != Flash_BufTail) && (flashBuf[Flash_BufTail].offset == 0xffff) )    
  { //Move head to the closest in buffer
    if(Flash_BufTail != 0 ){
      Flash_BufTail--;
    }
    else
    {
      Flash_BufTail = (FlashBufSize - 1);
    }
  } 
  //push back all the leading empty record, and move the head back to the first leading record
  while((Flash_BufHead != Flash_BufTail) && (flashBuf[Flash_BufHead].offset == 0xffff) )
  { //Move head to the closest in buffer
    if(++Flash_BufHead == FlashBufSize)  Flash_BufHead = 0;
  }  
}


/**
* @brief  store current data(16bit offset) into internal buffer
* @param  _offset: ofset position of the 16bit setting data 
* @warning [caution !!! not absolute offset address in flash !!! please follow the FlashOffsetIndex ]
* @retval None
*/
uint8_t FlashDataSet(uint16_t _offset, uint64_t _flashDat)
{
  uint64_t tmpryDat;
  if(!IsFlashBufFull())
  {
    FlashBufDeRegistered((_offset * 8), &tmpryDat); //find this offset has already got data, will be replace    
    if(++Flash_BufTail == FlashBufSize) {
      Flash_BufTail = 0;
    }  
    flashBuf[Flash_BufTail].offset = _offset * 8;               //store as 16bit effective offset as no odd address!!! 
    flashBuf[Flash_BufTail].flashDat = _flashDat;
    return true;
  }
  return false;                         //if return also mean buffer full
}

/**
* @brief  Internal buffer is full
* @param  None
* @retval ture/false
*/
uint8_t IsFlashBufFull(void) 
{
  int16_t result = (int16_t)Flash_BufHead - (int16_t)Flash_BufTail;
  if( (result == -(FlashBufSize-1)) || (result == 1)) 
  { 
    return (true);
  }  
  return (false);
}

uint8_t Reg2Ram(uint32_t _RegNum, uint16_t _Value)
{
  //uint8_t stage = 0;
  //bool bNoError = FALSE;
  //uint16_t Durationms;
  //int16_t FinalMecSpeedUnit;
  //int16_t FinalTorque;
  switch((FlashOffsetIndex)_RegNum)
  {

  default:
    break;
  }
  return TRUE;
}





// Includes -------------------------------------------------------------------
#include "scheduler.h"
#include "module_analog_0_10v.h"
//#include "module_analog_4_20ma.h"
#include "module_digital_inputs.h"
#include "module_digital_outputs.h"
#include "module_modbus.h"
#include "module_motor_com.h"
#include "module_motor_demand_multiplexer.h"
#include "module_pwm_input.h"

#define FLASH_BUFFER_SIZE 10
#define FLASH_SETTINGS_START_ADDRESS  0x0800F000
uint32_t  flashSettingsVersion_u32 = 0x00000001; // Version of flash setting version

uint8_t flash_buffer_head_u8 = 0;
uint8_t flash_buffer_tail_u8 = 0;

AnalogVolts_Control* flash_AnalogVoltsControl_ptr;
//Analog_4_20ma_Control* flash_AnalogAmpsControl_ptr;
DigitalInputs_Control* flash_DigitalInputsControl_ptr;
Digital_Outputs_Control* flash_DigitalOutputsControl_ptr;
Modbus_Control* flash_ModbusControl_ptr;
Motor_Com_Control* flash_MotorComControl_ptr;
MotorDemandMux_Control* flash_MotorDemandMuxControl_ptr;
PwmInput_Control* flash_PwmInputControl_ptr;
Gpio_Control* flash_GpioControl_ptr; 

// Flash states
enum
{	
  FLASH_SETTINGS_INIT,
  FLASH_SETTINGS_UPDATE,
  FLASH_SETTINGS_ERASE,
  NO_FLASH_SETTINGS_UPDATE,
};

enum
{
  FLASH_EMPTY = 1,
  FLASH_NOT_EMPTY,
  FLASH_CRC_EMPTY,
  FLASH_CRC_ERROR,
  FLASH_VERSION_ERROR,
};

// Buffer that stores data that need to flashed
typedef  struct 
{
  uint32_t address_u32;
  uint64_t flashData_u64;
}FlashBuf;

// Buffer that stores address offsets and pointers for both flash and RAM 
typedef struct
{
  uint16_t module_settings_begin_offset_u16;
  uint16_t module_settings_end_offset_u16;
  uint16_t module_settings_size_u16;
  uint8_t *module_settings_address_ptr;
  uint32_t module_flash_settings_address_ptr;
}FlashSettingsIndex;

FlashBuf flash_buffer[FLASH_BUFFER_SIZE]; // Buffer that stores data that need to flashed
FlashSettingsIndex flash_settings_index[TOTAL_NUM_OF_PROCESSES]; // Buffer that stores address offsets and pointers for both flash and RAM

// Init flash buffer to clear any junk values
/**
* @brief  Init flash_buffer to clear any junk values
* @param  None
*         
* @retval None
*/
void Flash_Buf_Init(void)
{
  for(uint8_t index = 0; index < FLASH_BUFFER_SIZE ; index++)
  {
    flash_buffer[index].address_u32 = 0xffff;
    flash_buffer[index].flashData_u64 = 0;
  }  
  flash_buffer_head_u8 = 0;
  flash_buffer_tail_u8 = 0;
}


/**
* @brief  Get all the addresses, begin/end address offset for settings structures of modules
* @param  None
*         
* @retval None
*/
uint16_t current_Index_u16 = 0;
void Get_Flash_Index(void)
{
  for(uint16_t current_Id_u16= MIN_MODULE_ID; current_Id_u16< TOTAL_NUM_OF_PROCESSES; current_Id_u16++)
  {
    // Module settings begin offset (previous module settings end index)
    flash_settings_index[current_Id_u16].module_settings_begin_offset_u16 = current_Index_u16; // Module Memory Index Begin
    uint8_t module_Index_u8 = 0;
    uint16_t settings_size_u16 = 0;
    switch(current_Id_u16)
    {
    case MODULE_ANALOG_0_10V:	
      module_Index_u8 = getProcessInfoIndex(MODULE_ANALOG_0_10V);
      flash_settings_index[current_Id_u16].module_settings_address_ptr= ((*(processInfoTable[module_Index_u8].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8); // Module Structured memory address
      flash_AnalogVoltsControl_ptr = (AnalogVolts_Control*)((*(processInfoTable[module_Index_u8].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8); // Module Structured memory address
      settings_size_u16 = sizeof((*flash_AnalogVoltsControl_ptr).analogVolts_Setting);      
      break;
      
    case MODULE_ANALOG_4_20MA:
      module_Index_u8 = getProcessInfoIndex(MODULE_ANALOG_4_20MA);
      flash_settings_index[current_Id_u16].module_settings_address_ptr= ((*(processInfoTable[module_Index_u8].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8); // Module Structured memory address
      flash_AnalogAmpsControl_ptr = (Analog_4_20ma_Control*)((*(processInfoTable[module_Index_u8].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8); // Module Structured memory address
      settings_size_u16 = sizeof((*flash_AnalogAmpsControl_ptr).analog_4_20mA_Setting);
      break;
      
    case MODULE_DIGITAL_INPUTS:
      module_Index_u8 = getProcessInfoIndex(MODULE_DIGITAL_INPUTS);
      flash_settings_index[current_Id_u16].module_settings_address_ptr= ((*(processInfoTable[module_Index_u8].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8); // Module Structured memory address
      flash_DigitalInputsControl_ptr = (DigitalInputs_Control*)((*(processInfoTable[module_Index_u8].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8); // Module Structured memory address
      settings_size_u16 = sizeof((*flash_DigitalInputsControl_ptr).digitalInputs_Setting);
      break;
      
    case MODULE_DIGITAL_OUTPUTS:
      module_Index_u8 = getProcessInfoIndex(MODULE_DIGITAL_OUTPUTS);
      flash_settings_index[current_Id_u16].module_settings_address_ptr= ((*(processInfoTable[module_Index_u8].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8); // Module Structured memory address
      flash_DigitalOutputsControl_ptr = (Digital_Outputs_Control*)((*(processInfoTable[module_Index_u8].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8); // Module Structured memory address
      settings_size_u16 = sizeof((*flash_DigitalOutputsControl_ptr).digital_outputs_Setting);
      break;
      
    case MODULE_MODBUS:
      module_Index_u8 = getProcessInfoIndex(MODULE_MODBUS);
      flash_settings_index[current_Id_u16].module_settings_address_ptr= ((*(processInfoTable[module_Index_u8].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8); // Module Structured memory address
      flash_ModbusControl_ptr = (Modbus_Control*)((*(processInfoTable[module_Index_u8].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8); // Module Structured memory address
      settings_size_u16 = sizeof((*flash_ModbusControl_ptr).modbus_Settings);
      break;
    
    case MODULE_MOTOR_COM:
      module_Index_u8 = getProcessInfoIndex(MODULE_MOTOR_COM);
      flash_settings_index[current_Id_u16].module_settings_address_ptr= ((*(processInfoTable[module_Index_u8].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8); // Module Structured memory address
      flash_MotorComControl_ptr = (Motor_Com_Control*)((*(processInfoTable[module_Index_u8].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8); // Module Structured memory address
      settings_size_u16 = sizeof((*flash_MotorComControl_ptr).motor_Setting);
      break;
      
    case MODULE_MOTOR_DEMAND_MUX:
      module_Index_u8 = getProcessInfoIndex(MODULE_MOTOR_DEMAND_MUX);
      flash_settings_index[current_Id_u16].module_settings_address_ptr= ((*(processInfoTable[module_Index_u8].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8); // Module Structured memory address
      flash_MotorDemandMuxControl_ptr = (MotorDemandMux_Control*)((*(processInfoTable[module_Index_u8].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8); // Module Structured memory address
      settings_size_u16 = sizeof((*flash_MotorDemandMuxControl_ptr).motorDemandMux_Settings);
      break;
    
    case MODULE_PWM_INPUT:
      module_Index_u8 = getProcessInfoIndex(MODULE_PWM_INPUT);
      flash_settings_index[current_Id_u16].module_settings_address_ptr= ((*(processInfoTable[module_Index_u8].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8); // Module Structured memory address
      flash_PwmInputControl_ptr = (PwmInput_Control*)((*(processInfoTable[module_Index_u8].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8); // Module Structured memory address
      settings_size_u16 = sizeof((*flash_PwmInputControl_ptr).pwmInput_Settings);
      break;
      
    case MODULE_GPIO:
      module_Index_u8 = getProcessInfoIndex(MODULE_GPIO);
      flash_settings_index[current_Id_u16].module_settings_address_ptr= ((*(processInfoTable[module_Index_u8].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8); // Module Structured memory address
      flash_GpioControl_ptr = (Gpio_Control*)((*(processInfoTable[module_Index_u8].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8); // Module Structured memory address
      settings_size_u16 = sizeof((*flash_GpioControl_ptr).gpio_Settings);
      break;
        
    }
    if(settings_size_u16 % 8 != 0)
    {
      current_Index_u16 = current_Index_u16 + settings_size_u16 + (8 - (settings_size_u16% 8 )); // Since min len we can write is 64 bits (8 bytes)
    }
    else
    {
      current_Index_u16 = current_Index_u16 + settings_size_u16; 
    }
    if(flash_settings_index[current_Id_u16].module_settings_end_offset_u16 != current_Index_u16)
    {
      // Module settings end offset (# of bytes + begin offset)
      flash_settings_index[current_Id_u16].module_settings_size_u16 = settings_size_u16;       // Size of settings structure. This does not have the 8 byte padding
      flash_settings_index[current_Id_u16].module_settings_end_offset_u16= current_Index_u16;  // Number of Bytes // Module Memory Index_End // Last address of the analog settings
    }
  }
}

// Update flash settings 
/*void Update_Flash(uint8_t module_id_u8, uint8_t flash_state_u8)
{
  switch(flash_state_u8)
  {
  case FLASH_SETTINGS_INIT:
    init_Flash_Settings();
    break;
  case FLASH_SETTINGS_UPDATE:
    update_Flash_Settings(module_id_u8);
    break;
  case FLASH_SETTINGS_ERASE:
    break;
  default:
    break;
  }
  
}*/



/**
* @brief  Init flash setting on first power up
* @param  index_u32  Index of where the data need to be flashed
*         ptr_u8     Address from where we have to copy data
* @retval None
*/
uint8_t init_Flash_Settings(void)
{
  uint16_t current_flash_index_u16 = 0;
  uint8_t error_u8 = 0;
  uint8_t current_Id = 0;
  for(current_Id= MIN_MODULE_ID; current_Id< TOTAL_NUM_OF_PROCESSES; current_Id++)
  {
    Flash_Buf_Init(); // Clear any junk values in buffer
    uint16_t module_settings_index_u8= 0;
    if(flash_settings_index[current_Id].module_settings_begin_offset_u16 != flash_settings_index[current_Id].module_settings_end_offset_u16)
    { 
      flash_settings_index[current_Id].module_flash_settings_address_ptr = flash_settings_index[current_Id].module_settings_begin_offset_u16 + FLASH_SETTINGS_START_ADDRESS;
      // Copy 8 bytes at a time into flash_buffer
      for(current_flash_index_u16 = flash_settings_index[current_Id].module_settings_begin_offset_u16; current_flash_index_u16 < flash_settings_index[current_Id].module_settings_end_offset_u16;  )
      {
        if(flash_settings_index[current_Id].module_settings_begin_offset_u16 != flash_settings_index[current_Id].module_settings_end_offset_u16)
        {  
          uint32_t address_u32 = FLASH_SETTINGS_START_ADDRESS+current_flash_index_u16;
          if( address_u32 > (FLASH_SETTINGS_START_ADDRESS + FLASH_PAGE_SIZE) )
          {
            error_u8 = 1;
          }
          else
          {
            copy_Data_To_Buffer( address_u32, (flash_settings_index[current_Id].module_settings_address_ptr + module_settings_index_u8) ); // 
          }
        }
        current_flash_index_u16 = current_flash_index_u16 + 8;
        module_settings_index_u8 = module_settings_index_u8 + 8;
      }
    }
    // Once all settings for a module are copied to buffer then flash them
    if(flash_buffer_tail_u8 != flash_buffer_head_u8)
    {
      flashBlockProgram8bytes( current_Id,(flash_settings_index[current_Id].module_settings_begin_offset_u16+FLASH_SETTINGS_START_ADDRESS), (flash_settings_index[current_Id].module_settings_end_offset_u16 - flash_settings_index[current_Id].module_settings_begin_offset_u16)/8 );
    }
  }
  return(error_u8);
  
}


/**
* @brief  Copy 8 bytes of data to buffer that needs to be flashed into flash
* @param  index_u32  Index of where the data need to be flashed
*         ptr_u8     Address from where we have to copy data
* @retval None
*/
void copy_Data_To_Buffer(uint32_t index_u32, uint8_t* ptr_u8)
{
  flash_buffer[flash_buffer_tail_u8].address_u32 = index_u32;  
  uint64_t data_u64 = 0;
  for(uint8_t byte_u8 = 0; byte_u8 < 8; byte_u8++)
  {
    data_u64 = (uint64_t)((uint64_t)data_u64 + (uint64_t)((uint64_t)(*(ptr_u8+(byte_u8))) << (byte_u8*8)) );
    
  }
  flash_buffer[flash_buffer_tail_u8].flashData_u64 = data_u64;
  flash_buffer_tail_u8++;
}


// If a module setting is update then the whole module setting in the flash need to be updated
/**
* @brief  Update a a specific block of momory related to a module
* @param  module_id_u8       The function caller module ID in case error occur within this function
* @retval successful
*/
uint8_t update_Flash_Settings(uint8_t module_id_u8)
{
  uint8_t error_u8=0;
  for(uint8_t current_Id= MIN_MODULE_ID; current_Id< TOTAL_NUM_OF_PROCESSES; current_Id++)
  {    
    if(module_id_u8 == current_Id)
    {
      FlashBufInit(); // Clear buffer
      // Copy module settings into buffer
      for(uint16_t current_flash_index_u16 = flash_settings_index[current_Id].module_settings_begin_offset_u16; current_flash_index_u16 < flash_settings_index[current_Id].module_settings_end_offset_u16; )
      {
        uint32_t address_u32 = FLASH_SETTINGS_START_ADDRESS + current_flash_index_u16;
        if( address_u32 > (FLASH_SETTINGS_START_ADDRESS + FLASH_PAGE_SIZE - 8) ) // Last 8 bytes for Flash Version (4 bytes) + CRC (4 bytes)
        {
          error_u8 = 1;
        }
        else
        {
          copy_Data_To_Buffer( address_u32, (flash_settings_index[current_Id].module_settings_address_ptr) ); // 
        }
        current_flash_index_u16= current_flash_index_u16 + 8;
      }      
      flashBlockProgram8bytes( current_Id,(flash_settings_index[current_Id].module_settings_begin_offset_u16+FLASH_SETTINGS_START_ADDRESS), (flash_buffer_tail_u8 - flash_buffer_head_u8) );      
    }
    
  }
  return(error_u8);
    
}


/**
* @brief  Program a data in flash_buffer into flash
* @param  module_id_u8       The function caller module ID in case error occur within this function
*         _TopageAddress  The starting address of the sink flash page/s
*          _length        The starting address of the source flash page/s
* @retval successful
*/
uint8_t flashBlockProgram8bytes(uint8_t module_id_u8, uint32_t _TopageAddress, uint32_t _length)
{
  uint8_t returnValue = TRUE;
  HAL_FLASH_Unlock();
  for(uint16_t index_u16 = flash_buffer_head_u8  ;  index_u16 < _length ; index_u16 ++)
  {
    if (!HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, flash_buffer[index_u16].address_u32, flash_buffer[index_u16].flashData_u64) == HAL_OK) //write 16bit data
    {
      // Error occurred while writing data in Flash memory.                                                                 
      setupSoftwareIRQ(module_id_u8, MODULE_ERR_LOGHANDLE, 0xE4, 0x00, 0x00, NULL);     //report the error address 
      returnValue = FALSE;
    } else
    {
      flash_buffer_head_u8++;
    }
  }
  // Lock the Flash to disable the flash control register access (recommended to protect the FLASH memory against possible unwanted operation) 
  HAL_FLASH_Lock();
  return returnValue;
}

/**
* @brief  Get address of a setting in flash memory
* @param  module_id_u8                 The function caller module ID in case error occur within this function
*         *module_address_ptr       Module settings strucuture address
*         *setting_address_ptr      Address of setting in the structure
* @retval setting_address_flash_ptr Address of setting in flash memory
*/
uint32_t flashGetSettingAddress(uint8_t module_id_u8, uint8_t *module_address_ptr, uint8_t *setting_address_ptr)
{
  uint16_t offset_u16 = 0;
  uint32_t module_falsh_address_ptr;
  uint32_t setting_address_flash_ptr;  
  module_falsh_address_ptr = flash_settings_index[module_id_u8].module_flash_settings_address_ptr;
  offset_u16 = setting_address_ptr - module_address_ptr ;
  setting_address_flash_ptr = module_falsh_address_ptr + offset_u16;
  return(setting_address_flash_ptr);
}

/**
* @brief  Check if flash is empty by looking at the CRC
* @param  module_id_u8                 The function caller module ID in case error occur within this function
*         *module_address_ptr       Module settings strucuture address
*         *setting_address_ptr      Address of setting in the structure
* @retval setting_address_flash_ptr Address of setting in flash memory
*/
uint8_t check_flash_empty(uint32_t page_start_address_u32, uint16_t page_size_u16)
{
  uint8_t *crc_address_u32;
  uint64_t crc_falsh_data_u64= 0; 
  crc_address_u32 = (uint8_t *)(page_start_address_u32 + page_size_u16 - 8); // Last 4 bytes is CRC
  crc_falsh_data_u64 = ((uint64_t)*crc_address_u32) << 56       + ((uint64_t)*(crc_address_u32 + 1)) << 48 + 
                       ((uint64_t)*(crc_address_u32 + 2)) << 40 + ((uint64_t)*(crc_address_u32 + 3)) << 32 + 
                       ((uint64_t)*(crc_address_u32 + 4)) << 24 + ((uint64_t)*(crc_address_u32 + 5)) << 16 +
                       ((uint64_t)*(crc_address_u32 + 6)) << 8  + ((uint64_t)*(crc_address_u32 + 7))  ;
  if( crc_falsh_data_u64 == 0xFFFFFFFF)
  {
    return(FLASH_EMPTY); 
  }
  else
  {
    return(FLASH_NOT_EMPTY);  
  }
  
}

/**
* @brief  Write CRC32 and flash version to flash
* @param  module_id_u8       The function caller module ID in case error occur within this function
*         _TopageAddress  The starting address of the sink flash page/s
* @retval successful
*/
uint8_t flashWriteCRC32Version(uint8_t module_id_u8, uint32_t _TopageAddress)
{
  uint8_t returnValue = TRUE;
  HAL_FLASH_Unlock();
  
  uint32_t uwCRCValue_u32 = Calculate_CRC32((uint32_t)((FLASH_PAGE_SIZE * NUMBER_OF_FLASH_PAGES) - 2) , (unsigned char*)_TopageAddress);    
  //put calculated CRC back to the last word of the page
  uint64_t data_u64 = ((uint64_t)uwCRCValue_u32 << 32) + flashSettingsVersion_u32; // Flash writes MSB LSB in senond and first address resp
  uint16_t offset_u16 = _TopageAddress + PAGE_SIZE - 8; // last 8 bytes are FLASH_VERSION (4 bytes) + CRC (4 bytes)
  if (!HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, offset_u16, data_u64) == HAL_OK) //write 64bit data
  {
    // Error occurred while writing data in Flash memory.                                                                 
    setupSoftwareIRQ(module_id_u8, MODULE_ERR_LOGHANDLE, 0xE4, 0x00, 0x00, NULL);     //report the error address
    returnValue = FALSE;
  }
  
  // Lock the Flash to disable the flash control register access (recommended to protect the FLASH memory against possible unwanted operation) 
  HAL_FLASH_Lock();
  return returnValue;
}

uint16_t value_u16 = 0;
void getSettingsFromFalsh(uint8_t module_id_u8)
{ // Get setting from flash and update the RAM
  uint16_t length_u16 = 0; // Lenght of module data in falsh 
  uint32_t module_falsh_address_ptr;  // Address of structure in FALSH      
  uint8_t *module_settings_struct_address_ptr; // Address of structure in RAM
  module_falsh_address_ptr = flash_settings_index[module_id_u8].module_flash_settings_address_ptr;
  module_settings_struct_address_ptr = flash_settings_index[module_id_u8].module_settings_address_ptr;
  length_u16 = flash_settings_index[module_id_u8].module_settings_size_u16;
  
  // Copy each byte info RAM from Flash
  for (uint16_t index_u16 = 0; index_u16 < length_u16; index_u16++)
  {
     *(module_settings_struct_address_ptr + index_u16) = *((uint8_t *)module_falsh_address_ptr + index_u16); // Copy data from falsh into RAM
  }
}