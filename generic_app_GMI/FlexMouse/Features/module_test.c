/**
*************************************************************************************
* @file    module_test.c 
* @author  Regal, Satya Akkina
* @version V1.0
* @date    Oct 05th 2021
* @brief   module to support testers
* @note    Provide functions that support various test requirements for production and debug
*************************************************************************************
*/

// Includes -------------------------------------------------------------------
#include "module_test.h"


#include "module_gpio.h"
#include "driver_usart2.h"
#include "module_usart2.h"
// Content --------------------------------------------------------------------
// Function Prototypes
void AssignTestModuleMem(void);
void Init_Module_Test_Setting(void);
void Init_Module_Test_Data(void);
void ProcessTestMessages(void);

Operating_Status Change_Operating_Mode(void);
void clear_Command_Data(void);
void clear_Password(void);

Usart2_Control* moduleTest_usart2_Control_ptr;

// Constants
#define ADMIN_PASSWORD (0X523667403141646d) // R6g@1Adm (ASCII) in Hex

// Module States
enum {
  MEMORY_INIT_MODULE,
  INIT_MODULE,
  RUN_MODULE,
  // Ddditional states to be added here as necessary.
  IRQ_MODULE = DEFAULT_IRQ_STATE,
  KILL_MODULE = KILL_APP
};

// External Variables
extern Ram_Buf sharedMemArray[TOTAL_NUM_OF_STRUCT_MEM_INSTANCES];
extern ProcessInfo processInfoTable[];

// Global variables specific to this module
static  Ram_Buf_Handle module_test_StructMem_u32;
ModuleTest_Control moduleTest_Control;
uint64_t module_test_time_u64 = 0;

// Define Pointers that will be used as References to other Modules, where applicable

// Local variable
Operating_Modes current_set_operating_mode_u16 = ENTER_STD_MODE;

/**
********************************************************************************************************************************
* @brief   State machine for Test Module
* @details
* @retval  return_state_u8
********************************************************************************************************************************
*/
uint8_t moduleTest_u32(uint8_t drv_identifier_u8, uint8_t previous_state_u8, uint8_t next_state_u8,
                            uint8_t irq_identifier_u8)
{
  //local variables  
  uint8_t return_state_u8 = MEMORY_INIT_MODULE;
  
  switch (next_state_u8)
  {
  case MEMORY_INIT_MODULE:
    {
      AssignTestModuleMem(); // Assign structured memory 
      return_state_u8 = INIT_MODULE;
      break;
    }
  case INIT_MODULE:
    {          
      Init_Module_Test_Setting();  // Initilize PWM Input settings        
      Init_Module_Test_Data();     // Init PWM input live data
      
      /*Attach Uart2 structured memory into this App*/         
      uint8_t Usart2index  = getProcessInfoIndex(MODULE_USART2); //return Process index from processInfo array
      moduleTest_usart2_Control_ptr = (Usart2_Control*)((*(processInfoTable[Usart2index].Sched_DrvData.p_masterSharedMem_u32)).p_ramBuf_u8);    //Get structured memory for USART2
      
//      // Get structured memory for TIM1 data
//      uint8_t module_TIM1_Index = getProcessInfoIndex(MODULE_TIM1);
//      tim1_LocalControl = (TIM1_Control*)((*(processInfoTable[module_TIM1_Index].Sched_DrvData.p_masterSharedMem_u32)).p_ramBuf_u8);
//      
//      // Get structured memory for GPIO module data
//      uint8_t module_gpio_index_u8 = getProcessInfoIndex(MODULE_GPIO);
//      gpio_PWMInputLocalControl = (Gpio_Control*)((*(processInfoTable[module_gpio_index_u8].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8);
      
      module_test_time_u64 = getSysCount() + moduleTest_Control.moduleTest_Settings.testCheckPeriod_u16;   // Store next poll time value for the module
      return_state_u8 = RUN_MODULE;
      break;
    }   
  case RUN_MODULE:                                                             
    {
      // Process test module every "testCheckPeriod" mSec
      if (getSysCount() >= module_test_time_u64) 
      { 
        if(current_set_operating_mode_u16 != moduleTest_Control.moduleTest_Settings.setOperatingMode_u16)
        { // Check to see if current operating mode needs to be changed
          moduleTest_Control.moduleTest_Data.operatingStatus_u16  = Change_Operating_Mode();
          current_set_operating_mode_u16 = moduleTest_Control.moduleTest_Settings.setOperatingMode_u16 ;
        }
        if(moduleTest_Control.moduleTest_Data.testCommandStatus_u16 != WAITING_TO_ENTER_TEST_MODE)
        { // In test mode already
          if( ( (moduleTest_Control.moduleTest_Settings.testCommand_u16)!= NO_TEST_CMD))
          {
            if((moduleTest_Control.moduleTest_Data.testCommandStatus_u16 == TEST_STATUS_OK) )
          { // Process test commands
            moduleTest_Control.moduleTest_Data.testCommandStatus_u16 = TEST_BUSY;
            ProcessTestMessages();
          }
          }
        }

      }     
      return_state_u8 = RUN_MODULE;
      break;
    }
  case IRQ_MODULE: 
    {
      // If there are more than one interrupts, from different drivers, you can identify each individually by:
      return_state_u8 = RUN_MODULE;
      break;
    }
    
  case KILL_MODULE: 
    {
      // Setting processStatus_u8 to PROCESS_STATUS_KILLED prevents the scheduler main loop from calling this module again.
      uint8_t table_index_u8 = getProcessInfoIndex(drv_identifier_u8);
      if (table_index_u8 != INDEX_NOT_FOUND) {
        processInfoTable[table_index_u8].Sched_DrvData.processStatus_u8 = PROCESS_STATUS_KILLED;
      }
      return_state_u8 = INIT_MODULE;
      break;
    }
  default:
    {
      return_state_u8 = KILL_MODULE; //10; 
      break;
    }
  }
  return return_state_u8;
} 

/**
********************************************************************************************************************************
* @brief   Assign structured memory
* @details Assign structured memory for Digital PWM control
* @retval  None
********************************************************************************************************************************
*/
//
void AssignTestModuleMem(void){   
  module_test_StructMem_u32 =  StructMem_CreateInstance(MODULE_TEST, sizeof(ModuleTest_Control), ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);
  (*module_test_StructMem_u32).p_ramBuf_u8 = (uint8_t *)&moduleTest_Control ;    // Map the TIM1 memory into the structured memory
  uint8_t Module_Digital_Pwm_Index = getProcessInfoIndex(MODULE_TEST);
  processInfoTable[Module_Digital_Pwm_Index].Sched_ModuleData.p_masterSharedMem_u32 = (Ram_Buf_Handle)module_test_StructMem_u32;
}

/**
********************************************************************************************************************************
* @brief   Initilize settings for this module
* @details 
* @retval  None
********************************************************************************************************************************
*/
void Init_Module_Test_Setting()
{
  clear_Password();     // Clear password
  moduleTest_Control.moduleTest_Settings.setOperatingMode_u16 = ENTER_STD_MODE;   // Operating mode set by Modbus or UP
  moduleTest_Control.moduleTest_Settings.testCommand_u16 = NO_TEST_CMD; // Test command to be processed
  moduleTest_Control.moduleTest_Settings.testCheckPeriod_u16 = 10; // Poll time for test module
  moduleTest_Control.moduleTest_Settings.flags_u16.empty01 = 0;    // Coil/falgs
  
  clear_Command_Data(); // Clear command array
}

/**
********************************************************************************************************************************
* @brief   Initilize live data for this module
* @details 
* @retval  None
********************************************************************************************************************************
*/
void Init_Module_Test_Data()
{
  moduleTest_Control.moduleTest_Data.operatingStatus_u16 = STD_MODE;       // Current operating mode. See Operating Modes
  moduleTest_Control.moduleTest_Data.testCommandStatus_u16 = WAITING_TO_ENTER_TEST_MODE;     // Test command to be processed
  moduleTest_Control.moduleTest_Data.discretes_u16.isAdminMode = FALSE; // Disable Admin mode
  moduleTest_Control.moduleTest_Data.discretes_u16.isTestMode = FALSE;  // Disable test mode
}

/**
********************************************************************************************************************************
* @brief   Process the test messages
* @details 
* @retval  None
********************************************************************************************************************************
*/
void ProcessTestMessages()
{
  switch(moduleTest_Control.moduleTest_Settings.testCommand_u16)
  {
    case ANALOG_VOLTS_INPUT_CALIB_CMD:
    {

      break;   
    }
    case ANALOG_AMPS_INPUT_CALIB_CMD:
    {
      
      break;   
    }
    case SET_USER_SETTINGS_CRC_CMD:
    {
      
      break;   
    }
    case SET_DEFAULT_SETTINGS_CRC_CMD:
    {
      
      break;   
    }
    case SET_APP_VERSION_CMD:
    {
      
      break;   
    }
    case SET_FLASH_VERSION_CMD:
    {
      
      break;   
    }
    case SET_DIGITAL_OUTPUT_CMD:
    {
      uint8_t output_num_u8 = 0;
      uint8_t pin_State_u8 = 0;
      output_num_u8 = (uint8_t)(moduleTest_Control.moduleTest_Settings.testCommandData_u16[0] & 0x00FF);
      pin_State_u8  = (uint8_t)((moduleTest_Control.moduleTest_Settings.testCommandData_u16[0] >>8) & 0x00FF);
      Module_Gpio_WriteGpioState(output_num_u8, pin_State_u8 );
      
      break;   
    }
    case SET_BURN_IN_CMD:
    {
      // Send burn-in command to motor control
      unsigned char speedTx[] = {0x55, 0x03, 0x23, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xCC, 0xCC};
      
      unsigned int speedLen = sizeof(speedTx);
      
      // Command 0 - Burn-in frequency
      // Command 1 - Burn-in Current
      // Command 2 - Burn-in Enable (LSB only)
      speedTx[5] = (unsigned char) ( (moduleTest_Control.moduleTest_Settings.testCommandData_u16[0] >> 8) & ( 0x00FF) ); // Frequency MSB
      speedTx[6] = (unsigned char) (moduleTest_Control.moduleTest_Settings.testCommandData_u16[0] & 0x00FF); // Frequency LSB
      speedTx[7] = (unsigned char) ( (moduleTest_Control.moduleTest_Settings.testCommandData_u16[1] >> 8) & ( 0x00FF) ); // Current MSB
      speedTx[8] = (unsigned char) (moduleTest_Control.moduleTest_Settings.testCommandData_u16[1] & 0x00FF); // Current LSB
      speedTx[9] = (unsigned char) ( (moduleTest_Control.moduleTest_Settings.testCommandData_u16[2]) & ( 0x00FF) ); // Burn-in enable
      
      RingBuf_WriteBlock((*moduleTest_usart2_Control_ptr).seqMemTX_u32, speedTx, &speedLen);
      
      break; 
    }
    default:
    {
      
      break;   
    }   
  }
  clear_Command_Data();
  moduleTest_Control.moduleTest_Settings.testCommand_u16 = NO_TEST_CMD;
  moduleTest_Control.moduleTest_Data.testCommandStatus_u16 = TEST_STATUS_OK;
}

/**
********************************************************************************************************************************
* @brief   Verify the admin password and sets "isAdminMode" flag accordingly
* @details 
* @retval  Operating_Status: Returns the PASSWORD_PASS or PASSWORD_FAIL
********************************************************************************************************************************
*/
Operating_Status Verify_Password()
{
  uint64_t set_password_u64 = 0;
  Operating_Status status_u16 = PASSWORD_FAIL;
  moduleTest_Control.moduleTest_Data.discretes_u16.isAdminMode = FALSE;
  set_password_u64 = (((uint64_t)moduleTest_Control.moduleTest_Settings.testCommandData_u16[3]) << 48) + 
                     (((uint64_t)moduleTest_Control.moduleTest_Settings.testCommandData_u16[2]) << 32) +
                     (((uint64_t)moduleTest_Control.moduleTest_Settings.testCommandData_u16[1]) << 16) +
                      ((uint64_t)moduleTest_Control.moduleTest_Settings.testCommandData_u16[0]);
  if(set_password_u64 == ADMIN_PASSWORD)
  {  
    status_u16 = PASSWORD_PASS;
    moduleTest_Control.moduleTest_Data.discretes_u16.isAdminMode = TRUE;
  }
  clear_Command_Data();
  return(status_u16);
}

/**
********************************************************************************************************************************
* @brief   Change the current operating mode
* @details 
* @retval  Operating_Status: Returns the status to change mode or fail
********************************************************************************************************************************
*/
Operating_Status Change_Operating_Mode()
{
  Operating_Status Operating_status_u16 = UNKNOWN_OPERATING_STATUS;
  Test_Command_Status test_command_status_u16 = WAITING_TO_ENTER_TEST_MODE;
  switch(moduleTest_Control.moduleTest_Settings.setOperatingMode_u16)
  {
  
    case ENTER_ADMIN_MODE: // Admin mode. Includes admin access
    {
      Operating_status_u16 = Verify_Password();
      
      if(Operating_status_u16 == PASSWORD_PASS)
      {
        Operating_status_u16 = ADMIN_MODE;
        moduleTest_Control.moduleTest_Data.discretes_u16.isTestMode = FALSE;
      } else
      {
        Operating_status_u16 = ENTER_ADMIN_MODE_FAIL;
      }
      break;
    }   
    case ENTER_TEST_MODE:
    {
      Operating_status_u16= ENTER_TEST_MODE_FAIL;
      if(moduleTest_Control.moduleTest_Data.discretes_u16.isAdminMode == TRUE)
      {
        Operating_status_u16 = TEST_MODE;
        test_command_status_u16 = TEST_STATUS_OK;
        moduleTest_Control.moduleTest_Data.discretes_u16.isTestMode = TRUE;
      } 
      break;    
    }
    case ENTER_HW_FCT_TEST_MODE: // FCT test mode. Includes admin access
    {
      Operating_status_u16= ENTER_HW_FCT_TEST_MODE_FAIL;
      if(moduleTest_Control.moduleTest_Data.discretes_u16.isAdminMode == TRUE)
      {
        Operating_status_u16 = HW_FCT_TEST_MODE;
        test_command_status_u16 = TEST_STATUS_OK;
        moduleTest_Control.moduleTest_Data.discretes_u16.isTestMode = TRUE;
      }      
      break;
    }
    case ENTER_HW_ICT_TEST_MODE: // ICT test mode. Includes admin access
    {  
      Operating_status_u16 = ENTER_HW_ICT_TEST_MODE_FAIL;
      if(moduleTest_Control.moduleTest_Data.discretes_u16.isAdminMode == TRUE)
      {
        Operating_status_u16 = HW_ICT_TEST_MODE;
        test_command_status_u16 = TEST_STATUS_OK;
        moduleTest_Control.moduleTest_Data.discretes_u16.isTestMode = TRUE;
      } 
      break;
    }
    case ENTER_HW_BI_TEST_MODE:  // BI test mode. Includes admin access
    {  
      Operating_status_u16 = ENTER_HW_BI_TEST_MODE_FAIL;
      if(moduleTest_Control.moduleTest_Data.discretes_u16.isAdminMode == TRUE)
      {
        Operating_status_u16 = HW_BI_TEST_MODE;
        test_command_status_u16 = TEST_STATUS_OK;
        moduleTest_Control.moduleTest_Data.discretes_u16.isTestMode = TRUE;
      } 
      break;
    }
    case ENTER_SW_TEST_MODE:  // Software test mode
    {  
      Operating_status_u16 = ENTER_HW_FCT_TEST_MODE_FAIL;
      if(moduleTest_Control.moduleTest_Data.discretes_u16.isAdminMode == TRUE)
      {
        Operating_status_u16 = SW_TEST_MODE;
        test_command_status_u16 = TEST_STATUS_OK;
        moduleTest_Control.moduleTest_Data.discretes_u16.isTestMode = TRUE;
      } 
      break;
    }
    
    case ENTER_STD_MODE: // Exit test mode and admin mode
    {
      Operating_status_u16 = STD_MODE;
      test_command_status_u16 = WAITING_TO_ENTER_TEST_MODE;
      moduleTest_Control.moduleTest_Data.discretes_u16.isTestMode = FALSE;
      moduleTest_Control.moduleTest_Data.discretes_u16.isAdminMode = FALSE;
      clear_Command_Data();      
      break;        
    }
    default:
    {
      Operating_status_u16 = ENTER_SW_TEST_MODE_FAIL;
      moduleTest_Control.moduleTest_Settings.setOperatingMode_u16 = ENTER_STD_MODE;
      moduleTest_Control.moduleTest_Data.discretes_u16.isTestMode = FALSE;
      break;
    }      
  } 
  moduleTest_Control.moduleTest_Data.testCommandStatus_u16 = test_command_status_u16;
  return(Operating_status_u16);
}

/**
********************************************************************************************************************************
* @brief   Clear command array buffer
* @details 
* @retval  None
********************************************************************************************************************************
*/
void clear_Command_Data()
{
  for(uint8_t index_u8=0; index_u8 < MAX_TEST_CMD_DATA_LENGTH_IN_WORDS; index_u8++)
  {
    moduleTest_Control.moduleTest_Settings.testCommandData_u16[index_u8]=0;
  } 
}


/**
********************************************************************************************************************************
* @brief   Clear password
* @details 
* @retval  None
********************************************************************************************************************************
*/
void clear_Password()
{
  for(uint8_t index_u8=0; index_u8 < MAX_PASSWORD_LENGTH_IN_WORDS; index_u8++)
  {
    moduleTest_Control.moduleTest_Settings.testCommandData_u16[index_u8]=0;
  } 
}