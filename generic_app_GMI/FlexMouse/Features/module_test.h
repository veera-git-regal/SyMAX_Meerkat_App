/**
  *****************************************************************************
  * @file    module_test.h 
  * @author  Regal, Satya Akkina
  * @version V1.0
  * @date    05-Oct-2021
  * @brief   Header for "Module Test"
  * @note    
  *****************************************************************************
  */

// Define to prevent recursive inclusion 
#ifndef _MODULE_TEST_H_
#define _MODULE_TEST_H_

#ifdef __cplusplus
extern "C" {
#endif
  
// Includes 
#include "driver_adc1.h" 

#include "structured_memory.h"
#include "scheduler.h"
  
extern Ram_Buf sharedMemArray[TOTAL_NUM_OF_STRUCT_MEM_INSTANCES];
extern ProcessInfo processInfoTable[];
extern Ram_Buf *module_Test_Settings_StructMem_u32;
extern Ram_Buf *module_Test_Data_StructMem_u32;

#define MAX_TEST_CMD_DATA_LENGTH_IN_WORDS 4  // Words (uint16_t)
#define MAX_PASSWORD_LENGTH_IN_WORDS 4  // Words (uint16_t)

// Operating modes
typedef enum 
{
  ENTER_STD_MODE = 0,           // Non-test mode
  ENTER_ADMIN_MODE,       // Adminstrator mode
  ENTER_TEST_MODE,        // Generic test mode. Includes admin access
  ENTER_HW_FCT_TEST_MODE, // Set to FCT test mode. Includes admin access
  ENTER_HW_ICT_TEST_MODE, // Set to ICT test mode. Includes admin access
  ENTER_HW_BI_TEST_MODE,  // Set to BI test mode. Includes admin access
  ENTER_EOL_TEST_MODE,    // Set to EOL test mode. Includes admin access
  ENTER_SW_TEST_MODE,     // Set to ths for SW debugging test. Includes admin access
  UNKNOWN_OPERATING_MODE = 0xFFFF
}Operating_Modes;

// Operating Status
typedef enum
{
  STD_MODE = 0, 
  OPERATION_MODE_OK, // Normal operating mode 
  PASSWORD_FAIL,    // Entered password wrong
  PASSWORD_PASS,    // Password is accepted
  ADMIN_MODE,       // Adminstrator mode
  TEST_MODE,        // Test mode
  HW_FCT_TEST_MODE, // FCT test mode. Includes admin access
  HW_ICT_TEST_MODE, // ICT test mode. Includes admin access
  HW_BI_TEST_MODE,  // BI test mode. Includes admin access
  EOL_TEST_MODE,    // EOL test mode.
  SW_TEST_MODE,     // Software test mode
  
  ENTER_ADMIN_MODE_FAIL,       // Enter admin mode failed
  ENTER_TEST_MODE_FAIL,        // Faile to enter the test mode
  ENTER_HW_FCT_TEST_MODE_FAIL, // Enter hardware FCT test mode fail
  ENTER_HW_ICT_TEST_MODE_FAIL, // Enter hardware ICT test mode fail
  ENTER_HW_BI_TEST_MODE_FAIL,  // Enter hardwre burn-in test mode fail
  ENTER_SW_TEST_MODE_FAIL,     // Enter software tese mode fail
  
  UNKNOWN_OPERATING_STATUS = 0xFFFF
}Operating_Status;

// Test commands
enum
{
  //WAITING_TO_ENTER_TEST_MODE = 0,
  NO_TEST_CMD = 0,
  ANALOG_VOLTS_INPUT_CALIB_CMD,
  ANALOG_AMPS_INPUT_CALIB_CMD,
  SET_USER_SETTINGS_CRC_CMD,
  SET_DEFAULT_SETTINGS_CRC_CMD,
  SET_APP_VERSION_CMD,
  SET_FLASH_VERSION_CMD,
  SET_DIGITAL_OUTPUT_CMD,
  SET_BURN_IN_CMD,
  
  UNKNOWN_TEST_CMD = 0xFFFF  
};


// Test command status
typedef enum
{
  WAITING_TO_ENTER_TEST_MODE = 0,
  WAITING_FOR_TEST_COMMAND,
  TEST_STATUS_OK,
  TEST_BUSY,
  ANALOG_VOLTS_INPUT_CALIB,
  ANALOG_AMPS_INPUT_CALIB,
  SET_USER_SETTINGS_CRC,
  SET_DEFAULT_SETTINGS_CRC,
  SET_APP_VERSION,
  SET_FLASH_VERSION,
  ANALOG_VOLTS_INPUT_CALIB_PASS,
  ANALOG_AMPS_INPUT_CALIB_PASS,
  SET_USER_SETTINGS_CRC_PASS,
  SET_DEFAULT_SETTINGS_CRC_PASS,
  SET_APP_VERSION_PASS,
  SET_FLASH_VERSION_PASS,
  ANALOG_VOLTS_INPUT_CALIB_FAIL,
  ANALOG_AMPS_INPUT_CALIB_FAIL,
  SET_USER_SETTINGS_CRC_FAIL,
  SET_DEFAULT_SETTINGS_CRC_FAIL,
  SET_APP_VERSION_FAIL,
  SET_FLASH_VERSION_FAIL,
  
  UNKNOWN_TEST_STATUS  = 0xFFFF  
}Test_Command_Status;

//******************* Module Test Control (inside shared memory) ************  
// Module Test settings
typedef struct
{  
  //uint16_t	bits_convert_u16;			 // Union structure of all 16 bits in the field				
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
} ModuleTest_Flags;

struct ModuleTest_Settings
{ 
//  uint16_t setPassword_u16[MAX_PASSWORD_LENGTH_IN_WORDS];
//  uint16_t testCommandData[MAX_TEST_CMD_LENGTH];
//  uint16_t testCommandLenght;
  Operating_Modes setOperatingMode_u16;  // Operating mode set by Modbus or UP
  uint16_t testCommand_u16;       // Test command to be processed
  uint16_t testCommandData_u16[MAX_TEST_CMD_DATA_LENGTH_IN_WORDS];       // Test command to be processed
  uint16_t testCheckPeriod_u16;   // Poll time for test module
  ModuleTest_Flags flags_u16;     // Coil/flags
};

typedef struct
{  
  uint16_t	isAdminMode:1;   
  uint16_t	isTestMode:1; 
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
} ModuleTest_Discretes;

// Live Test Data
struct ModuleTest_Data
{  
  Operating_Status operatingStatus_u16;       // Current operating mode. See Operating Modes
  uint16_t testCommandStatus_u16;     // Test command to be processed
  ModuleTest_Discretes discretes_u16; // discrete bit indicators
};

typedef struct{
 struct ModuleTest_Settings moduleTest_Settings ;
 struct ModuleTest_Data moduleTest_Data;
}ModuleTest_Control;


//******* end of Module Test Control (inside shared memory) ****************

#ifdef __cplusplus
}
#endif

#endif /* _MODULE_TEST_H_ */

