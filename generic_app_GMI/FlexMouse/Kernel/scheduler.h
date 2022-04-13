/**
  ********************************************************************************************************************************
  * @file    scheduler.h 
  * @author  Pamela Lee
  * @brief   Header of c++ function/s for the kernel scheduler. 
  * @details Modules and drivers are collectively known as processes. This file contains function for scheduling processes. The
  *             file also contains functions for performing system resource requests such as creating and destroying sequential
  *             and/or structured memory instances. The function scheduler_run is the 'main loop' of the FlexMouse architecture.
  ********************************************************************************************************************************
  */

/* Define to prevent recursive inclusion ---------------------------------------------------------------------------------------*/
#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

//#include "stm32f4xx_hal.h" 
#include "stm32g0xx_hal.h" //SPA

/*USE_FULL_LL_DRIVER
HSE_VALUE=8000000
HSE_STARTUP_TIMEOUT=100
LSE_STARTUP_TIMEOUT=5000
LSE_VALUE=32768
EXTERNAL_CLOCK_VALUE=12288000
HSI_VALUE=16000000
LSI_VALUE=32000
VDD_VALUE=3300
PREFETCH_ENABLE=0
INSTRUCTION_CACHE_ENABLE=1
DATA_CACHE_ENABLE=1*/

#include "../Regal/typedef.h"

#include "sequential_memory.h"
#include "structured_memory.h"

/* Content ---------------------------------------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
  
//static uint64_t tickCounter = 0; //SPA
    
/* global parameters -----------------------------------------------------------------------------------------------------------*/  
static uint8_t reallocError = 0;       
    
/* User parameters -------------------------------------------------------------------------------------------------------------*/
// General
extern __root const uint32_t App_CRC @ "app_crc32_rom";// = 0x00000000; // This is a placeholder for App Firmware Checksum (CRC32)


#define MODULE_GPIO_ID                MODULE_GPIO
#define MODULE_GPIO_FUNCTION_POINTER  &moduleGPIO_u32
#define MODULE_GPIO_TOTAL_SEQ         0
#define MODULE_GPIO_TOTAL_STRUCT      1
#define MODULE_GPIO_PREV_STATE        0
#define MODULE_GPIO_NEXT_STATE        0
#define MODULE_GPIO_IRQ_STATUS        DEFAULT_IRQ_STATE
#define MODULE_GPIO_PROCESS_STATUS    0
#define MODULE_GPIO_MASTER_SHARED_MEM 0

#define MODULE_TIM1_ID                MODULE_TIM1
#define MODULE_TIM1_FUNCTION_POINTER  &moduleTim1_u32
#define MODULE_TIM1_TOTAL_SEQ         0
#define MODULE_TIM1_TOTAL_STRUCT      1
#define MODULE_TIM1_PREV_STATE        0
#define MODULE_TIM1_NEXT_STATE        0
#define MODULE_TIM1_IRQ_STATUS        DEFAULT_IRQ_STATE
#define MODULE_TIM1_PROCESS_STATUS    0
#define MODULE_TIM1_MASTER_SHARED_MEM 0

#define MODULE_USART2_ID                MODULE_USART2
#define MODULE_USART2_FUNCTION_POINTER  &moduleUsart2_u32
#define MODULE_USART2_TOTAL_SEQ         4
#define MODULE_USART2_TOTAL_STRUCT      1
#define MODULE_USART2_PREV_STATE        0
#define MODULE_USART2_NEXT_STATE        0
#define MODULE_USART2_IRQ_STATUS        DEFAULT_IRQ_STATE
#define MODULE_USART2_PROCESS_STATUS    0
#define MODULE_USART2_MASTER_SHARED_MEM 0

#define MODULE_ADC1_ID                MODULE_ADC1
#define MODULE_ADC1_FUNCTION_POINTER  &moduleADC1_u32
#define MODULE_ADC1_TOTAL_SEQ         0
#define MODULE_ADC1_TOTAL_STRUCT      1
#define MODULE_ADC1_PREV_STATE        0
#define MODULE_ADC1_NEXT_STATE        0
#define MODULE_ADC1_IRQ_STATUS        DEFAULT_IRQ_STATE
#define MODULE_ADC1_PROCESS_STATUS    0
#define MODULE_ADC1_MASTER_SHARED_MEM 0

#define MODULE_ANALOG_0_10V_ID                MODULE_ANALOG_0_10V
#define MODULE_ANALOG_0_10V_FUNCTION_POINTER  &moduleAnalog_0_10V
#define MODULE_ANALOG_0_10V_TOTAL_SEQ         0
#define MODULE_ANALOG_0_10V_TOTAL_STRUCT      1
#define MODULE_ANALOG_0_10V_PREV_STATE        0
#define MODULE_ANALOG_0_10V_NEXT_STATE        0
#define MODULE_ANALOG_0_10V_IRQ_STATUS        DEFAULT_IRQ_STATE
#define MODULE_ANALOG_0_10V_PROCESS_STATUS    0
#define MODULE_ANALOG_0_10V_MASTER_SHARED_MEM 0

#define MODULE_MOTOR_COM_ID                 MODULE_MOTOR_COM
#define MODULE_MOTOR_COM_FUNCTION_POINTER   &module_Motor_Com_u32
#define MODULE_MOTOR_COM_TOTAL_SEQ          0
#define MODULE_MOTOR_COM_TOTAL_STRUCT       1
#define MODULE_MOTOR_COM_PREV_STATE         0
#define MODULE_MOTOR_COM_NEXT_STATE         0
#define MODULE_MOTOR_COM_IRQ_STATUS         DEFAULT_IRQ_STATE
#define MODULE_MOTOR_COM_PROCESS_STATUS     0
#define MODULE_MOTOR_COM_MASTER_SHARED_MEM  0

#define MODULE_FLASH_BLK_SETTING_ID                MODULE_FLASH_BLK_SETTING
#define MODULE_FLASH_BLK_SETTING_FUNCTION_POINTER  &module_Flash_Blk_Setting_u32
#define MODULE_FLASH_BLK_SETTING_TOTAL_SEQ         0
#define MODULE_FLASH_BLK_SETTING_TOTAL_STRUCT      1
#define MODULE_FLASH_BLK_SETTING_PREV_STATE        0
#define MODULE_FLASH_BLK_SETTING_NEXT_STATE        0
#define MODULE_FLASH_BLK_SETTING_IRQ_STATUS        DEFAULT_IRQ_STATE
#define MODULE_FLASH_BLK_SETTING_PROCESS_STATUS    0
#define MODULE_FLASH_BLK_SETTING_MASTER_SHARED_MEM 0

#define MODULE_MOTOR_FW_UPDATE_ID                MODULE_MOTOR_FW_UPDATE
#define MODULE_MOTOR_FW_UPDATE_FUNCTION_POINTER  &module_Motor_FW_Update_u32
#define MODULE_MOTOR_FW_UPDATE_TOTAL_SEQ         0
#define MODULE_MOTOR_FW_UPDATE_TOTAL_STRUCT      1
#define MODULE_MOTOR_FW_UPDATE_PREV_STATE        0
#define MODULE_MOTOR_FW_UPDATE_NEXT_STATE        0
#define MODULE_MOTOR_FW_UPDATE_IRQ_STATUS        DEFAULT_IRQ_STATE
#define MODULE_MOTOR_FW_UPDATE_PROCESS_STATUS    0
#define MODULE_MOTOR_FW_UPDATE_MASTER_SHARED_MEM 0

#define MODULE_DIGITAL_INPUTS_ID                   MODULE_DIGITAL_INPUTS
#define MODULE_DIGITAL_INPUTS_FUNCTION_POINTER     &module_Digital_Inputs_u32
#define MODULE_DIGITAL_INPUTS_TOTAL_SEQ            0
#define MODULE_DIGITAL_INPUTS_TOTAL_STRUCT         1
#define MODULE_DIGITAL_INPUTS_PREV_STATE           0
#define MODULE_DIGITAL_INPUTS_NEXT_STATE           0
#define MODULE_DIGITAL_INPUTS_IRQ_STATUS           DEFAULT_IRQ_STATE
#define MODULE_DIGITAL_INPUTS_PROCESS_STATUS       0
#define MODULE_DIGITAL_INPUTS_MASTER_SHARED_MEM    0

#define MODULE_MOTOR_DEMAND_MUX_ID                   MODULE_MOTOR_DEMAND_MUX
#define MODULE_MOTOR_DEMAND_MUX_FUNCTION_POINTER     &moduleMotorDemandMultiplexer_u32
#define MODULE_MOTOR_DEMAND_MUX_TOTAL_SEQ            0
#define MODULE_MOTOR_DEMAND_MUX_TOTAL_STRUCT         1
#define MODULE_MOTOR_DEMAND_MUX_PREV_STATE           0
#define MODULE_MOTOR_DEMAND_MUX_NEXT_STATE           0
#define MODULE_MOTOR_DEMAND_MUX_IRQ_STATUS           DEFAULT_IRQ_STATE
#define MODULE_MOTOR_DEMAND_MUX_PROCESS_STATUS       0
#define MODULE_MOTOR_DEMAND_MUX_MASTER_SHARED_MEM    0

#define MODULE_PWM_INPUT_ID                   MODULE_PWM_INPUT
#define MODULE_PWM_INPUT_FUNCTION_POINTER     &modulePWM_Input_u32
#define MODULE_PWM_INPUT_TOTAL_SEQ            0
#define MODULE_PWM_INPUT_TOTAL_STRUCT         1
#define MODULE_PWM_INPUT_PREV_STATE           0
#define MODULE_PWM_INPUT_NEXT_STATE           0
#define MODULE_PWM_INPUT_IRQ_STATUS           DEFAULT_IRQ_STATE
#define MODULE_PWM_INPUT_PROCESS_STATUS       0
#define MODULE_PWM_INPUT_MASTER_SHARED_MEM    0

// ======= Modbus Protocol Modules =======
#define MODULE_USART1_ID                MODULE_USART1
#define MODULE_USART1_FUNCTION_POINTER  &moduleUsart1
#define MODULE_USART1_TOTAL_SEQ         3
#define MODULE_USART1_TOTAL_STRUCT      1
#define MODULE_USART1_PREV_STATE        0
#define MODULE_USART1_NEXT_STATE        0
#define MODULE_USART1_IRQ_STATUS        DEFAULT_IRQ_STATE
#define MODULE_USART1_PROCESS_STATUS    0
#define MODULE_USART1_MASTER_SHARED_MEM 0

#define MODULE_MODBUS_ID                MODULE_MODBUS
#define MODULE_MODBUS_FUNCTION_POINTER  &moduleModbus
#define MODULE_MODBUS_TOTAL_SEQ         0
#define MODULE_MODBUS_TOTAL_STRUCT      1
#define MODULE_MODBUS_PREV_STATE        0
#define MODULE_MODBUS_NEXT_STATE        0
#define MODULE_MODBUS_IRQ_STATUS        DEFAULT_IRQ_STATE
#define MODULE_MODBUS_PROCESS_STATUS    0
#define MODULE_MODBUS_MASTER_SHARED_MEM 0

// Digital Outputs Module
#define MODULE_DIGITAL_OUTPUTS_ID                   MODULE_DIGITAL_OUTPUTS
#define MODULE_DIGITAL_OUTPUTS_FUNCTION_POINTER     &module_Digital_Outputs_u32
#define MODULE_DIGITAL_OUTPUTS_TOTAL_SEQ            0
#define MODULE_DIGITAL_OUTPUTS_TOTAL_STRUCT         1
#define MODULE_DIGITAL_OUTPUTS_PREV_STATE           0
#define MODULE_DIGITAL_OUTPUTS_NEXT_STATE           0
#define MODULE_DIGITAL_OUTPUTS_IRQ_STATUS           DEFAULT_IRQ_STATE
#define MODULE_DIGITAL_OUTPUTS_PROCESS_STATUS       0
#define MODULE_DIGITAL_OUTPUTS_MASTER_SHARED_MEM    0

// Analog 4-20mA
#define MODULE_ANALOG_4_20MA_ID                   MODULE_ANALOG_4_20MA
#define MODULE_ANALOG_4_20MA_FUNCTION_POINTER     &moduleAnalog_4_20mA
#define MODULE_ANALOG_4_20MA_TOTAL_SEQ            0
#define MODULE_ANALOG_4_20MA_TOTAL_STRUCT         1
#define MODULE_ANALOG_4_20MA_PREV_STATE           0
#define MODULE_ANALOG_4_20MA_NEXT_STATE           0
#define MODULE_ANALOG_4_20MA_IRQ_STATUS           DEFAULT_IRQ_STATE
#define MODULE_ANALOG_4_20MA_PROCESS_STATUS       0
#define MODULE_ANALOG_4_20MA_MASTER_SHARED_MEM    0

// SPI
#define MODULE_SPI1_ID                MODULE_SPI
#define MODULE_SPI1_FUNCTION_POINTER  &moduleSPI1_u32
#define MODULE_SPI1_TOTAL_SEQ         0
#define MODULE_SPI1_TOTAL_STRUCT      1
#define MODULE_SPI1_PREV_STATE        0
#define MODULE_SPI1_NEXT_STATE        0
#define MODULE_SPI1_IRQ_STATUS        DEFAULT_IRQ_STATE
#define MODULE_SPI1_PROCESS_STATUS    0
#define MODULE_SPI1_MASTER_SHARED_MEM 0

#define MODULE_ERR_LOGHANDLE_ID                MODULE_ERR_LOGHANDLE
#define MODULE_ERR_LOGHANDLE_FUNCTION_POINTER  &moduleErrorLog_u32
#define MODULE_ERR_LOGHANDLE_TOTAL_SEQ         0
#define MODULE_ERR_LOGHANDLE_TOTAL_STRUCT      1
#define MODULE_ERR_LOGHANDLE_PREV_STATE        0
#define MODULE_ERR_LOGHANDLE_NEXT_STATE        0
#define MODULE_ERR_LOGHANDLE_IRQ_STATUS        DEFAULT_IRQ_STATE
#define MODULE_ERR_LOGHANDLE_PROCESS_STATUS    0
#define MODULE_ERR_LOGHANDLE_MASTER_SHARED_MEM 0

#define MODULE_FLASH_ID                MODULE_FLASH
#define MODULE_FLASH_FUNCTION_POINTER  &moduleFlash_u32
#define MODULE_FLASH_TOTAL_SEQ         0
#define MODULE_FLASH_TOTAL_STRUCT      1
#define MODULE_FLASH_PREV_STATE        0
#define MODULE_FLASH_NEXT_STATE        0
#define MODULE_FLASH_IRQ_STATUS        DEFAULT_IRQ_STATE
#define MODULE_FLASH_PROCESS_STATUS    0
#define MODULE_FLASH_MASTER_SHARED_MEM 0

#define MODULE_TEST_ID                 MODULE_TEST
#define MODULE_TEST_FUNCTION_POINTER   &moduleTest_u32
#define MODULE_TEST_TOTAL_SEQ          2
#define MODULE_TEST_TOTAL_STRUCT       1
#define MODULE_TEST_PREV_STATE         0
#define MODULE_TEST_NEXT_STATE         0
#define MODULE_TEST_IRQ_STATUS         DEFAULT_IRQ_STATE
#define MODULE_TEST_PROCESS_STATUS     0
#define MODULE_TEST_MASTER_SHARED_MEM  0
// Process Status
#define PROCESS_STATUS_RUNNING 0x00
#define PROCESS_STATUS_KILLED  0xFF
#define PROCESS_STATUS_PAUSED  0x05

// Process Ranges
#define MIN_MODULE_ID     0
#define MAX_MODULE_ID     191
#define MAX_NUM_OF_MODULES MAX_MODULE_ID - MIN_MODULE_ID + 1
#define MIN_IRQ_ID     0
#define MAX_IRQ_ID     254
#define MAX_NUM_OF_DRVS MAX_DRV_ID - MIN_DRV_ID + 1
#define MAX_PROCESS_ID    255

// Interrupt Parameters
#define SHIFTER                 1
#define GPIO_SOFTWARE_IRQ 0x02
#define PROCESS_IRQ_ID    0
#define NO_IRQS           0
#define DEFAULT_IRQ_STATE 200

// Default States
#define DEFAULT_PREV_STATE 0
#define DEFAULT_NEXT_STATE     0

#define INDEX_NOT_FOUND 255

#define KILL_APP 255

// Watch Dog Timer
#define NUM_OF_625_MS_INC 6

/**
  ********************************************************************************************************************************
  * @brief   Generate Process ID for each module and/or driver.
  * @details 1. All module and drivers should be declared here.
  *          2. The position in the enum defines the process ID for each module/driver.
  *          3. The lower the position the higher the priority.
  *                 Position 0 is the highest module priority.
  *                 Position 192 is the highest driver priority.
  *          4. Modules should be declared at positions 0 to 191.
  *          5. Drivers should be declared at positions 192 to 254.
  *          6. END_OF_PROCESS_ID is reserved.
  ********************************************************************************************************************************
  */
#define TOTAL_NUM_OF_PROCESSES 17 // REVIEW: Moved to near the enum
enum Processes {
    MODULE_FLASH= MIN_MODULE_ID, // MODULE_FLASH should be the first process
    MODULE_GPIO,
    MODULE_TIM1,
    MODULE_USART2,
    MODULE_ADC1,
    MODULE_ANALOG_0_10V,
    MODULE_ANALOG_4_20MA,
    MODULE_DIGITAL_INPUTS,
    MODULE_PWM_INPUT,        
    MODULE_USART1, // modbus
    MODULE_SPI,
    MODULE_MODBUS,    
    MODULE_MOTOR_DEMAND_MUX,    
    MODULE_MOTOR_COM,
    MODULE_DIGITAL_OUTPUTS,
    MODULE_ERR_LOGHANDLE,
	MODULE_TEST,
    //MODULE_FLASH_BLK_SETTING,
    //MODULE_MOTOR_FW_UPDATE,
    END_OF_PROCESS_ID = MAX_PROCESS_ID,
};

/**
  ********************************************************************************************************************************
  * @brief   Generate Memory Instance ID for each Structured Memory Instance
  * @details 1. Used to generate Array Sizes and for checking the full array of memory instances
  *          2. The position is not used as of 10/5/2020
  ********************************************************************************************************************************
  */

// Cannot use Enum, because need TOTAL_NUM_OF_STRUCT_MEM_INSTANCES pre-compiler for compiler directives
// - REVIEW: TOTAL_NUM_OF_STRUCT_MEM_INSTANCES only used pre-comile for Array Zero Detect and Size double check
// #define STRUCT_MEM_ID_MODULE_ADC1_DATA_BUFFER 0
// #define STRUCT_MEM_ID_MODULE_0_10V_DATA_BUFFER 1
// #define STRUCT_MEM_ID_MODULE_MOTOR_COM_BUFFER 2
// #define STRUCT_MEM_ID_MODULE_USART2_BUFFER 3
// #define TOTAL_NUM_OF_STRUCT_MEM_INSTANCES 8 // should always be the last member of this enum

#define TOTAL_NUM_OF_STRUCT_MEM_INSTANCES TOTAL_NUM_OF_PROCESSES // TODO: This should be a count
// TODO: Struct Memory seems to be indexed by module id and not structured memory instance, 
// - this was found because MODULE_GPIO not having a Struct mem instance led to null pointers for the last module

// Ensure that we do not declare arrays of size 0, if Struct Memory is not used in a project
#if TOTAL_NUM_OF_STRUCT_MEM_INSTANCES == 0
#define STRUCT_MEM_ARRAY_SIZE 1
#else
#define STRUCT_MEM_ARRAY_SIZE TOTAL_NUM_OF_STRUCT_MEM_INSTANCES
#endif

// Generate a warning if TOTAL_NUM_OF_STRUCT_MEM_INSTANCES != Amount declared in the Module Setup Tables
#define STRUCT_MEM_SETUP_TABLE_COUNT (MODULE_GPIO_TOTAL_STRUCT + MODULE_TIM1_TOTAL_STRUCT+ MODULE_USART2_TOTAL_STRUCT + MODULE_ADC1_TOTAL_STRUCT \
          + MODULE_ANALOG_0_10V_TOTAL_STRUCT + MODULE_MOTOR_COM_TOTAL_STRUCT + MODULE_DIGITAL_INPUTS_TOTAL_STRUCT \
          + MODULE_MOTOR_DEMAND_MUX_TOTAL_STRUCT + MODULE_PWM_INPUT_TOTAL_STRUCT + MODULE_USART1_TOTAL_STRUCT +  MODULE_MODBUS_TOTAL_STRUCT \
          + MODULE_DIGITAL_OUTPUTS_TOTAL_STRUCT + MODULE_ANALOG_4_20MA_TOTAL_STRUCT + MODULE_ERR_LOGHANDLE_TOTAL_STRUCT + MODULE_FLASH_BLK_SETTING_TOTAL_STRUCT \
          + MODULE_SPI1_TOTAL_STRUCT + MODULE_MOTOR_FW_UPDATE_TOTAL_STRUCT + MODULE_FLASH_TOTAL_STRUCT + MODULE_TEST_SETTING_TOTAL_STRUCT) // Used as a double check-only

#if STRUCT_MEM_SETUP_TABLE_COUNT != TOTAL_NUM_OF_STRUCT_MEM_INSTANCES
// TODO: reimplement warning once TODO regarding 'indexed by module id' above is resolved
// #warning TOTAL_NUM_OF_STRUCT_MEM_INSTANCES does not equal amount derived from tables (STRUCT_MEM_SETUP_TABLE_COUNT)!
//printf("%d\n", XSTR(TOTAL_NUM_OF_STRUCT_MEM_INSTANCES))
#endif

/**
  ********************************************************************************************************************************
  * @brief   Generate Memory Instance ID for each Sequential Memory Instance
  * @details 1. Used to generate Array Sizes and for checking the full array of memory instances
  *          2. The position is not used as of 10/5/2020
  ********************************************************************************************************************************
  */

// Cannot use Enum, because need TOTAL_NUM_OF_SEQ_MEM_INSTANCES pre-compiler for compiler directives
// - REVIEW: TOTAL_NUM_OF_SEQ_MEM_INSTANCES only used pre-comile for Array Zero Detect and Size double check
#define SEQ_MEM_ID_MODULE_USART2_INTERNAL 0
#define SEQ_MEM_ID_MODULE_USART2_RX_TM 1 
#define SEQ_MEM_ID_MODULE_USART2_RX 2
#define SEQ_MEM_ID_MODULE_USART2_TX 3
#define SEQ_MEM_ID_MODULE_USART1_INTERNAL 4
#define SEQ_MEM_ID_MODULE_USART1_RX 5
#define SEQ_MEM_ID_MODULE_USART1_TX 6
#define TOTAL_NUM_OF_SEQ_MEM_INSTANCES 7 // should always be the last member of this enum
//};
      
// Ensure that we do not declare arrays of size 0, if Struct Memory is not used in a project
#if TOTAL_NUM_OF_SEQ_MEM_INSTANCES == 0
#define SEQ_MEM_ARRAY_SIZE 1
#else
#define SEQ_MEM_ARRAY_SIZE TOTAL_NUM_OF_SEQ_MEM_INSTANCES
#endif
// Generate a warning if TOTAL_NUM_OF_SEQ_MEM_INSTANCES != Amount declared in the Module Setup Tables
#define SEQ_MEM_SETUP_TABLE_COUNT (MODULE_GPIO_TOTAL_SEQ + MODULE_TIM1_TOTAL_SEQ + MODULE_USART2_TOTAL_SEQ + MODULE_ADC1_TOTAL_SEQ \
          + MODULE_ANALOG_0_10V_TOTAL_SEQ + MODULE_MOTOR_COM_TOTAL_SEQ + MODULE_PWM_INPUT_TOTAL_SEQ \
          + MODULE_MOTOR_DEMAND_MUX_TOTAL_SEQ + MODULE_DIGITAL_INPUTS_TOTAL_SEQ + MODULE_USART1_TOTAL_SEQ + MODULE_MODBUS_TOTAL_SEQ \
          + MODULE_DIGITAL_INPUTS_TOTAL_SEQ+ MODULE_ANALOG_4_20MA_TOTAL_SEQ + MODULE_ERR_LOGHANDLE_TOTAL_SEQ + \
            MODULE_FLASH_BLK_SETTING_TOTAL_SEQ + MODULE_MOTOR_FW_UPDATE_TOTAL_SEQ + MODULE_SPI1_TOTAL_SEQ + MODULE_FLASH_TOTAL_SEQ + MODULE_TEST_UPDATE_TOTAL_SEQ) // Used as a double check-only
#if SEQ_MEM_SETUP_TABLE_COUNT != TOTAL_NUM_OF_SEQ_MEM_INSTANCES
#warning TOTAL_NUM_OF_SEQ_MEM_INSTANCES does not equal amount derived from tables (SEQ_MEM_SETUP_TABLE_COUNT)!
#endif



/**
  ********************************************************************************************************************************
  * @brief   
  * @details 
  * @param   moduleId_u8
  * @param   prevState_u8
  * @param   nextState_u8
  * @param   irqId_u8
  * @return  
  ********************************************************************************************************************************
  */
uint8_t moduleApp_u32(uint8_t moduleId_u8, uint8_t prevState_u8, uint8_t nextState_u8,
                        uint8_t irqId_u8);


/**
  ********************************************************************************************************************************
  * @brief   
  * @details 
  * @param   moduleId_u8
  * @param   prevState_u8
  * @param   nextState_u8
  * @param   irqId_u8
  * @return  
  ********************************************************************************************************************************
  */
uint8_t module_Motor_FW_Update_u32(uint8_t moduleIDd_u8, uint8_t prev_state_u8, uint8_t next_state_u8,
                               uint8_t irq_id_u8);

/**
  ********************************************************************************************************************************
  * @brief   
  * @details 
  * @param   driver_identifier_u8
  * @param   prevState_u8
  * @param   nextState_u8
  * @param   interruptIdentifier_u8
  * @return  
  ********************************************************************************************************************************
  */
uint8_t moduleUsart1(uint8_t driver_identifier_u8, uint8_t prevState_u8, uint8_t nextState_u8,
                           uint8_t interruptIdentifier_u8);

/**
  ********************************************************************************************************************************
  * @brief   
  * @details 
  * @param   drv_id_u8
  * @param   prevState_u8
  * @param   nextState_u8
  * @param   irqId_u8
  * @return  
  ********************************************************************************************************************************
  */
uint8_t moduleUsart2_u32(uint8_t drv_id_u8, uint8_t prevState_u8, uint8_t nextState_u8,
                           uint8_t irqId_u8);
                           
/**
  ********************************************************************************************************************************
  * @brief   
  * @details 
  * @param   drv_id_u8
  * @param   prevState_u8
  * @param   nextState_u8
  * @param   irqId_u8
  * @return  
  ********************************************************************************************************************************
  */
uint8_t moduleSPI1_u32(uint8_t drv_id_u8, uint8_t prevState_u8, uint8_t nextState_u8,
                           uint8_t irqId_u8);

/**
  ********************************************************************************************************************************
  * @brief   
  * @details 
  * @param   drv_id_u8
  * @param   prevState_u8
  * @param   nextState_u8
  * @param   irqId_u8
  * @return  
  ********************************************************************************************************************************
  */
uint8_t moduleGPIO_u32(uint8_t drv_id_u8, uint8_t prevState_u8, uint8_t nextState_u8,
                         uint8_t irqId_u8);

/**
  ********************************************************************************************************************************
  * @brief   
  * @details 
  * @param   drv_id_u8
  * @param   prevState_u8
  * @param   nextState_u8
  * @param   irqId_u8
  * @return  
  ********************************************************************************************************************************
  */
uint8_t p_moduleMotorControl_u32(uint8_t drv_id_u8, uint8_t prevState_u8, uint8_t nextState_u8,
                                 uint8_t irqId_u8);

/**
  ********************************************************************************************************************************
  * @brief   
  * @details 
  * @param   drv_id_u8
  * @param   prevState_u8
  * @param   nextState_u8
  * @param   irqId_u8
  * @return  
  ********************************************************************************************************************************
  */
uint8_t p_moduleRTC_u32(uint8_t drv_id_u8, uint8_t prevState_u8, uint8_t nextState_u8,
                                 uint8_t irqId_u8); 
/**
  ********************************************************************************************************************************
  * @brief   
  * @details 
  * @param   drv_id_u8
  * @param   prevState_u8
  * @param   nextState_u8
  * @param   irqId_u8
  * @return  
  ********************************************************************************************************************************
  */
uint8_t p_moduleMeerkatSafetyCore_u32(uint8_t drv_id_u8, uint8_t prevState_u8, uint8_t nextState_u8,
                                 uint8_t irqId_u8); // TODO: These are 'functions' and should not follow 'pointer' naming conventions

/**
  ********************************************************************************************************************************
  * @brief   
  * @details 
  * @param   drv_id_u8
  * @param   prevState_u8
  * @param   nextState_u8
  * @param   irqId_u8
  * @return  
  ********************************************************************************************************************************
  */
uint8_t moduleADC1_u32(uint8_t drv_id_u8, uint8_t prevState_u8, uint8_t nextState_u8,
                                 uint8_t irqId_u8); // TODO: These are 'functions' and should not follow 'pointer' naming conventions

/**
  ********************************************************************************************************************************
  * @brief   
  * @details 
  * @param   drv_id_u8
  * @param   prevState_u8
  * @param   nextState_u8
  * @param   irqId_u8
  * @return  
  ********************************************************************************************************************************
  */
uint8_t moduleAnalog_0_10V(uint8_t drv_id_u8, uint8_t prevState_u8, uint8_t nextState_u8,
                                 uint8_t irqId_u8); // TODO: These are 'functions' and should not follow 'pointer' naming conventions

/**
  ********************************************************************************************************************************
  * @brief   
  * @details 
  * @param   drv_id_u8
  * @param   prevState_u8
  * @param   nextState_u8
  * @param   irqId_u8
  * @return  
  ********************************************************************************************************************************
  */

uint8_t module_Motor_Com_u32(uint8_t drv_id_u8, uint8_t prev_state_u8, uint8_t next_state_u8,
                               uint8_t irq_id_u8);


/**
  ********************************************************************************************************************************
  * @brief   
  * @details 
  * @param   drv_id_u8
  * @param   prevState_u8
  * @param   nextState_u8
  * @param   irqId_u8
  * @return  
  ********************************************************************************************************************************
  */
uint8_t module_Flash_Blk_Setting_u32(uint8_t drv_id_u8, uint8_t prev_state_u8, uint8_t next_state_u8,
                               uint8_t irq_id_u8);




/**
  ********************************************************************************************************************************
  * @brief   
  * @details 
  * @param   drv_id_u8
  * @param   prevState_u8
  * @param   nextState_u8
  * @param   irqId_u8
  * @return  
  ********************************************************************************************************************************
  */
uint8_t dummy(uint8_t drv_id_u8, uint8_t prevState_u8, uint8_t nextState_u8,
                                 uint8_t irqId_u8);

/**
  ********************************************************************************************************************************
  * @brief   
  * @details 
  * @param   drv_id_u8
  * @param   prevState_u8
  * @param   nextState_u8
  * @param   irqId_u8
  * @return  
  ********************************************************************************************************************************
  */

uint8_t module_Digital_Inputs_u32(uint8_t drv_identifier_u8, uint8_t previous_state_u8, uint8_t next_state_u8,
                               uint8_t irq_identifier_u8);


/**
  ********************************************************************************************************************************
  * @brief   State machine for digital outputs module
  * @details
  * @param   drv_identifier_u8, previous_state_u8, next_stat_u8, irq_identfier_u8
  * @retval  return_state_u8
  ********************************************************************************************************************************
*/
uint8_t module_Digital_Outputs_u32(uint8_t drv_identifier_u8, uint8_t previous_state_u8, uint8_t next_state_u8,
                               uint8_t irq_identifier_u8);

/**
  ********************************************************************************************************************************
  * @brief   
  * @details 
  * @param   drv_id_u8
  * @param   prevState_u8
  * @param   nextState_u8
  * @param   irqId_u8
  * @return  
  ********************************************************************************************************************************
  */
uint8_t modulePWM_Input_u32(uint8_t drv_id_u8, uint8_t prevState_u8, uint8_t nextState_u8,
                                 uint8_t irqId_u8);


/**
  ********************************************************************************************************************************
  * @brief   Module for Tim1 used for PWM input
  * @details 
  * @param   drv_id_u8
  * @param   prevState_u8
  * @param   nextState_u8
  * @param   irqId_u8
  * @return  
  ********************************************************************************************************************************
  */
uint8_t moduleTim1_u32(uint8_t drv_id_u8, uint8_t prev_state_u8, uint8_t next_state_u8,
                       uint8_t irq_id_u8);

/**
  ********************************************************************************************************************************
  * @brief   
  * @details 
  * @param   drv_id_u8
  * @param   prevState_u8
  * @param   nextState_u8
  * @param   irqId_u8
  * @return  
  ********************************************************************************************************************************
  */

uint8_t moduleMotorDemandMultiplexer_u32(uint8_t drv_identifier_u8, uint8_t previous_state_u8, uint8_t next_state_u8,
                               uint8_t irq_identifier_u8);

/**
  ********************************************************************************************************************************
  * @brief   
  * @details 
  * @param   drv_id_u8
  * @param   prevState_u8
  * @param   nextState_u8
  * @param   irqId_u8
  * @return  
  ********************************************************************************************************************************
  */

uint8_t moduleModbus(uint8_t drv_identifier_u8, uint8_t previous_state_u8, uint8_t next_state_u8,
                               uint8_t irq_identifier_u8);


/**
  ********************************************************************************************************************************
  * @brief   State machine for Analog 4-20mA module
  * @details 
  * @param   drv_id_u8
  * @param   prevState_u8
  * @param   nextState_u8
  * @param   irqId_u8
  * @return  return_state_u8
  ********************************************************************************************************************************
*/
uint8_t moduleAnalog_4_20mA(uint8_t drv_identifier_u8, uint8_t previous_state_u8, uint8_t next_state_u8,
                               uint8_t irq_identifier_u8);
/**
********************************************************************************************************************************
* @brief   State machine for Test Module
* @details
* @retval  return_state_u8
********************************************************************************************************************************
*/
uint8_t moduleTest_u32(uint8_t drv_identifier_u8, uint8_t previous_state_u8, uint8_t next_state_u8,
                            uint8_t irq_identifier_u8);

/**
  ********************************************************************************************************************************
  * @brief   Module Callback Intended to be called by the SysTick Interrupt
  * @details  Used by the Meerkat Clock Test
  * @param  
  ********************************************************************************************************************************
 */
void Meerkat_SafetyCore_SysTickCallback(void);

/**
  ********************************************************************************************************************************
  * @brief   Function pointer type definition for modules/drivers.
  * @details 
  ********************************************************************************************************************************
  */

typedef uint8_t (*p_process_u8)(uint8_t moduleId_u8, uint8_t prevState_u8, uint8_t nextState_u8,
                                uint8_t irqId_u8);

#pragma pack(1) // Specifies the packing alignment for data to be 1 byte, instead of the default value of 8 bytes, to save memory.

/**
  ********************************************************************************************************************************
  * @brief   Structure for all module/driver system control parameters.
  * @details The strucure for modules and drivers share the same space in memory. The parameters are similar, but not identical.
  *          Note on modules:
  *             Each module can be called by more than one driver's interrupt. (One 64-bit bit-oriented  parameter per driver.)
  *          Note on drivers:
  *             The driver interrupt will call the module defined by that driver's attachedModuleID
  *             The driver interrupt will pass control to the irqState_u8 of the associated module.
  ********************************************************************************************************************************
  */
typedef union {
    /**
    ******************************************************************************************************************************
    * @brief   Structure parameters for Sched_ModuleData struct.
    * @param   moduleId_u8     ID of the module currently being run by the scheduler.
    * @param   p_module_u32             Pointer to the module currently being run.
    * @param   totalSeqMemInstances_u8 Number of sequential memory instances allocated to the module currently being run.
    * @param   totalStructMemInstances_u8 Number of structured memory instances allocated to the module currently being run.
    * @param   prevState_u8        Reference to the last stage/state that this module ran.
    * @param   nextState_u8            Reference to the next stage/state that this module will run.
    * @param   irqState_u8       Reference to the interrupt stage/state for the module being run.
    * @param   processStatus_u8        This process is 0x00= running, 0xff= deleted, 0x05 = paused
    * @param   p_masterSharedMem_u32   Pointer to structured memory instance of the module being run.
    * @param   attachedDrvsReg_u64 64-Bit data structure for bit-wise declaration of the associated drivers of this module.
    *                                   Bit 0 corresponds to the driver at address 192, bit 1 to the driver at address 193, etc...
    * @param   irqId_u8  Parameter is system reserved. Indicates the driver that owns the interrupt that was triggered.
    ******************************************************************************************************************************
    */
    struct {
        uint8_t moduleId_u8;
        p_process_u8 p_module_u32;
        uint8_t totalSeqMemInstances_u8;
        uint8_t totalStructMemInstances_u8;
        uint8_t prevState_u8;
        uint8_t nextState_u8;
        uint8_t irqState_u8;
        uint8_t processStatus_u8;
        Ram_Buf_Handle p_masterSharedMem_u32;
        uint8_t irqType_u8;
        uint8_t irqDat_u8;
    } Sched_ModuleData;

    /**
    ******************************************************************************************************************************
    * @brief   Structure parameters for Sched_DrvData struct. 
    * @param   drvId_u8          ID of the driver currently being run by the scheduler. Same memory location as moduleId_u8.
    * @param   p_drv_u32         Pointer to the driver currently being run. Same memory location as p_module_u32.
    * @param   totalSeqMemInstances_u8       Number of sequential memory instances allocated to the driver currently being run.
    * @param   totalStructMemInstances_u8      Number of structured memory instances allocated to the driver currently being run.
    * @param   prevState_u8  Reference to the last stage/state that this driver ran.
    * @param   nextState_u8      Reference to the next stage/state that this driver will run.
    * @param   irqState_u8 Reference to the interrupt stage/state for the driver being run.
    * @param   processStatus_u8  This process is 0x00= running, 0xff= deleted, 0x05 = paused
    * @param   p_masterSharedMem_u32 Pointer to structured memory instance of the driver being run.
    * @param   irqType_u8       this will indicate what kind of interrupt (for details please read the software interrupt document) 
    * @param   irqDat_u8         interrupt triggered module to pass data to response module
    * @param   irqDat1_len_u8    Parameter for interrupt triggered module to pass data to response module (multi-byte data use 
    *                                       data pointer of irqDatPt_u8 andthis byte will be data length of irqDatPt_u8 pointer) .
    * @param   irqDatPt_u8      Data pointer,usually set as NULL pointer for no extended data [for example only two bytes if data] 
    *                                          (else irqDat_len_u8 = data length of this data pointer
    ******************************************************************************************************************************
    */
    struct {
        uint8_t drvId_u8;
        p_process_u8 p_drv_u32;
        uint8_t totalSeqMemInstances_u8;
        uint8_t totalStructMemInstances_u8;
        uint8_t prevState_u8;
        uint8_t nextState_u8;
        uint8_t irqState_u8;
        uint8_t processStatus_u8;
        Ram_Buf_Handle p_masterSharedMem_u32;
        uint8_t irqType_u8;
        uint8_t irqDat_u8;
        uint8_t irqDat1_len_u8;
        uint8_t* irqDatPt_u8;
    } Sched_DrvData;

} ProcessInfo;

#pragma pack() // Declaration that the packing alignment return to the default of 8 bytes, from the temporary value of 1 byte.

/**
  ********************************************************************************************************************************
  * @brief   Initializes shared sequential and structured memory.
  * @details Frees previously allocated memory.
  ********************************************************************************************************************************
  */
uint8_t Sched_Initialize();

/**
  ********************************************************************************************************************************
  * @brief   Not used.
  * @details 
  ********************************************************************************************************************************
  */
uint8_t Sched_InitializeDrvs();

/**
  ********************************************************************************************************************************
  * @brief   Kernel scheduler main loop.
  * @details 
  ********************************************************************************************************************************
  */
void Sched_Run();

/**
  ********************************************************************************************************************************
  * @brief   Returns the index of processInfoTable corresponding to the specified moduleId_u8.
  * @details For each process, if the process is running, run the process and update the previous state and next state of that
  *             process.
  * @param   moduleId_u8
  * @return  Return the index of processInfoTable.
  ********************************************************************************************************************************
  */
uint8_t getProcessInfoIndex(uint8_t moduleId_u8);

/**
  ********************************************************************************************************************************
  * @brief   System Callback called by the SysTick Interrupt
  * @details  Used by the Meerkat Clock Test
  * @param  
  ********************************************************************************************************************************
  */
void HAL_SYSTICK_Callback(void);

/**
  ********************************************************************************************************************************
  * @brief   Initializes and starts the watchdog.
  * @details (IWDG_PR, IWDG_RLR and IWDG_WINR registers)
  * @param   timeout_u8
  ********************************************************************************************************************************
  */
// void Watchdog_Initialize(uint8_t timeout_u8);
/**
  ********************************************************************************************************************************
  * @brief   Initializes and starts the watchdog.
  * @details (IWDG_PR, IWDG_RLR and IWDG_WINR registers)
  * @param   timeout_u8
  ********************************************************************************************************************************
  */
uint64_t getSysCount(void);

/**
  ********************************************************************************************************************************
  * @brief   CRC .
  * @details 
  * @param  
  ********************************************************************************************************************************
*/
uint16_t Calculate_CRC(uint8_t BufSize, unsigned char* aDataBuf);

/**
  ********************************************************************************************************************************
  * @brief   Calculate 32bit CRC
  * @details 
  * @param  
  ********************************************************************************************************************************
*/
uint32_t Calculate_CRC32(uint8_t BufSize, unsigned char* aDataBuf);
/**
  ********************************************************************************************************************************
  * @brief   Reload (kick) the watchdog.
  * @details 
  * @param  
  ********************************************************************************************************************************
  */
void Watchdog_Reload(void);

/**
  ********************************************************************************************************************************
  * @brief   Setup a software interrupt 
  * @details find out and set the correct bit in the software interrupt bit table SoftwareIrqBitPt[IrqGroupIndx_u8] 
  * @param  
  ********************************************************************************************************************************
  */
void setupSoftwareIRQ(uint8_t SENDER_MODULE_ID, uint8_t RECEIVER_MODULE_ID, uint8_t _irqType_u8, uint8_t _irqDat_u8, uint8_t _irqDat1_len_u8, uint8_t * _irqDatPt_u8);

/**
  ********************************************************************************************************************************
  * @brief   
  * @details 
  * @param   drv_id_u8
  * @param   prevState_u8
  * @param   nextState_u8
  * @param   irqId_u8
  * @return  
  ********************************************************************************************************************************
  */
uint8_t moduleErrorLog_u32(uint8_t drv_id_u8, uint8_t prevState_u8, uint8_t nextState_u8,
                           uint8_t irqId_u8);

/**
  ********************************************************************************************************************************
  * @brief   Module to read and write flash
  * @details 
  * @param   drv_id_u8
  * @param   prevState_u8
  * @param   nextState_u8
  * @param   irqId_u8
  * @return  
  ********************************************************************************************************************************
  */
uint8_t moduleFlash_u32(uint8_t drv_id_u8, uint8_t prev_state_u8, uint8_t next_state_u8, uint8_t irq_id_u8);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _SCHEDULER_H_ */