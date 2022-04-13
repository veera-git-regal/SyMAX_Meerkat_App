/**
  ***************************************************************************************************
  * @file    module_modbus_drive_map.h 
  * @author  Regal Myron Mychal
  * @version V1.0
  * @date    15-Jun-2021
  * @brief   Macros for defining register map in drive micro
  * @note    
  ***************************************************************************************************
  */
//^** Tips: APPs/Drivers adding process example step6  [refer to user manual ) 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MODULE_MODBUS_DRIVE_MAP_H_
#define _MODULE_MODBUS_DRIVE_MAP_H_

/* Includes ------------------------------------------------------------------*/
#include "structured_memory.h"
#include "scheduler.h"
#include "ring_buffer.h"
#include "module_modbus.h"

extern  Modbus_RTU_Control modbus_RTU_control;					// structure from modbus module
extern  DriveDynamic_Control drive_dynamic_control;

// Template for module modbus address map structure
#define	DRIVE_MAX_COILS		(16)		// maximum number of modbus coils allowed for a single module
#define	DRIVE_MAX_DISCRETES	(24)		// maximum number of modbus discrete registers allowed for a single module
#define DRIVE_MAX_INPUTS	(64)		// maximum number of modbus input registers allowed for a single module
#define	DRIVE_MAX_HOLDINGS	(152)		// maximum bumber of modbus holding registers allowed for a single module

uint16_t	void_drive_start_of_coils_pu16[1];
uint16_t	void_drive_start_of_discretes_pu16[1];
uint16_t	void_drive_start_of_inputs_pu16[1];
uint16_t	void_drive_start_of_holdings_pu16[1];

// TEMPLATE number of each type of modbus data element in the module
#define MODULE_DRIVE_NUMBER_OF_COILS			0
#define MODULE_DRIVE_NUMBER_OF_COIL_REGISTERS	0
#define MODULE_DRIVE_NUMBER_OF_DISCRETES		0
#define MODULE_DRIVE_NUMBER_OF_DISCRETE_REGISTERS	0
#define MODULE_DRIVE_NUMBER_OF_INPUTS			0
#define MODULE_DRIVE_NUMBER_OF_HOLDINGS		    0

// TEMPLATE address for the start of each element type within the module
#define MODULE_DRIVE_MODBUS_GROUP				(0)
#define MODULE_DRIVE_MODBUS_GROUP_BASE_ADDRESS	(MODULE_DRIVE_MODBUS_GROUP<<8)
#define	MODULE_DRIVE_START_OF_COILS_ADDRESS		(MODULE_DRIVE_MODBUS_GROUP_BASE_ADDRESS + 0)
#define MODULE_DRIVE_START_OF_DISCRETES_ADDRESS	(MODULE_DRIVE_START_OF_COILS_ADDRESS + DRIVE_MAX_COILS)
#define MODULE_DRIVE_START_OF_INPUTS_ADDRESS	(MODULE_DRIVE_START_OF_DISCRETES_ADDRESS + DRIVE_MAX_DISCRETES)
#define MODULE_DRIVE_START_OF_HOLDINGS_ADDRESS	(MODULE_DRIVE_START_OF_INPUTS_ADDRESS + DRIVE_MAX_INPUTS)
#define	MODULE_DRIVE_END_OF_GROUP				(MODULE_DRIVE_START_OF_HOLDINGS_ADDRESS + DRIVE_MAX_HOLDINGS)

// TEMPLATE
uint16_t*	module_drive_start_of_coils_pu16;			// first register in coil subgroup
uint16_t*	module_drive_start_of_discretes_pu16;		// first register in discrete subgroup
uint16_t*	module_drive_start_of_inputs_pu16;			// first register in inputs subgroup
uint16_t*	module_drive_start_of_holdings_pu16;		// first register in holdings subgroup

// Drive Dynamic Grou - Group 0
// number of each type of modbus data element in the module
#define MODULE_DRIVE_DYNAMIC_NUMBER_OF_COILS			(1)
#define MODULE_DRIVE_DYNAMIC_NUMBER_OF_COIL_REGISTERS	(1)
#define MODULE_DRIVE_DYNAMIC_NUMBER_OF_DISCRETES		(1)
#define MODULE_DRIVE_DYNAMIC_NUMBER_OF_DISCRETE_REGISTERS	(1)
#define MODULE_DRIVE_DYNAMIC_NUMBER_OF_INPUTS			(8)
#define MODULE_DRIVE_DYNAMIC_NUMBER_OF_HOLDINGS			(5)

// address for the start of each element type within the module
#define MODULE_DRIVE_DYNAMIC_MODBUS_GROUP				(0)
#define MODULE_DRIVE_DYNAMIC_MODBUS_GROUP_BASE_ADDRESS	(MODULE_DRIVE_DYNAMIC_MODBUS_GROUP<<8)
#define	MODULE_DRIVE_DYNAMIC_START_OF_COILS_ADDRESS		(MODULE_DRIVE_DYNAMIC_MODBUS_GROUP_BASE_ADDRESS + 0)
#define MODULE_DRIVE_DYNAMIC_START_OF_DISCRETES_ADDRESS	(MODULE_DRIVE_DYNAMIC_START_OF_COILS_ADDRESS + DRIVE_MAX_COILS)
#define MODULE_DRIVE_DYNAMIC_START_OF_INPUTS_ADDRESS	(MODULE_DRIVE_DYNAMIC_START_OF_DISCRETES_ADDRESS + DRIVE_MAX_DISCRETES)
#define MODULE_DRIVE_DYNAMIC_START_OF_HOLDINGS_ADDRESS	(MODULE_DRIVE_DYNAMIC_START_OF_INPUTS_ADDRESS + DRIVE_MAX_INPUTS)
#define	MODULE_DRIVE_DYNAMIC_END_OF_GROUP				(MODULE_DRIVE_DYNAMIC_START_OF_HOLDINGS_ADDRESS + DRIVE_MAX_HOLDINGS)

uint16_t*	module_drive_dynamic_start_of_coils_pu16		= (uint16_t*) (&(drive_dynamic_control.drive_dynamic_settings.flags_u16));			// first register in coil subgroup
uint16_t*	module_drive_dynamic_start_of_discretes_pu16	= (uint16_t*) (&(drive_dynamic_control.drive_dynamic_data.discretes_u16));		// first register in discrete subgroup
uint16_t*	module_drive_dynamic_start_of_inputs_pu16		= (uint16_t*) (&(drive_dynamic_control.drive_dynamic_data));		// first register in holdings subgroup	  
uint16_t*	module_drive_dynamic_start_of_holdings_pu16		= (uint16_t*) (&(drive_dynamic_control.drive_dynamic_settings));			// first register in inputs subgroup

#endif /* _MODULE_MODBUS_DRIVE_H_ */

