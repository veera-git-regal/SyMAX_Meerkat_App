/**
  ***************************************************************************************************
  * @file    module_modbus_application_map.h 
  * @author  Regal Myron Mychal
  * @version V1.0
  * @date    15-Jun-2021
  * @brief   Macros for defining register map in applicatin micro
  * @note    
  ***************************************************************************************************
  */
//^** Tips: APPs/Drivers adding process example step6  [refer to user manual ) 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MODULE_MODBUS_APPLICATION_MAP_H_
#define _MODULE_MODBUS_APPLICATION_MAP_H_

/* Includes ------------------------------------------------------------------*/
#include "structured_memory.h"
#include "scheduler.h"
#include "ring_buffer.h"
#include "module_analog_0_10v.h"
#include "module_analog_4_20ma.h"
#include "module_digital_inputs.h"
#include "module_digital_outputs.h"
//#include "module_flash_blk_setting.h"
#include "module_motor_com.h"
#include "module_motor_demand_multiplexer.h"
//#include "module_Motor_FW_Update.h"
#include "module_pwm_input.h"
#include "module_modbus.h"
#include "ZZ_module_flash.h"
#include "module_test.h"

extern	DigitalInputs_Control digitalInputs_Control;			// structure from digital inputs module
extern	Digital_Outputs_Control digital_Outputs_Control;			// structure from digital inputs module
extern	AnalogVolts_Control analogVolts_Control;				// structure from analog inputs 0_to_10V module
extern	PwmInput_Control pwmInput_Control;						// structure from pwm inputsA module
extern  Modbus_RTU_Control modbus_RTU_Control;					// structure from modbus module
extern	Analog_4_20ma_Control analog_4_20ma_Control_ptr;		// structure from analog inputs 4_to_20mA module
extern  Motor_Com_Control motor_Com_Control;
extern  MotorDemandMux_Control motorDemandMux_Control;			// structure from motor demand multiplexer module
//extern  Motor_FW_Update_Control motor_FW_Update_Control;		// structure from motor firmware update module
//extern	Flash_Blk_Setting_Control *flash_blk_setting_Control;	// structure from flash block setings module
//extern  ModuleFlash_Control moduleFlash_Control;                 // structure from flash
extern  ModuleTest_Control moduleTest_Control;                  // Structure from test module
extern  ApplicationID_Control application_id_control;

// Template for module modbus address map structure
#define	MAX_COILS		(16)			// maximum number of modbus coils allowed for a single module
#define	MAX_DISCRETES	(24)		// maximum number of modbus discrete registers allowed for a single module
#define MAX_INPUTS		(64)		// maximum number of modbus input registers allowed for a single module
#define	MAX_HOLDINGS	(152)		// maximum bumber of modbus holding registers allowed for a single module

uint16_t	void_start_of_coils_pu16[1];			// placeholders
uint16_t	void_start_of_discretes_pu16[1];
uint16_t	void_start_of_inputs_pu16[1];
uint16_t	void_start_of_holdings_pu16[1];

// number of each type of modbus data element in the module
#define MODULE_NUMBER_OF_COILS			0
#define MODULE_NUMBER_OF_COIL_REGISTERS	0
#define MODULE_NUMBER_OF_DISCRETES		0
#define MODULE_NUMBER_OF_DISCRETE_REGISTERS	0
#define MODULE_NUMBER_OF_INPUTS			0
#define MODULE_NUMBER_OF_HOLDINGS		0

// address for the start of each element type within the module
#define MODULE_MODBUS_GROUP					(0)
#define MODULE_MODBUS_GROUP_BASE_ADDRESS	(MODULE_MODBUS_GROUP<<8)
#define	MODULE_START_OF_COILS_ADDRESS		(MODULE_MODBUS_GROUP_BASE_ADDRESS + 0)
#define MODULE_START_OF_DISCRETES_ADDRESS	(MODULE_START_OF_COILS_ADDRESS + MAX_COILS)
#define MODULE_START_OF_INPUTS_ADDRESS		(MODULE_START_OF_DISCRETES_ADDRESS + MAX_DISCRETES)
#define MODULE_START_OF_HOLDINGS_ADDRESS	(MODULE_START_OF_INPUTS_ADDRESS + MAX_INPUTS)
#define	MODULE_END_OF_GROUP					(MODULE_START_OF_HOLDINGS_ADDRESS + MAX_HOLDINGS)

uint16_t*	module_start_of_coils_pu16;			// first register in coil subgroup
uint16_t*	module_start_of_discretes_pu16;		// first register in discrete subgroup
uint16_t*	module_start_of_inputs_pu16;		// first register in inputs subgroup
uint16_t*	module_start_of_holdings_pu16;		// first register in holdings subgroup

// structure for analog input 0-10v
// number of each type of modbus data element in the module
#define MODULE_ANALOG_INPUTS_0TO10_NUMBER_OF_COILS			(5)
#define MODULE_ANALOG_INPUTS_0TO10_NUMBER_OF_COIL_REGISTERS	(1)
#define MODULE_ANALOG_INPUTS_0TO10_NUMBER_OF_DISCRETES		(6)
#define MODULE_ANALOG_INPUTS_0TO10_NUMBER_OF_DISCRETE_REGISTERS	(1)
#define MODULE_ANALOG_INPUTS_0TO10_NUMBER_OF_INPUTS			(15) // (status, filtered, 32-bit input value x3, DEmand Percent, is_invert, is_enabled)
#define MODULE_ANALOG_INPUTS_0TO10_NUMBER_OF_HOLDINGS		(21) // (input function x3, debounce limit, input_polarity, input enable, demand percent x8)

// address for the start of each element type within the module
#define MODULE_ANALOG_INPUTS_0TO10_GROUP						(136)			// analog inputs 0-10V group 
#define MODULE_ANALOG_INPUTS_0TO10_BASE_ADDRESS					(MODULE_ANALOG_INPUTS_0TO10_GROUP<<8)
#define	MODULE_ANALOG_INPUTS_0TO10_START_OF_COILS_ADDRESS		(MODULE_ANALOG_INPUTS_0TO10_BASE_ADDRESS + 0)
#define MODULE_ANALOG_INPUTS_0TO10_START_OF_DISCRETES_ADDRESS	(MODULE_ANALOG_INPUTS_0TO10_START_OF_COILS_ADDRESS + MAX_COILS)
#define MODULE_ANALOG_INPUTS_0TO10_START_OF_INPUTS_ADDRESS		(MODULE_ANALOG_INPUTS_0TO10_START_OF_DISCRETES_ADDRESS + MAX_DISCRETES)
#define MODULE_ANALOG_INPUTS_0TO10_START_OF_HOLDINGS_ADDRESS	(MODULE_ANALOG_INPUTS_0TO10_START_OF_INPUTS_ADDRESS + MAX_INPUTS)
#define	MODULE_ANALOG_INPUTS_0TO10_END_OF_GROUP					(MODULE_ANALOG_INPUTS_0TO10_START_OF_HOLDINGS_ADDRESS + MAX_HOLDINGS)

uint16_t*	module_analog_inputs_0TO10_start_of_coils_pu16			= (uint16_t*) &(analogVolts_Control.analogVolts_Setting.flags_u16);			// first register in coil subgroup
uint16_t*	module_analog_inputs_0TO10_start_of_discretes_pu16		= (uint16_t*) &(analogVolts_Control.analogVolts_Data.discretes_u16);			// first register in discrete subgroup
uint16_t*	module_analog_inputs_0TO10_start_of_inputs_pu16			= (uint16_t*) &(analogVolts_Control.analogVolts_Data);			// first register in inputs subgroup
uint16_t*	module_analog_inputs_0TO10_start_of_holdings_pu16		= (uint16_t*) &(analogVolts_Control.analogVolts_Setting);		// first register in holdings subgroup

// structure for digital inputs
// number of each type of modbus data element in the module
#define MODULE_DIGITAL_INPUTS_NUMBER_OF_COILS			0
#define MODULE_DIGITAL_INPUTS_NUMBER_OF_COIL_REGISTERS	0
#define MODULE_DIGITAL_INPUTS_NUMBER_OF_DISCRETES		(2)
#define MODULE_DIGITAL_INPUTS_NUMBER_OF_DISCRETE_REGISTERS		(1)
#define MODULE_DIGITAL_INPUTS_NUMBER_OF_INPUTS			(19) // (status, filtered, 32-bit input value x3, DEmand Percent, is_invert, is_enabled)
#define MODULE_DIGITAL_INPUTS_NUMBER_OF_HOLDINGS		(19) // (input function x3, debounce limit, input_polarity, input enable, demand percent x8)

// address for the start of each element type within the module
#define MODULE_DIGITAL_INPUTS_GROUP							(137)			// digital inputs group 
#define MODULE_DIGITAL_INPUTS_BASE_ADDRESS					(MODULE_DIGITAL_INPUTS_GROUP<<8)
#define	MODULE_DIGITAL_INPUTS_START_OF_COILS_ADDRESS		(MODULE_DIGITAL_INPUTS_BASE_ADDRESS + 0)
#define MODULE_DIGITAL_INPUTS_START_OF_DISCRETES_ADDRESS	(MODULE_DIGITAL_INPUTS_START_OF_COILS_ADDRESS + MAX_COILS)
#define MODULE_DIGITAL_INPUTS_START_OF_INPUTS_ADDRESS		(MODULE_DIGITAL_INPUTS_START_OF_DISCRETES_ADDRESS + MAX_DISCRETES)
#define MODULE_DIGITAL_INPUTS_START_OF_HOLDINGS_ADDRESS		(MODULE_DIGITAL_INPUTS_START_OF_INPUTS_ADDRESS + MAX_INPUTS)
#define	MODULE_DIGITAL_INPUTS_END_OF_GROUP					(MODULE_DIGITAL_INPUTS_START_OF_HOLDINGS_ADDRESS + MAX_HOLDINGS)

uint16_t*	module_digital_inputs_start_of_coils_pu16		= (uint16_t*) &(void_start_of_coils_pu16[0]);	
uint16_t*	module_digital_inputs_start_of_discretes_pu16	= (uint16_t*) &(digitalInputs_Control.digitalInputs_Data.discretes_u16);		// first register in discrete subgroup
uint16_t*	module_digital_inputs_start_of_inputs_pu16		= (uint16_t*) &(digitalInputs_Control.digitalInputs_Data);		// first register in holdings subgroup
uint16_t*	module_digital_inputs_start_of_holdings_pu16	= (uint16_t*) &(digitalInputs_Control.digitalInputs_Setting);			// first register in inputs subgroup

// structure for PWM input

// number of each type of modbus data element in the module
#define MODULE_PWM_INPUT_NUMBER_OF_COILS		(4)
#define MODULE_PWM_INPUT_NUMBER_OF_COIL_REGISTERS	(1)
#define MODULE_PWM_INPUT_NUMBER_OF_DISCRETES		(9)
#define MODULE_PWM_INPUT_NUMBER_OF_DISCRETE_REGISTERS	(1)
#define MODULE_PWM_INPUT_NUMBER_OF_INPUTS		(16) // (status, filtered, 32-bit input value x3, DEmand Percent, is_invert, is_enabled)
#define MODULE_PWM_INPUT_NUMBER_OF_HOLDINGS		(21) // (input function x3, debounce limit, input_polarity, input enable, demand percent x8)

// address for the start of each element type within the module
#define MODULE_PWM_INPUT_GROUP						(138)			// analog inputs 0-10V group 
#define MODULE_PWM_INPUT_BASE_ADDRESS				(MODULE_PWM_INPUT_GROUP<<8)
#define	MODULE_PWM_INPUT_START_OF_COILS_ADDRESS		(MODULE_PWM_INPUT_BASE_ADDRESS + 0)
#define MODULE_PWM_INPUT_START_OF_DISCRETES_ADDRESS	(MODULE_PWM_INPUT_START_OF_COILS_ADDRESS + MAX_COILS)
#define MODULE_PWM_INPUT_START_OF_INPUTS_ADDRESS	(MODULE_PWM_INPUT_START_OF_DISCRETES_ADDRESS + MAX_DISCRETES)
#define MODULE_PWM_INPUT_START_OF_HOLDINGS_ADDRESS	(MODULE_PWM_INPUT_START_OF_INPUTS_ADDRESS + MAX_INPUTS)
#define	MODULE_PWM_INPUT_END_OF_GROUP				(MODULE_PWM_INPUT_START_OF_HOLDINGS_ADDRESS + MAX_HOLDINGS)

uint16_t*	module_pwm_input_start_of_coils_pu16		= (uint16_t*) &(pwmInput_Control.pwmInput_Settings.flags_u16);			// first register in coil subgroup
uint16_t*	module_pwm_input_start_of_discretes_pu16	= (uint16_t*) &(pwmInput_Control.pwmInput_Data.discretes_u16);		// first register in discrete subgroup
uint16_t*	module_pwm_input_start_of_inputs_pu16		= (uint16_t*) &(pwmInput_Control.pwmInput_Data);			// first register in inputs subgroup
uint16_t*	module_pwm_input_start_of_holdings_pu16		= (uint16_t*) &(pwmInput_Control.pwmInput_Settings);		// first register in holdings subgroup

// structure for MODBUS module
// number of each type of modbus data element in the module
#define MODULE_MODBUS_RTU_NUMBER_OF_COILS			(6)
#define MODULE_MODBUS_RTU_NUMBER_OF_COIL_REGISTERS	(1)
#define MODULE_MODBUS_RTU_NUMBER_OF_DISCRETES		(0)
#define MODULE_MODBUS_RTU_NUMBER_OF_INPUTS			(1) // (status, filtered, 32-bit input value x3, DEmand Percent, is_invert, is_enabled)
#define MODULE_MODBUS_RTU_NUMBER_OF_HOLDINGS		(5) // (input function x3, debounce limit, input_polarity, input enable, demand percent x8)

// address for the start of each element type within the module
#define MODULE_MODBUS_RTU_GROUP							(139)			// analog inputs 0-10V group 
#define MODULE_MODBUS_RTU_BASE_ADDRESS					(MODULE_MODBUS_RTU_GROUP<<8)
#define	MODULE_MODBUS_RTU_START_OF_COILS_ADDRESS		(MODULE_MODBUS_RTU_BASE_ADDRESS + 0)
#define MODULE_MODBUS_RTU_START_OF_DISCRETES_ADDRESS	(MODULE_MODBUS_RTU_START_OF_COILS_ADDRESS + MAX_COILS)
#define MODULE_MODBUS_RTU_START_OF_INPUTS_ADDRESS		(MODULE_MODBUS_RTU_START_OF_DISCRETES_ADDRESS + MAX_DISCRETES)
#define MODULE_MODBUS_RTU_START_OF_HOLDINGS_ADDRESS		(MODULE_MODBUS_RTU_START_OF_INPUTS_ADDRESS + MAX_INPUTS)
#define	MODULE_MODBUS_RTU_END_OF_GROUP					(MODULE_MODBUS_RTU_START_OF_HOLDINGS_ADDRESS + MAX_HOLDINGS)

uint16_t*	module_modbus_rtu_start_of_coils_pu16		= (uint16_t*) (&(modbus_RTU_Control.modbus_RTU_Settings.flags_u16));			// first register in coil subgroup
uint16_t*	module_modbus_rtu_start_of_discretes_pu16	= (uint16_t*) (&(void_start_of_discretes_pu16[0]));		// first register in discrete subgroup
uint16_t*	module_modbus_rtu_start_of_holdings_pu16	= (uint16_t*) (&(modbus_RTU_Control.modbus_RTU_Settings));		// first register in holdings subgroup	  
uint16_t*	module_modbus_rtu_start_of_inputs_pu16		= (uint16_t*) (&(modbus_RTU_Control.modbus_RTU_Data));			// first register in inputs subgroup

// structure for analg input 4-20 mA
// number of each type of modbus data element in the module
#define MODULE_ANALOG_INPUTS_4TO20_NUMBER_OF_COILS			(5)
#define MODULE_ANALOG_INPUTS_4TO20_NUMBER_OF_COIL_REGISTERS	(1)
#define MODULE_ANALOG_INPUTS_4TO20_NUMBER_OF_DISCRETES		(6)
#define MODULE_ANALOG_INPUTS_4TO20_NUMBER_OF_DISCRETE_REGISTERS (1)
#define MODULE_ANALOG_INPUTS_4TO20_NUMBER_OF_INPUTS			(16) // (status, filtered, 32-bit input value x3, DEmand Percent, is_invert, is_enabled)
#define MODULE_ANALOG_INPUTS_4TO20_NUMBER_OF_HOLDINGS		(23) // (input function x3, debounce limit, input_polarity, input enable, demand percent x8)

// address for the start of each element type within the module
#define MODULE_ANALOG_INPUTS_4TO20_GROUP						(140)			// analog inputs 0-10V group 
#define MODULE_ANALOG_INPUTS_4TO20_BASE_ADDRESS					(MODULE_ANALOG_INPUTS_4TO20_GROUP<<8)
#define	MODULE_ANALOG_INPUTS_4TO20_START_OF_COILS_ADDRESS		(MODULE_ANALOG_INPUTS_4TO20_BASE_ADDRESS + 0)
#define MODULE_ANALOG_INPUTS_4TO20_START_OF_DISCRETES_ADDRESS	(MODULE_ANALOG_INPUTS_4TO20_START_OF_COILS_ADDRESS + MAX_COILS)
#define MODULE_ANALOG_INPUTS_4TO20_START_OF_INPUTS_ADDRESS		(MODULE_ANALOG_INPUTS_4TO20_START_OF_DISCRETES_ADDRESS + MAX_DISCRETES)
#define MODULE_ANALOG_INPUTS_4TO20_START_OF_HOLDINGS_ADDRESS	(MODULE_ANALOG_INPUTS_4TO20_START_OF_INPUTS_ADDRESS + MAX_INPUTS)
#define	MODULE_ANALOG_INPUTS_4TO20_END_OF_GROUP					(MODULE_ANALOG_INPUTS_4TO20_START_OF_HOLDINGS_ADDRESS + MAX_HOLDINGS)

uint16_t*	module_analog_inputs_4TO20_start_of_coils_pu16			= (uint16_t*) &(analog_4_20ma_Control_ptr.analog_4_20mA_Setting.flags_u16);			// first register in coil subgroup
uint16_t*	module_analog_inputs_4TO20_start_of_discretes_pu16		= (uint16_t*) &(analog_4_20ma_Control_ptr.analog_4_20mA_Data.discretes_u16);		// first register in discrete subgroup
uint16_t*	module_analog_inputs_4TO20_start_of_inputs_pu16			= (uint16_t*) &(analog_4_20ma_Control_ptr.analog_4_20mA_Data);			// first register in inputs subgroup
uint16_t*	module_analog_inputs_4TO20_start_of_holdings_pu16		= (uint16_t*) &(analog_4_20ma_Control_ptr.analog_4_20mA_Setting);		// first register in holdings subgroup

// structure for Demand Multiplexer
// number of each type of modbus data element in the module
#define MODULE_DEMAND_MULTIPLEXER_NUMBER_OF_COILS			0
#define MODULE_DEMAND_MULTIPLEXER_NUMBER_OF_COIL_REGISTERS	0
#define MODULE_DEMAND_MULTIPLEXER_NUMBER_OF_DISCRETES		0
#define MODULE_DEMAND_MULTIPLEXER_NUMBER_OF_DISCRETE_REGISTERS	0
#define MODULE_DEMAND_MULTIPLEXER_NUMBER_OF_INPUTS			(2) // 
#define MODULE_DEMAND_MULTIPLEXER_NUMBER_OF_HOLDINGS		(8) // (input function x3, debounce limit, input_polarity, input enable, demand percent x8)

// address for the start of each element type within the module
#define MODULE_DEMAND_MULTIPLEXER_GROUP							(141)			// demand multiplexer group
#define MODULE_DEMAND_MULTIPLEXER_BASE_ADDRESS					(MODULE_DEMAND_MULTIPLEXER_GROUP<<8)

#define	MODULE_DEMAND_MULTIPLEXER_START_OF_COILS_ADDRESS		(MODULE_DEMAND_MULTIPLEXER_BASE_ADDRESS + 0)
#define MODULE_DEMAND_MULTIPLEXER_START_OF_DISCRETES_ADDRESS	(MODULE_DEMAND_MULTIPLEXER_START_OF_COILS_ADDRESS + MAX_COILS)
#define MODULE_DEMAND_MULTIPLEXER_START_OF_INPUTS_ADDRESS		(MODULE_DEMAND_MULTIPLEXER_START_OF_DISCRETES_ADDRESS + MAX_DISCRETES)
#define MODULE_DEMAND_MULTIPLEXER_START_OF_HOLDINGS_ADDRESS		(MODULE_DEMAND_MULTIPLEXER_START_OF_INPUTS_ADDRESS + MAX_INPUTS)
#define	MODULE_DEMAND_MULTIPLEXER_END_OF_GROUP					(MODULE_DEMAND_MULTIPLEXER_START_OF_HOLDINGS_ADDRESS + MAX_HOLDINGS)

uint16_t*	module_demand_multiplexer_start_of_coils_pu16			= (uint16_t*) &(void_start_of_coils_pu16[0]);			// first register in coil subgroup
uint16_t*	module_demand_multiplexer_start_of_discretes_pu16		= (uint16_t*) &(void_start_of_discretes_pu16[0]);		// first register in discrete subgroup
uint16_t*	module_demand_multiplexer_start_of_inputs_pu16			= (uint16_t*) &(motorDemandMux_Control.motorDemandMux_Data);			// first register in inputs subgroup
uint16_t*	module_demand_multiplexer_start_of_holdings_pu16		= (uint16_t*) &(motorDemandMux_Control.motorDemandMux_Settings);		// first register in holdings subgroup

// structure for Firmware update
/***
// number of each type of modbus data element in the module
#define MODULE_FIRMWARE_UPDATE_NUMBER_OF_COILS			0
#define MODULE_FIRMWARE_UPDATE_NUMBER_OF_DISCRETES		0
#define MODULE_FIRMWARE_UPDATE_NUMBER_OF_INPUTS			(0) // 
#define MODULE_FIRMWARE_UPDATE_NUMBER_OF_HOLDINGS		(5) // 

// address for the start of each element type within the module
#define MODULE_FIRMWARE_UPDATE_GROUP						(142)			// demand multiplexer group
#define MODULE_FIRMWARE_UPDATE_BASE_ADDRESS					(MODULE_FIRMWARE_UPDATE_GROUP<<8)

#define	MODULE_FIRMWARE_UPDATE_START_OF_COILS_ADDRESS		(MODULE_FIRMWARE_UPDATE_BASE_ADDRESS + 0)
#define MODULE_FIRMWARE_UPDATE_START_OF_DISCRETES_ADDRESS	(MODULE_FIRMWARE_UPDATE_START_OF_COILS_ADDRESS + MAX_COILS)
#define MODULE_FIRMWARE_UPDATE_START_OF_INPUTS_ADDRESS		(MODULE_FIRMWARE_UPDATE_START_OF_DISCRETES_ADDRESS + MAX_DISCRETES)
#define MODULE_FIRMWARE_UPDATE_START_OF_HOLDINGS_ADDRESS	(MODULE_FIRMWARE_UPDATE_START_OF_INPUTS_ADDRESS + MAX_INPUTS)
#define	MODULE_FIRMWARE_UPDATE_END_OF_GROUP					(MODULE_FIRMWARE_UPDATE_START_OF_HOLDINGS_ADDRESS + MAX_HOLDINGS)

uint16_t*	module_firmware_update_start_of_coils_pu16			= (uint16_t*) (&(void_start_of_coils_pu16[0]));			// first register in coil subgroup
uint16_t*	module_firmware_update_start_of_discretes_pu16		= (uint16_t*) (&(void_start_of_coils_pu16[0]));		// first register in discrete subgroup
uint16_t*	module_firmware_update_start_of_inputs_pu16			= (uint16_t*) (&(void_start_of_coils_pu16[0]));		// first register in inputs subgroup
uint16_t*	module_firmware_update_start_of_holdings_pu16		= (uint16_t*) (&(motor_FW_Update_Control));		// first register in holdings subgroup

// structure for Flash Block Settings

// number of each type of modbus data element in the module
#define MODULE_FLASH_BLOCK_NUMBER_OF_COILS			0
#define MODULE_FLASH_BLOCK_NUMBER_OF_DISCRETES		0
#define MODULE_FLASH_BLOCK_NUMBER_OF_INPUTS			(1)		// 
#define MODULE_FLASH_BLOCK_NUMBER_OF_HOLDINGS		(0)	// 

// address for the start of each element type within the module
#define MODULE_FLASH_BLOCK_GROUP						(143)			// flash block group 
#define MODULE_FLASH_BLOCK_BASE_ADDRESS					(MODULE_FLASH_BLOCK_GROUP<<8)
#define	MODULE_FLASH_BLOCK_START_OF_COILS_ADDRESS		(MODULE_FLASH_BLOCK_BASE_ADDRESS + 0)
#define MODULE_FLASH_BLOCK_START_OF_DISCRETES_ADDRESS	(MODULE_FLASH_BLOCK_START_OF_COILS_ADDRESS + MAX_COILS)
#define MODULE_FLASH_BLOCK_START_OF_INPUTS_ADDRESS		(MODULE_FLASH_BLOCK_START_OF_DISCRETES_ADDRESS + MAX_DISCRETES)
#define MODULE_FLASH_BLOCK_START_OF_HOLDINGS_ADDRESS	(MODULE_FLASH_BLOCK_START_OF_INPUTS_ADDRESS + MAX_INPUTS)
#define	MODULE_FLASH_BLOCK_END_OF_GROUP					(MODULE_FLASH_BLOCK_START_OF_HOLDINGS_ADDRESS + MAX_HOLDINGS)

uint16_t*  module_flash_block_start_of_coils_pu16		= (uint16_t*) &(void_start_of_coils_pu16[0]);	  // First register in coil subgroup
uint16_t*  module_flash_block_start_of_discretes_pu16	= (uint16_t*) &(void_start_of_discretes_pu16[0]); // First register in discrete subgroup
uint16_t*  module_flash_block_start_of_holdings_pu16	= (uint16_t*) &(void_start_of_holdings_pu16[0]);  // First register in discrete subgroup
uint16_t*  module_flash_block_start_of_inputs_pu16		= (uint16_t*) &(flash_blk_setting_Control);       // First register in inputs subgroup
***/

// number of each type of modbus data element in the module
#define MODULE_DIGITAL_OUTPUTS_NUMBER_OF_COILS			0
#define MODULE_DIGITAL_OUTPUTS_NUMBER_OF_COIL_REGISTERS	0
#define MODULE_DIGITAL_OUTPUTS_NUMBER_OF_DISCRETES		(1)
#define MODULE_DIGITAL_OUTPUTS_NUMBER_OF_DISCRETE_REGISTERS		(1)
#define MODULE_DIGITAL_OUTPUTS_NUMBER_OF_INPUTS			(4)		// 
#define MODULE_DIGITAL_OUTPUTS_NUMBER_OF_HOLDINGS		(34)	// 

// address for the start of each element type within the module
#define MODULE_DIGITAL_OUTPUTS_GROUP						(144)			// digal outputs group 
#define MODULE_DIGITAL_OUTPUTS_BASE_ADDRESS					(MODULE_DIGITAL_OUTPUTS_GROUP<<8)
#define	MODULE_DIGITAL_OUTPUTS_START_OF_COILS_ADDRESS		(MODULE_DIGITAL_OUTPUTS_BASE_ADDRESS + 0)
#define MODULE_DIGITAL_OUTPUTS_START_OF_DISCRETES_ADDRESS	(MODULE_DIGITAL_OUTPUTS_START_OF_COILS_ADDRESS + MAX_COILS)
#define MODULE_DIGITAL_OUTPUTS_START_OF_INPUTS_ADDRESS		(MODULE_DIGITAL_OUTPUTS_START_OF_DISCRETES_ADDRESS + MAX_DISCRETES)
#define MODULE_DIGITAL_OUTPUTS_START_OF_HOLDINGS_ADDRESS	(MODULE_DIGITAL_OUTPUTS_START_OF_INPUTS_ADDRESS + MAX_INPUTS)
#define	MODULE_DIGITAL_OUTPUTS_END_OF_GROUP					(MODULE_DIGITAL_OUTPUTS_START_OF_HOLDINGS_ADDRESS + MAX_HOLDINGS)

uint16_t*	module_digital_outputs_start_of_coils_pu16		= (uint16_t*) &(void_start_of_coils_pu16[0]);			// first register in coil subgroup
uint16_t*	module_digital_outputs_start_of_discretes_pu16	= (uint16_t*) (&(digital_Outputs_Control.digital_outputs_Data.discretes_u16));		// first register in discrete subgroup
uint16_t*	module_digital_outputs_start_of_holdings_pu16	= (uint16_t*) (&(digital_Outputs_Control.digital_outputs_Setting));		// first register in holdings subgroup	  
uint16_t*	module_digital_outputs_start_of_inputs_pu16		= (uint16_t*) (&(digital_Outputs_Control.digital_outputs_Data));			// first register in inputs subgroup

// number of each type of modbus data element in the module
#define MODULE_MOTOR_COMMUNICATION_NUMBER_OF_COILS			0
#define MODULE_MOTOR_COMMUNICATION_NUMBER_OF_COIL_REGISTERS	0
#define MODULE_MOTOR_COMMUNICATION_NUMBER_OF_DISCRETES		(1)
#define MODULE_MOTOR_COMMUNICATION_NUMBER_OF_DISCRETE_REGISTERS		(1)
#define MODULE_MOTOR_COMMUNICATION_NUMBER_OF_INPUTS			(15)		// 
#define MODULE_MOTOR_COMMUNICATION_NUMBER_OF_HOLDINGS		(9)	// 

// address for the start of each element type within the module
#define MODULE_MOTOR_COMMUNICATION_GROUP						(145)			// digal outputs group 
#define MODULE_MOTOR_COMMUNICATION_BASE_ADDRESS					(MODULE_MOTOR_COMMUNICATION_GROUP<<8)
#define	MODULE_MOTOR_COMMUNICATION_START_OF_COILS_ADDRESS		(MODULE_MOTOR_COMMUNICATION_BASE_ADDRESS + 0)
#define MODULE_MOTOR_COMMUNICATION_START_OF_DISCRETES_ADDRESS	(MODULE_MOTOR_COMMUNICATION_START_OF_COILS_ADDRESS + MAX_COILS)
#define MODULE_MOTOR_COMMUNICATION_START_OF_INPUTS_ADDRESS		(MODULE_MOTOR_COMMUNICATION_START_OF_DISCRETES_ADDRESS + MAX_DISCRETES)
#define MODULE_MOTOR_COMMUNICATION_START_OF_HOLDINGS_ADDRESS	(MODULE_MOTOR_COMMUNICATION_START_OF_INPUTS_ADDRESS + MAX_INPUTS)
#define	MODULE_MOTOR_COMMUNICATION_END_OF_GROUP					(MODULE_MOTOR_COMMUNICATION_START_OF_HOLDINGS_ADDRESS + MAX_HOLDINGS)

uint16_t* module_motor_communication_start_of_coils_pu16		= (uint16_t*) &(void_start_of_coils_pu16[0]);			// first register in coil subgroup
uint16_t* module_motor_communication_start_of_discretes_pu16	= (uint16_t*) (&(motor_Com_Control.motor_Metering_Data.discretes_u16));		// first register in discrete subgroup
uint16_t* module_motor_communication_start_of_holdings_pu16	= (uint16_t*) (&(motor_Com_Control.motor_Setting));	// first register in holdings subgroup	  
uint16_t* module_motor_communication_start_of_inputs_pu16		= (uint16_t*) (&(motor_Com_Control.motor_Metering_Data));		// first register in inputs subgroup

// Number of each type of modbus data element in the module
#define MODULE_TEST_NUMBER_OF_COILS		 (0)
#define MODULE_TEST_NUMBER_OF_COIL_REGISTERS	 (1)
#define MODULE_TEST_NUMBER_OF_DISCRETES		 (2)
#define MODULE_TEST_NUMBER_OF_DISCRETE_REGISTERS (1)
#define MODULE_TEST_NUMBER_OF_INPUTS		 (2)	// 
#define MODULE_TEST_NUMBER_OF_HOLDINGS		 (7)	//

// Address for the start of each element type within the module
#define MODULE_TEST_GROUP						(131)	// Test module group 
#define MODULE_TEST_BASE_ADDRESS				(MODULE_TEST_GROUP<<8)
#define	MODULE_TEST_START_OF_COILS_ADDRESS		(MODULE_TEST_BASE_ADDRESS + 0)
#define MODULE_TEST_START_OF_DISCRETES_ADDRESS	(MODULE_TEST_START_OF_COILS_ADDRESS + MAX_COILS)
#define MODULE_TEST_START_OF_INPUTS_ADDRESS		(MODULE_TEST_START_OF_DISCRETES_ADDRESS + MAX_DISCRETES)
#define MODULE_TEST_START_OF_HOLDINGS_ADDRESS	(MODULE_TEST_START_OF_INPUTS_ADDRESS + MAX_INPUTS)
#define	MODULE_TEST_END_OF_GROUP				(MODULE_TEST_START_OF_HOLDINGS_ADDRESS + MAX_HOLDINGS)

uint16_t*  module_test_start_of_coils_pu16	= (uint16_t*) &(moduleTest_Control.moduleTest_Settings.flags_u16);   // First register in coil subgroup
uint16_t*  module_test_start_of_discretes_pu16	= (uint16_t*) (&(moduleTest_Control.moduleTest_Data.discretes_u16)); // First register in discrete subgroup
uint16_t*  module_test_start_of_holdings_pu16	= (uint16_t*) (&(moduleTest_Control.moduleTest_Settings));	   // First register in holdings subgroup	  
uint16_t*  module_test_start_of_inputs_pu16	= (uint16_t*) (&(moduleTest_Control.moduleTest_Data)); // First register in inputs subgroup

// number of each type of modbus data element in the module
#define MODULE_APP_ID_NUMBER_OF_COILS			(1)
#define MODULE_APP_ID_NUMBER_OF_COIL_REGISTERS	(1)
#define MODULE_APP_ID_NUMBER_OF_DISCRETES		(1)
#define MODULE_APP_ID_NUMBER_OF_DISCRETE_REGISTERS		(1)
#define MODULE_APP_ID_NUMBER_OF_INPUTS			(1)		// 
#define MODULE_APP_ID_NUMBER_OF_HOLDINGS		(3)	// 

// address for the start of each element type within the module
#define MODULE_APP_ID_GROUP							(130)			// application testing group 
#define MODULE_APP_ID_BASE_ADDRESS					(MODULE_APP_ID_GROUP<<8)
#define	MODULE_APP_ID_START_OF_COILS_ADDRESS		(MODULE_APP_ID_BASE_ADDRESS + 0)
#define MODULE_APP_ID_START_OF_DISCRETES_ADDRESS	(MODULE_APP_ID_START_OF_COILS_ADDRESS + MAX_COILS)
#define MODULE_APP_ID_START_OF_INPUTS_ADDRESS		(MODULE_APP_ID_START_OF_DISCRETES_ADDRESS + MAX_DISCRETES)
#define MODULE_APP_ID_START_OF_HOLDINGS_ADDRESS		(MODULE_APP_ID_START_OF_INPUTS_ADDRESS + MAX_INPUTS)
#define	MODULE_APP_ID_END_OF_GROUP					(MODULE_APP_ID_START_OF_HOLDINGS_ADDRESS + MAX_HOLDINGS)

uint16_t*	module_application_id_start_of_coils_pu16		= (uint16_t*) (&(application_id_control.application_id_settings.flags_u16));			// first register in coil subgroup
uint16_t*	module_application_id_start_of_discretes_pu16	= (uint16_t*) (&(application_id_control.application_id_data.discretes_u16));		// first register in discrete subgroup
uint16_t*	module_application_id_start_of_holdings_pu16	= (uint16_t*) (&(application_id_control.application_id_settings));		// first register in holdings subgroup	  
uint16_t*	module_application_id_start_of_inputs_pu16		= (uint16_t*) (&(application_id_control.application_id_data));			// first register in inputs subgroup

#endif /* _MODULE_MODBUS_APPLICATION_MAP_H_ */

