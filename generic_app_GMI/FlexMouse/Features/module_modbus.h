/**
  ********************************************************************************************************************************
  * @file    module_modbus.h 
  * @author  Myron Mychal
  * @brief   This is the header for the modbus module.
  * @details This header file creates the modbus constants.
  ********************************************************************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MODULE_MODBUS_H_
#define _MODULE_MODBUS_H_

/* Includes ------------------------------------------------------------------*/
#include "structured_memory.h"
#include "scheduler.h"
#include "driver_adc1.h"
#include "module_motor_com.h"
#include "zz_module_flash.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
                            CONSTANTS
*******************************************************************************/
#define MODBUS_ADDRESS 247 // Default Modbus Address
#define MODBUS_MIN_MESSAGE_LEN  5 // minimum number of bytes for a MODBUS packet

// Temporary definitions for testing the module - can be thron away eventually MRM
#define	STOP	0
#define	START	1

#define	BYTE_TO_WORD_MSB	256		// shift of 2^8 = 256 to put into upper byte in a word
#define	BYTE_TO_WORD_LSB	1		// shift of 2^0 = 1   to put into lower byte in a word

#define	BYTE_0		0		// Byte 0 in a Modbus packet
#define	BYTE_1		1
#define	BYTE_2		2
#define	BYTE_3		3
#define	BYTE_4		4
#define	BYTE_5		5
#define	BYTE_6		6
#define	BYTE_7		7
#define	BYTE_8		8
#define	BYTE_9		9
#define	BYTE_10		10

#define	EXCEPTION_FUNCTION_CODE_OFFSET	128

#define CRC_LENGTH						2 // crc_hi and crc_lo
#define	MINIMUM_RESPONSE_LENGTH			5
#define MINIMUM_WRITE_HEADER_LENGTH		7
#define	WRITE_HEADER_LENGTH				7 // slave address, function code, starting coil address MSB, LSB, number of coils MBS, LSB, Number of Data Bytes containing coil data bits
#define WRITE_SINGLE_HEADER_LENGTH		6 // slave address, function code, starting address MSB, LSB, Data MSB, LSB
#define	WRITE_RESPONSE_HEADER_LENGTH	6 // slave address, function code, starting address MSB, LSB, Number of total bytes written MSB, LSB
#define	READ_HEADER_LENGTH				6 // slave address, function code, starting address MSB, LSB, Total number of registers read MSB, LSB
#define READ_RESPONSE_HEADER_LENGTH		3 // slave address, function code, number of bytes in response
#define	MINIMUM_READ_RESPONSE_LENGTH	7 // slave address, function code, number of registers read, starting address MSB/LSB + CRC low and high 
#define	MINIMUM_WRITE_RESPONSE_LENGTH	8 // slave address, function code, number of registers written, strting address MSB/LSB + CRC low and high 
#define	EXCEPTION_RESPONSE_LENGTH		5 // slave address, function code, exception code + CRC LSB and MSB
#define READ_RECORDS_HEADER_LENGTH		10 // slave address, function code, # of bytes in message, reference type, file # hi and lo, data address hi and lo, number of data bytes(registers?) hi and low
#define READ_RECORDS_RESPONSE_HEADER_LENGTH 5 // slave address, function code, # of bytes in response, number of bytes in data, refrence byte
#define WRITE_RECORDS_HEADER_LENGTH		1
#define WRITE_RECORDS_RESPONSE_HEADER_LENGTH 1
#define MODBUS_REFERENCE_TYPE			6 // used for read/write records functions and is always a 6
#define UP_HEADER_LENGTH 				7 // universal protocol header length			
  
  
// macro definitions for MODBUS Protocol
#define MODBUS_FUNCTION_CODE_READ_COILS 					0x01
#define MODBUS_FUNCTION_CODE_READ_DISCRETE_INPUTS 			0x02
#define MODBUS_FUNCTION_CODE_READ_HOLDING_REGISTERS 		0x03
#define MODBUS_FUNCTION_CODE_READ_INPUT_REGISTERS 			0x04
#define MODBUS_FUNCTION_CODE_WRITE_COIL 					0x05
#define MODBUS_FUNCTION_CODE_WRITE_SINGLE_HOLDING_REGISTER 	0x06
#define MODBUS_FUNCTION_CODE_WRITE_MULTIPLE_COILS 			0x0F  
#define MODBUS_FUNCTION_CODE_WRITE_HOLDING_REGISTERS 		0x10
#define MODBUS_FUNCTION_CODE_READ_RECORDS 					0x14
#define MODBUS_FUNCTION_CODE_WRITE_RECORDS 					0x15  
#define MODBUS_FUNCTION_CODE_UTILITY                        0xFB

// file numbers for read/write records
#define DATA_LOGGER_FILE_NUMBER					0x00
#define COMPONENT_DATA_FILE_NUMBER				0x01
#define	APPLICATION_CONFIGURATION_FILE_NUMBER	0x02
#define	APPLICATION_PROGRAM_FILE_NUMBER			0x03
#define	DRIVE_CONFIGURATION_FILE_NUMBER			0x04
#define DRIVE_PROGRAM_FILE_NUMBER				0x05

// file address minimums
#define DATA_LOGGER_ADDRESS_MIN					0x0000
#define DATA_LOGGER_ADDRESS_MAX					0x01FF
#define	COMPONENT_DATA_ADDRESS_MIN				0x0200
#define COMPONENT_DATA_ADDRESS_MAX				0x02FF
#define DRIVE_CONFIGURATION_ADDRESS_MIN			0x0000
#define DRIVE_CONFIGURATION_ADDRESS_MAX			0x01FF

// internal flash pages files for function code 20/21 messages
#define	APP_FLASH_SETTINGS_PAGE				0x01
#define	APP_FLASH_SETTINGS_MIRROR_PAGE		0x02
#define	DRIVE_FLASH_SETTINGS_PAGE			0x03
#define	DRIVE_FLASH_SETTINGS_MIRROR_PAGE	0x04
  
#define MODUBS_FUNCTION_CODE_UTILITY_MIN_MESSAGE_LEN        7 // Address, FunctionCode, D1, D2, D3, cksum, cksum

#define	MODBUS_EXCEPTION_01	0x01		// Illegal Function Code
#define	MODBUS_EXCEPTION_02	0x02		// Ilegal Data Address
#define	MODBUS_EXCEPTION_03	0x03		// Illegal Data Value
#define	MODBUS_EXCEPTION_04	0x04		// Slave Device Failure
#define	MODBUS_EXCEPTION_05	0x05		// Acknowledge
#define MODBUS_EXCEPTION_06	0x06		// Slave Device Busy

#define	MODBUS_COIL_ADDRESS 	0x0001  
#define MODBUS_COIL_VALUE_ON 	0xFF00	// value for coil that is in ON state
#define MODBUS_COIL_VALUE_OFF 	0x0000	// value for coil that is in OFF state
#define DEFAULT_BAUD 115200

typedef struct 
{  
  //uint16_t	bits_convert_u16;		// Union structure of all 16 bits in the field				
  uint16_t  is_happy:1;              	// Set to "1" if loss of analog is detected
  uint16_t  is_working:1;       		// Set to "1" if analog voltage is decreasing
  uint16_t  is_sleeping:1;  			// Set to "1" if lower end hysteresis need to be enabled
  uint16_t  is_enabled:1;				// Set to "1" if upper end hysteresis need to be enabled  
  uint16_t  is_powered:1;				// Set to "1" if demand goes from 0 to above min demand.
  uint16_t  is_over:1;					// Set to "1" if the measured voltage is above "digitalOnVolts"
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
  //uint16_t  empty_bits_bf:10;			 // fill out the rest of the field
} Modbus_RTU_flags;  
  
// Modbus RTU settings
struct Modbus_RTU_Settings
{  
  uint16_t baud_rate_u16;             	// baud rate
  uint16_t data_bits_u16;				// data bits
  uint16_t stop_bits_u16;				// stop bits
  uint16_t parity_u16;					// parity
  uint16_t motor_address_u16;			// motor's follower address
  Modbus_RTU_flags flags_u16;	// flags
};

// Live Modbus Data
struct Modbus_RTU_Data
{  
  uint16_t messages_received_u16;		// number of modbus messages received
};

typedef struct{
 struct Modbus_RTU_Settings modbus_RTU_Settings;
 struct Modbus_RTU_Data modbus_RTU_Data;  
}Modbus_RTU_Control;

typedef struct 
{  
  uint16_t  is_enabled:1;				// Set to "1" if application ID is enabled
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
} ApplicationID_flags;    

typedef struct {
  uint16_t	minor_software_rev_u16;
  uint16_t	median_software_rev_u16;
  uint16_t	major_software_rev_u16;
  ApplicationID_flags flags_u16;
} ApplicationID_Settings;

typedef struct 
{  
  uint16_t  is_enabled:1;				// Set to "1" if application ID is enabled  
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
} ApplicationID_Discretes;    
  
typedef struct {
  uint16_t	errorCode_u16;
  ApplicationID_Discretes discretes_u16;
} ApplicationID_Data;

typedef struct{
  ApplicationID_Settings application_id_settings;
  ApplicationID_Data application_id_data;  
} ApplicationID_Control;

// Drive Dynamic group:
typedef struct 
{  
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
} DriveDynamic_flags;  

typedef struct {
  uint16_t	commanded_speed_u16;
  uint16_t	commanded_demand_u16;
  uint16_t	start_command_u16;
  uint16_t	demand_source_u16;
  uint16_t	direction_u16;
  DriveDynamic_flags flags_u16;
} DriveDynamic_Settings;

typedef struct 
{  // unused - empty
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
  uint16_t  enpty16:1;
} DriveDynamic_Discretes;    

typedef struct {
  uint16_t state_u16;
  uint16_t faults_u16;
  uint16_t regal_mc_state_u16;
  uint16_t direction_u16;
  uint16_t bus_voltage_u16;
  uint16_t measured_speed_u16;
  uint16_t measured_torque_u16;
  uint16_t shaft_power_u16;
  uint16_t ipm_temperature_u16;
  int16_t phase_current_ia_s16;
  int16_t phase_current_ib_s16;
  DriveDynamic_Discretes discretes_u16;
} DriveDynamic_Data;

typedef struct{
  DriveDynamic_Settings drive_dynamic_settings;
  DriveDynamic_Data drive_dynamic_data;  
} DriveDynamic_Control;

typedef struct{
  uint8_t device_address_u8;
  uint8_t function_code_u8;
  uint8_t length_u8;
  uint8_t is_exception_u8;
  uint8_t exception_type_u8;
} Modbus_Message;

// TODO: Move all these extern function declarations to a separate file once we finalize the motor system.
//
extern void Modbus_UpdateStoredStatus(uint8_t status);
extern void Modbus_UpdateStoredFaults(uint16_t faults);
extern void Modbus_UpdateStoredBusVoltage(uint16_t bus_voltage);
//
extern void Modbus_UpdateStoredDirection(uint8_t direction);
extern void Modbus_UpdateStoredMeasuredSpeed(int16_t measured_speed);
extern void Modbus_UpdateStoredTorque(int16_t torque);
//
extern void Modbus_UpdateStoredPower(int16_t power);
extern void Modbus_UpdateStoredTemperature(int16_t temperature);
extern void Modbus_PassEEPROMData(uint8_t *data_address_pu8, uint16_t length_u16);
extern void Modbus_PassDriveFlashData(uint8_t *data_address_pu8, uint16_t length_u16);
extern void Utility_ExecuteOperation(uint8_t function_id, uint8_t function_parameter, uint8_t function_subparameter); // TODO: Move to own File

extern void HarmonicInjection_ModbusUpdate(uint8_t hi_enable_u8, int16_t *hi_amplitudes_s16, uint8_t *hi_angle_multipliers_u8, uint16_t *hi_angle_offsets_u16, uint8_t* hi_is_inverted_u8, uint16_t* hi_min_speed_u16, uint16_t* hi_max_speed_u16);

// TODO: REPLACE CONSTANTS BELOW THIS LINE WITH MORE DESCRIPTIVE NAMES OR REMOVE THEM

/***************Start of Modbus Control (inside shared memory)******************/
struct Modbus_Settings{
  uint16_t baudRate_u16;        //4800, 9600, 14400, 19200, 38400, 57600, 115200
  uint8_t data_Bits_u8;         // 8 or 9
  uint8_t parity_u8;            //1= Odd; 2 = Even; 0 = None;
  uint8_t stop_Bits_u8;         //1 or 2
  uint8_t flow_Control;         //0 = None; 1= Xon/Xoff; 2 = Hardware
  uint8_t modbus_Slave_Address_u8;   //modbus address
  uint8_t modbus_Master_Address_u8; //modbus master address 
};

struct Modbus_Data{
  bool is_first_Valid_Msg;           //Flag indicating first valid msg received.
  bool is_modbus_Activity;           //Flag indicating recent serial activity.
  bool is_admin_Mode;                //Flag indicating enter Administrator Mode  
};

typedef struct{
 struct Modbus_Settings modbus_Settings ;
 struct Modbus_Data modbus_Data;  
}Modbus_Control;

typedef enum
{
    MB_READ_REGISTER,			// Read holding register values and pass
    MB_WRITE_REGISTER           // Write to holding register values
} MBRegisterMode;

typedef enum
{
    MB_NO_ERROR,                // no error
    MB_ILLEGAL_ADDRESS,         // illegal register address
    MB_ILLEGAL_ARGUMENT,        // illegal register argument
    MB_TIMED_OUT                //
} MBErrorCode;

#endif /* _MODULE_MODBUS_H_ */
