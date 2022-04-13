/**
********************************************************************************************************************************
* @file    module_modbus.c 
* @author  Myron Mychal
* @brief   This is a module that handles MODBUS responses o to valid MODBUS requests
* @details rocesses MODBUS funtion requst by responding with appropriate data
********************************************************************************************************************************
*/

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "driver_usart1.h"
#include "driver_usart2.h"
#include "module_modbus_application_map.h"
#include "module_modbus_drive_map.h"
#include "module_motor_demand_multiplexer.h"
//#include "main.h"
#include "stm32g0xx_ll_iwdg.h" // TODO: Watchdog is for Bootloader Resets only. Move Bootloader Features to a better location

typedef struct ModbusDataRangeStruct
{
  uint16_t	minimum_value_u16;
  uint16_t	maximum_value_u16;
} ModbusDataRange;

typedef struct ModbusMapBlockStruct
{
  uint16_t *start_of_data_pu16;		// a pointer to where the sequential data used by the motor is stored
  uint16_t start_address_u16;	        // the modbus start address
  uint8_t number_of_registers_u8;	// number registers from the start address in this struct
  //ModbusDataRange data_range_pu16[];	// store the minimum values for the data in this block (sequentially)
  //uint16_t maximum_values_pu16[];	// store the maximum values for the data in this block (sequentially)	
} ModbusMapBlock;

typedef struct ModbusCoilMapBlockStruct
{
  uint16_t *start_of_coil_data_pu16;		// a pointer to where the sequential data used by the motor is stored
  uint16_t start_coil_address_u16;	    	// the modbus start address
  uint8_t number_of_coil_registers_u8;	// number registers from the start address in this struct
  uint8_t number_of_coils_u8;				// number registers from the start address in this struct 
} ModbusCoilMapBlock;

// Now each modbus block has to be defined to point to a valid data structure and exactly match its sizes
// MODBUS BLocks:
//
// Monitor
// Operation

int16_t  hi_amplitude_s16[8]; 		
uint8_t  hi_angle_multiplier_hi_u8[8];
uint16_t hi_angle_offset_u16[8]; 		
uint8_t  hi_is_phase_inverted_u8[8]; 	
uint16_t hi_min_speed_u16[8]; 		
uint16_t hi_max_speed_u16[8]; 		
uint8_t  hi_is_harmonic_injection_allowed_hi_u8; 

uint64_t tt_ModbusLinkLostStop;
#define MODBUS_LOST_LINK_TIME 3000//10000

#define NUM_MODBUS_REGISTERS_5000s 1 // 49
uint16_t modbusRegisters5000s[NUM_MODBUS_REGISTERS_5000s] = {
  0,		// HI ENable/Disable		5000	
/*  0,		// Amplitude 1				5001	
  0,		// Amplitude 2				5002	
  0,		// Amplitude 3				5003	
  0,		// Amplitude 4				5004	
  0,		// Amplitude 5				5005		
  0,		// Amplitude 6				5006	
  0,		// Amplitude 7				5007	
  0,		// Amplitude 8				5008	
  0,		// Angle Multiplier 1 		5009	
  0,		// Angle Multiplier 2 		5010	
  0,		// Angle Multiplier 3 		5011	
  0,		// Angle Multiplier 4 		5012
  0,		// Angle Multiplier 5 		5013	
  0,		// Angle Multiplier 6 		5014  
  0,		// Angle Multiplier 7 		5015	
  0,		// Angle Multiplier 8 		5016	
  0,		// Angle Offset 1 			5017
  0,		// Angle Offset 2 			5018
  0,		// Angle Offset 3 			5019
  0,		// Angle Offset 4 			5020
  0,		// Angle Offset 5 			5021
  0,		// Angle Offset 6 			5022
  0,		// Angle Offset 7 			5023
  0,		// Angle Offset 8 			5024
  0,		// Phase Inversion 1 		5025
  0,		// Phase Inversion 2 		5026
  0,		// Phase Inversion 3 		5027
  0,		// Phase Inversion 4 		5028
  0,		// Phase Inversion 5 		5029
  0,		// Phase Inversion 6 		5030
  0,		// Phase Inversion 7 		5031
  0,		// Phase Inversion 8 		5032
  0,		// Minimum Speed 1 			5033
  0,		// Minimum Speed 2 			5034
  0,		// Minimum Speed 3 			5035
  0,		// Minimum Speed 4 			5036
  0,		// Minimum Speed 5 			5037
  0,		// Minimum Speed 6 			5038
  0,		// Minimum Speed 7 			5039
  0,		// Minimum Speed 8 			5040
  2250,		// Maximum Speed 1 			5041
  2250,		// Maximum Speed 2 			5042
  2250,		// Maximum Speed 3 			5043
  2250,		// Maximum Speed 4 			5044
  2250,		// Maximum Speed 5 			5045
  2250,		// Maximum Speed 6 			5046
  2250,		// Maximum Speed 7 			5047
  2250		// Maximum Speed 8 			5048  
*/
};
  
ModbusMapBlock BlockHarmonic =
{
  //	(uint16_t *)&measued_speed_u16,	// first data value in block
  modbusRegisters5000s,				// first data value in block
  5000,						// starting address for block
  NUM_MODBUS_REGISTERS_5000s,			// number of elements in block
};

ModbusCoilMapBlock BlockAnalog0TO10V_Coils =
{
  0,				// first data value in block
  MODULE_ANALOG_INPUTS_0TO10_START_OF_COILS_ADDRESS,	// starting address for block
  MODULE_ANALOG_INPUTS_0TO10_NUMBER_OF_COIL_REGISTERS,	// number of coil words in block
  MODULE_ANALOG_INPUTS_0TO10_NUMBER_OF_COILS,			// number of coil bits in block
};

ModbusCoilMapBlock BlockPWMInputs_Coils =
{
  0,				// first data value in block
  MODULE_PWM_INPUT_START_OF_COILS_ADDRESS,	// starting address for block
  MODULE_PWM_INPUT_NUMBER_OF_COIL_REGISTERS,	// number of coil words in block
  MODULE_PWM_INPUT_NUMBER_OF_COILS,			// number of coil bits in block
};

ModbusMapBlock BlockAnalog0TO10V_Inputs =
{
  0,				// first data value in block
  MODULE_ANALOG_INPUTS_0TO10_START_OF_INPUTS_ADDRESS,	// starting address for block
  MODULE_ANALOG_INPUTS_0TO10_NUMBER_OF_INPUTS,			// number of elements in block					
};

ModbusMapBlock BlockAnalog0TO10V_Holdings =
{
  0,				// first data value in block
  MODULE_ANALOG_INPUTS_0TO10_START_OF_HOLDINGS_ADDRESS,	// starting address for block
  MODULE_ANALOG_INPUTS_0TO10_NUMBER_OF_HOLDINGS,		// number of elements in block				
};



ModbusCoilMapBlock BlockDigitalInputs_Discrete =
{
  0,				// first data value in block
  MODULE_DIGITAL_INPUTS_START_OF_DISCRETES_ADDRESS,	// starting address for block
  MODULE_DIGITAL_INPUTS_NUMBER_OF_DISCRETE_REGISTERS,	// number of coil words in block
  MODULE_DIGITAL_INPUTS_NUMBER_OF_DISCRETES,			// number of coil bits in block
};

ModbusMapBlock BlockDigitalInputs_Inputs =
{
  0,				// first data value in block
  MODULE_DIGITAL_INPUTS_START_OF_INPUTS_ADDRESS,	// starting address for block
  MODULE_DIGITAL_INPUTS_NUMBER_OF_INPUTS,			// number of elements in block				
};

ModbusMapBlock BlockDigitalInputs_Holdings =
{
  0,				// first data value in block
  MODULE_DIGITAL_INPUTS_START_OF_HOLDINGS_ADDRESS,	// starting address for block
  MODULE_DIGITAL_INPUTS_NUMBER_OF_HOLDINGS,			// number of elements in block					
};

ModbusCoilMapBlock BlockDigitalOutputs_Discrete =
{
  0,				// first data value in block
  MODULE_DIGITAL_OUTPUTS_START_OF_DISCRETES_ADDRESS,	// starting address for block
  MODULE_DIGITAL_OUTPUTS_NUMBER_OF_DISCRETE_REGISTERS,	// number of coil words in block
  MODULE_DIGITAL_OUTPUTS_NUMBER_OF_DISCRETES,			// number of coil bits in block
};

ModbusMapBlock BlockDigitalOutputs_Inputs =
{
  0,				// first data value in block
  MODULE_DIGITAL_OUTPUTS_START_OF_INPUTS_ADDRESS,	// starting address for block
  MODULE_DIGITAL_OUTPUTS_NUMBER_OF_INPUTS,			// number of elements in block				
};

ModbusMapBlock BlockDigitalOutputs_Holdings =
{
  0,				// first data value in block
  MODULE_DIGITAL_OUTPUTS_START_OF_HOLDINGS_ADDRESS,	// starting address for block
  MODULE_DIGITAL_OUTPUTS_NUMBER_OF_HOLDINGS,		// number of elements in block					
};


ModbusMapBlock BlockPWMInputs_Inputs =
{
  0,				// first data value in block
  MODULE_PWM_INPUT_START_OF_INPUTS_ADDRESS,	// starting address for block
  MODULE_PWM_INPUT_NUMBER_OF_INPUTS,			// number of elements in block				
};

ModbusMapBlock BlockPWMInputs_Holdings =
{
  0,				// first data value in block
  MODULE_PWM_INPUT_START_OF_HOLDINGS_ADDRESS,	// starting address for block
  MODULE_PWM_INPUT_NUMBER_OF_HOLDINGS,		// number of elements in block			
};

ModbusCoilMapBlock BlockModbusRTU_Coils =
{
  0,				// first data value in block
  MODULE_MODBUS_RTU_START_OF_COILS_ADDRESS,	// starting address for block
  MODULE_MODBUS_RTU_NUMBER_OF_COIL_REGISTERS,	// number of coil words in block
  MODULE_MODBUS_RTU_NUMBER_OF_COILS,			// number of coil bits in block
};

ModbusCoilMapBlock BlockAnalog0TO10V_Discrete =
{
  0,				// first data value in block
  MODULE_ANALOG_INPUTS_0TO10_START_OF_DISCRETES_ADDRESS,	// starting address for block
  MODULE_ANALOG_INPUTS_0TO10_NUMBER_OF_DISCRETE_REGISTERS,			// number of coil words in block
  MODULE_ANALOG_INPUTS_0TO10_NUMBER_OF_DISCRETES,			// number of coil bits in block
};

ModbusCoilMapBlock BlockPWMInputs_Discrete =
{
  0,				// first data value in block
  MODULE_PWM_INPUT_START_OF_DISCRETES_ADDRESS,	// starting address for block
  MODULE_PWM_INPUT_NUMBER_OF_DISCRETE_REGISTERS,			// number of coil words in block
  MODULE_PWM_INPUT_NUMBER_OF_DISCRETES,			// number of coil bits in block
};

ModbusMapBlock BlockModbusRTU_Inputs =
{
  0,				// first data value in block
  MODULE_MODBUS_RTU_START_OF_INPUTS_ADDRESS,	// starting address for block
  MODULE_MODBUS_RTU_NUMBER_OF_INPUTS,			// number of elements in block					
};

ModbusMapBlock BlockModbusRTU_Holdings =
{
  0,				// first data value in block
  MODULE_MODBUS_RTU_START_OF_HOLDINGS_ADDRESS,	// starting address for block
  MODULE_MODBUS_RTU_NUMBER_OF_HOLDINGS,		// number of elements in block				
};

ModbusCoilMapBlock BlockAnalog4TO20_Coils =
{
  0,				// first data value in block
  MODULE_ANALOG_INPUTS_4TO20_START_OF_COILS_ADDRESS,	// starting address for block
  MODULE_ANALOG_INPUTS_4TO20_NUMBER_OF_COIL_REGISTERS,	// number of coil words in block
  MODULE_ANALOG_INPUTS_4TO20_NUMBER_OF_COILS,			// number of coil bits in block
};

ModbusCoilMapBlock BlockAnalog4TO20_Discrete =
{
  0,				// first data value in block
  MODULE_ANALOG_INPUTS_4TO20_START_OF_DISCRETES_ADDRESS,	// starting address for block
  MODULE_ANALOG_INPUTS_4TO20_NUMBER_OF_DISCRETE_REGISTERS,	// number of coil words in block
  MODULE_ANALOG_INPUTS_4TO20_NUMBER_OF_DISCRETES,			// number of coil bits in block
};

ModbusMapBlock BlockAnalog4TO20_Inputs =
{
  0,				// first data value in block
  MODULE_ANALOG_INPUTS_4TO20_START_OF_INPUTS_ADDRESS,	// starting address for block
  MODULE_ANALOG_INPUTS_4TO20_NUMBER_OF_INPUTS,			// number of elements in block				
};

ModbusMapBlock BlockAnalog4TO20_Holdings =
{
  0,				// first data value in block
  MODULE_ANALOG_INPUTS_4TO20_START_OF_HOLDINGS_ADDRESS,	// starting address for block
  MODULE_ANALOG_INPUTS_4TO20_NUMBER_OF_HOLDINGS,		// number of elements in block				
};

ModbusMapBlock BlockDemandMultiplexer_Inputs =
{
  0,				// first data value in block
  MODULE_DEMAND_MULTIPLEXER_START_OF_INPUTS_ADDRESS,	// starting address for block
  MODULE_DEMAND_MULTIPLEXER_NUMBER_OF_INPUTS,			// number of elements in block					
};

ModbusMapBlock BlockDemandMultiplexer_Holdings =
{
  0,				// first data value in block
  MODULE_DEMAND_MULTIPLEXER_START_OF_HOLDINGS_ADDRESS,	// starting address for block
  MODULE_DEMAND_MULTIPLEXER_NUMBER_OF_HOLDINGS,		// number of elements in block				
};

ModbusCoilMapBlock BlockMotorCommunication_Discrete =
{
  0,				// first data value in block
  MODULE_MOTOR_COMMUNICATION_START_OF_DISCRETES_ADDRESS,	// starting address for block
  MODULE_MOTOR_COMMUNICATION_NUMBER_OF_DISCRETE_REGISTERS,	// number of coil words in block
  MODULE_MOTOR_COMMUNICATION_NUMBER_OF_DISCRETES,			// number of coil bits in block
};

ModbusMapBlock BlockMotorCommunication_Inputs =
{
  0,				// first data value in block
  MODULE_MOTOR_COMMUNICATION_START_OF_INPUTS_ADDRESS,	// starting address for block
  MODULE_MOTOR_COMMUNICATION_NUMBER_OF_INPUTS,			// number of elements in block					
};

ModbusMapBlock BlockMotorCommunication_Holdings =
{
  0,				// first data value in block
  MODULE_MOTOR_COMMUNICATION_START_OF_HOLDINGS_ADDRESS,	// starting address for block
  MODULE_MOTOR_COMMUNICATION_NUMBER_OF_HOLDINGS,		// number of elements in block					
};

/***
ModbusMapBlock BlockFirmwareUpdate_Inputs =
{
  0,				// first data value in block
  MODULE_FIRMWARE_UPDATE_START_OF_INPUTS_ADDRESS,	// starting address for block
  MODULE_FIRMWARE_UPDATE_NUMBER_OF_INPUTS,			// number of elements in block					
};

ModbusMapBlock BlockFirmwareUpdate_Holdings =
{
  0,				// first data value in block
  MODULE_FIRMWARE_UPDATE_START_OF_HOLDINGS_ADDRESS,	// starting address for block
  MODULE_FIRMWARE_UPDATE_NUMBER_OF_HOLDINGS,		// number of elements in block				
};

ModbusMapBlock BlockFlashBlock_Inputs =
{
  0,				// first data value in block
  MODULE_FLASH_BLOCK_START_OF_INPUTS_ADDRESS,	// starting address for block
  MODULE_FLASH_BLOCK_NUMBER_OF_INPUTS,			// number of elements in block				
};
***/

/***
ModbusCoilMapBlock BlockApplicationID_Coils =
{
  0,				// first data value in block
  MODULE_APP_ID_START_OF_COILS_ADDRESS,	// starting address for block
  MODULE_APP_ID_NUMBER_OF_COIL_REGISTERS,	// number of coil words in block
  MODULE_APP_ID_NUMBER_OF_COILS,			// number of coil bits in block
};

ModbusMapBlock BlockApplicationID_Holdings =
{
  0,				// first data value in block
  MODULE_APP_ID_START_OF_HOLDINGS_ADDRESS,	// starting address for block
  MODULE_APP_ID_NUMBER_OF_HOLDINGS,			// number of elements in block					
};

ModbusCoilMapBlock BlockApplicationID_Discretes =
{
  0,				// first data value in block
  MODULE_APP_ID_START_OF_DISCRETES_ADDRESS,	// starting address for block
  MODULE_APP_ID_NUMBER_OF_DISCRETE_REGISTERS,	// number of coil words in block
  MODULE_APP_ID_NUMBER_OF_DISCRETES,			// number of coil bits in block
};

ModbusMapBlock BlockApplicationID_Inputs =
{
  0,				// first data value in block
  MODULE_APP_ID_START_OF_INPUTS_ADDRESS,	// starting address for block
  MODULE_APP_ID_NUMBER_OF_INPUTS,			// number of elements in block				
};
***/

ModbusCoilMapBlock BlockDriveDynamic_Coils =
{
  0,				// first data value in block
  MODULE_DRIVE_DYNAMIC_START_OF_COILS_ADDRESS,	// starting address for block
  MODULE_DRIVE_DYNAMIC_NUMBER_OF_COIL_REGISTERS,	// number of coil words in block
  MODULE_DRIVE_DYNAMIC_NUMBER_OF_COILS,			// number of coil bits in block
};

ModbusMapBlock BlockDriveDynamic_Holdings =
{
  0,				// first data value in block
  MODULE_DRIVE_DYNAMIC_START_OF_HOLDINGS_ADDRESS,	// starting address for block
  MODULE_DRIVE_DYNAMIC_NUMBER_OF_HOLDINGS,			// number of elements in block					
};

ModbusCoilMapBlock BlockDriveDynamic_Discretes =
{
  0,				// first data value in block
  MODULE_DRIVE_DYNAMIC_START_OF_DISCRETES_ADDRESS,	// starting address for block
  MODULE_DRIVE_DYNAMIC_NUMBER_OF_DISCRETE_REGISTERS,	// number of coil words in block
  MODULE_DRIVE_DYNAMIC_NUMBER_OF_DISCRETES,			// number of coil bits in block
};

ModbusMapBlock BlockDriveDynamic_Inputs =
{
  0,				// first data value in block
  MODULE_DRIVE_DYNAMIC_START_OF_INPUTS_ADDRESS,	// starting address for block
  MODULE_DRIVE_DYNAMIC_NUMBER_OF_INPUTS,			// number of elements in block				
};

ModbusCoilMapBlock *masterCoilBlocks[] =
{
  //&BlockTest,
  //&BlockCoils,					// segregated into drive groups
  &BlockAnalog0TO10V_Coils,
  &BlockAnalog0TO10V_Discrete,  
  &BlockAnalog4TO20_Coils,
  &BlockAnalog4TO20_Discrete, 
  &BlockDigitalInputs_Discrete,
  &BlockDigitalOutputs_Discrete,
  &BlockPWMInputs_Coils,
  &BlockPWMInputs_Discrete, 
  &BlockModbusRTU_Coils,  
  &BlockMotorCommunication_Discrete,
  //&BlockApplicationID_Coils,
  //&BlockApplicationID_Discretes,
  &BlockDriveDynamic_Coils,
  &BlockDriveDynamic_Discretes,  
};

ModbusMapBlock *masterBlocks[] =
{
  //&BlockTest,
  //&BlockMonitor,				// moved to drive dynamic inputs
  //&BlockOperation,				// moved to drive drive dynamic holdings
  &BlockDriveDynamic_Inputs,
  &BlockDriveDynamic_Holdings,
  //&BlockHarmonic,
  &BlockAnalog0TO10V_Inputs,
  &BlockAnalog0TO10V_Holdings,  
  &BlockDigitalInputs_Inputs,
  &BlockDigitalInputs_Holdings,
  &BlockDigitalOutputs_Inputs,
  &BlockDigitalOutputs_Holdings,   
  &BlockPWMInputs_Inputs,
  &BlockPWMInputs_Holdings, 
  //&BlockApplicationID_Inputs,
  //&BlockApplicationID_Holdings,
  &BlockModbusRTU_Inputs,
  &BlockModbusRTU_Holdings,
  &BlockAnalog4TO20_Inputs,
  &BlockAnalog4TO20_Holdings,
  &BlockDemandMultiplexer_Inputs,
  &BlockDemandMultiplexer_Holdings,
  &BlockMotorCommunication_Inputs,
  &BlockMotorCommunication_Holdings,
  //&BlockFirmwareUpdate_Holdings,
  //&BlockFlashBlock_Inputs,
};

uint16_t number_of_modbus_blocks_u16 = sizeof(masterBlocks)/sizeof(masterBlocks[0]);
uint16_t number_of_modbus_coil_blocks_u16 = sizeof(masterCoilBlocks)/sizeof(masterCoilBlocks[0]);

enum {
  MEMORY_INIT_MODULE,
  INIT_MODULE,
  RUN_MODULE,
  // Ddditional states to be added here as necessary.
  IRQ_MODULE = DEFAULT_IRQ_STATE,
  KILL_MODULE = KILL_APP
};

// Global variables
extern ProcessInfo processInfoTable[];
extern Usart2_Control* usart2Control_AppLocal;
Usart1_Control* usart1Control_Modbus;
Modbus_RTU_Control modbus_RTU_Control;
ApplicationID_Control application_id_control;
DriveDynamic_Control drive_dynamic_control;
Modbus_Message last_eeprom_request;				// last modbus message sent performing an eeprom request
Modbus_Message last_drive_flash_request;		// last modbus message sent peforming a drive-ide flash request

static Ram_Buf_Handle module_Control_StructMem_u32;


#define ENABLE_MODBUS_PROTOCOLBUF_RX_FIXED_LEN 1
#if ENABLE_MODBUS_PROTOCOLBUF_RX_FIXED_LEN >= 1
// This is a one-shot buffer, that is written to and read from in single calls.
// - it does not currently need to be tracked for current index because of this.
#define FIXED_MODBUS_PROTOCOLBUF_RX_MAX_LENGTH USART1_SINGLE_MESSAGE_RX_BUF_SIZE // Inclusive (this value is accepted) 
uint8_t fixedModbus_ProtocolBufRX_Length = 0;
uint8_t fixedModbus_ProtocolBufRX[FIXED_MODBUS_PROTOCOLBUF_RX_MAX_LENGTH];
uint8_t* modbus_ProtocolBufRX = fixedModbus_ProtocolBufRX;
#else // if ENABLE_MODBUS_PROTOCOLBUF_RX_FIXED_LEN <= 0
uint8_t* modbus_ProtocolBufRX;
#endif // if ENABLE_MODBUS_PROTOCOLBUF_RX_FIXED_LEN <= 0


#define ENABLE_MODBUS_PROTOCOLBUF_TX_FIXED_LEN 1
#if ENABLE_MODBUS_PROTOCOLBUF_TX_FIXED_LEN >= 1
// This is a one-shot buffer, that is written to and read from in single calls.
// - it does not currently need to be tracked for current index because of this.
#define FIXED_MODBUS_PROTOCOLBUF_TX_MAX_LENGTH USART1_SINGLE_MESSAGE_TX_BUF_SIZE // Inclusive (this value is accepted) 
uint8_t fixedModbus_ProtocolBufTX_Length = 0;
uint8_t fixedModbus_ProtocolBufTX[FIXED_MODBUS_PROTOCOLBUF_TX_MAX_LENGTH];
uint8_t* modbus_ProtocolBufTX = fixedModbus_ProtocolBufTX;
#else // if ENABLE_MODBUS_PROTOCOLBUF_TX_FIXED_LEN <= 0
uint8_t* modbus_ProtocolBufTX;
#endif // if ENABLE_MODBUS_PROTOCOLBUF_TX_FIXED_LEN <= 0

Modbus_RTU_Control modbus_RTU_control;

// Local Function Prototypes
void Modbus_ParseReceivedMessages(void);
ModbusMapBlock *find_modbus_block(uint16_t desired_address_u16, uint16_t number_of_registers_u16);
ModbusCoilMapBlock *find_modbus_coil_block(uint16_t desired_coil_address_u16, uint16_t number_of_coils_u16);

MBErrorCode ProcessMBHoldingRegister(uint8_t * data_buffer_pu8, uint16_t starting_address_u16, uint16_t number_of_registers_u16, MBRegisterMode eMode);
MBErrorCode ProcessMBCoilRegister_New(uint8_t * data_buffer_pu8, uint16_t desired_coil_address_u16, uint16_t number_of_coils_u16, MBRegisterMode eMode);
MBErrorCode ProcessMBReadRecords(uint8_t * data_buffer_pu8, uint16_t offset_address_u16, uint16_t number_of_registers_u16, uint8_t file_number_u8);
MBErrorCode ProcessMBWriteRecords(uint8_t * data_buffer_pu8, uint16_t flash_offset_address_u16, uint16_t number_of_flash_words_u16, uint8_t file_number_u8);
MBErrorCode ProcessMBDiscreteRegister(uint8_t * data_buffer_pu8, uint16_t starting_address_u16, uint16_t number_of_registers_u16, MBRegisterMode eMode);
void Build_UPDrive_Flash_Request(uint16_t data_address_u16, uint8_t length_u8);
void Build_UPEEPROM_Request(uint16_t data_address_u16, uint8_t length_u8);
void AssignModuleMemModbus(void);
void Modbus_JumpToBootloader(void);

uint16_t calculateModbusCRC(uint8_t *buf_pu8, uint16_t length_u16);

void UpdateMotorDemandMultiplexer(void) {
  uint16_t commanded_speed = drive_dynamic_control.drive_dynamic_settings.commanded_speed_u16;
  uint16_t commanded_demand = drive_dynamic_control.drive_dynamic_settings.commanded_demand_u16;
  uint16_t commanded_start = drive_dynamic_control.drive_dynamic_settings.start_command_u16;
  uint16_t demand_source = drive_dynamic_control.drive_dynamic_settings.demand_source_u16;
  uint16_t direction = drive_dynamic_control.drive_dynamic_settings.direction_u16;
  MotorDemandMux_ModbusUpdate(commanded_speed, commanded_demand, commanded_start, demand_source, direction);
}

void UpdateHarmonicInjectionParameters(void) {
  uint8_t	index_u8 = 0;	
  hi_is_harmonic_injection_allowed_hi_u8 	= modbusRegisters5000s[0]; 
  for(index_u8 = 0; index_u8 < 8; index_u8++) {
  	hi_amplitude_s16[index_u8] 			= (int16_t)  modbusRegisters5000s[1+index_u8]; 
  	hi_angle_multiplier_hi_u8[index_u8] 	= (uint8_t)  modbusRegisters5000s[9+index_u8];
  	hi_angle_offset_u16[index_u8] 			= (uint16_t) modbusRegisters5000s[17+index_u8];
  	hi_is_phase_inverted_u8[index_u8] 		= (uint8_t)  modbusRegisters5000s[25+index_u8];
  	hi_min_speed_u16[index_u8] 			= (uint16_t) modbusRegisters5000s[33+index_u8];
  	hi_max_speed_u16[index_u8] 			= (uint16_t) modbusRegisters5000s[41+index_u8];  
  }
  HarmonicInjection_ModbusUpdate(hi_is_harmonic_injection_allowed_hi_u8, hi_amplitude_s16, hi_angle_multiplier_hi_u8, hi_angle_offset_u16, hi_is_phase_inverted_u8, hi_min_speed_u16, hi_max_speed_u16);
}

// module functions
uint8_t moduleModbus(uint8_t drv_id_u8, uint8_t prev_state_u8, uint8_t next_state_u8, uint8_t irq_id_u8) {
  uint8_t return_state_u8 = MEMORY_INIT_MODULE;
  switch (next_state_u8) {
  case MEMORY_INIT_MODULE:
    {
      //AssignModuleMemModbus(); // Assign structured memory to Analog 0-10V setting and data 
      return_state_u8 = INIT_MODULE;
	  // Modbus RTU settings
	  modbus_RTU_Control.modbus_RTU_Settings.baud_rate_u16 = 24;	// baud rate = 24*4800 = 115200
	  modbus_RTU_Control.modbus_RTU_Settings.data_bits_u16 = 8;		// data bits
	  modbus_RTU_Control.modbus_RTU_Settings.stop_bits_u16 = 1;		// stop bits
	  modbus_RTU_Control.modbus_RTU_Settings.parity_u16 = 0;		// parity 0 = NONE, 1 = ODD; 2 = EVEN
	  modbus_RTU_Control.modbus_RTU_Settings.motor_address_u16 = MODBUS_ADDRESS;			// motor's MDBUS follower address
	  modbus_RTU_Control.modbus_RTU_Settings.flags_u16.is_happy = 1;
	  modbus_RTU_Control.modbus_RTU_Settings.flags_u16.is_working = 1;
	  modbus_RTU_Control.modbus_RTU_Settings.flags_u16.is_sleeping = 0;
	  modbus_RTU_Control.modbus_RTU_Settings.flags_u16.is_enabled = 0;
	  modbus_RTU_Control.modbus_RTU_Settings.flags_u16.is_powered = 0;
	  modbus_RTU_Control.modbus_RTU_Settings.flags_u16.is_over = 1;

	  modbus_RTU_Control.modbus_RTU_Data.messages_received_u16 = 2112;
	  
	  application_id_control.application_id_settings.minor_software_rev_u16 = 0x3036;	// Revision 06
	  application_id_control.application_id_settings.median_software_rev_u16 = 0x3052;	// Revision 0R
	  application_id_control.application_id_settings.major_software_rev_u16 = 0x3036;	// Revision 00	  
	  application_id_control.application_id_settings.flags_u16.is_enabled = 1;
	  
	  application_id_control.application_id_data.errorCode_u16 = 2112;	  
	  application_id_control.application_id_data.discretes_u16.is_enabled = 1;
	  
	  // map modbus registers to corresponding modules
	  BlockAnalog0TO10V_Coils.start_of_coil_data_pu16	= module_analog_inputs_0TO10_start_of_coils_pu16;
	  BlockAnalog0TO10V_Discrete.start_of_coil_data_pu16	= module_analog_inputs_0TO10_start_of_discretes_pu16;
	  BlockAnalog0TO10V_Inputs.start_of_data_pu16 		= module_analog_inputs_0TO10_start_of_inputs_pu16;
	  BlockAnalog0TO10V_Holdings.start_of_data_pu16 	= module_analog_inputs_0TO10_start_of_holdings_pu16;
	  
	  BlockDigitalInputs_Discrete.start_of_coil_data_pu16 = module_digital_inputs_start_of_discretes_pu16;
	  BlockDigitalInputs_Inputs.start_of_data_pu16 		= module_digital_inputs_start_of_inputs_pu16;
	  BlockDigitalInputs_Holdings.start_of_data_pu16 	= module_digital_inputs_start_of_holdings_pu16;
	  BlockDigitalInputs_Discrete.start_of_coil_data_pu16 = module_digital_outputs_start_of_discretes_pu16;
	  
	  BlockDigitalOutputs_Inputs.start_of_data_pu16 	= module_digital_outputs_start_of_inputs_pu16;
	  BlockDigitalOutputs_Holdings.start_of_data_pu16 	= module_digital_outputs_start_of_holdings_pu16;
	  
          BlockPWMInputs_Coils.start_of_coil_data_pu16	        = module_pwm_input_start_of_coils_pu16;
	  BlockPWMInputs_Discrete.start_of_coil_data_pu16	= module_pwm_input_start_of_discretes_pu16;
	  BlockPWMInputs_Inputs.start_of_data_pu16 			= module_pwm_input_start_of_inputs_pu16;
	  BlockPWMInputs_Holdings.start_of_data_pu16 		= module_pwm_input_start_of_holdings_pu16;
	  
	  BlockModbusRTU_Coils.start_of_coil_data_pu16		= module_modbus_rtu_start_of_coils_pu16;
	  BlockModbusRTU_Inputs.start_of_data_pu16 			= module_modbus_rtu_start_of_inputs_pu16;
	  BlockModbusRTU_Holdings.start_of_data_pu16 		= module_modbus_rtu_start_of_holdings_pu16;
	  
	  BlockAnalog4TO20_Coils.start_of_coil_data_pu16	= module_analog_inputs_4TO20_start_of_coils_pu16;
	  BlockAnalog4TO20_Discrete.start_of_coil_data_pu16	= module_analog_inputs_4TO20_start_of_discretes_pu16;
	  BlockAnalog4TO20_Inputs.start_of_data_pu16 		= module_analog_inputs_4TO20_start_of_inputs_pu16;
	  BlockAnalog4TO20_Holdings.start_of_data_pu16 		= module_analog_inputs_4TO20_start_of_holdings_pu16;	
	  
	  BlockMotorCommunication_Discrete.start_of_coil_data_pu16	= module_motor_communication_start_of_discretes_pu16;
	  BlockMotorCommunication_Inputs.start_of_data_pu16 		= module_motor_communication_start_of_inputs_pu16;
	  BlockMotorCommunication_Holdings.start_of_data_pu16 		= module_motor_communication_start_of_holdings_pu16;	
	  
	  BlockDemandMultiplexer_Inputs.start_of_data_pu16 		= module_demand_multiplexer_start_of_inputs_pu16;
	  BlockDemandMultiplexer_Holdings.start_of_data_pu16 	= module_demand_multiplexer_start_of_holdings_pu16;	  	  
	  
	  //BlockFirmwareUpdate_Inputs.start_of_data_pu16 	= module_firmware_update_start_of_inputs_pu16;
	  
	  //BlockFirmwareUpdate_Holdings.start_of_data_pu16 	= module_firmware_update_start_of_holdings_pu16;	  
	  
	  //BlockFlashBlock_Inputs.start_of_data_pu16 		= module_flash_block_start_of_inputs_pu16;
	  
	  
	  //BlockApplicationID_Coils.start_of_coil_data_pu16		= module_application_id_start_of_coils_pu16;
	  //BlockApplicationID_Discretes.start_of_coil_data_pu16	= module_application_id_start_of_discretes_pu16;
	  //BlockApplicationID_Inputs.start_of_data_pu16 			= module_application_id_start_of_inputs_pu16;
	  //BlockApplicationID_Holdings.start_of_data_pu16 		= module_application_id_start_of_holdings_pu16;	

	  BlockDriveDynamic_Coils.start_of_coil_data_pu16		= module_drive_dynamic_start_of_coils_pu16;
	  BlockDriveDynamic_Discretes.start_of_coil_data_pu16	= module_drive_dynamic_start_of_discretes_pu16;
	  BlockDriveDynamic_Inputs.start_of_data_pu16 			= module_drive_dynamic_start_of_inputs_pu16;
	  BlockDriveDynamic_Holdings.start_of_data_pu16 		= module_drive_dynamic_start_of_holdings_pu16;		  
	  
	  // initialize last eeprom request to have an exception
	  last_eeprom_request.device_address_u8 = MODBUS_ADDRESS;
	  last_eeprom_request.function_code_u8 = MODBUS_FUNCTION_CODE_READ_RECORDS;
	  last_eeprom_request.length_u8 = 0;
	  last_eeprom_request.is_exception_u8 = TRUE;
	  last_eeprom_request.exception_type_u8 = MODBUS_EXCEPTION_04; // device failure

	  last_drive_flash_request.device_address_u8 = MODBUS_ADDRESS;
	  last_drive_flash_request.function_code_u8 = MODBUS_FUNCTION_CODE_READ_RECORDS;
	  last_drive_flash_request.length_u8 = 0;
	  last_drive_flash_request.is_exception_u8 = TRUE;
	  last_drive_flash_request.exception_type_u8 = MODBUS_EXCEPTION_04; // device failure	  
	  
	  break;
    }
  case INIT_MODULE: 
    {
      // initialize test registers for modbus
      //initMODBUSRegisterMap();
      /* Attach Uart1 shared memory into this Module */

      tt_ModbusLinkLostStop = 0xFFFFFFFFFFFFFFFF; // Restart Modbus Link Lost Timer
      
      uint8_t Usart1index  = getProcessInfoIndex(MODULE_USART1);              //return Process index from processInfo array with the Uart2 driver
      usart1Control_Modbus = (Usart1_Control*) ((*(processInfoTable[Usart1index].Sched_DrvData.p_masterSharedMem_u32)).p_ramBuf_u8);
      return_state_u8 = RUN_MODULE ;
      break;
    }
  case RUN_MODULE: 
    {
      //unsigned char received_slave_address_u8 = (*usart1Control_Modbus).seqMem_RawRx;
      //unsigned char received_function_code_u8 = (*usart1Control_Modbus).seqMem_RawRx;
      // test CRC
      // test length
      
      //Ring_Buf_Handle seqMem_ModbusRx;
      //Ring_Buf_Handle seqMemTX;
      //Ring_Buf_Handle seqMem_RawRx;
      //int16_t motorSpeed_s16;
      //uint16_t motorStatus_u16;
      //uint8_t errorCode_u8;
      //} Usart1_Control;		

      if(getSysCount() >= tt_ModbusLinkLostStop) // Modbus Link Lost
      {            
        MotorDemandMux_ModbusUpdate(0, 0, 1, 5, 1);
      }  	  
      Modbus_ParseReceivedMessages();
	/*  if(RingBuf_GetUsedNumOfElements((*usart1Control_Modbus).seqMemTX) == 0 ) {
		//if no message are to be sent then flip the direction of the tranciever to receive new messages
		Set_DE_Mode(MODBUS_FOLLOWER_RX);
	  }*/
	  
      return_state_u8 = RUN_MODULE ;
      break;
    }
  case KILL_MODULE: 
    {
      // The USART1 driver module must only be executed once.
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


/**
********************************************************************************
* @brief   Assign structured memory
* @details Assign structured memory for digital outputs module
* @param   None 
* @return  None
********************************************************************************
*/
void AssignModuleMemModbus(void){   
  module_Control_StructMem_u32 =  StructMem_CreateInstance(MODULE_MODBUS, sizeof(Modbus_Control), ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);
  (*module_Control_StructMem_u32).p_ramBuf_u8 = (uint8_t *)&modbus_RTU_Control ;    // Map the ADC1 memory into the structured memory
  uint8_t module_modbus_index_u8 = getProcessInfoIndex(MODULE_MODBUS);
  processInfoTable[module_modbus_index_u8].Sched_ModuleData.p_masterSharedMem_u32 = (Ram_Buf_Handle)module_Control_StructMem_u32;
}

uint8_t modbus_message_counter_u8 = 0; // !errorTemporary

// Checks Message Length: Returns 0, if length accepted, Returns 1 if buffer overflow
uint8_t Modbus_ReallocateTxBufferForLength(uint8_t message_length) {
#if ENABLE_MODBUS_PROTOCOLBUF_TX_FIXED_LEN >= 1
  if (message_length > FIXED_MODBUS_PROTOCOLBUF_TX_MAX_LENGTH) {
    return 1; // Message would overflow buffer
  }
#else // if ENABLE_MODBUS_PROTOCOLBUF_TX_FIXED_LEN <= 0
  if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, message_length)) == NULL) {
    reallocError++;
    return 1;
  }
#endif // if ENABLE_MODBUS_PROTOCOLBUF_TX_FIXED_LEN <= 0
  return 0; // Message will fit in allocated buffer
}

uint8_t Modbus_ReallocateRxBufferForLength(uint8_t message_length) {
#if ENABLE_MODBUS_PROTOCOLBUF_RX_FIXED_LEN >= 1
  if (message_length > FIXED_MODBUS_PROTOCOLBUF_RX_MAX_LENGTH) {
    return 1; // Message would overflow buffer
  }
#else // if ENABLE_MODBUS_PROTOCOLBUF_RX_FIXED_LEN <= 0
  if((modbus_ProtocolBufRX = (uint8_t *) realloc(modbus_ProtocolBufRX, message_length)) == NULL) {
    reallocError++;
    return 1;
  }
#endif // if ENABLE_MODBUS_PROTOCOLBUF_RX_FIXED_LEN <= 0
  return 0; // Message will fit in allocated buffer
}


void Modbus_ParseReceivedMessages(void) {
  // TODO: Accept Messages of Various Lengths, will require updates to driver_usart1 as well.
  // TODO: Length Checking for individual Messages
  uint32_t DataLen2_u32		= MODBUS_MIN_MESSAGE_LEN;
  uint32_t responseLength_u32 = (uint32_t) MINIMUM_RESPONSE_LENGTH;
  uint8_t number_of_coil_bytes_u8 = 0;
  
  DataLen2_u32 = RingBuf_GetUsedNumOfElements((*usart1Control_Modbus).seqMem_ModbusRx);
  if(DataLen2_u32 >= MODBUS_MIN_MESSAGE_LEN) {
    // if((modbus_ProtocolBufRX = (unsigned char*) realloc(modbus_ProtocolBufRX,DataLen2_u32)) == NULL) reallocError++;     //allocate the right frame size of memory for buffer
    uint8_t error_occurred = Modbus_ReallocateRxBufferForLength(DataLen2_u32);
    if (error_occurred) {
      // Read All Data (Clear the Buffer),  so we don't get stuck with the Buffer full
      while (DataLen2_u32 > 0) {
        if (DataLen2_u32 > FIXED_MODBUS_PROTOCOLBUF_RX_MAX_LENGTH) {
          // REVIEW: Replace with RingBuf_ClearContents? Much less processing
          unsigned int read_length = FIXED_MODBUS_PROTOCOLBUF_RX_MAX_LENGTH;
          RingBuf_ReadBlock((*usart1Control_Modbus).seqMem_ModbusRx, modbus_ProtocolBufRX, &read_length); //extract the whole frame
          DataLen2_u32 -= FIXED_MODBUS_PROTOCOLBUF_RX_MAX_LENGTH;
        } else {
          unsigned int read_length = DataLen2_u32;
          RingBuf_ReadBlock((*usart1Control_Modbus).seqMem_ModbusRx, modbus_ProtocolBufRX, &read_length); //extract the whole frame
          DataLen2_u32 = 0;
        }
      }
      // Exit Gracefully
      return;
    }
    
    RingBuf_ReadBlock((*usart1Control_Modbus).seqMem_ModbusRx, modbus_ProtocolBufRX, &DataLen2_u32); //extract the whole frame into he temporary modbus_ProtocolBuf
    //decode and perform the CMD function
    uint8_t slave_address_u8 = modbus_ProtocolBufRX[BYTE_0];
    uint8_t function_code_u8 = modbus_ProtocolBufRX[BYTE_1]; // does not include address or checksum, does include this length byte
    uint8_t crc_lo_u8 = modbus_ProtocolBufRX[DataLen2_u32 - 2];
    uint8_t crc_hi_u8 = modbus_ProtocolBufRX[DataLen2_u32 - 1];
    uint16_t register_address_u16 		= modbus_ProtocolBufRX[BYTE_2]*BYTE_TO_WORD_MSB + modbus_ProtocolBufRX[BYTE_3]*BYTE_TO_WORD_LSB;
    uint16_t number_of_registers_u16	= modbus_ProtocolBufRX[BYTE_4]*BYTE_TO_WORD_MSB + modbus_ProtocolBufRX[BYTE_5]*BYTE_TO_WORD_LSB;		
    uint16_t crc_received_u16 = (uint16_t) ((crc_hi_u8*256) | (crc_lo_u8));
    
    //All characters in msg included in checksum
    //for (index_u8 = 0; index_u8 < (DataLen2_u32 - 2); index_u8++) {
    //	UpdateChecksum(modbus_ProtocolBufRX[index_u8], &localCHK);
    //}
    //FixChecksum(&localCHK);
    //uint8_t myBoolean;
    //myBoolean = isChecksumValid(modbus_ProtocolBufRX, DataLen2_u32 - 2, &localCHK);
    
    uint16_t crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufRX, (DataLen2_u32 - 2));
    //uint16_t alt_calculated_u16 = calculateMODBUSCRC(modbus_ProtocolBufRX, (DataLen2_u32 - 2));
    //uint16_t crc_calculated_u16 = Modbus_CalculateCrc((DataLen2_u32 - 2) , modbus_ProtocolBufRX);
    
    // MRM: Validate CRC
    if(crc_received_u16 != crc_calculated_u16) {
      // send exception for bad CRC
      responseLength_u32 = EXCEPTION_RESPONSE_LENGTH;
      uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
      if (error_occurred) {
        return;
      }
      // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;
      
      // build header
      modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
      modbus_ProtocolBufTX[BYTE_1] = function_code_u8 + EXCEPTION_FUNCTION_CODE_OFFSET;
      modbus_ProtocolBufTX[BYTE_2] = (uint8_t) MODBUS_EXCEPTION_04;
      crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufRX, (DataLen2_u32 - 2));			
      modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (crc_calculated_u16 >> 0);								// crc LSB
      modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (crc_calculated_u16 >> 8);								// crc MSB
           
      // write to seq Mem structure
      RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf								  
#if ENABLE_MODBUS_PROTOCOLBUF_TX_FIXED_LEN <= 0
      if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX,1)) == NULL) reallocError++;	
#endif //ENABLE_MODBUS_PROTOCOLBUF_TX_FIXED_LEN <= 0
      
      return;
    }
    
    // Validate modbus slave address
    if (slave_address_u8 != MODBUS_ADDRESS) {
      // MRM: return exception code for invalid slave address
      responseLength_u32 = EXCEPTION_RESPONSE_LENGTH;
      uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
      if (error_occurred) {
        return;
      }
      // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;
      
      // build exception header
      modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
      modbus_ProtocolBufTX[BYTE_1] = function_code_u8 + EXCEPTION_FUNCTION_CODE_OFFSET;
      modbus_ProtocolBufTX[BYTE_2] = (uint8_t) MODBUS_EXCEPTION_04;
      crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufRX, (DataLen2_u32 - 2));			
      modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (crc_calculated_u16 >> 0);								// crc LSB
      modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (crc_calculated_u16 >> 8);								// crc MSB
      
      // write to seq Mem structure
      RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf								  
#if ENABLE_MODBUS_PROTOCOLBUF_TX_FIXED_LEN <= 0
      if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX,1)) == NULL) reallocError++;	
#endif //ENABLE_MODBUS_PROTOCOLBUF_TX_FIXED_LEN <= 0		  		  
      return;
    }
    
    // MRM: Validate message length length
    //if (DataLen2_u32 < MODBUS_MIN_MESSAGE_LEN) {
    //  	// MRM: return exception code for invalid message lengths
    //    return;
    //}
    tt_ModbusLinkLostStop = getSysCount() + MODBUS_LOST_LINK_TIME; // Restart Modbus Link Lost Timer
    switch(function_code_u8) 
    {
    case MODBUS_FUNCTION_CODE_UTILITY: {
        if (DataLen2_u32 < MODUBS_FUNCTION_CODE_UTILITY_MIN_MESSAGE_LEN) {
          return;
        } 
        //        
        Utility_ExecuteOperation(modbus_ProtocolBufRX[BYTE_2], modbus_ProtocolBufRX[BYTE_3], modbus_ProtocolBufRX[BYTE_4]);
        // Modbus_JumpToBootloader();
    }
      //case MODBUS_FUNCTION_CODE_READ_COILS:												
    case MODBUS_FUNCTION_CODE_READ_COILS: {
      //if (ProcessMBCoilRegister(modbus_ProtocolBufTX+READ_RESPONSE_HEADER_LENGTH, register_address_u16, number_of_registers_u16, MB_READ_REGISTER) == MB_ILLEGAL_ADDRESS) {
      if (ProcessMBCoilRegister_New(modbus_ProtocolBufTX+READ_RESPONSE_HEADER_LENGTH, register_address_u16, number_of_registers_u16, MB_READ_REGISTER) == MB_ILLEGAL_ADDRESS) {
		// send exception code for bad address
        responseLength_u32 = EXCEPTION_RESPONSE_LENGTH;
        uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
        if (error_occurred) {
          return;
        }
        // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;
        // build header
        modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
        modbus_ProtocolBufTX[BYTE_1] = function_code_u8 + EXCEPTION_FUNCTION_CODE_OFFSET;
        modbus_ProtocolBufTX[BYTE_2] = (uint8_t) MODBUS_EXCEPTION_02;
        // build CRC
        crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
        modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (crc_calculated_u16 >> 0);								// crc LSB
        modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (crc_calculated_u16 >> 8);								// crc MSB
        // write to seq Mem structure
        RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf									
      }
      else {
        // build response message and send
        if (number_of_registers_u16 % 8 == 0)
          number_of_coil_bytes_u8 = number_of_registers_u16/8;
        else
          number_of_coil_bytes_u8 = number_of_registers_u16/8 + 1;
        responseLength_u32 = READ_RESPONSE_HEADER_LENGTH + number_of_coil_bytes_u8 + CRC_LENGTH;	// exactly one partial byte for coils			  
        uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
        if (error_occurred) {
          return;
        }
        // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;	
        // build header
        modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
        modbus_ProtocolBufTX[BYTE_1] = function_code_u8;
        modbus_ProtocolBufTX[BYTE_2] = (uint8_t) number_of_coil_bytes_u8;								// 1 byte per multiple of 8 coils
        //modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (register_address_u16 >> 8);								// starting register
        //modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (register_address_u16 >> 0);					
        // build CRC
        crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
        modbus_ProtocolBufTX[responseLength_u32 - 2] = (uint8_t) (crc_calculated_u16 >> 0);					// crc LSB
        modbus_ProtocolBufTX[responseLength_u32 - 1] = (uint8_t) (crc_calculated_u16 >> 8);					// crc MSB
        RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf	
        uint16_t alt_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
      }  				  
      break;
    }
    //case MODBUS_FUNCTION_CODE_READ_DISCRETE_INPUTS:
    case MODBUS_FUNCTION_CODE_READ_DISCRETE_INPUTS: {
      //if (ProcessMBCoilRegister(modbus_ProtocolBufTX+READ_RESPONSE_HEADER_LENGTH, register_address_u16, number_of_registers_u16, MB_READ_REGISTER) == MB_ILLEGAL_ADDRESS) {
      if (ProcessMBCoilRegister_New(modbus_ProtocolBufTX+READ_RESPONSE_HEADER_LENGTH, register_address_u16, number_of_registers_u16, MB_READ_REGISTER) == MB_ILLEGAL_ADDRESS) {
        // send exception code for bad address
        responseLength_u32 = EXCEPTION_RESPONSE_LENGTH;
        uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
        if (error_occurred) {
          return;
        }
        // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;
        // build header
        modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
        modbus_ProtocolBufTX[BYTE_1] = function_code_u8 + EXCEPTION_FUNCTION_CODE_OFFSET;
        modbus_ProtocolBufTX[BYTE_2] = (uint8_t) MODBUS_EXCEPTION_02;
        // build CRC
        crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
        modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (crc_calculated_u16 >> 0);								// crc LSB
        modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (crc_calculated_u16 >> 8);								// crc MSB
        // write to seq Mem structure
        RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf					
      }
      else {
        if (number_of_registers_u16 % 8 == 0)
          number_of_coil_bytes_u8 = number_of_registers_u16/8;
        else
          number_of_coil_bytes_u8 = number_of_registers_u16/8 + 1;
        // build response message and send
        responseLength_u32 = READ_RESPONSE_HEADER_LENGTH + number_of_coil_bytes_u8 + CRC_LENGTH;
        uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
        if (error_occurred) {
          return;
        }
        // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;	
        // build header
        modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
        modbus_ProtocolBufTX[BYTE_1] = function_code_u8;
        modbus_ProtocolBufTX[BYTE_2] = (uint8_t) (number_of_coil_bytes_u8);								// 2 bytes per registers
        //modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (register_address_u16 >> 8);								// starting register
        //modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (register_address_u16 >> 0);					
        // build CRC
        crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
        modbus_ProtocolBufTX[responseLength_u32 - 2] = (uint8_t) (crc_calculated_u16 >> 0);					// crc LSB
        modbus_ProtocolBufTX[responseLength_u32 - 1] = (uint8_t) (crc_calculated_u16 >> 8);					// crc MSB
        RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf	
        uint16_t alt_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
      }  
      break;
    }			  
    //case MODBUS_FUNCTION_CODE_READ_HOLDING_REGISTERS: {
    case MODBUS_FUNCTION_CODE_READ_HOLDING_REGISTERS: {
      if (ProcessMBHoldingRegister(modbus_ProtocolBufTX+READ_RESPONSE_HEADER_LENGTH, register_address_u16, number_of_registers_u16, MB_READ_REGISTER) == MB_ILLEGAL_ADDRESS) {
        // send exception code for bad address
        responseLength_u32 = EXCEPTION_RESPONSE_LENGTH;
        uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
        if (error_occurred) {
          return;
        }
        // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;
        // build header
        modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
        modbus_ProtocolBufTX[BYTE_1] = function_code_u8 + EXCEPTION_FUNCTION_CODE_OFFSET;
        modbus_ProtocolBufTX[BYTE_2] = (uint8_t) MODBUS_EXCEPTION_02;
        // build CRC
        crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
        modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (crc_calculated_u16 >> 0);								// crc LSB
        modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (crc_calculated_u16 >> 8);								// crc MSB
        // write to seq Mem structure
        RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf					
      }
      else {
        // build response message and send
        responseLength_u32 = READ_RESPONSE_HEADER_LENGTH + 2*number_of_registers_u16 + CRC_LENGTH;
        uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
        if (error_occurred) {
          return;
        }
        // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;	
        // build header
        modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
        modbus_ProtocolBufTX[BYTE_1] = function_code_u8;
        modbus_ProtocolBufTX[BYTE_2] = (uint8_t) (2*number_of_registers_u16);								// 2 bytes per registers
        //modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (register_address_u16 >> 8);								// starting register
        //modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (register_address_u16 >> 0);					
        // build CRC
        crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
        modbus_ProtocolBufTX[responseLength_u32 - 2] = (uint8_t) (crc_calculated_u16 >> 0);					// crc LSB
        modbus_ProtocolBufTX[responseLength_u32 - 1] = (uint8_t) (crc_calculated_u16 >> 8);					// crc MSB
        RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf	
        uint16_t alt_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
      }  
      break;
    }			  
    //case MODBUS_FUNCTION_CODE_READ_INPUT_REGISTERS:
    case MODBUS_FUNCTION_CODE_READ_INPUT_REGISTERS: {			
      if (ProcessMBHoldingRegister(modbus_ProtocolBufTX+READ_RESPONSE_HEADER_LENGTH, register_address_u16, number_of_registers_u16, MB_READ_REGISTER) == MB_ILLEGAL_ADDRESS) {
        // send exception code for bad address
        responseLength_u32 = EXCEPTION_RESPONSE_LENGTH;
        uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
        if (error_occurred) {
          return;
        }
        // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;
        // build header
        modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
        modbus_ProtocolBufTX[BYTE_1] = function_code_u8 + EXCEPTION_FUNCTION_CODE_OFFSET;
        modbus_ProtocolBufTX[BYTE_2] = (uint8_t) MODBUS_EXCEPTION_02;
        // build CRC
        crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
        modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (crc_calculated_u16 >> 0);								// crc LSB
        modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (crc_calculated_u16 >> 8);								// crc MSB
        // write to seq Mem structure
        RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf					
      }
      else {
        // build response message and send
        responseLength_u32 = READ_RESPONSE_HEADER_LENGTH + 2*number_of_registers_u16 + CRC_LENGTH;
        uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
        if (error_occurred) {
          return;
        }
        // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;	
        // build header
        modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
        modbus_ProtocolBufTX[BYTE_1] = function_code_u8;
        modbus_ProtocolBufTX[BYTE_2] = (uint8_t) (2*number_of_registers_u16);								// 2 bytes per registers
        //modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (register_address_u16 >> 8);								// starting register
        //modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (register_address_u16 >> 0);					
        // build CRC
        crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
        modbus_ProtocolBufTX[responseLength_u32 - 2] = (uint8_t) (crc_calculated_u16 >> 0);					// crc LSB
        modbus_ProtocolBufTX[responseLength_u32 - 1] = (uint8_t) (crc_calculated_u16 >> 8);					// crc MSB
        RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf	
        uint16_t alt_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
      }
      break;
    }
    //case MODBUS_FUNCTION_CODE_WRITE_SINGLE_HOLDING_REGISTER:
    case MODBUS_FUNCTION_CODE_WRITE_SINGLE_HOLDING_REGISTER: {
      if (ProcessMBHoldingRegister(modbus_ProtocolBufRX+WRITE_SINGLE_HEADER_LENGTH, register_address_u16, 1, MB_WRITE_REGISTER) == MB_ILLEGAL_ADDRESS) {
        // send exception code for ilegal address
        responseLength_u32 = EXCEPTION_RESPONSE_LENGTH;
        uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
        if (error_occurred) {
          return;
        }
        // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;
        // build header
        modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
        modbus_ProtocolBufTX[BYTE_1] = function_code_u8 + EXCEPTION_FUNCTION_CODE_OFFSET;
        modbus_ProtocolBufTX[BYTE_2] = (uint8_t) MODBUS_EXCEPTION_02;
        // build CRC
        crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
        modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (crc_calculated_u16 >> 0);					// crc LSB
        modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (crc_calculated_u16 >> 8);					// crc MSB
        // write to seq Mem structure
        RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf					
      }
      else {
        responseLength_u32 = WRITE_RESPONSE_HEADER_LENGTH + CRC_LENGTH;
        uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
        if (error_occurred) {
          return;
        }
        // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;	
        // build header
        modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
        modbus_ProtocolBufTX[BYTE_1] = function_code_u8;
        modbus_ProtocolBufTX[BYTE_2] = (uint8_t) (register_address_u16 >> 8);				// starting register MSB/LSB
        modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (register_address_u16 >> 0);					
        modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (modbus_ProtocolBufRX[WRITE_SINGLE_HEADER_LENGTH]);			// number of registers written MSB/LSB
        modbus_ProtocolBufTX[BYTE_5] = (uint8_t) (modbus_ProtocolBufRX[WRITE_SINGLE_HEADER_LENGTH+1]);				
        // build CRC
        crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
        modbus_ProtocolBufTX[BYTE_6] = (uint8_t) (crc_calculated_u16 >> 0);					// crc LSB
        modbus_ProtocolBufTX[BYTE_7] = (uint8_t) (crc_calculated_u16 >> 8);					// crc MSB
        RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf					
        
        UpdateMotorDemandMultiplexer(); // TODO: Don't call for every write (only when updates detected)
      }
      break;
    }			  
    //case MODBUS_FUNCTION_CODE_WRITE_MULTIPLE_COILS:
    case MODBUS_FUNCTION_CODE_WRITE_MULTIPLE_COILS:{
      if (number_of_registers_u16 % 8 == 0)
        number_of_coil_bytes_u8 = number_of_registers_u16/8;
      else
        number_of_coil_bytes_u8 = number_of_registers_u16/8 + 1;			  
//      if (ProcessMBCoilRegister(modbus_ProtocolBufRX+WRITE_HEADER_LENGTH, register_address_u16, number_of_registers_u16, MB_WRITE_REGISTER) == MB_ILLEGAL_ADDRESS) {
      if (ProcessMBCoilRegister_New(modbus_ProtocolBufRX+WRITE_HEADER_LENGTH, register_address_u16, number_of_registers_u16, MB_WRITE_REGISTER) == MB_ILLEGAL_ADDRESS) {
        // send exception code for ilegal address
        responseLength_u32 = EXCEPTION_RESPONSE_LENGTH;
        uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
        if (error_occurred) {
          return;
        }
        // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;
        // build header
        modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
        modbus_ProtocolBufTX[BYTE_1] = function_code_u8 + EXCEPTION_FUNCTION_CODE_OFFSET;
        modbus_ProtocolBufTX[BYTE_2] = (uint8_t) MODBUS_EXCEPTION_02;
        // build CRC
        crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
        modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (crc_calculated_u16 >> 0);					// crc LSB
        modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (crc_calculated_u16 >> 8);					// crc MSB
        // write to seq Mem structure
        RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf					
      }
      else {
        responseLength_u32 = WRITE_RESPONSE_HEADER_LENGTH + CRC_LENGTH;
        uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
        if (error_occurred) {
          return;
        }
        // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;	
        // build heade
        modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
        modbus_ProtocolBufTX[BYTE_1] = function_code_u8;
        modbus_ProtocolBufTX[BYTE_2] = (uint8_t) (register_address_u16 >> 8);				// starting register MSB/LSB
        modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (register_address_u16 >> 0);					
        modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (number_of_registers_u16 >> 8);			// number of registers written MSB/LSB
        modbus_ProtocolBufTX[BYTE_5] = (uint8_t) (number_of_registers_u16 >> 0);				
        // build CRC
        crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
        modbus_ProtocolBufTX[BYTE_6] = (uint8_t) (crc_calculated_u16 >> 0);					// crc LSB
        modbus_ProtocolBufTX[BYTE_7] = (uint8_t) (crc_calculated_u16 >> 8);					// crc MSB
        RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf					
        
        UpdateMotorDemandMultiplexer(); // TODO: Don't call for every write (only when updates detected)
      }				
      break;
    }
    //case MODBUS_FUNCTION_CODE_WRITE_HOLDING_REGISTERS: {
    case MODBUS_FUNCTION_CODE_WRITE_HOLDING_REGISTERS: {
      if (ProcessMBHoldingRegister(modbus_ProtocolBufRX+WRITE_HEADER_LENGTH, register_address_u16, number_of_registers_u16, MB_WRITE_REGISTER) == MB_ILLEGAL_ADDRESS) {
        // send exception code for ilegal address
        responseLength_u32 = EXCEPTION_RESPONSE_LENGTH;
        uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
        if (error_occurred) {
          return;
        }
        // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;
        // build header
        modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
        modbus_ProtocolBufTX[BYTE_1] = function_code_u8 + EXCEPTION_FUNCTION_CODE_OFFSET;
        modbus_ProtocolBufTX[BYTE_2] = (uint8_t) MODBUS_EXCEPTION_02;
        // build CRC
        crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
        modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (crc_calculated_u16 >> 0);					// crc LSB
        modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (crc_calculated_u16 >> 8);					// crc MSB
        // write to seq Mem structure
        RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf					
      }
      else {
        responseLength_u32 = WRITE_RESPONSE_HEADER_LENGTH + CRC_LENGTH;
        uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
        if (error_occurred) {
          return;
        }
        // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;	
        // build header
        modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
        modbus_ProtocolBufTX[BYTE_1] = function_code_u8;
        modbus_ProtocolBufTX[BYTE_2] = (uint8_t) (register_address_u16 >> 8);				// starting register MSB/LSB
        modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (register_address_u16 >> 0);					
        modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (number_of_registers_u16 >> 8);			// number of registers written MSB/LSB
        modbus_ProtocolBufTX[BYTE_5] = (uint8_t) (number_of_registers_u16 >> 0);				
        // build CRC
        crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
        modbus_ProtocolBufTX[BYTE_6] = (uint8_t) (crc_calculated_u16 >> 0);					// crc LSB
        modbus_ProtocolBufTX[BYTE_7] = (uint8_t) (crc_calculated_u16 >> 8);					// crc MSB
        RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf					
        
        UpdateMotorDemandMultiplexer(); // TODO: Don't call for every write (only when updates detected)
      }
      break;
    }			  
	case MODBUS_FUNCTION_CODE_READ_RECORDS: {
      uint16_t file_address_u16 			= modbus_ProtocolBufRX[BYTE_6]*BYTE_TO_WORD_MSB + modbus_ProtocolBufRX[BYTE_7]*BYTE_TO_WORD_LSB;
      uint16_t number_of_file_words_u16		= modbus_ProtocolBufRX[BYTE_8]*BYTE_TO_WORD_MSB + modbus_ProtocolBufRX[BYTE_9]*BYTE_TO_WORD_LSB;		
	  uint8_t  file_number_u8				= modbus_ProtocolBufRX[BYTE_5];
	  
	  // read requet to EEPROM on the drive side
	  if((file_number_u8 == DATA_LOGGER_FILE_NUMBER) || (file_number_u8 == COMPONENT_DATA_FILE_NUMBER)) {
		last_eeprom_request.device_address_u8 = slave_address_u8;
		last_eeprom_request.function_code_u8 = MODBUS_FUNCTION_CODE_READ_RECORDS;
		last_eeprom_request.length_u8 = number_of_file_words_u16;
		// test for exceptions
		switch(file_number_u8) {
		  case DATA_LOGGER_FILE_NUMBER: {
			if((file_address_u16 < DATA_LOGGER_ADDRESS_MIN) || (file_address_u16 > DATA_LOGGER_ADDRESS_MAX)) {
			  last_eeprom_request.is_exception_u8 = TRUE;
			  last_eeprom_request.exception_type_u8 = MODBUS_EXCEPTION_03; // illegal data value
			}
			else
			  last_eeprom_request.is_exception_u8 = FALSE;
			break;
		  }  
		  case COMPONENT_DATA_FILE_NUMBER: {
			if((file_address_u16 < COMPONENT_DATA_ADDRESS_MIN) || (file_address_u16 > COMPONENT_DATA_ADDRESS_MAX)) {
			  last_eeprom_request.is_exception_u8 = TRUE;
			  last_eeprom_request.exception_type_u8 = MODBUS_EXCEPTION_03; // illegal data value
			}
			else
			  last_eeprom_request.is_exception_u8 = FALSE;
			break;
		  }
		}
		if(last_eeprom_request.is_exception_u8 == FALSE) {
		  Build_UPEEPROM_Request(file_address_u16, (uint8_t) (2*number_of_file_words_u16));
		}
		else
		  Modbus_PassEEPROMData((uint8_t*) (NULL), 0);
		return;
	  }

	  // read requet to EEPROM on the drive side
	  if(file_number_u8 == DRIVE_CONFIGURATION_FILE_NUMBER) {
		last_drive_flash_request.device_address_u8 = slave_address_u8;
		last_drive_flash_request.function_code_u8 = MODBUS_FUNCTION_CODE_READ_RECORDS;
		last_drive_flash_request.length_u8 = number_of_file_words_u16;
		// test for exceptions
		if((file_address_u16 < DRIVE_CONFIGURATION_ADDRESS_MIN) || (file_address_u16 > DRIVE_CONFIGURATION_ADDRESS_MAX)) {
		  last_drive_flash_request.is_exception_u8 = TRUE;
		  last_drive_flash_request.exception_type_u8 = MODBUS_EXCEPTION_03; // illegal data value
		}
		else
		  last_drive_flash_request.is_exception_u8 = FALSE;

		if(last_drive_flash_request.is_exception_u8 == FALSE) {
			Build_UPDrive_Flash_Request(file_address_u16, (uint8_t) (2*number_of_file_words_u16));
		}
		else // generate exception response immediately
		  Modbus_PassDriveFlashData((uint8_t*) (NULL), 0);
		return;
	  }
	  	  
	  if (ProcessMBReadRecords(modbus_ProtocolBufRX+READ_RECORDS_HEADER_LENGTH, file_address_u16, number_of_file_words_u16, file_number_u8) == MB_ILLEGAL_ADDRESS) {
		// send exception code for ilegal address
		  responseLength_u32 = EXCEPTION_RESPONSE_LENGTH;
		  uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
		  if (error_occurred) {
			return;
		  }
		  // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;
		  // build header
		  modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
		  modbus_ProtocolBufTX[BYTE_1] = function_code_u8 + EXCEPTION_FUNCTION_CODE_OFFSET;
		  modbus_ProtocolBufTX[BYTE_2] = (uint8_t) MODBUS_EXCEPTION_02;
		  // build CRC
		  crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
		  modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (crc_calculated_u16 >> 0);					// crc LSB
		  modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (crc_calculated_u16 >> 8);					// crc MSB
		  // write to seq Mem structure
		  RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf					
		}
		else {
		  responseLength_u32 = READ_RECORDS_RESPONSE_HEADER_LENGTH + (2*number_of_registers_u16)+ CRC_LENGTH;
		  uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
		  if (error_occurred) {
			return;
		  }
		  // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;	
		  // build header
		  modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
		  modbus_ProtocolBufTX[BYTE_1] = function_code_u8;
		  modbus_ProtocolBufTX[BYTE_2] = (2*number_of_file_words_u16) + 3;	// total byte count for message in bytes
		  modbus_ProtocolBufTX[BYTE_3] = (2*number_of_file_words_u16) + 1;	// total byte count for data, counts hte number of bytes read/written to flash
		  modbus_ProtocolBufTX[BYTE_4] = MODBUS_REFERENCE_TYPE;	// Always 0x06
		  
		  // build CRC
		  crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
		  modbus_ProtocolBufTX[responseLength_u32 - 2] = (uint8_t) (crc_calculated_u16 >> 0);					// crc LSB
		  modbus_ProtocolBufTX[responseLength_u32 - 1] = (uint8_t) (crc_calculated_u16 >> 8);					// crc MSB
		  RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf	
		  uint16_t alt_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2); // debugging
		  		  
		  UpdateMotorDemandMultiplexer(); // TODO: Don't call for every write (only when updates detected)
        } 
	  break;
	}
	case MODBUS_FUNCTION_CODE_WRITE_RECORDS: { 
      uint16_t file_address_u16 			= modbus_ProtocolBufRX[BYTE_6]*BYTE_TO_WORD_MSB + modbus_ProtocolBufRX[BYTE_7]*BYTE_TO_WORD_LSB;
      uint16_t number_of_file_words_u16		= modbus_ProtocolBufRX[BYTE_8]*BYTE_TO_WORD_MSB + modbus_ProtocolBufRX[BYTE_9]*BYTE_TO_WORD_LSB;		
	  uint8_t  file_number_u8				= modbus_ProtocolBufRX[BYTE_5];
	  
	  // read requet to EEPROM on the drive side
	  if((file_number_u8 == DATA_LOGGER_FILE_NUMBER) || (file_number_u8 == COMPONENT_DATA_FILE_NUMBER)) {
		last_eeprom_request.device_address_u8 = slave_address_u8;
		last_eeprom_request.function_code_u8 = MODBUS_FUNCTION_CODE_WRITE_RECORDS;
		last_eeprom_request.length_u8 = number_of_file_words_u16;
		// test for exceptions
		switch(file_number_u8) {
		  case DATA_LOGGER_FILE_NUMBER: {
			if((file_address_u16 < DATA_LOGGER_ADDRESS_MIN) || (file_address_u16 > DATA_LOGGER_ADDRESS_MAX)) {
			  last_eeprom_request.is_exception_u8 = TRUE;
			  last_eeprom_request.exception_type_u8 = MODBUS_EXCEPTION_03; // illegal data value
			}
			else
			  last_eeprom_request.is_exception_u8 = FALSE;
			break;
		  }  
		  case COMPONENT_DATA_FILE_NUMBER: {
			if((file_address_u16 < COMPONENT_DATA_ADDRESS_MIN) || (file_address_u16 > COMPONENT_DATA_ADDRESS_MAX)) {
			  last_eeprom_request.is_exception_u8 = TRUE;
			  last_eeprom_request.exception_type_u8 = MODBUS_EXCEPTION_03; // illegal data value
			}
			else
			  last_eeprom_request.is_exception_u8 = FALSE;
			break;
		  }
		}
		if(last_eeprom_request.is_exception_u8 == FALSE) {
		  Build_UPEEPROM_Request(file_address_u16, (uint8_t) (2*number_of_file_words_u16));
		}
		else
		  Modbus_PassEEPROMData((uint8_t*) (NULL), 0);
		return;
	  }

	  // read requet to EEPROM on the drive side
	  if(file_number_u8 == DRIVE_CONFIGURATION_FILE_NUMBER) {
		last_drive_flash_request.device_address_u8 = slave_address_u8;
		last_drive_flash_request.function_code_u8 = MODBUS_FUNCTION_CODE_READ_RECORDS;
		last_drive_flash_request.length_u8 = number_of_file_words_u16;
		// test for exceptions
		if((file_address_u16 < DRIVE_CONFIGURATION_ADDRESS_MIN) || (file_address_u16 > DRIVE_CONFIGURATION_ADDRESS_MAX)) {
		  last_drive_flash_request.is_exception_u8 = TRUE;
		  last_drive_flash_request.exception_type_u8 = MODBUS_EXCEPTION_03; // illegal data value
		}
		else
		  last_drive_flash_request.is_exception_u8 = FALSE;

		if(last_drive_flash_request.is_exception_u8 == FALSE) {
			Build_UPDrive_Flash_Request(file_address_u16, (uint8_t) (2*number_of_file_words_u16));
		}
		else // generate exception response immediately
		  Modbus_PassDriveFlashData((uint8_t*) (NULL), 0);
		return;
	  }	  
	  
	  if (ProcessMBWriteRecords(modbus_ProtocolBufRX+WRITE_RECORDS_HEADER_LENGTH, file_address_u16, number_of_file_words_u16, file_number_u8) == MB_ILLEGAL_ADDRESS) {
		  // send exception code for ilegal address
		  responseLength_u32 = EXCEPTION_RESPONSE_LENGTH;
		  uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
		  if (error_occurred) {
			return;
		  }
		  // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;
		  // build header
		  modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
		  modbus_ProtocolBufTX[BYTE_1] = function_code_u8 + EXCEPTION_FUNCTION_CODE_OFFSET;
		  modbus_ProtocolBufTX[BYTE_2] = (uint8_t) MODBUS_EXCEPTION_02;
		  // build CRC
		  crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
		  modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (crc_calculated_u16 >> 0);					// crc LSB
		  modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (crc_calculated_u16 >> 8);					// crc MSB
		  // write to seq Mem structure
		  RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf					
		}
		else {
		  responseLength_u32 = WRITE_RECORDS_RESPONSE_HEADER_LENGTH + CRC_LENGTH;
		  uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
		  if (error_occurred) {
			return;
		  }
		  // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;	
		  // build header
		  modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
		  modbus_ProtocolBufTX[BYTE_1] = function_code_u8;
		  modbus_ProtocolBufTX[BYTE_2] = (uint8_t) (register_address_u16 >> 8);				// starting register MSB/LSB
		  modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (register_address_u16 >> 0);					
		  modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (number_of_registers_u16 >> 8);			// number of registers written MSB/LSB
		  modbus_ProtocolBufTX[BYTE_5] = (uint8_t) (number_of_registers_u16 >> 0);				
		  // build CRC
		  crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
		  modbus_ProtocolBufTX[BYTE_6] = (uint8_t) (crc_calculated_u16 >> 0);					// crc LSB
		  modbus_ProtocolBufTX[BYTE_7] = (uint8_t) (crc_calculated_u16 >> 8);					// crc MSB
		  RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf					
		  
		  UpdateMotorDemandMultiplexer(); // TODO: Don't call for every write (only when updates detected)
		}		  
	  break;
	}
    default: 
      {
        // function code is not supported, so send an exception
        responseLength_u32 = EXCEPTION_RESPONSE_LENGTH;
        uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
        if (error_occurred) {
          return;
        }
        // if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;
        // build header
        modbus_ProtocolBufTX[BYTE_0] = slave_address_u8;
        modbus_ProtocolBufTX[BYTE_1] = function_code_u8 + EXCEPTION_FUNCTION_CODE_OFFSET;
        modbus_ProtocolBufTX[BYTE_2] = (uint8_t) MODBUS_EXCEPTION_01;									// illegal function code
        // build CRC
        crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
        modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (crc_calculated_u16 >> 0);								// crc LSB
        modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (crc_calculated_u16 >> 8);								// crc MSB
        // write to seq Mem structure
        RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf								  
        break;
      }
    }
  }
  // Minimize Size of Dynamically Allocated Buffers
#if ENABLE_MODBUS_PROTOCOLBUF_RX_FIXED_LEN <= 0
  if((modbus_ProtocolBufRX = (uint8_t *) realloc(modbus_ProtocolBufRX,1)) == NULL) reallocError++;
#endif // ENABLE_MODBUS_PROTOCOLBUF_RX_FIXED_LEN <= 0
#if ENABLE_MODBUS_PROTOCOLBUF_TX_FIXED_LEN <= 0
  if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX,1)) == NULL) reallocError++;	
#endif //ENABLE_MODBUS_PROTOCOLBUF_TX_FIXED_LEN <= 0
}

MBErrorCode ProcessMBHoldingRegister(uint8_t * data_buffer_pu8, uint16_t starting_address_u16, uint16_t number_of_registers_u16, MBRegisterMode eMode) {
  MBErrorCode    eStatus = MB_ILLEGAL_ADDRESS;
  ModbusMapBlock *block = find_modbus_block(starting_address_u16, number_of_registers_u16);
  
  if (block) {
    //block_1 =(uint16_t) ( block->start_address_u16);
    //offset_1 = starting_address_u16;
    uint16_t index_u16 = starting_address_u16 - block->start_address_u16;
    
    while (number_of_registers_u16 > 0 ) {
      if (eMode == MB_WRITE_REGISTER) {
        block->start_of_data_pu16[index_u16] = ((uint16_t)*data_buffer_pu8++) << 8;
        block->start_of_data_pu16[index_u16] |= (uint16_t)*data_buffer_pu8++;
      }
      else {
        *data_buffer_pu8++ = (uint8_t)(block->start_of_data_pu16[index_u16] >> 8);
        *data_buffer_pu8++ = (uint8_t)(block->start_of_data_pu16[index_u16] & 0xff);
      }
      ++index_u16;
      --number_of_registers_u16;
    }
    eStatus = MB_NO_ERROR;
  }
  return eStatus;
}

MBErrorCode ProcessMBCoilRegister_New(uint8_t * data_buffer_pu8, uint16_t desired_coil_address_u16, uint16_t number_of_coils_u16, MBRegisterMode eMode) {
  MBErrorCode    eStatus = MB_ILLEGAL_ADDRESS;
  ModbusCoilMapBlock *block = find_modbus_coil_block(desired_coil_address_u16, number_of_coils_u16);
  uint8_t number_of_bytes_in_block_u8 = 0;		// number of bytes to represent all coils in this block
  // TODO: Change the below into a pointer without fixed length MRM
  uint8_t modbus_coil_data[6];					// a temporary array of all of the coils currently in this modbus block; will be used to access proper coils
  uint8_t index_u8;
  uint8_t mask_u8 = 0xff;
  
  if (block) {	
	// Get all existing coil data from block
	for(index_u8=0; index_u8<(block->number_of_coil_registers_u8); index_u8++)
	{
		modbus_coil_data[index_u8*2] = (uint8_t)(block->start_of_coil_data_pu16[index_u8] & 0x00ff);	  // low byte of the first register
		modbus_coil_data[index_u8*2+1] = (uint8_t)(block->start_of_coil_data_pu16[index_u8] >> 8);  	  // high byte of the first register
		number_of_bytes_in_block_u8 += 2;												  // count bytes
	}
	
	// where to start indexing coils
	uint16_t coil_index_u16 = desired_coil_address_u16 - block->start_coil_address_u16;
	  
	uint16_t number_of_offset_bytes_u16 = (coil_index_u16 / 8);			// number of full bytes to shift
	uint16_t number_of_offset_bits_u16 = (coil_index_u16 % 8);			// number of full bits to shift
	number_of_bytes_in_block_u8 -= number_of_offset_bytes_u16;

	if (eMode == MB_WRITE_REGISTER) {
	  uint64_t current_coil_data_u64 = 0;
	  uint64_t new_coil_data_u64 = 0;
	  uint64_t low_mask_u64 = 0x0000FFFFFFFFFFFF;
	  uint64_t mask_u64 = 0xFFFFFFFFFFFFFFFF;
	  mask_u64 <<= number_of_coils_u16 + coil_index_u16;
	  low_mask_u64 = low_mask_u64 >> (48 - coil_index_u16);
	  mask_u64 = mask_u64 | low_mask_u64;
		
	  uint16_t	modbus_register_u16 = 0;
	  for(index_u8=0; index_u8<3; index_u8++)
	  {
	      modbus_register_u16 = block->start_of_coil_data_pu16[index_u8];
		  current_coil_data_u64 = current_coil_data_u64 | (((uint64_t) (modbus_register_u16)) << (16*index_u8));	// create new wrte block of 0's = full length of coil block
	  }

	  uint8_t number_of_bytes_to_write_u8 = 0;
	  if(((number_of_coils_u16 + number_of_offset_bits_u16) % 8) > 0)
	  	number_of_bytes_to_write_u8 = (number_of_coils_u16 + number_of_offset_bits_u16)/8+1;	// add extra byte for spillover bits
	  else
		number_of_bytes_to_write_u8 = (number_of_coils_u16 + number_of_offset_bits_u16)/8;		// no bits spillover into next byte
	  if(number_of_bytes_to_write_u8 == 0)
		number_of_bytes_to_write_u8 = 1; // must have a minimum of one coil byte to write
	  
	  index_u8=number_of_offset_bytes_u16;
	  do {
		  new_coil_data_u64 = new_coil_data_u64 | (((uint64_t) (*data_buffer_pu8++)) << ((8*index_u8++) + number_of_offset_bits_u16)); 
	  } while(index_u8 < (number_of_offset_bytes_u16 + number_of_bytes_to_write_u8));
	  
	  current_coil_data_u64 = ((current_coil_data_u64 & mask_u64) | (new_coil_data_u64 & ~mask_u64));
	  
	  for(index_u8=0; index_u8<(block->number_of_coil_registers_u8); index_u8++)
		block->start_of_coil_data_pu16[index_u8] = (uint16_t) (((current_coil_data_u64)>>(16*index_u8)) & (0xFFFF));
	  
	  eStatus = MB_NO_ERROR;
    }
    else {	  // performing a read
	  // byte shift
	  for(index_u8=0; index_u8<((block->number_of_coil_registers_u8*2) - number_of_offset_bytes_u16); index_u8++){
		  modbus_coil_data[index_u8] = modbus_coil_data[index_u8+number_of_offset_bytes_u16];	  // shift data dwn by number_of_offset_bytes
	  }
	  
	  // bit shift8
	  mask_u8 = 0xFF;
	  mask_u8 >>= (8-number_of_offset_bits_u16);
	  for(index_u8=0; index_u8<(number_of_bytes_in_block_u8); index_u8++)
	  {
		  modbus_coil_data[index_u8]  = ((modbus_coil_data[index_u8])   >> number_of_offset_bits_u16);	  // shift data dwn by number_of_offset_bytes
		  if(index_u8 == number_of_bytes_in_block_u8-1)
		  	modbus_coil_data[index_u8] |= (((0xFF) & mask_u8) << (8-number_of_offset_bits_u16));	  				// shift data dwn by number_of_offset_bits		
		  else
		  	modbus_coil_data[index_u8] |= (((modbus_coil_data[index_u8+1]) & mask_u8) << (8-number_of_offset_bits_u16));	  				// shift data dwn by number_of_offset_bits		
	  }
	  // pad last byte with proper number of 0's
	  mask_u8 = 0xFF;
	  uint8_t number_of_coils_in_result_u8 = number_of_coils_u16 / 8;
	  mask_u8 >>= (8-(number_of_coils_u16 % 8));
	  uint8_t old_value_u8 = modbus_coil_data[number_of_coils_in_result_u8];
	  modbus_coil_data[number_of_coils_in_result_u8] = (old_value_u8 & mask_u8);

	  for(index_u8=0; index_u8<(number_of_bytes_in_block_u8); index_u8++){
		  *data_buffer_pu8++ = modbus_coil_data[index_u8];	  // shift data dwn by number_of_offset_bytes
	  }	  
    }
    eStatus = MB_NO_ERROR;
  }
  return eStatus;
}

MBErrorCode ProcessMBDiscreteRegister(uint8_t * data_buffer_pu8, uint16_t starting_address_u16, uint16_t number_of_registers_u16, MBRegisterMode eMode) {
  MBErrorCode    eStatus = MB_ILLEGAL_ADDRESS;
  ModbusMapBlock *block = find_modbus_block(starting_address_u16, number_of_registers_u16);
  
  if (block) {
    //block_1 =(uint16_t) ( block->start_address_u16);
    //offset_1 = starting_address_u16;
    uint16_t index_u16 = starting_address_u16 - block->start_address_u16;
    
    while (number_of_registers_u16 > 0 ) {
      if (eMode == MB_WRITE_REGISTER) {
        block->start_of_data_pu16[index_u16] = ((uint16_t)*data_buffer_pu8++) << 8;
        block->start_of_data_pu16[index_u16] |= (uint16_t)*data_buffer_pu8++;
      }
      else {
        *data_buffer_pu8++ = (uint8_t)(block->start_of_data_pu16[index_u16] >> 8);
        *data_buffer_pu8++ = (uint8_t)(block->start_of_data_pu16[index_u16] & 0xff);
      }
      ++index_u16;
      --number_of_registers_u16;
    }
    eStatus = MB_NO_ERROR;
  }
  return eStatus;
}

MBErrorCode ProcessMBWriteRecords(uint8_t * data_buffer_pu8, uint16_t flash_offset_address_u16, uint16_t number_of_flash_words_u16, uint8_t file_number_u8) {
  MBErrorCode    eStatus = MB_ILLEGAL_ADDRESS;
  uint8_t	*to_page_address_pu8;	// data page to perform the reading from
  uint8_t	is_write_successful_u8;
  //uint16_t	flash_page_offset_u16;	// offset to base page address to read the 8-byte block
  //uint8_t 
  //set up flash page address and ffset address based on info in mobus message
  if(file_number_u8 == APP_FLASH_SETTINGS_PAGE) 
	to_page_address_pu8 = (uint8_t*) (FLASH_USER_START_ADDR); 
  else if(file_number_u8 == APP_FLASH_SETTINGS_MIRROR_PAGE) 
	to_page_address_pu8 = (uint8_t*) (MIRROR_FLASH_BLOCK_SETTING_PAGE); 
  else
	return eStatus;	// illegal page selected
  
  if(flash_offset_address_u16 < PAGE_SIZE) {
	if ((number_of_flash_words_u16 % 8) != 0)
	  return eStatus;	// illegal block of data - not a multiple of 64 bits
	else {
		is_write_successful_u8 = flashBlockProgram(0, (uint32_t)to_page_address_pu8, data_buffer_pu8, (2*number_of_flash_words_u16));    //MRM - not sure if this is the correct function to call
		if((is_write_successful_u8))
		  eStatus = MB_NO_ERROR;	  		// data is good; consider alternate exception response
	}
  }
  else
	return eStatus; // illegal offset selected
  return eStatus;
}

MBErrorCode ProcessMBReadRecords(uint8_t * data_buffer_pu8, uint16_t offset_address_u16, uint16_t number_of_registers_u16, uint8_t file_number_u8) {
  MBErrorCode    eStatus = MB_ILLEGAL_ADDRESS;
  uint64_t	flash_data_u64;				// data block to be read from flash
  uint64_t	byte_mask_u64;				// used to help parse bytes from 64-bit data
  uint8_t	*from_page_address_pu8;		// data page to perform the reading from
  uint16_t	block_index_u16 = 0;		// used to index through the data in the modbus packet
  uint8_t	byte_index_u8 = 0;			// for parsing 64-bit data into 8 bytes in buffer 

  //set up flash page address and ffset address based on info in mobus message
  if(file_number_u8 == APP_FLASH_SETTINGS_PAGE) 
	from_page_address_pu8 = (uint8_t*) (FLASH_USER_START_ADDR); 
  else if(file_number_u8 == APP_FLASH_SETTINGS_MIRROR_PAGE) 
	from_page_address_pu8 = (uint8_t*) (MIRROR_FLASH_BLOCK_SETTING_PAGE); 
  else
	return eStatus;	// illegal page selected
  
  if(offset_address_u16 < PAGE_SIZE) {
	if ((number_of_registers_u16 % 8) != 0)
	  return eStatus;	// illegal block of data - not a multiple of 64 bits
	else {
	  for(block_index_u16 = 0; block_index_u16 < (2*number_of_registers_u16); block_index_u16+=8) {
		//take data_buffer values - 8 bytes and stuff them into uint64_t variable
		flash_data_u64 = FlashRead64Bits((uint8_t*) from_page_address_pu8, offset_address_u16 + block_index_u16); //no updated data then just read the old data
		byte_mask_u64 = (uint64_t) (0xFF00000000000000);		// start with highest byte
		for(byte_index_u8=0; byte_index_u8 < 8; byte_index_u8++) {
		  *data_buffer_pu8++ = (uint8_t) ( ((flash_data_u64 & byte_mask_u64) >> ((7-byte_index_u8) & (0xFF)) ) );
		  byte_mask_u64 >>= 8;
		}
	  }
	  eStatus = MB_NO_ERROR;	  // data is good
	}
  }
  else
	return eStatus; // illegal offset selected

  return eStatus;	  
}

MBErrorCode ProcessMBReadEEPROMRecords(uint8_t * data_buffer_pu8, uint16_t offset_address_u16, uint16_t number_of_registers_u16, uint8_t file_number_u8) {
  MBErrorCode    eStatus = MB_ILLEGAL_ADDRESS;
  uint64_t	flash_data_u64;				// data block to be read from flash
  uint64_t	byte_mask_u64;				// used to help parse bytes from 64-bit data
  uint8_t	*from_page_address_pu8;		// data page to perform the reading from
  uint16_t	block_index_u16 = 0;		// used to index through the data in the modbus packet
  uint8_t	byte_index_u8 = 0;			// for parsing 64-bit data into 8 bytes in buffer 

  //set up flash page address and ffset address based on info in mobus message
  
  // check if EEPROM file number and addresses are correct for a read from eeprom
  
  if(file_number_u8 == APP_FLASH_SETTINGS_PAGE) 
	from_page_address_pu8 = (uint8_t*) (FLASH_USER_START_ADDR); 
  else if(file_number_u8 == APP_FLASH_SETTINGS_MIRROR_PAGE) 
	from_page_address_pu8 = (uint8_t*) (MIRROR_FLASH_BLOCK_SETTING_PAGE); 
  else
	return eStatus;	// illegal page selected
  
  if(offset_address_u16 < PAGE_SIZE) {
	if ((number_of_registers_u16 % 8) != 0)
	  return eStatus;	// illegal block of data - not a multiple of 64 bits
	else {
	  for(block_index_u16 = 0; block_index_u16 < (2*number_of_registers_u16); block_index_u16+=8) {
		//take data_buffer values - 8 bytes and stuff them into uint64_t variable
		flash_data_u64 = FlashRead64Bits((uint8_t*) from_page_address_pu8, offset_address_u16 + block_index_u16); //no updated data then just read the old data
		byte_mask_u64 = (uint64_t) (0xFF00000000000000);		// start with highest byte
		for(byte_index_u8=0; byte_index_u8 < 8; byte_index_u8++) {
		  *data_buffer_pu8++ = (uint8_t) ( ((flash_data_u64 & byte_mask_u64) >> ((7-byte_index_u8) & (0xFF)) ) );
		  byte_mask_u64 >>= 8;
		}
	  }
	  eStatus = MB_NO_ERROR;	  // data is good
	}
  }
  else
	return eStatus; // illegal offset selected

  return eStatus;	  
}


//
// Given a modbus start address and number of registers,
// return the mapping block that contains those values, or 0 (null) if no match 
//
ModbusMapBlock* find_modbus_block(uint16_t desired_address_u16, uint16_t number_of_registers_u16) {
  uint16_t block_index_u16;
  for (block_index_u16 = 0; block_index_u16 < number_of_modbus_blocks_u16; ++block_index_u16)
  {
    ModbusMapBlock *block = masterBlocks[block_index_u16];
    if ((block->start_address_u16 <= desired_address_u16) && (desired_address_u16 + number_of_registers_u16 <= block->start_address_u16 + block->number_of_registers_u8))
      return block;
  }
  return 0;
}

//
// Given a modbus start coil address and number of coils,
// return the mapping block that contains those values, or 0 (null) if no match 
//
ModbusCoilMapBlock* find_modbus_coil_block(uint16_t desired_coil_address_u16, uint16_t number_of_coils_u16) {
  uint16_t block_index_u16;
  for (block_index_u16 = 0; block_index_u16 < number_of_modbus_coil_blocks_u16; ++block_index_u16)
  {
    ModbusCoilMapBlock *block = masterCoilBlocks[block_index_u16];
    if ((block->start_coil_address_u16 <= desired_coil_address_u16) && (desired_coil_address_u16 + number_of_coils_u16 <= block->start_coil_address_u16 + block->number_of_coils_u8))
      return block;
  }
  return 0;
}

// Compute the MODBUS RTU CRC
uint16_t calculateModbusCRC(uint8_t *buf_pu8, uint16_t length_u16)
{
  uint16_t crc_u16 = 0xFFFF;
  
  for (uint16_t pos_u16 = 0; pos_u16 < length_u16; pos_u16++) {
    crc_u16 ^= (uint16_t)buf_pu8[pos_u16];          // XOR byte into least sig. byte of crc
    
    for (uint8_t index_u8 = 8; index_u8 != 0; index_u8--) {    // Loop over each bit
      if ((crc_u16 & 0x0001) != 0) {      // If the LSB is set
        crc_u16 >>= 1;                    // Shift right and XOR 0xA001
        crc_u16 ^= 0xA001;
      }
      else                            // Else LSB is not set
        crc_u16 >>= 1;                    // Just shift right
    }
  }
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  return crc_u16;  
}

// TODO: Move Register Storage to a Universal location to be used by universal protocol, modbus, and bluetooth modules
//

void Modbus_UpdateStoredStatus(uint8_t status){
  drive_dynamic_control.drive_dynamic_data.state_u16 = status;
}
void Modbus_UpdateStoredFaults(uint16_t faults){
  drive_dynamic_control.drive_dynamic_data.faults_u16 = faults;  
}
void Modbus_UpdateStoredBusVoltage(uint16_t bus_voltage){
  drive_dynamic_control.drive_dynamic_data.bus_voltage_u16 = bus_voltage;
}
//
void Modbus_UpdateStoredDirection(uint8_t direction){
  drive_dynamic_control.drive_dynamic_data.direction_u16 = direction;
}
void Modbus_UpdateStoredMeasuredSpeed(int16_t measured_speed){
  Actual_Measured_Speed=measured_speed;
  drive_dynamic_control.drive_dynamic_data.measured_speed_u16 = measured_speed;
}
void Modbus_UpdateStoredTorque(int16_t torque){
  drive_dynamic_control.drive_dynamic_data.measured_torque_u16 = torque;  
}
//
void Modbus_UpdateStoredPower(int16_t power){
  drive_dynamic_control.drive_dynamic_data.shaft_power_u16 = power;  
}
void Modbus_UpdateStoredTemperature(int16_t temperature){
  drive_dynamic_control.drive_dynamic_data.ipm_temperature_u16 = temperature;
}

void Modbus_UpdateStoredRegalMCStatus(uint16_t regal_mc_status_data_u16)
{
  drive_dynamic_control.drive_dynamic_data.regal_mc_state_u16 = regal_mc_status_data_u16;
}

void Modbus_UpdateStoredPhaseCurrentIa(int16_t current_ia_s16)
{
  drive_dynamic_control.drive_dynamic_data.phase_current_ia_s16 = current_ia_s16;
}

void Modbus_UpdateStoredPhaseCurrentIb(int16_t current_ib_s16)
{
  drive_dynamic_control.drive_dynamic_data.phase_current_ib_s16 = current_ib_s16;  
}


void Modbus_PassDriveFlashData(uint8_t *data_address_pu8, uint16_t length_u16) {
  	// pass data from UP buffer's data address of length length_u16 to MODBUS usart 
	uint32_t responseLength_u32 = 0;
	uint8_t index_u8 = 0;
	uint16_t crc_calculated_u16 = 0;
  
	// procss if an eception occurred
	if((last_drive_flash_request.is_exception_u8 == TRUE) || (data_address_pu8 == NULL))
	{
	  responseLength_u32 = EXCEPTION_RESPONSE_LENGTH;
	  uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
	  if (error_occurred) {
		return;
	  }
		// if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;
	  // build header
	  modbus_ProtocolBufTX[BYTE_0] = last_drive_flash_request.device_address_u8;
	  modbus_ProtocolBufTX[BYTE_1] = last_drive_flash_request.function_code_u8 + EXCEPTION_FUNCTION_CODE_OFFSET;
	  modbus_ProtocolBufTX[BYTE_2] = last_drive_flash_request.exception_type_u8;
	  // build CRC
	  crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
	  modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (crc_calculated_u16 >> 0);					// crc LSB
	  modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (crc_calculated_u16 >> 8);					// crc MSB
	  // write to seq Mem structure
	  RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf					
	}
	else {
	  if(last_drive_flash_request.function_code_u8 == MODBUS_FUNCTION_CODE_READ_RECORDS) {
		responseLength_u32 = READ_RECORDS_RESPONSE_HEADER_LENGTH + (2*length_u16)+ CRC_LENGTH;
		uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
		if (error_occurred) {
		  return;
		}
		// build header
		modbus_ProtocolBufTX[BYTE_0] = last_drive_flash_request.device_address_u8;
		modbus_ProtocolBufTX[BYTE_1] = last_drive_flash_request.function_code_u8;
		modbus_ProtocolBufTX[BYTE_2] = (2*last_drive_flash_request.length_u8) + 3;	// total byte count for message in bytes
		modbus_ProtocolBufTX[BYTE_3] = (2*last_drive_flash_request.length_u8) + 1;	// total byte count for data, counts hte number of bytes read/written to flash
		modbus_ProtocolBufTX[BYTE_4] = MODBUS_REFERENCE_TYPE;	// Always 0x06
		
		// fill modbus TX with UP data from EEPROM command
		for(index_u8 = 0; index_u8 < length_u16; index_u8++)
		  modbus_ProtocolBufTX[index_u8] = (*data_address_pu8++);
		
		// build CRC
		crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
		modbus_ProtocolBufTX[responseLength_u32 - 2] = (uint8_t) (crc_calculated_u16 >> 0);					// crc LSB
		modbus_ProtocolBufTX[responseLength_u32 - 1] = (uint8_t) (crc_calculated_u16 >> 8);					// crc MSB
		RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf	
		//uint16_t alt_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u16 - 2); // debugging
	  }
	  else {
		responseLength_u32 = WRITE_RECORDS_RESPONSE_HEADER_LENGTH /*+ (2*length_u16) */ + CRC_LENGTH;
		uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
		if (error_occurred) {
		  return;
		}
		// build header
		modbus_ProtocolBufTX[BYTE_0] = last_drive_flash_request.device_address_u8;
		modbus_ProtocolBufTX[BYTE_1] = last_drive_flash_request.function_code_u8;
		modbus_ProtocolBufTX[BYTE_2] = (2*last_drive_flash_request.length_u8) + 3;	// total byte count for message in bytes
		modbus_ProtocolBufTX[BYTE_3] = (2*last_drive_flash_request.length_u8) + 1;	// total byte count for data, counts hte number of bytes read/written to flash
		modbus_ProtocolBufTX[BYTE_4] = MODBUS_REFERENCE_TYPE;	// Always 0x06
		
		// fill modbus TX with UP data from EEPROM command
		for(index_u8 = 0; index_u8 < length_u16; index_u8++)
		  modbus_ProtocolBufTX[index_u8] = (*data_address_pu8++);
		
		// build CRC
		crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
		modbus_ProtocolBufTX[responseLength_u32 - 2] = (uint8_t) (crc_calculated_u16 >> 0);					// crc LSB
		modbus_ProtocolBufTX[responseLength_u32 - 1] = (uint8_t) (crc_calculated_u16 >> 8);					// crc MSB
		RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf	
		//uint16_t alt_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u16 - 2); // debugging		
	  }
	}
}

void Modbus_PassEEPROMData(uint8_t* data_address_pu8, uint16_t length_u16) {
	// pass data from UP buffer's data address of length length_u16 to MODBUS usart 
	uint32_t responseLength_u32 = 0;
	uint8_t index_u8 = 0;
	uint16_t crc_calculated_u16 = 0;
  
	// procss if an eception occurred
	if((last_eeprom_request.is_exception_u8 == TRUE) || (data_address_pu8 == NULL))
	{
	  responseLength_u32 = EXCEPTION_RESPONSE_LENGTH;
	  uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
	  if (error_occurred) {
		return;
	  }
		// if((modbus_ProtocolBufTX = (uint8_t *) realloc(modbus_ProtocolBufTX, responseLength_u32)) == NULL) reallocError++;
	  // build header
	  modbus_ProtocolBufTX[BYTE_0] = last_eeprom_request.device_address_u8;
	  modbus_ProtocolBufTX[BYTE_1] = last_eeprom_request.function_code_u8 + EXCEPTION_FUNCTION_CODE_OFFSET;
	  modbus_ProtocolBufTX[BYTE_2] = last_eeprom_request.exception_type_u8;
	  // build CRC
	  crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
	  modbus_ProtocolBufTX[BYTE_3] = (uint8_t) (crc_calculated_u16 >> 0);					// crc LSB
	  modbus_ProtocolBufTX[BYTE_4] = (uint8_t) (crc_calculated_u16 >> 8);					// crc MSB
	  // write to seq Mem structure
	  RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf					
	}
	else {
	  if(last_eeprom_request.function_code_u8 == MODBUS_FUNCTION_CODE_READ_RECORDS) {
		responseLength_u32 = READ_RECORDS_RESPONSE_HEADER_LENGTH + (2*length_u16)+ CRC_LENGTH;
		uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
		if (error_occurred) {
		  return;
		}
		// build header
		modbus_ProtocolBufTX[BYTE_0] = last_eeprom_request.device_address_u8;
		modbus_ProtocolBufTX[BYTE_1] = last_eeprom_request.function_code_u8;
		modbus_ProtocolBufTX[BYTE_2] = (2*last_eeprom_request.length_u8) + 3;	// total byte count for message in bytes
		modbus_ProtocolBufTX[BYTE_3] = (2*last_eeprom_request.length_u8) + 1;	// total byte count for data, counts hte number of bytes read/written to flash
		modbus_ProtocolBufTX[BYTE_4] = MODBUS_REFERENCE_TYPE;	// Always 0x06
		
		// fill modbus TX with UP data from EEPROM command
		for(index_u8 = 0; index_u8 < length_u16; index_u8++)
		  modbus_ProtocolBufTX[index_u8] = (*data_address_pu8++);
		
		// build CRC
		crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
		modbus_ProtocolBufTX[responseLength_u32 - 2] = (uint8_t) (crc_calculated_u16 >> 0);					// crc LSB
		modbus_ProtocolBufTX[responseLength_u32 - 1] = (uint8_t) (crc_calculated_u16 >> 8);					// crc MSB
		RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf	
		//uint16_t alt_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u16 - 2); // debugging
	  }
	  else {
		responseLength_u32 = WRITE_RECORDS_RESPONSE_HEADER_LENGTH /*+ (2*length_u16) */ + CRC_LENGTH;
		uint8_t error_occurred = Modbus_ReallocateTxBufferForLength(responseLength_u32);
		if (error_occurred) {
		  return;
		}
		// build header
		modbus_ProtocolBufTX[BYTE_0] = last_eeprom_request.device_address_u8;
		modbus_ProtocolBufTX[BYTE_1] = last_eeprom_request.function_code_u8;
		modbus_ProtocolBufTX[BYTE_2] = (2*last_eeprom_request.length_u8) + 3;	// total byte count for message in bytes
		modbus_ProtocolBufTX[BYTE_3] = (2*last_eeprom_request.length_u8) + 1;	// total byte count for data, counts hte number of bytes read/written to flash
		modbus_ProtocolBufTX[BYTE_4] = MODBUS_REFERENCE_TYPE;	// Always 0x06
		
		// fill modbus TX with UP data from EEPROM command
		for(index_u8 = 0; index_u8 < length_u16; index_u8++)
		  modbus_ProtocolBufTX[index_u8] = (*data_address_pu8++);
		
		// build CRC
		crc_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u32 - 2);
		modbus_ProtocolBufTX[responseLength_u32 - 2] = (uint8_t) (crc_calculated_u16 >> 0);					// crc LSB
		modbus_ProtocolBufTX[responseLength_u32 - 1] = (uint8_t) (crc_calculated_u16 >> 8);					// crc MSB
		RingBuf_WriteBlock((*usart1Control_Modbus).seqMemTX, modbus_ProtocolBufTX, &responseLength_u32); 	//extract the whole frame into he temporary modbus_ProtocolBuf	
		//uint16_t alt_calculated_u16 = calculateModbusCRC(modbus_ProtocolBufTX, responseLength_u16 - 2); // debugging		
	  }
	}
}

// Bootloader Features (START)
// TODO: In this section, move to their own file
#define UTILITY_FUNCTION_ID_JUMP_TO_PARTITION 1 
#define UTILITY_SECURITY_CODE 99
#define PARTITION_ID_BOOTLOADER 1
void Utility_ExecuteOperation(uint8_t function_id, uint8_t function_parameter, uint8_t function_subparameter) {
  switch (function_id)
  {
    case UTILITY_FUNCTION_ID_JUMP_TO_PARTITION:
      { // function_parameter = partition id, function_subparameter = sercurity code
        if (function_subparameter ==  UTILITY_SECURITY_CODE) { // only execute if security code was entered
          if (function_parameter == PARTITION_ID_BOOTLOADER) {
            Modbus_JumpToBootloader();
          }
        }
        break;
      }
    default:
      {
        break;
      }

  }
}

void Build_UPDrive_Flash_Request(uint16_t data_address_u16, uint8_t length_u8) {
	// Send update speed command to motor control
	uint8_t requestFlashTx[9];		// UP message 0x80 is length 9
    uint32_t tx_length_u32;
	
	requestFlashTx[0] = 0x55;		// standard header from master
	requestFlashTx[1] = 0x02;		// length of data = 2 because only one address, which is a word
	requestFlashTx[2] = 0x79;		// command to read data from drve side flash
	requestFlashTx[3] = 0x00;
	requestFlashTx[4] = 0x00;
	requestFlashTx[5] = (uint8_t) ((data_address_u16 >> 8) & (0xFF));
	requestFlashTx[6] = (uint8_t) ((data_address_u16 >> 0) & (0xFF));
	requestFlashTx[7] = 0xCC;
	requestFlashTx[8] = 0xCC;
	
	tx_length_u32 = UP_HEADER_LENGTH + 2;
	RingBuf_WriteBlock((*usart2Control_AppLocal).seqMemTX_u32, requestFlashTx, &tx_length_u32);
}

void Build_UPEEPROM_Request(uint16_t data_address_u16, uint8_t length_u8) {
	// Send update speed command to motor control
	uint8_t requestEEPROMTx[9];		// UP message 0x80 is length 9
    uint32_t tx_length_u32;
	
	requestEEPROMTx[0] = 0x55;		// standard header from master
	requestEEPROMTx[1] = 0x02;		// length of data = 2 because only one address, which is a word
	requestEEPROMTx[2] = 0x80;		// command to read data from EEPROM
	requestEEPROMTx[3] = 0x00;
	requestEEPROMTx[4] = 0x00;
	requestEEPROMTx[5] = (uint8_t) ((data_address_u16 >> 8) & (0xFF));
	requestEEPROMTx[6] = (uint8_t) ((data_address_u16 >> 0) & (0xFF));
	requestEEPROMTx[7] = 0xCC;
	requestEEPROMTx[8] = 0xCC;
	
	tx_length_u32 = UP_HEADER_LENGTH + 2;
	RingBuf_WriteBlock((*usart2Control_AppLocal).seqMemTX_u32, requestEEPROMTx, &tx_length_u32);
}

void Modbus_WatchDogInitialize(void) { // REVIEW: This is not currently used, as watchdog is initialized in application.
    LL_IWDG_Enable(IWDG);
    LL_IWDG_EnableWriteAccess(IWDG);
    LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_4);
    //LL_IWDG_SetWindow(IWDG, 4095);
    LL_IWDG_SetReloadCounter(IWDG, 4095);
    while (LL_IWDG_IsReady(IWDG) != 1) {
    }
//    LL_IWDG_ReloadCounter(IWDG);
    LL_IWDG_SetWindow(IWDG, 4095);
}
// !errorTemporary: Watchdog Timer (END)
void Modbus_JumpToBootloader(void) {
    // Reboot via Watchdog Reset: 
	// - This works, when bootloader starts at address 0x08000000
    Modbus_WatchDogInitialize();
    while (1) {}
}
// Bootloader Features (END)