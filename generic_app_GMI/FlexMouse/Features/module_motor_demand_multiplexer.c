/**
********************************************************************************************************************************
* @file    module_motor_demand_multiplexer.c 
* @author  Justin Moon
* @brief   Combine multiple motor demand sources into a single motor demand output.
* @details This module is to be referenced by 'module_motor_com' to send a demand value to the motor control MCU.
********************************************************************************************************************************
*/

// Includes -------------------------------------------------------------------
#include "module_motor_demand_multiplexer.h"

#include "module_motor_com.h"
#include "driver_usart2.h"

// - Dependent Inclusions (only include modules that exist)
// - 0-10V Analog Input
#ifdef MODULE_ANALOG_0_10V_ID
  #include "module_analog_0_10v.h"
#endif
// - 4-20mA Analog Input
#ifdef MODULE_ANALOG_4_20MA_ID
  #include "module_analog_4_20ma.h"
#endif
// - Digital Inputs
#ifdef MODULE_DIGITAL_INPUTS_ID
  #include "module_digital_inputs.h"
#endif
// - PWM Input
#ifdef MODULE_PWM_INPUT_ID
  #include "module_pwm_input.h"
#endif
// - Modbus
#ifdef MODULE_MODBUS_ID
  #include "module_modbus.h"
#endif

// Content --------------------------------------------------------------------
// - Function Prototypes
void MotorDemandMux_AssignModuleMem(void);
void MotorDemandMux_InitStructPointers(void);
void MotorDemandMux_InitSystem(void);
void MotorDemandMux_Update(void);
uint16_t MotorDemandMux_GetSourceByPriority(void);
uint16_t MotorDemandMux_GetActiveValue(uint16_t active_demand_source);

// - Constants
// -- Module States
enum {
  MEMORY_INIT_MODULE,
  INIT_MODULE,
  RUN_MODULE,
  // Ddditional states to be added here as necessary.
  IRQ_MODULE = DEFAULT_IRQ_STATE,
  KILL_MODULE = KILL_APP
};

// - External Variables
extern ProcessInfo processInfoTable[];

// - Global variables specific to this module
static  Ram_Buf_Handle motorDemandMux_StructMem_u32;
MotorDemandMux_Control motorDemandMux_Control; // Main Structure of Settings and Data
uint64_t motorDemandMux_PollTime_u64 = 0;      // Poll time for the module
// - Global variables stored by this module regarding the sxtate of other modules
uint16_t motorDemandMux_ModbusSpeed = 0;
uint16_t motorDemandMux_ModbusDemand = 0;
uint16_t motorDemandMux_ModbusStart = 0;
uint16_t motorDemandMux_ModbusDirection = 0;



// -- Define Pointers that will be used as References to other Modules, where applicable
// Motor_Com_Control* motorDemandMux_MotorCom_Control_ptr; // REVIEW: Only used to set motor direction

// --- 0-10V Analog Input
#ifdef MODULE_ANALOG_0_10V_ID
  AnalogVolts_Control *motorDemandMux_analog_0_10v_Control_ptr;
#endif
// --- 4-20mA Analog Input
#ifdef MODULE_ANALOG_4_20MA_ID
  Analog_4_20ma_Control *motorDemandMux_analog_4_20ma_Control_ptr;
#endif
// --- Digital Inputs
#ifdef MODULE_DIGITAL_INPUTS_ID
  DigitalInputs_Control *motorDemandMux_digitalInputsControl_ptr;
#endif
// --- PWM Input
#ifdef MODULE_PWM_INPUT_ID
  PwmInput_Control *motorDemandMux_pwmInputControl_ptr;
#endif
// --- Modbus
#ifdef MODULE_MODBUS_ID
  Modbus_Control *motorDemandMux_modbusControl_ptr;
#endif


// - external functions
void MotorCom_UpdateMotorDirection(uint16_t direction); // TODO: Remove this once motor direction is stored in this module

/**
********************************************************************************************************************************
* @brief   State machine for app module
* @details
* @param   drv_identifier_u8, previous_state_u8, next_stat_u8, irq_identfier_u8
* @retval  return_state_u8
********************************************************************************************************************************
*/
uint8_t moduleMotorDemandMultiplexer_u32(uint8_t drv_identifier_u8, uint8_t previous_state_u8, uint8_t next_state_u8,
                                           uint8_t irq_identifier_u8) 
{
   
  uint8_t return_state_u8 = MEMORY_INIT_MODULE;
  switch (next_state_u8) {
    case MEMORY_INIT_MODULE:
    {
      MotorDemandMux_AssignModuleMem(); // Assign structured memory to Analog 0-10V setting and data 
      return_state_u8 = INIT_MODULE;
      break;
    }
  case INIT_MODULE: {
    // Assign structured memory for module
    //MotorDemandMux_AssignModuleMem();
    MotorDemandMux_InitSystem();
    MotorDemandMux_InitStructPointers();
    
    motorDemandMux_PollTime_u64 = getSysCount() + MODULE_MOTOR_DEMAND_MUX_POLL_TIME; // Store time tick value
    return_state_u8 = RUN_MODULE;
    break;
  }
  case RUN_MODULE: {
    // Process analog input every "AnalogPeriod" mSec
    if (getSysCount() >= motorDemandMux_PollTime_u64) // Time above poll time
    {
      motorDemandMux_PollTime_u64 = getSysCount() + MODULE_MOTOR_DEMAND_MUX_POLL_TIME; // Next poll time
      // User Code
      // - Update Demand Here
      MotorDemandMux_Update();
    }
    return_state_u8 = RUN_MODULE;
    break;
  }
  case IRQ_MODULE: {
    // If there are more than one interrupts, from different drivers, you can identify each individually by:
    // tableIndex_u8 = getProcessInfoIndex(irq_identifier_u8);
    // Then use processInfoTable[tableIndex_u8] to tailor your response appropriately.
    return_state_u8 = RUN_MODULE;
    break;
  }
  
  case KILL_MODULE: {
    // Setting processStatus_u8 to PROCESS_STATUS_KILLED prevents the scheduler main loop from calling this module again.
    uint8_t table_index_u8 = getProcessInfoIndex(drv_identifier_u8);
    if (table_index_u8 != INDEX_NOT_FOUND) {
      processInfoTable[table_index_u8].Sched_DrvData.processStatus_u8 = PROCESS_STATUS_KILLED;
    }
    return_state_u8 = INIT_MODULE;
    break;
  }
  default:
    return_state_u8 = KILL_MODULE; //10;
    break;
  }
   
  return return_state_u8;
}
  
/**
********************************************************************************************************************************
* @brief   Assign structured memory
* @details Assign structured memory for motorDemandMux_Control
* @param   None 
* @return  None
********************************************************************************************************************************
*/
void MotorDemandMux_AssignModuleMem(void) {
  motorDemandMux_StructMem_u32 =  StructMem_CreateInstance(MODULE_MOTOR_DEMAND_MUX, sizeof(MotorDemandMux_Control), ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST); 
  // REVIEW: Above statement uses sizeof(MotorDemandMux_Control), but I believe that to be inaccurate, as that is the size of the object that is 
  // - pointed to by  p_rambuf, not the StructMem object. Fix this, if needed.
  
  (*motorDemandMux_StructMem_u32).p_ramBuf_u8 = (uint8_t *)&motorDemandMux_Control ;    // Map the ADC1 memory into the structured memory
  uint8_t module_index_u8 = getProcessInfoIndex(MODULE_MOTOR_DEMAND_MUX);
  processInfoTable[module_index_u8].Sched_ModuleData.p_masterSharedMem_u32 = (Ram_Buf_Handle)motorDemandMux_StructMem_u32;
}

/**
********************************************************************************************************************************
* @brief   Initialize Pointers to Structures of Other Modules that this Module Depends Upon
* @details Only Called Once at module Initialization
* @param   None 
* @return  None
********************************************************************************************************************************
*/
void MotorDemandMux_InitStructPointers(void) {
  uint8_t module_index_u8;
  // Initialize pointers to structures of other modules used by this module
  // - Motor COM - REVIEW: Temporarily Added for Access to Motor Direction (which should be handled by this module in the future)
  // TODO: Pointer to MODULE_MOTOR_COM does not work (cyclic dependency, in future motor direction should be held in this module)
  // module_index_u8  = getProcessInfoIndex(MODULE_MOTOR_COM);   //return Process index from processInfo array
  // motorDemandMux_MotorCom_Control_ptr = (Motor_Com_Control*)((*(processInfoTable[module_index_u8].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8);    //Get structured memory 
  // - Demand Input Sources
#ifdef MODULE_ANALOG_0_10V_ID
  module_index_u8 = getProcessInfoIndex(MODULE_ANALOG_0_10V);
  motorDemandMux_analog_0_10v_Control_ptr =
    (AnalogVolts_Control *) ((*(processInfoTable[module_index_u8].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8);
#endif
#ifdef MODULE_ANALOG_4_20MA_ID
  module_index_u8 = getProcessInfoIndex(MODULE_ANALOG_4_20MA);
  motorDemandMux_analog_4_20ma_Control_ptr =
    (Analog_4_20ma_Control *) ((*(processInfoTable[module_index_u8].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8);
#endif
#ifdef MODULE_DIGITAL_INPUTS_ID
  module_index_u8 = getProcessInfoIndex(MODULE_DIGITAL_INPUTS);
  motorDemandMux_digitalInputsControl_ptr =
    (DigitalInputs_Control *) ((*(processInfoTable[module_index_u8].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8);
#endif
#ifdef MODULE_PWM_INPUT_ID
  module_index_u8 = getProcessInfoIndex(MODULE_PWM_INPUT);
  motorDemandMux_pwmInputControl_ptr =
    (PwmInput_Control *) ((*(processInfoTable[module_index_u8].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8);
#endif  
}

/**
********************************************************************************************************************************
* @brief   Initialize RAM Objects Used by This Module
* @details None
* @param   None 
* @return  None
********************************************************************************************************************************
*/
void MotorDemandMux_InitSystem(void) {
  // Init settings
  motorDemandMux_Control.motorDemandMux_Settings.demandSource_u16 = MOTOR_DEMAND_SOURCE_PRIORITY_SYSTEM;
  motorDemandMux_Control.motorDemandMux_Settings.control_Mode_u16 = DEMAND_MODE_SPEED; 
  motorDemandMux_Control.motorDemandMux_Settings.modbusPriority_u16 = 2;         // Demand priority. 1 being highest
  motorDemandMux_Control.motorDemandMux_Settings.analog0_10vPriority_u16 = 1;   // Demand priority. 1 being highest
  motorDemandMux_Control.motorDemandMux_Settings.digitalInputsPriority_u16 = 3; // Demand priority. 1 being highest
  motorDemandMux_Control.motorDemandMux_Settings.pwmInputPriority_u16 = 4;      // Demand priority. 1 being highest
  motorDemandMux_Control.motorDemandMux_Settings.analog4_20mAPriority_u16 = 5;  // Demand priority. 1 being highest  
  motorDemandMux_Control.motorDemandMux_Settings.maxPriority_u16 = 5;            // Demand priority. 1 being highest

  // Init Data
  motorDemandMux_Control.motorDemandMux_Data.demandValue_u16 = 0;  
}

/**
********************************************************************************************************************************
* @brief   Update the overall output demand stored by this module.
* @details This function chooses from a variety of demand sources and filters them down to a single demand based on settings.
* @param   None 
* @return  None
********************************************************************************************************************************
*/
void MotorDemandMux_Update(void) {
  uint16_t active_demand_source_u16 = 0;
  uint16_t output_demand_u16 = 0;
  uint16_t demand_source_index_u16 = motorDemandMux_Control.motorDemandMux_Settings.demandSource_u16;
  
  
  // Determine which demand source that we should look at.
  if (demand_source_index_u16 != MOTOR_DEMAND_SOURCE_PRIORITY_SYSTEM) {
    active_demand_source_u16 = demand_source_index_u16;
  } else {
    active_demand_source_u16 = MotorDemandMux_GetSourceByPriority();
    motorDemandMux_Control.motorDemandMux_Data.currentDemandSource_u16 = active_demand_source_u16;
  }
  
  // Get Demand from the active source
  output_demand_u16 = MotorDemandMux_GetActiveValue(active_demand_source_u16);
  motorDemandMux_Control.motorDemandMux_Data.demandValue_u16 = output_demand_u16;
}

extern Usart2_Control* usart2Control_AppLocal;

#define ADC_FAULT_INJECTION 0xAA
#define REGISTER_FAULT_INJECTION 0xBB
#define CLOCK_FAULT_INJECTION 0xCC
#define RAM_FAULT_INJECTION 0xDD
#define ROM_FAULT_INJECTION 0xEE
/**
********************************************************************************************************************************
* @brief   Get the Demand Value of a Specified Demand Source.
* @details This function pulls the values from the structure pointers assigned in MotorDemandMux_InitStructPointers
* @param   demand_source_id_u16 
* @return  active_demand_value_u16
********************************************************************************************************************************
*/

uint16_t MotorDemandMux_GetActiveValue(uint16_t demand_source_id_u16) {
  uint16_t active_demand_value_u16 = 0;
  uint16_t active_demand_source_u16;
  volatile char fault_injected =0;
  if (demand_source_id_u16 == MOTOR_DEMAND_SOURCE_PRIORITY_SYSTEM) {
    active_demand_source_u16 = MotorDemandMux_GetSourceByPriority();
  } else {
    active_demand_source_u16 = demand_source_id_u16;
  }
#ifdef MODULE_DIGITAL_INPUTS_ID  
  if((*motorDemandMux_digitalInputsControl_ptr).digitalInputs_Data.discretes_u16.is_motorEnabled == FALSE) 
  {
    active_demand_value_u16 = 0;
  } else {
#endif   
    switch (active_demand_source_u16) {
    case MOTOR_DEMAND_SOURCE_ANALOG_0_10V:
#ifdef MODULE_ANALOG_0_10V_ID
      active_demand_value_u16 = (*motorDemandMux_analog_0_10v_Control_ptr).analogVolts_Data.analogDemandPercent_u16;
#endif
      break;
    case MOTOR_DEMAND_SOURCE_ANALOG_4_20MA:
      {
#ifdef MODULE_ANALOG_4_20MA_ID
        active_demand_value_u16 = (*motorDemandMux_analog_4_20ma_Control_ptr).analog_4_20mA_Data.analogDemandPercent_u16;
#endif
        break;
      }
    case MOTOR_DEMAND_SOURCE_DIGITAL_INPUTS:
      {
#ifdef MODULE_DIGITAL_INPUTS_ID
        active_demand_value_u16 = (*motorDemandMux_digitalInputsControl_ptr).digitalInputs_Data.discreteDemandPercent;
#endif
        break;
      }
    case MOTOR_DEMAND_SOURCE_PWM_INPUT:
      {
#ifdef MODULE_PWM_INPUT_ID
        active_demand_value_u16 = (uint16_t)(*motorDemandMux_pwmInputControl_ptr).pwmInput_Data.pwmInputDemandPercentage_u16;
#endif
        break;
      }
    case MOTOR_DEMAND_SOURCE_SERIAL_COMMAND:
      {
#ifdef MODULE_MODBUS_ID
        if (motorDemandMux_ModbusStart <= 0) {
          active_demand_value_u16 = 0;
        } 
        // active demand percent (takes priority over speed)
        else if (motorDemandMux_ModbusDemand > 0) {
          /************************************************************/
          uint32_t fault_length;
          
          switch(motorDemandMux_ModbusDemand)
          {
            case ADC_FAULT_INJECTION: // send 31,1,0,0
              unsigned char adc_fault[]= {0x55, 0x01, 0x1F, 0x0, 0x0,0x0 ,0xCC, 0xCC};//{31,1,0,0};
              fault_length = sizeof(adc_fault);
                               
               RingBuf_WriteBlock((*usart2Control_AppLocal).seqMemTX_u32, adc_fault, &fault_length);
               fault_injected = 1;
              
            break;
            case REGISTER_FAULT_INJECTION: // send 31,1,1,0
              unsigned char register_fault[]={0x55, 0x01, 0x1F, 0x0,0x0,0x1 ,0xCC, 0xCC};
              fault_length = sizeof(register_fault);
               RingBuf_WriteBlock((*usart2Control_AppLocal).seqMemTX_u32, register_fault, &fault_length);
               fault_injected = 1;
            break;
            case CLOCK_FAULT_INJECTION: // send 31,1,2,0
              unsigned char clock_fault[]={0x55, 0x01, 0x1F, 0x0, 0x0,0x2 ,0xCC, 0xCC};;
              fault_length = sizeof(clock_fault);
               RingBuf_WriteBlock((*usart2Control_AppLocal).seqMemTX_u32, clock_fault, &fault_length);
               fault_injected = 1;
            break;
            case RAM_FAULT_INJECTION: // send 31,1,3,0
              unsigned char RAM_fault[]={0x55, 0x01, 0x1F, 0x0, 0x0,0x3 ,0xCC, 0xCC};;
              fault_length = sizeof(RAM_fault);
               RingBuf_WriteBlock((*usart2Control_AppLocal).seqMemTX_u32, RAM_fault, &fault_length);
               fault_injected = 1;
            break;
            case ROM_FAULT_INJECTION: // send 31,1,4,0
              unsigned char ROM_fault[]={0x55, 0x01, 0x1F, 0x0, 0x0,0x4 ,0xCC, 0xCC};;
              fault_length = sizeof(ROM_fault);
               RingBuf_WriteBlock((*usart2Control_AppLocal).seqMemTX_u32, ROM_fault, &fault_length);
               fault_injected = 1;
            break;
            
          }
          
          /************************************************************/
          if(fault_injected ==0){
          if ((motorDemandMux_ModbusDemand < 10000)) { // TODO: Magic Number DEMAND_PERCENT_MAX
            active_demand_value_u16 = motorDemandMux_ModbusDemand ;
          } else { // limit to 100% demand
            active_demand_value_u16 = 10000; // TODO: Magic Number
          }
          }
          motorDemandMux_ModbusDemand = 0;
         // active demand speed (second priority)
        } 
        if (motorDemandMux_ModbusSpeed < 0xFFFF) // Ignore 0xFFFF speed from Modbus since its not valid
        {
          active_demand_value_u16 = MotorCom_ConvertSpeedToDemandPercentage(motorDemandMux_ModbusSpeed);
        } else {
          active_demand_value_u16 = 0;
        }
#endif // def MODULE_MODBUS_ID
        break;
      }
    default: // No Known demand source
      break;
    }
    
#ifdef MODULE_DIGITAL_INPUTS_ID // Needed this for the else part of if((*motorDemandMux_digitalInputsControl_ptr).digitalInputs_Data.discretes_u16.is_motorEnabled == FALSE). This makes sure digital inputs module is available 
  //if ( ((*motorDemandMux_digitalInputsControl_ptr).digitalInputs_Data.is_invertDirection == TRUE) && (motorDemandMux_Control.motorDemandMux_Settings.control_Mode_u16 == DEMAND_MODE_SPEED) )
    //{
      //active_demand_value_u16 = active_demand_value_u16 * -1; // Negative speed corresponds to speed in oppsite direction
    //}
  }
#endif
  return active_demand_value_u16;
}

/**
********************************************************************************************************************************
* @brief   Get the demand source id of the highest priority demand source that is active.
* @details This is used by 'MotorDemandMux_GetActiveValue' to determine which source to look at.
* @param   None 
* @return  demand_source_id_u16
********************************************************************************************************************************
*/

uint16_t MotorDemandMux_GetSourceByPriority(void) {
  uint16_t new_demand_u16 = 0;
  uint8_t new_demand_source_u8 = 0;  
  uint16_t demand_source_id_u16 = 0;
  uint8_t previous_demand_source_u8 = 0;
  uint16_t previous_demand_u16 = 0;
  //uint16_t current_demand_u16 = 0;
  //uint8_t current_demand_source_u8 = 0;  
  //uint8_t highest_priority_source_id_u8 = 0; // ID of highest priority
  
  for(uint8_t priority_u8 = motorDemandMux_Control.motorDemandMux_Settings.maxPriority_u16; priority_u8 > 0; priority_u8--)
  { // Loop through lowest (max_Priority_u8) to highest priority (1)
      #ifdef MODULE_MODBUS_ID
      if(motorDemandMux_Control.motorDemandMux_Settings.modbusPriority_u16 == priority_u8)
      {
        //new_demand_u16 = motorDemandMux_ModbusDemand;              // Demand from Modbus
        if( motorDemandMux_ModbusSpeed == 0)
        {
          new_demand_u16 = motorDemandMux_ModbusDemand;              // Demand from Modbus
        }else 
        {
          new_demand_u16 = MotorCom_ConvertSpeedToDemandPercentage(motorDemandMux_ModbusSpeed); // Demand from Modbus
        }
        
        new_demand_source_u8 = MOTOR_DEMAND_SOURCE_SERIAL_COMMAND; // Modbus source          
        //break;
      }
      #endif
    
      #ifdef MODULE_ANALOG_0_10V_ID
      if( motorDemandMux_Control.motorDemandMux_Settings.analog0_10vPriority_u16 == priority_u8)
      {
        new_demand_u16 = (*motorDemandMux_analog_0_10v_Control_ptr).analogVolts_Data.analogDemandPercent_u16; // Demand from analog 0-10V input
        new_demand_source_u8 = MOTOR_DEMAND_SOURCE_ANALOG_0_10V;  // Analog 0-10V source          
        //break;
      }
      #endif
    
      #ifdef MODULE_DIGITAL_INPUTS_ID
      if( motorDemandMux_Control.motorDemandMux_Settings.digitalInputsPriority_u16 == priority_u8)
      {
        new_demand_u16 = (*motorDemandMux_digitalInputsControl_ptr).digitalInputs_Data.discreteDemandPercent;          // Demand from digital inputs
        new_demand_source_u8 = MOTOR_DEMAND_SOURCE_DIGITAL_INPUTS; // Digital input source
        //break;
      }
      #endif
    
      #ifdef MODULE_PWM_INPUT_ID
      if( motorDemandMux_Control.motorDemandMux_Settings.pwmInputPriority_u16 == priority_u8)
      {
        new_demand_u16 = (uint16_t)(*motorDemandMux_pwmInputControl_ptr).pwmInput_Data.pwmInputDemandPercentage_u16;         // Demand from PWM input
        new_demand_source_u8 = MOTOR_DEMAND_SOURCE_PWM_INPUT;     // PWM input source
        //break;
      }
      #endif
    
      #ifdef MODULE_ANALOG_4_20MA_ID
      if( motorDemandMux_Control.motorDemandMux_Settings.analog4_20mAPriority_u16 == priority_u8)
      {
        new_demand_u16 = (*motorDemandMux_analog_4_20ma_Control_ptr).analog_4_20mA_Data.analogDemandPercent_u16;         // Demand from analog 4-20mA
        new_demand_source_u8 = MOTOR_DEMAND_SOURCE_ANALOG_4_20MA; // Analog 4-20mA source
        //break;
      }
      #endif
    
    if(new_demand_u16 > 0)
    {
      demand_source_id_u16 = new_demand_source_u8;    
      previous_demand_u16 = new_demand_u16;
      previous_demand_source_u8 = new_demand_source_u8;
    } else
    {
      if(previous_demand_u16 != 0)
      {
        demand_source_id_u16 = previous_demand_source_u8;
      }else
      {
        demand_source_id_u16 = DEFAULT_MOTOR_DEMAND_SOURCE;
      }
    }
    //if(priority_u8 == 1)
    //{
     // highest_priority_source_id_u8 = new_demand_source_u8;
    //}
  }
  return demand_source_id_u16;
}

/**
********************************************************************************************************************************
* @brief   Get the demand source id of the highest priority demand source that is active.
* @details This is used by 'MotorDemandMux_GetActiveValue' to determine which source to look at.
* @param   None 
* @return  demand_source_id_u16
********************************************************************************************************************************
*/
void MotorDemandMux_ModbusUpdate(uint16_t commanded_speed, uint16_t commanded_demand, uint16_t commanded_start, uint16_t demand_source, uint16_t direction) {
  motorDemandMux_ModbusSpeed = commanded_speed;
  motorDemandMux_ModbusDemand = commanded_demand;
  motorDemandMux_ModbusStart = commanded_start;
  if (demand_source < MOTOR_DEMAND_SOURCES_TOTAL) {// todo: magic number - NO_DEMAND_SOURCE
    if(demand_source != MOTOR_DEMAND_SOURCE_SERIAL_COMMAND)
    { // Only update settings when the source is other then Modbus
      motorDemandMux_Control.motorDemandMux_Settings.demandSource_u16 = demand_source;
    }
    motorDemandMux_Control.motorDemandMux_Data.currentDemandSource_u16 = demand_source;
  }
  motorDemandMux_ModbusDirection = direction;
  // TODO: Move All Motor Limtis and (non-communication related) Settings from module_motor_com to motor_demand_multiplexer
  //  (*motorDemandMux_MotorCom_Control_ptr).motor_Setting.motor_Direction_u8 = direction ? CW: CCW;
  MotorCom_UpdateMotorDirection(direction);
}

// void MotorDemandMux_ModbusConvertSpeedToDemand(uint16_t speed_rpm) {

// }
