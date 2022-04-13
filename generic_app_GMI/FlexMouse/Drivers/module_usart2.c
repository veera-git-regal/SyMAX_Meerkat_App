/**
********************************************************************************************************************************
* @file    module_usart2.c 
* @author  Pamela Lee
* @brief   Main driver module for USART2 Communication.
* @details This module initializes the USART2 port and attaches the pre-selected fixed memory allocation to the module.
To Transmitt data in the RUN_MODULE case: put data into seqMemTX_u32, and call this function:
*             USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
********************************************************************************************************************************
*/

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "module_usart2.h"

#include "driver_usart2.h"
#include "module_motor_com.h"
#define LED_ONBOARD_Pin LL_GPIO_PIN_6 //SPA REVIEW.

/* Content ---------------------------------------------------------------------------------------------------------------------*/
/* Uarts handle declaration */
extern void Delay(__IO uint32_t nTime);

extern Ram_Buf sharedMemArray[STRUCT_MEM_ARRAY_SIZE];
extern ProcessInfo processInfoTable[];

//extern Ram_Buf *usart2StructMem_u32;
extern Ram_Buf_Handle usart2StructMem_u32;
//Usart2_Control* usart2_Module_Control;

#define ENABLE_RXCMD_USART2_FIXED_LEN 1
#if ENABLE_RXCMD_USART2_FIXED_LEN >= 1
// This is a one-shot buffer, that is written to and read from in single calls.
// - it does not currently need to be tracked for current index because of this.
// - REVIEW: does RX_FLASHUPDATECMD_LENGTH need to be this big?
// -- Made it this big so 'write packets' could have as much data as possible.
#define FIXED_RXCMD_USART2_MAX_LENGTH  TX_RX_BUF_SIZE // Inclusive (this value is accepted). REVIEW: Size, left high for flash reads 
unsigned char fixedRxCMD_Usart2_Length = 0;
unsigned char fixedRxCMD_Usart2[FIXED_RXCMD_USART2_MAX_LENGTH];
unsigned char* RxCMD_Usart2 = fixedRxCMD_Usart2;
#else // if ENABLE_RXCMD_USART2_FIXED_LEN <= 0
unsigned char* RxCMD_Usart2;
#endif // if ENABLE_RXCMD_USART2_FIXED_LEN <= 0
int16_t Actual_Measured_Speed;
extern uint8_t usart2CaptureLen;
extern uint8_t UniProtocolState;
extern __IO uint8_t indexTx;

extern Usart2_Control usart2Control;

// Function prototypes
void assign_UART2_ModuleMem(uint8_t);
void parseMessage_BulkMonitoring(unsigned int);
void parseMessage_DriveFlashData(unsigned int data_length);
void parseMessage_EEPROMData(unsigned int);

enum {
  MEMORY_INIT_MODULE,
  INIT_MODULE,
  RUN_MODULE,
  // additional states to be added here as necessary.
  IRQ_MODULE = DEFAULT_IRQ_STATE,
  KILL_MODULE = KILL_APP
};

#define UNIVERSAL_PROTOCOL_HEADER_LENGTH 7
#define MESSAGE_LENGTH_UP_BULK_MONITORING (14 + UNIVERSAL_PROTOCOL_HEADER_LENGTH)

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
//
extern void Modbus_PassEEPROMData(uint8_t* data_address_pu8, uint16_t length_u16);
extern void Modbus_PassDriveFlashData(uint8_t *data_address_pu8, uint16_t data_length);

#define UNIVERSAL_PROTOCOL_DATA_START_INDEX 5
volatile uint8_t test_counter_bulk_monitoring = 0;
int16_t Motor_Avg_Speed_s16;
uint8_t Motor_Avg_Counter_u8;


uint8_t moduleUsart2_u32(uint8_t drv_id_u8, uint8_t prev_state_u8, uint8_t next_state_u8, uint8_t irq_id_u8) {
  uint8_t return_state_u8 = MEMORY_INIT_MODULE;
  switch (next_state_u8) {
  case MEMORY_INIT_MODULE:
    {
      assign_UART2_ModuleMem(drv_id_u8); // Assign structured memory
      return_state_u8 = INIT_MODULE;
      break;
    }
  case INIT_MODULE: 
    {
      // Initialize UART
      
      usart2_Init(); // USART init after memory allocation since USART needs buffers during init
      //assign_UART2_ModuleMem();
      
      // Find the structured memory for the UART2 driver module, by searching for the UART2 onwer id.
      /* Ram_Buf_Handle this_ram_buf_u32;
      for (uint8_t struct_mem_index_u8 = 0; struct_mem_index_u8 < TOTAL_NUM_OF_STRUCT_MEM_INSTANCES;
      struct_mem_index_u8++) {
      this_ram_buf_u32 = &sharedMemArray[struct_mem_index_u8];
      if (RamBuf_GetOwner(this_ram_buf_u32) == drv_id_u8) {
      usart2StructMem_u32 = &sharedMemArray[struct_mem_index_u8];
    }
    }
      
      // Attach the structured memory to the process's master shared memory.
      uint8_t table_index_u8 = getProcessInfoIndex(drv_id_u8);
      if (table_index_u8 != INDEX_NOT_FOUND) {
      processInfoTable[table_index_u8].Sched_DrvData.irqState_u8 = DEFAULT_IRQ_STATE;
      processInfoTable[table_index_u8].Sched_DrvData.p_masterSharedMem_u32 =
      usart2StructMem_u32;
    }
      
      //Get structured memory for ADC1 data
      usart2Control = (Usart2_Control*)((*(processInfoTable[table_index_u8].Sched_DrvData.p_masterSharedMem_u32)).p_ramBuf_u8);*/
      usart2CaptureLen = UniHeaderlen;                                 //pam bug without this
      return_state_u8 = RUN_MODULE;
      break;
    }
  case RUN_MODULE: 
    {
      if(RingBuf_GetUsedNumOfElements((usart2Control).seqMem_InternalPipe_u32) >= usart2CaptureLen )
      {
        LL_USART_DisableIT_RXNE(USART2); // disable interrupts so new data doesn't come in while we are editing receive buffer
        // Idle line detection: Only interpret messages when transmission is complete
        // - This Idle Line Patches a bug with lost data due to interpretation of incomplete packets.
        // -- TODO: When this is disabled, firmware infrequently crashes (main loop no longer executes) when interpreting incomplete packets.
        // --- locate the source of the crash and patch, as this likely could still cause problems.
        uint32_t idle_line = Usart2IdlePatch_RxIsIdle(); // LL_USART_IsActiveFlag_IDLE(USART2) (Emulation Note: this is polled) 
        if (idle_line) {
          protocolHeaderfetch();
          // LL_USART_ClearFlag_IDLE(USART2); // Clear the idle line flag.
        }
        LL_USART_EnableIT_RXNE(USART2); // re-enable the receive interrupt after parsing the received data
      }
      uint8_t TxLen = UniHeaderlen;
      //if((RingBuf_GetUsedNumOfElements((Ring_Buf_Handle)((*(usart2Control).seqMemTX_u32).p_ringBuf_u8)) >= TxLen) && !indexTx)
      if((RingBuf_GetUsedNumOfElements((usart2Control).seqMemTX_u32) >= TxLen) && !indexTx)
      {
        TxProcess();
      }
      /********* for MVP only Pam ******************************************/
      
      unsigned int DataLen2 = (unsigned int)UniHeaderlen;
      //if(RingBuf_GetUsedNumOfElements((Ring_Buf_Handle)((*(usart2Control).seqMemRX_u32).p_ringBuf_u8)) >= DataLen2 )
      if(RingBuf_GetUsedNumOfElements((usart2Control).seqMemRX_u32) >= DataLen2 )
      {        
        
#if ENABLE_RXCMD_USART2_FIXED_LEN >= 1
        // No need to check length here, this buffer must be at least longer than the header.
#else // if ENABLE_RXCMD_USART2_FIXED_LEN <= 0
        RxCMD_Usart2 = (unsigned char*) realloc(RxCMD_Usart2,DataLen2);     
#endif // if ENABLE_RXCMD_USART2_FIXED_LEN <= 0
        RingBuf_Observe((usart2Control).seqMemRX_u32, RxCMD_Usart2, 0, &DataLen2);  
        
        //calculate the total number of frame
        DataLen2 = ((unsigned int)RxCMD_Usart2[1] & 0x3F) + (unsigned int)UniHeaderlen;
        
#if ENABLE_RXCMD_USART2_FIXED_LEN >= 1
        // Check for possible buffer overflow
        if (DataLen2 <= FIXED_RXCMD_USART2_MAX_LENGTH) { // Normal Case: Read message into buffer
          RingBuf_ReadBlock((usart2Control).seqMemRX_u32, RxCMD_Usart2, &DataLen2); //extract the whole frame into the buffer
        } else { // Overflow 
          // Read All Data (Clear the Buffer)
          while (DataLen2 > 0) {
            if (DataLen2 > FIXED_RXCMD_USART2_MAX_LENGTH) {
              // REVIEW: Replace with RingBuf_ClearContents? Much less processing
              unsigned int read_length = FIXED_RXCMD_USART2_MAX_LENGTH;
              RingBuf_ReadBlock((usart2Control).seqMemRX_u32, RxCMD_Usart2, &read_length); //extract the whole frame
              // RingBuf_ReadBlock((*usart2Control).seqMemTX_u32, headerFramebuf, &read_length);             //copy the complete frame into buffer
              DataLen2 -= FIXED_RXCMD_USART2_MAX_LENGTH;
            } else {
              RingBuf_ReadBlock((usart2Control).seqMemRX_u32, RxCMD_Usart2, &DataLen2); //extract the whole frame
              // RingBuf_ReadBlock((*usart2Control).seqMemTX_u32, headerFramebuf, &DataLen2);             //copy the complete frame into buffer
              DataLen2 = 0;
            }
          }
          // Exit Gracefully, so that we don't interpret the corrupted message
          return_state_u8 = RUN_MODULE;
          return return_state_u8;
        }
#else // if ENABLE_RXCMD_USART2_FIXED_LEN <= 0
        RxCMD_Usart2 = (unsigned char*) realloc(RxCMD_Usart2,DataLen2);     //allocate the right frame size of memory for buffer
        RingBuf_ReadBlock((usart2Control).seqMemRX_u32, RxCMD_Usart2, &DataLen2); //extract the whole frame
#endif // if ENABLE_RXCMD_USART2_FIXED_LEN <= 0
        
        
        switch(RxCMD_Usart2[2])
        {
          
        case BUS_VOLTS_CMD:       //  0x40 // Read bus volts
          {
            uint16_t bus_volts_u16 = RxCMD_Usart2[6]; 
            bus_volts_u16 += (uint16_t) RxCMD_Usart2[5] << 8;
            (usart2Control).busVolts_u16 = bus_volts_u16;
            break;
          }
        case MOTOR_STATUS_CMD:    //  0x41 // Motor status
          {
            uint16_t Mot_Status = RxCMD_Usart2[6]; 
            Mot_Status += (uint16_t) RxCMD_Usart2[5] << 8;
            (usart2Control).motorStatus_u16 = Mot_Status;
            if(Mot_Status != 0)
            {
              LL_GPIO_TogglePin(GPIOC, LED_ONBOARD_Pin); //on board LED output // SPA Reveiw. This function should not be part of module.
            }
            break;
          }
        case MOTOR_DIR_CMD :      //  0x42 // Motor direction
          {
            int16_t motor_dir_s16 = RxCMD_Usart2[6]; 
            motor_dir_s16 += (int16_t) RxCMD_Usart2[5] << 8;
            (usart2Control).motorDir_s16 = motor_dir_s16;
            break;
          }
        case BULK_MONITORING_CMD: // 0x4D
          {
            parseMessage_BulkMonitoring(DataLen2);
            break;
          }
        case HEART_BEAT_CMD:      //  0x4E // Heart beat
          { //this would basically send a packet (with a time delay info (to be specified later)) to motor-side and will be used as a heartbeat
            unsigned char HeartBeatTx[] = {0x55, 0x02, 0x4E, 0x00, 0x00, 0x0F, 0xA0, 0xCC, 0xCC};
            unsigned int HeartBeatLen = sizeof(HeartBeatTx);         
            RingBuf_WriteBlock((usart2Control).seqMemTX_u32, HeartBeatTx, &HeartBeatLen);
            
            // TODO: (RPa) please add the necessary sequence on the app-side if not receiving any valid packet from the motor-side
            // to complete the loop of HeartBeat
            break;
          }
        case ELECTRICAL_INFO_CMD: //  0x4F // Motor volts, phase current & power
          {
            int16_t motor_volts_s16 = RxCMD_Usart2[6]; 
            motor_volts_s16 += (int16_t) RxCMD_Usart2[5] << 8;
            (usart2Control).motorVolts_s16 = motor_volts_s16;
            
            int16_t motor_phase_amps_s16 = RxCMD_Usart2[8]; 
            motor_phase_amps_s16 += (int16_t) RxCMD_Usart2[7] << 8;
            (usart2Control).motorPhaseCurrent_s16 = motor_phase_amps_s16;
            
            int16_t motor_output_power_s16 = RxCMD_Usart2[10]; 
            motor_output_power_s16 += (int16_t) RxCMD_Usart2[9] << 8;
            (usart2Control).motorOutputPower_s16 = motor_output_power_s16;
            
            break;
          }
        case MEASURED_SPEED_CMD:  //  0x60 // Measured speed
          {
            int16_t Mea_speed = RxCMD_Usart2[6];
            Mea_speed += (int16_t) RxCMD_Usart2[5] << 8;
            Mea_speed=abs(Mea_speed);
            (usart2Control).motorSpeed_s16 = Mea_speed;              
            Modbus_UpdateStoredMeasuredSpeed(Mea_speed);
           /* Motor_Avg_Speed_s16+=Mea_speed;
            Motor_Avg_Counter_u8++;
            if(Motor_Avg_Counter_u8>=2){
              Actual_Measured_Speed=Motor_Avg_Speed_s16/Motor_Avg_Counter_u8;
              Modbus_UpdateStoredMeasuredSpeed(Actual_Measured_Speed);
              Motor_Avg_Counter_u8=0;
              Motor_Avg_Speed_s16=0;
            }*/
            
            break;
          }
        case MEASURED_TORQUE_CMD: //  0x61 // Measured Torque
          {
            int16_t motor_torque_s16 = RxCMD_Usart2[6]; 
            motor_torque_s16 += (int16_t) RxCMD_Usart2[5] << 8;
            (usart2Control).motorOutputTorque_s16 = motor_torque_s16;
            break;
          }
        case THERMO_MECHANICAL_INFO_CMD:  // 0x6F // Measured torque & temperature
          {
            int16_t motor_torque_s16 = RxCMD_Usart2[6]; 
            motor_torque_s16 += (int16_t) RxCMD_Usart2[5] << 8;
            (usart2Control).motorOutputTorque_s16 = motor_torque_s16;
            
            int16_t module_temperature_s16 = RxCMD_Usart2[8]; 
            module_temperature_s16 += (int16_t) RxCMD_Usart2[7] << 8;
            (usart2Control).moduleTemperature_s16 = module_temperature_s16;
            
            break;
          }
		case DRIVE_FLASH_READ_CMD: // 0x79
          {
            parseMessage_DriveFlashData(DataLen2);
            break;
          }
		case EEPROM_READ_DATA_CMD: // 0x80
          {
            parseMessage_EEPROMData(DataLen2);
            break;
          }
        case SINGLE_REG_READ_CMD: //  0x07 // Read single regiser
          {
            break;
          }
          //case REG_STATUS_CMD:      //  0x72 // Read register status
        case MULT_DATA_REQ1_CMD:  //  0xB0 // Read multiple commands 1
          {
            break;
          }
        case MULT_DATA_REQ2_CMD:  //  0xB1 // Read multiple commands 2
          {
            break;
          }
          
          /*case 0x60:
          {
          int16_t Mea_speed = RxCMD_Usart2[6];
          Mea_speed += (int16_t) RxCMD_Usart2[5] << 8;
          (*usart2Control).motorSpeed_s16 = Mea_speed;     
          break;
        }
        case 0x41:
          {
          uint16_t Mot_Status = RxCMD_Usart2[6]; 
          Mot_Status += (uint16_t) RxCMD_Usart2[5] << 8;
          (*usart2Control).motorStatus_u16 = Mot_Status;
          if(Mot_Status != 0)
          {
          LL_GPIO_TogglePin(GPIOC, LED_ONBOARD_Pin); //on board LED output // SPA Reveiw. This function should not be part of module.
        }
          break;
        }*/
          
        default:
          break;
        }
      }
#if ENABLE_RXCMD_USART2_FIXED_LEN <= 0
      RxCMD_Usart2 = (unsigned char*) realloc(RxCMD_Usart2,1);   
#endif // ENABLE_RXCMD_USART2_FIXED_LEN <= 0
      
      return_state_u8 = RUN_MODULE;
      break;
    }
  case KILL_MODULE: 
    {
      // The USART2 driver module must only be executed once.
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
********************************************************************************************************************************
* @brief   Assign structured/sequential memory
* @details Assign structured/sequential memory for USAR2 module
* @param   None 
* @return  None
********************************************************************************************************************************
*/
void assign_UART2_ModuleMem(uint8_t drv_id_u8){  
  //System call create a buffer for this driver need to be bigger than 1 complete frame 
  usart2InternalSeqMem_u32 = SeqMem_CreateInstance(MODULE_USART2, TX_RX_BUF_SIZE, ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);
  //System call create a buffer for final packet receiver buffer 
  usart2SeqMemRX_transparentMode_u32 = SeqMem_CreateInstance(MODULE_USART2, TX_RX_BUF_SIZE, ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);
  //System call create a buffer for final packet receiver buffer 
  usart2SeqMemRX_u32 = SeqMem_CreateInstance(MODULE_USART2, TX_RX_BUF_SIZE, ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);
  //System call create a buffer for Tx data
  usart2SeqMemTX_u32 = SeqMem_CreateInstance(MODULE_USART2, TX_RX_BUF_SIZE, ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);      
  
  usart2StructMem_u32 =  StructMem_CreateInstance(MODULE_USART2, sizeof(Usart2_Control), ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);//System call create a structured memory for this driver [should map it back to this driver local struct]
  (*usart2StructMem_u32).p_ramBuf_u8 = (uint8_t *)&usart2Control ;    // Map the usart2Control memory into the structured memory
  uint8_t usart2_index_u8 = getProcessInfoIndex(MODULE_USART2);
  processInfoTable[usart2_index_u8].Sched_DrvData.irqState_u8 = DEFAULT_IRQ_STATE;
  processInfoTable[usart2_index_u8].Sched_DrvData.p_masterSharedMem_u32 = (Ram_Buf_Handle)usart2StructMem_u32;
  
  usart2Control.seqMemRX_transparentMode_u32 = usart2SeqMemRX_transparentMode_u32;
  usart2Control.seqMemTX_u32 = usart2SeqMemTX_u32;
  usart2Control.seqMemRX_u32 = usart2SeqMemRX_u32;
  usart2Control.seqMem_InternalPipe_u32 = usart2InternalSeqMem_u32;
  usart2Control.errorCode_u8 = 0;
  usart2Control.UsartMode_u8 = 0;
}

void parseMessage_BulkMonitoring(unsigned int data_length) { 
  if (data_length < MESSAGE_LENGTH_UP_BULK_MONITORING) {
    return;
  } else {
    test_counter_bulk_monitoring += 1;
    // Received message will be of the following format.
    // uint8_t message[] = {0x55, 14, 0x4D, 0x00, 0x00, 
    //                      0xff, 0xff, 0xff, 0xff, 0xff, // status_u8, faults_u16, bus_voltage_u16
    //                      0xff, 0xff, 0xff, 0xff, 0xff, // direction_u8, speed_i16, torque_i16
    //                      0xff, 0xff, 0xff, 0xff, // power_i16, temperature_i16
    //                      0xCC, 0xCC};// Just send 0x5AA5 to App-side and get a response
    
    uint8_t index = UNIVERSAL_PROTOCOL_DATA_START_INDEX;
    // 
    uint8_t status = RxCMD_Usart2[index++];
    uint16_t faults = RxCMD_Usart2[index++] << 8;
    faults += RxCMD_Usart2[index++];
    uint16_t regal_mc_status_u16 = (uint16_t) RxCMD_Usart2[index++];
    
    uint16_t bus_voltage = RxCMD_Usart2[index++] << 8;
    bus_voltage += RxCMD_Usart2[index++];
    //
    uint8_t direction = RxCMD_Usart2[index++];
    
    int16_t measured_speed = RxCMD_Usart2[index++] << 8;
    measured_speed += RxCMD_Usart2[index++];
    measured_speed=abs(measured_speed);
    // Measured Torque
    int16_t torque = RxCMD_Usart2[index++] << 8;
    torque += RxCMD_Usart2[index++];
    //
    int16_t power = RxCMD_Usart2[index++] << 8;
    power += RxCMD_Usart2[index++];
    
    int16_t temperature = RxCMD_Usart2[index++] << 8;
    temperature += RxCMD_Usart2[index++]; 
    
     // Measured phase peak current Ia
        int16_t ia_s16 = RxCMD_Usart2[index++] << 8;
        ia_s16 += RxCMD_Usart2[index++];
        // Measured phase peak current Ib
        int16_t ib_s16 = RxCMD_Usart2[index++] << 8;
        ib_s16 += RxCMD_Usart2[index++];

        // Phase currents Ia and Ib
        int16_t current_ia_s16 = RxCMD_Usart2[index++] << 8;
        current_ia_s16 += RxCMD_Usart2[index++];
        int16_t current_ib_s16 = RxCMD_Usart2[index++] << 8;
        current_ib_s16 += RxCMD_Usart2[index++];


    
    // Send Data to Motor Parameter Table (used by modbus)
    // - TODO: Motor State Stuff should go in it's own module, so that both Universal Protocol, Bluetooth, and Modbus can use it
    Modbus_UpdateStoredStatus(status);
    Modbus_UpdateStoredFaults(faults);
     Modbus_UpdateStoredRegalMCStatus(regal_mc_status_u16);
    Modbus_UpdateStoredBusVoltage(bus_voltage);
    //
    Modbus_UpdateStoredDirection(direction);
    Modbus_UpdateStoredMeasuredSpeed(measured_speed);
    Modbus_UpdateStoredTorque(torque);
    //
    Modbus_UpdateStoredPower(power);
    Modbus_UpdateStoredTemperature(temperature);
    
    Modbus_UpdateStoredPhaseCurrentIa(current_ia_s16);
    Modbus_UpdateStoredPhaseCurrentIb(current_ib_s16);
    
  }
}

void parseMessage_DriveFlashData(unsigned int data_length) {     
    uint8_t index_u8 = UNIVERSAL_PROTOCOL_DATA_START_INDEX;
    uint8_t *data_address_pu8 = &(RxCMD_Usart2[index_u8]);
    
    // Send Data to Modbus Module to process Modbus response
	Modbus_PassDriveFlashData(data_address_pu8, data_length);
} 

void parseMessage_EEPROMData(unsigned int data_length) {     
    uint8_t index_u8 = UNIVERSAL_PROTOCOL_DATA_START_INDEX;
    uint8_t *data_address_pu8 = &(RxCMD_Usart2[index_u8]);
    
    // Send Data to Modbus Module to process Modbus response
	Modbus_PassEEPROMData(data_address_pu8, data_length);
}