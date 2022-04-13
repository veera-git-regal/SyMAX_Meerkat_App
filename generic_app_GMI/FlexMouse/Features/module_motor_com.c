/**
***************************************************************************************************
* @file    module_motor_com.c 
* @author  Regal Pamela Lee
* @version V1.0
* @date    7-Jul-2020
* @brief   Main function/s of control motor speed by 0 to 10V
* @note   
***************************************************************************************************
*/

// Includes -------------------------------------------------------------------
#include "module_motor_com.h"
#include "driver_usart2.h"
#include "module_usart2.h"
//#include "module_analog_0_10v.h"
#include "module_motor_demand_multiplexer.h"
#include "module_gpio.h"
#include "module_digital_inputs.h"
#include "macros.h"

// Content --------------------------------------------------------------------
// TODO: Move 

// - Function Prototypes
void init_Motor_Setting(void);
void assign_Module_Motor_Com_Mem();
extern uint64_t getSysCount(void);
void init_Motor_Data(void);
void send_Periodic_Data_Request(uint8_t);

// - Constants
#define DemandPollPeriod 100         //time period for checking and sending 0-10V and speed data to motor board
#define	HarmonicInitialPollPeriod 200		 // time for updating the Harmonic Injection data
#define TIME_BETWEEN_STAGGERED_HI_TX_BLOCKS 200	// time between staggered blocks of UP messages being transmitted for harmonic injection

// Application Constants
// #define MIN_COMMANDABLE_SPEED	300  // RPM
// #define MAX_COMMANDABLE_SPEED	2500 //MAX_APP_SPEED_RPM 
// #define ADC12B  4096
// #define motorOnThreadhold       10   // percentage of 0 to 10V for turn on Motor
// #define motorOffThreadhold       5   // percentage of 0 to 10V for turn on Motor
// #define MotorMaxLimPercent      90   // Max motor speed in percentage 90%
#define STARTUP_STATE_DELAY     2000 // Delay before state machine enters "startMotor" state
//#define SpeedRatePerADCValue   (MAX_COMMANDABLE_SPEED - MIN_COMMANDABLE_SPEED) / ((MotorMaxLimPercent *100) - (MIN_COMMANDABLE_SPEED * 1000))

// -- Module States
enum                                                                            //Default APPs/Driver stage template
{ 
  MEMORY_INIT_MODULE,
  AppInit,
  AppStart,
  //any other stage in here !!!
  startMotor,
  slowDnMotor2Stop,
  stopMotor,
  waitForIdle,
  WaitReset,
  SpdUpdate,
  HarmonicUpdate,
  
  //above 200 will be all interrupt for this APP
  AppIrq = 200,
  killApp = 255
};

// - External Variables
extern Ram_Buf sharedMemArray[STRUCT_MEM_ARRAY_SIZE];
extern ProcessInfo processInfoTable[];
static Ram_Buf_Handle motor_Com_StructMem_u32;

extern int16_t  hi_amplitude_s16[8]; 		
extern uint8_t  hi_angle_multiplier_hi_u8[8];
extern uint16_t hi_angle_offset_u16[8]; 		
extern uint8_t  hi_is_phase_inverted_u8[8]; 	
extern uint16_t hi_min_speed_u16[8]; 		
extern uint16_t hi_max_speed_u16[8]; 		
extern uint8_t  hi_is_harmonic_injection_allowed_hi_u8; 

// - Global variables specific to this module
uint64_t tt_DemandTime;        //time tick storage for 0-10 V 
uint64_t tt_HarmonicUpdateTime;	// Time to send Harmnic Injection data
//bool isMotorOn = FALSE;
uint64_t tt_StopMotorResume;   //time tick storage motor stop resume period 
unsigned char readMotorDataCommands[] = {BUS_VOLTS_CMD, MOTOR_STATUS_CMD, MOTOR_DIR_CMD, ELECTRICAL_INFO_CMD, MEASURED_SPEED_CMD, THERMO_MECHANICAL_INFO_CMD,BULK_MONITORING_CMD};
uint16_t readMotorDataTimeInterval_u16[] = {100, 100, 100, 100, 100, 100,100};
uint8_t index_read_motor_data_command = 0;
uint8_t motor_data_commands_len_u8 = sizeof(readMotorDataTimeInterval_u16)/( sizeof(readMotorDataTimeInterval_u16[0]) );
//uint8_t local_motor_dir_u8 = CCW;
// Harmonic Injection Variables
/*uint8_t	 hi_enable_u8 = 0;
int16_t  hi_amplitude_s16[8] 			= {0,0,0,0,0,0,0,0}; 
uint8_t  hi_angle_multiplier_u8[8] 	    = {0,0,0,0,0,0,0,0};
uint16_t hi_angle_offset_u16[8] 		= {0,0,0,0,0,0,0,0};
uint8_t  hi_is_phase_inverted_u8[8] 	= {0,0,0,0,0,0,0,0};
uint16_t hi_min_speed_u16[8] 			= {0,0,0,0,0,0,0,0};
uint16_t hi_max_speed_u16[8] 			= {2250, 2250, 2250, 2250, 2250, 2250, 2250, 2250}; 
*/
// -- Define Pointers that will be used as References to other Modules, where applicable
Usart2_Control* usart2Control_AppLocal;
Motor_Com_Control motor_Com_Control; // Main Structure of Settings and Data
MotorDemandMux_Control *motorDemandMuxControl_ptr;
DigitalInputs_Control* digitalInputControl_motorCom_ptr;

     

/**
********************************************************************************************************************************
* @brief   State machine for Motor Com module
* @details
* @param   drv_identifier_u8, previous_state_u8, next_stat_u8, irq_identfier_u8
* @retval  return_state_u8
********************************************************************************************************************************
*/ 
//^**Tips: APPs/Drivers adding process example step7 (Add the Additional funtion itself)
uint8_t module_Motor_Com_u32(uint8_t drv_id_u8, uint8_t prev_state_u8, uint8_t next_state_u8,
                             uint8_t irq_id_u8)
{ 
  uint8_t  return_state_u8 = MEMORY_INIT_MODULE; 
  uint16_t output_demand = 0; 
  static uint8_t blocks_to_transmit_u8 = 0,harmonic_data_cnt=0;
	  uint8_t index_u8 = 0;

  
  switch (next_state_u8)
  {
  case MEMORY_INIT_MODULE:
    {
      assign_Module_Motor_Com_Mem(); // Assign structured memory to Analog 0-10V setting and data 
      return_state_u8 = AppInit;
      break;
    }
  case AppInit:                                                              //initial stage
    {   
      //assign_Module_Motor_Com_Mem(); //Assign structured memory
      init_Motor_Setting(); //initilize motor settings
      init_Motor_Data(); // Init live motor data
      /*Attach Uart2 structured memory into this App*/         
      uint8_t Usart2index  = getProcessInfoIndex(MODULE_USART2); //return Process index from processInfo array
      usart2Control_AppLocal = (Usart2_Control*)((*(processInfoTable[Usart2index].Sched_DrvData.p_masterSharedMem_u32)).p_ramBuf_u8);    //Get structured memory for USART2
      
      // Attached digital input structure memory
      uint8_t digtial_input_index_u8  = getProcessInfoIndex(MODULE_DIGITAL_INPUTS); //return Process index from processInfo array
      digitalInputControl_motorCom_ptr = (DigitalInputs_Control*)((*(processInfoTable[digtial_input_index_u8].Sched_DrvData.p_masterSharedMem_u32)).p_ramBuf_u8);    //Get structured memory for USART2
      
      /*Attach Motor Demand Multiplexer module structured memory into this App*/     
      uint8_t module_index = getProcessInfoIndex(MODULE_MOTOR_DEMAND_MUX);   //return Process index from processInfo array
      motorDemandMuxControl_ptr = (MotorDemandMux_Control*)((*(processInfoTable[module_index].Sched_DrvData.p_masterSharedMem_u32)).p_ramBuf_u8);
      
      output_demand = (*motorDemandMuxControl_ptr).motorDemandMux_Data.demandValue_u16;
      
      tt_DemandTime = getSysCount() + DemandPollPeriod;                          //store time tick value 
      tt_HarmonicUpdateTime = getSysCount() + HarmonicInitialPollPeriod;		// store time tick value for Harmonic message updates
      return_state_u8 = AppStart;//startMotor;// stopMotor; 
      break;
    }       
  case AppStart:
    { 
      if(index_read_motor_data_command < motor_data_commands_len_u8)
      {
     // send_Periodic_Data_Request(index_read_motor_data_command); // Send UP message to get motor data periodically
      }
      
      if (getSysCount() >= tt_DemandTime) 
      {
        motor_Com_Control.motor_Metering_Data.demand_Output_Percent_u16 = (uint16_t)CALCULATE_PERCENT( abs((*usart2Control_AppLocal).motorSpeed_s16) , (motor_Com_Control.motor_Setting.max_Speed_u16))*100; //xxx.yy% = XXXYY format
        output_demand = (*motorDemandMuxControl_ptr).motorDemandMux_Data.demandValue_u16; // Percent reference demand
        motor_Com_Control.motor_Metering_Data.demand_Reference_Percent_u16 = output_demand;
        (motor_Com_Control.motor_Metering_Data.motor_Status_u16) = (*usart2Control_AppLocal).motorStatus_u16;
        uint32_t temp_result = (uint32_t)(output_demand * (motor_Com_Control.motor_Setting.max_Speed_u16));
        motor_Com_Control.motor_Metering_Data.demand_Reference_Speed_u16 = (uint16_t)(temp_result/(float)10000); //Convert Speed ref % to RPM
        
        if ( (!motor_Com_Control.motor_Metering_Data.discretes_u16.is_Motor_On) ) {
      //    if(getSysCount() >= tt_DemandTime + STARTUP_STATE_DELAY)
      //    { // Ensures digital inputs states are updated before starting the motor
            if(motor_Com_Control.motor_Metering_Data.demand_Reference_Speed_u16 >= motor_Com_Control.motor_Setting.min_Speed_u16) //Demand > min speed
            {
              return_state_u8 = startMotor;
              break;
            }
     //     }
        }else{
          if(motor_Com_Control.motor_Metering_Data.demand_Reference_Speed_u16 < (motor_Com_Control.motor_Setting.min_Speed_u16 - motor_Com_Control.motor_Setting.hysteresis_Speed_u16)) //Demand > min speed
          {             
            return_state_u8 = slowDnMotor2Stop;
            break;
          }
        }
           // change state machine to update Harmonic messages
	/*  if (getSysCount() >= tt_HarmonicUpdateTime) {
		return_state_u8 = HarmonicUpdate;
		break;
	  }*/
        //if (getSysCount() >= tt_HarmonicUpdateTime) {
		//return_state_u8 = HarmonicUpdate;
          //index_read_motor_data_command=4;//4;
          //send_Periodic_Data_Request(index_read_motor_data_command); // Send UP message to get motor data periodically
//tt_HarmonicUpdateTime = getSysCount() + TIME_BETWEEN_STAGGERED_HI_TX_BLOCKS;

		//break;
//	  }
        
        return_state_u8 = SpdUpdate;
        break;             
      } 
		  
      return_state_u8 = AppStart;
      break;
    }
  case SpdUpdate:
    {
      // Send update speed command to motor control
      unsigned char speedTx[] = {0x55, 0x03, 0x21, 0x00, 0x00, 0xff, 0xff, 0xff, 0xCC, 0xCC};
      
      unsigned int speedLen = sizeof(speedTx);
      speedTx[5] = (unsigned char) (((uint16_t) (motor_Com_Control.motor_Metering_Data.demand_Reference_Speed_u16) & 0xff00) >> 8);
      speedTx[6] = (unsigned char) (motor_Com_Control.motor_Metering_Data.demand_Reference_Speed_u16) & 0xff;
      speedTx[7] = (unsigned char) (motor_Com_Control.motor_Metering_Data.motor_Actual_Direction_u16);
      
      if(motor_Com_Control.motor_Metering_Data.discretes_u16.is_Motor_On)
      {
        RingBuf_WriteBlock((*usart2Control_AppLocal).seqMemTX_u32, speedTx, &speedLen);
      }
      
      tt_DemandTime = getSysCount() + DemandPollPeriod;                          //update next time tick value 
	  
      return_state_u8 = AppStart;
      break;
    }
  case HarmonicUpdate:
	{

	  if(blocks_to_transmit_u8 == 0)
	  {
		
            // Update Harmonic Injection and send to motor control
		// Enable/Disable
            if(harmonic_data_cnt==0)
            {
		unsigned char harmonicEnableTx[8] = {0x55, 0x01, 0x30, 0x00, 0x00, 0xff, 0xCC, 0xCC};
		unsigned int harmonicEnableTxLen = 8;//sizeof(harmonicEnableTx);
              harmonicEnableTx[5] = (unsigned char) (hi_is_harmonic_injection_allowed_hi_u8);
		RingBuf_WriteBlock((*usart2Control_AppLocal).seqMemTX_u32, harmonicEnableTx, &harmonicEnableTxLen);
                harmonic_data_cnt++;
                return_state_u8 = AppStart;
                break;
            }
              
            
            if(harmonic_data_cnt==1)
            {
		// Amplitudes
		unsigned char harmonicAmplitudessTx[23] = {0x55, 0x10, 0x31, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                                                         0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xCC, 0xCC};
		unsigned int harmonicAmplitudesTxLen = 23;//sizeof(harmonicAmplitudessTx);
		unsigned char har_amp_index=5;
                
                for(index_u8 = 0; index_u8 < 8; index_u8++) {
                  
		  harmonicAmplitudessTx[har_amp_index] = (unsigned char) (((hi_amplitude_s16[index_u8]) & 0xff00) >> 8);
                  har_amp_index++;
		  harmonicAmplitudessTx[har_amp_index] = (unsigned char) (((hi_amplitude_s16[index_u8]) & 0x00ff) >> 0);
                  har_amp_index++;
		}	
                
                               
		RingBuf_WriteBlock((*usart2Control_AppLocal).seqMemTX_u32, harmonicAmplitudessTx, &harmonicAmplitudesTxLen);
		harmonic_data_cnt++;
            return_state_u8 = AppStart;
                break;
          }
            
		// Angle Multipliers
            if(harmonic_data_cnt==2){
		unsigned char harmonicMultipliersTx[15] = {0x55, 0x08, 0x32, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                                                         0xff, 0xff, 0xCC, 0xCC};
		unsigned int harmonicMultipliersTxLen = 15;//sizeof(harmonicMultipliersTx);
		for(index_u8 = 0; index_u8 < 8; index_u8++) {
		  harmonicMultipliersTx[5+index_u8] = (unsigned char) (hi_angle_multiplier_hi_u8[index_u8]);
		}
		RingBuf_WriteBlock((*usart2Control_AppLocal).seqMemTX_u32, harmonicMultipliersTx, &harmonicMultipliersTxLen);
	  
		blocks_to_transmit_u8++;	// increment counter to next transmit block
		tt_HarmonicUpdateTime = getSysCount() + TIME_BETWEEN_STAGGERED_HI_TX_BLOCKS;
      	return_state_u8 = AppStart;  // revert back to AppStart state
                harmonic_data_cnt++;
            
	  	break;	 
          }
	  }
	  
	  if(blocks_to_transmit_u8 == 1)
	  {
            if(harmonic_data_cnt==3)
            {
		// Angle Offsets
		unsigned char harmonicOffsetsTx[23] = {0x55, 0x10, 0x33, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
                                                    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xCC, 0xCC};
		unsigned int harmonicOffsetsTxLen = 23;// sizeof(harmonicOffsetsTx);
                unsigned char har_offset_index=5;
                
		for(index_u8 = 0; index_u8 < 8; index_u8++)
                {
		  harmonicOffsetsTx[har_offset_index] = (unsigned char) (((hi_angle_offset_u16[index_u8]) & 0xff00) >> 8);
		 har_offset_index++;
                  harmonicOffsetsTx[har_offset_index] = (unsigned char) (((hi_angle_offset_u16[index_u8]) & 0x00ff) >> 0);
		har_offset_index++;
                }	  
		RingBuf_WriteBlock((*usart2Control_AppLocal).seqMemTX_u32, harmonicOffsetsTx, &harmonicOffsetsTxLen);
                harmonic_data_cnt++;
                return_state_u8 = AppStart;
	         break;
            }
		// Inversions
            if(harmonic_data_cnt==4)
            {
		unsigned char harmonicInversionsTx[15] = {0x55, 0x08, 0x34, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 
                                                        0xff, 0xff, 0xff, 0xCC, 0xCC};
		unsigned int harmonicInversionsTxLen = 15;//sizeof(harmonicInversionsTx);
		for(index_u8 = 0; index_u8 < 8; index_u8++) {
		  harmonicInversionsTx[5+index_u8] = (unsigned char) (hi_is_phase_inverted_u8[index_u8]);
		}
		RingBuf_WriteBlock((*usart2Control_AppLocal).seqMemTX_u32, harmonicInversionsTx, &harmonicInversionsTxLen);
                harmonic_data_cnt++;
		
            }
		blocks_to_transmit_u8++;	// increment couner to next transmit block
		//blocks_to_transmit_u8 = 0;
		tt_HarmonicUpdateTime = getSysCount() + TIME_BETWEEN_STAGGERED_HI_TX_BLOCKS;
      	return_state_u8 = AppStart;  // revert back to AppStart state
	  	break;	 
	  }
  
	  if(blocks_to_transmit_u8 == 2)
	  {
            if(harmonic_data_cnt==5)
            {
		// Minimum Speeds
		unsigned char harmonicMinSpeedsTx[23] = {0x55, 0x10, 0x35, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
                                                       0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xCC, 0xCC};
		unsigned int harmonicMinSpeedsTxLen = 23;//sizeof(harmonicMinSpeedsTx);
		unsigned char harmonic_minspeed_index=5;
                for(index_u8 = 0; index_u8 < 8; index_u8++) {
		  harmonicMinSpeedsTx[harmonic_minspeed_index] = (unsigned char) (((hi_min_speed_u16[index_u8]) & 0xff00) >> 8);
		  harmonic_minspeed_index++;
                  harmonicMinSpeedsTx[harmonic_minspeed_index] = (unsigned char) (((hi_min_speed_u16[index_u8]) & 0x00ff) >> 0);
		  harmonic_minspeed_index++;
                }	  
		RingBuf_WriteBlock((*usart2Control_AppLocal).seqMemTX_u32, harmonicMinSpeedsTx, &harmonicMinSpeedsTxLen);
	        harmonic_data_cnt++;
                return_state_u8 = AppStart;
                break;
            }
		if(harmonic_data_cnt==6)
                {
            // Maximum Speeds
		unsigned char harmonicMaxSpeedsTx[23] = {0x55, 0x10, 0x36, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
                                                       0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xCC, 0xCC};
		unsigned int harmonicMaxSpeedsTxLen = 23;//sizeof(harmonicMaxSpeedsTx);
                unsigned char harmonic_maxspeed_index=5;
		
                for(index_u8 = 0; index_u8 < 8; index_u8++) {
		  harmonicMaxSpeedsTx[harmonic_maxspeed_index] = (unsigned char) (((hi_max_speed_u16[index_u8]) & 0xff00) >> 8);
		  harmonic_maxspeed_index++;
                  harmonicMaxSpeedsTx[harmonic_maxspeed_index] = (unsigned char) (((hi_max_speed_u16[index_u8]) & 0x00ff) >> 0);
		 harmonic_maxspeed_index++;
                }	  
		RingBuf_WriteBlock((*usart2Control_AppLocal).seqMemTX_u32, harmonicMaxSpeedsTx, &harmonicMaxSpeedsTxLen);
		harmonic_data_cnt=0;
                }
		blocks_to_transmit_u8 = 0;	// reset block counter back to beginning
	  }
	  
	  tt_HarmonicUpdateTime = getSysCount() + TIME_BETWEEN_STAGGERED_HI_TX_BLOCKS;      
         
          return_state_u8 = AppStart;  // revert back to AppStart state
	  break;  
	}
  case startMotor:
    { // To start motor send SetSpeed command
      // No longer need to send start command to start motor  
      
      // Set direction of rotation based on digital inputs
      // - TODO: Replace this section with MotorDemandMux_GetDirection()
      if((*digitalInputControl_motorCom_ptr).digitalInputs_Data.discretes_u16.is_invertDirection)
      {
        if(motor_Com_Control.motor_Setting.motor_Direction_u16 == CCW) 
        {
          motor_Com_Control.motor_Metering_Data.motor_Actual_Direction_u16= CW;
        } else
        {
          motor_Com_Control.motor_Metering_Data.motor_Actual_Direction_u16 = CCW;
        }
      } else
      {
        motor_Com_Control.motor_Metering_Data.motor_Actual_Direction_u16 = motor_Com_Control.motor_Setting.motor_Direction_u16;
      }
      
      motor_Com_Control.motor_Metering_Data.discretes_u16.is_Motor_On = TRUE;
      return_state_u8 = AppStart;
      //send_Periodic_Data_Request(); // Send UP message to get motor data periodically
      tt_DemandTime = getSysCount() + DemandPollPeriod; // REVIEW: Keeps frames from getting sandwiched together, not necessary in this case
      break;
    }
  case slowDnMotor2Stop:
    { // Slow down the motor to stop
      //LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_6); //SPA
      unsigned char speedTx[] = {0x55, 0x03, 0x21, 0x00, 0x00, 0xff, 0xff, 0xff, 0xCC, 0xCC};
      unsigned int speedLen = sizeof(speedTx);
      //put 200 rpm lower speed 
      speedTx[5] = (unsigned char) (((uint16_t)(motor_Com_Control.motor_Setting.min_Speed_u16) & 0xff00) >> 8);
      speedTx[6] = (unsigned char) (motor_Com_Control.motor_Setting.min_Speed_u16) & 0xff;  
      speedTx[7] = (unsigned char) (motor_Com_Control.motor_Metering_Data.motor_Actual_Direction_u16);
      
      if(motor_Com_Control.motor_Metering_Data.discretes_u16.is_Motor_On)
      {
        RingBuf_WriteBlock((*usart2Control_AppLocal).seqMemTX_u32, speedTx, &speedLen);
      }
      return_state_u8 = stopMotor;
      break;
    }       
  case stopMotor: 
    {   
      unsigned int speedLen;
      if(!(motor_Com_Control.motor_Metering_Data.discretes_u16.is_Motor_On) || (motor_Com_Control.motor_Metering_Data.motor_Status_u16 != 0)) //motor is off already or any fault happen
      { // in unknow situation
        //LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_6); //SPA
        unsigned char speedTx[] = {0x55, 0x03, 0x21, 0x00, 0x00, 0x00, 0x00, 0xff, 0xCC, 0xCC}; // Send speed set point of "0"
        speedLen = sizeof(speedTx);
        speedTx[7] = (unsigned char) (motor_Com_Control.motor_Metering_Data.motor_Actual_Direction_u16);
        RingBuf_WriteBlock((*usart2Control_AppLocal).seqMemTX_u32, speedTx, &speedLen);
        //unsigned char speedTx1[] = {0x55, 0x00, 0x01, 0x00, 0x00, 0xCC, 0xCC};
        //speedLen = sizeof(speedTx1);
        //RingBuf_WriteBlock((*usart2Control_AppLocal).seqMemTX_u32, speedTx1, &speedLen); //send stop command  
        tt_StopMotorResume = getSysCount() + MotorStopResumePeriodMax;                 //set wait time delay to max
        return_state_u8 = waitForIdle;
        break;
      }
      else
      { // Ramp down speed to stop the motor
        if((*usart2Control_AppLocal).motorSpeed_s16 <= ((motor_Com_Control.motor_Setting.min_Speed_u16) + 10) )   //keep track of slow down
        { //issue final stop command and set time delay
          //unsigned char speedTx1[] = {0x55, 0x00, 0x01, 0x00, 0x00, 0xCC, 0xCC}; // Send speed of "0" instead
          unsigned char speedTx1[] = {0x55, 0x03, 0x21, 0x00, 0x00, 0x00, 0x00, 0xff, 0xCC, 0xCC}; // Send speed set point of "0"
          speedLen = sizeof(speedTx1);
          speedTx1[7] = (unsigned char) (motor_Com_Control.motor_Metering_Data.motor_Actual_Direction_u16);
          RingBuf_WriteBlock((*usart2Control_AppLocal).seqMemTX_u32, speedTx1, &speedLen);
          //RBWriteBlk((*usart2Control_AppLocal).TxPipe->SystemIndex, speedTx1, &speedLen);  //send stop command 
          tt_StopMotorResume = getSysCount() + MotorStopResumePeriod;                 //set normal wait time delay
          
          return_state_u8 = waitForIdle;
          break;
        }
        else
        {
          //if user turn back on before motor speed drop below abs min rpm
          if(motor_Com_Control.motor_Metering_Data.demand_Reference_Speed_u16 >= motor_Com_Control.motor_Setting.min_Speed_u16)
          {
            //LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_6); //SPA
            return_state_u8 = SpdUpdate;
            break; 
          }            
        }
      }
      
      return_state_u8 = stopMotor; 
      break;
    }
  case waitForIdle:
    {
      if(getSysCount() >= tt_StopMotorResume)  //wait for the motor stop period before restart                        
      {
        // No longer need fault ack message
        //unsigned char speedTx[] = {0x55, 0x00, 0x03, 0x00, 0x00, 0xCC, 0xCC}; //Send faluty ack
        //unsigned int speedLen = sizeof(speedTx);
        //RingBuf_WriteBlock((*usart2Control_AppLocal).seqMemTX_u32, speedTx, &speedLen);
        tt_StopMotorResume = 0;
        motor_Com_Control.motor_Metering_Data.discretes_u16.is_Motor_On = FALSE;
        
        return_state_u8 = WaitReset;       
        break;
      } 
      return_state_u8 = waitForIdle;
      break;  
    }
  case WaitReset:
    { 
      //output_demand = (*motorDemandMuxControl_ptr).motorDemandMux_Data.demandValue_u16;
      //(motor_Com_Control.motor_Metering_Data.motor_Status_u16) = (*usart2Control_AppLocal).motorStatus_u16;
      //uint32_t temp_result = (uint32_t)(output_demand * (motor_Com_Control.motor_Setting.max_Speed_u16));
      //motor_Com_Control.motor_Metering_Data.demand_Reference_Speed_u16 = (uint16_t)(temp_result/(float)10000); //Convert Speed ref % to RPM
      
      if( (motor_Com_Control.motor_Metering_Data.demand_Reference_Speed_u16) < (motor_Com_Control.motor_Setting.min_Speed_u16))
      {
        //LL_GPIO_ResetOutputPin(GPIOC, LED_ONBOARD_Pin); //on board LED output //SPA REVIEW we cant use this function here
        return_state_u8 = AppStart;
        break;
      }
      return_state_u8 = WaitReset;
      break;
    }
  case AppIrq:
    {
      break;
    }               
  case killApp:
    {
      return_state_u8 = AppInit;
      break;
    }
  default:
    {
      return_state_u8 = killApp;   
      break;
    }
  }
  return return_state_u8;
}

/**
********************************************************************************************************************************
* @brief   Assign structured memory
* @details Assign structured memory for motor_Com_Control
* @param   None 
* @return  None
********************************************************************************************************************************
*/
void assign_Module_Motor_Com_Mem(){  
  motor_Com_StructMem_u32 =  StructMem_CreateInstance(MODULE_MOTOR_COM, sizeof(Motor_Com_Control), ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);//System call create a structured memory for this driver [should map it back to this driver local struct]
  (*motor_Com_StructMem_u32).p_ramBuf_u8 = (uint8_t *)&motor_Com_Control ;    // Map the motor_Com_Control memory into the structured memory
  uint8_t module_motor_com_index_u8 = getProcessInfoIndex(MODULE_MOTOR_COM);
  processInfoTable[module_motor_com_index_u8].Sched_ModuleData.p_masterSharedMem_u32 = (Ram_Buf_Handle)motor_Com_StructMem_u32;
}

/**
********************************************************************************************************************************
* @brief   Initilize all settings
* @details Read settings from the RAM and initilize the settings.
* @param   None 
* @retval  None
********************************************************************************************************************************
*/
void init_Motor_Setting(){
  motor_Com_Control.motor_Setting.control_Mode_u16 = 0;      //00 = Speed 01 = torque;
  motor_Com_Control.motor_Setting.comm_Address_u16 = 55;     //address for univeral protocol
#if HARDWARE_VERSION == HARDWARE_VERSION_BULLRUNNER
  motor_Com_Control.motor_Setting.max_Speed_u16 = 9800;      //Maximum allowed speed in RPM 0dp
  motor_Com_Control.motor_Setting.min_Speed_u16 = 1000;       //minumum speed in RPM 0dp
  motor_Com_Control.motor_Setting.hysteresis_Speed_u16 = 0; //hysteresis speed in RPM 0dp  
  motor_Com_Control.motor_Setting.max_Torque_u16 = 10;        //Maximum allowed torque
  motor_Com_Control.motor_Setting.min_Torque_u16 = 0;       //Minmium torque Nm 
#elif HARDWARE_VERSION == HARDWARE_VERSION_1p3KW
  motor_Com_Control.motor_Setting.max_Speed_u16 = 1800;      //Maximum allowed speed in RPM 0dp
  motor_Com_Control.motor_Setting.min_Speed_u16 = 300;       //minumum speed in RPM 0dp
  motor_Com_Control.motor_Setting.hysteresis_Speed_u16 = 0; //hysteresis speed in RPM 0dp  
  motor_Com_Control.motor_Setting.max_Torque_u16 = 10;        //Maximum allowed torque
  motor_Com_Control.motor_Setting.min_Torque_u16 = 0;       //Minmium torque Nm
  motor_Com_Control.motor_Setting.motor_Direction_u16 = CCW;  // Default motor direction
  motor_Com_Control.motor_Metering_Data.motor_Actual_Direction_u16 = motor_Com_Control.motor_Setting.motor_Direction_u16;
#else // if HARDWARE_VERSION == HARDWARE_VERSION_4KW
  motor_Com_Control.motor_Setting.max_Speed_u16 = 1800;      //Maximum allowed speed in RPM 0dp
  motor_Com_Control.motor_Setting.min_Speed_u16 = 300;       //minumum speed in RPM 0dp
  motor_Com_Control.motor_Setting.hysteresis_Speed_u16 = 0; //hysteresis speed in RPM 0dp  
  motor_Com_Control.motor_Setting.max_Torque_u16 = 0;        //Maximum allowed torque
  motor_Com_Control.motor_Setting.min_Torque_u16 = 10;       //Minmium torque Nm
  // Changed default dircion to clockwise
  motor_Com_Control.motor_Setting.motor_Direction_u16 = CCW;  // Default motor direction
  motor_Com_Control.motor_Metering_Data.motor_Actual_Direction_u16 = motor_Com_Control.motor_Setting.motor_Direction_u16;  
#endif
}

/**
********************************************************************************************************************************
* @brief   Initilize all live data
* @details 
* @param   None 
* @retval  None
********************************************************************************************************************************
*/
void init_Motor_Data()
{
  motor_Com_Control.motor_Metering_Data.demand_Output_Percent_u16=0;
  motor_Com_Control.motor_Metering_Data.demand_Output_Speed_u16 = 0;
  motor_Com_Control.motor_Metering_Data.demand_Output_Torque_u16 = 0;
  motor_Com_Control.motor_Metering_Data.demand_Reference_Percent_u16 = 0;
  motor_Com_Control.motor_Metering_Data.demand_Reference_Speed_u16 = 0;
  motor_Com_Control.motor_Metering_Data.demand_Reference_Torque_u16 = 0;
  motor_Com_Control.motor_Metering_Data.motor_alarms_u32 = 0;
  motor_Com_Control.motor_Metering_Data.motor_Status_u16 = 0;
  motor_Com_Control.motor_Metering_Data.discretes_u16.is_Motor_On = FALSE;
}

/**
********************************************************************************************************************************
* @brief   Initilize all digital input settings and live data
* @details Read settings from the RAM and initilize the settings. This function need run in order to update the input function/mode.
* @param   (uint16_t)demand_percent_f in xxxyy formare which is equal to xxx.yy%
* @retval  None
********************************************************************************************************************************
*/
uint16_t MotorCom_ConvertSpeedToDemandPercentage(uint16_t speed_rpm) {
  uint16_t min_speed = motor_Com_Control.motor_Setting.min_Speed_u16;
  uint16_t max_speed = motor_Com_Control.motor_Setting.max_Speed_u16;
  //uint16_t out_speed = 0;
  if (speed_rpm <= 0) {
    return 0;
  } else if (speed_rpm < min_speed) {
    return 1; // TODO: magic number 1=min demand percentage
  } else if (speed_rpm > max_speed) {
    return 10000; // TODO: magic number 10000=max demand percentage
  } else {
    //uint16_t speed_shifted = speed_rpm - min_speed;
    //uint16_t speed_range = max_speed - min_speed;
    //float demand_percent_f = (speed_shifted/(float)speed_range)*10000; // TODO magic number 10000=max demand percentage
    float demand_percent_f = (speed_rpm/(float)max_speed)*10000; // TODO magic number 10000=max demand percentage
    return ((uint16_t)demand_percent_f); 
  }
  
  
  // if (demand_percentage > 0) {
  //   float speed_calc_f = ((float)(demand_percentage)/10000) * (max_speed - min_speed);
  //   out_speed = min_speed + (demand_percentage/10000)/()
  // } // else {}
  // return out_speed;
}


/**
********************************************************************************************************************************
* @brief   Send Universal Protocol messages to read motor data periodically
* @details readMotorDataCommands and readMotorDataTimeInterval_u16 contains the commands and the time interval(mSec)
* @param   None
* @retval  None
********************************************************************************************************************************
*/
void send_Periodic_Data_Request(uint8_t cmd_index_u8)
{
  
  unsigned char readMotorDataCmdTx[] = {0x55, 0x02, 0x40, 0x00, 0x00, 0xff, 0xff, 0xCC, 0xCC}; // Send speed set point of "0"
  unsigned int read_motor_data_cmd_len_u8 = sizeof(readMotorDataCmdTx);

  //uint8_t motor_data_commands_len_u8 = sizeof(readMotorDataTimeInterval_u16);
  //for(uint8_t index_u8 = 0; index_u8 < motor_data_commands_len_u8; index_u8++)
  //{
  readMotorDataCmdTx[2] = (unsigned char) ((uint8_t)readMotorDataCommands[cmd_index_u8]);
  readMotorDataCmdTx[5] = (unsigned char) (readMotorDataTimeInterval_u16[cmd_index_u8]>>8);
  readMotorDataCmdTx[6] = (unsigned char) readMotorDataTimeInterval_u16[cmd_index_u8];
  RingBuf_WriteBlock((*usart2Control_AppLocal).seqMemTX_u32, readMotorDataCmdTx, &read_motor_data_cmd_len_u8);
  //} 
  index_read_motor_data_command++;
  if(index_read_motor_data_command>=7){index_read_motor_data_command=0;}
}

// uint16_t MotorCom_GetMaxSpeed(void) {
//   uint16_t speed_value = motor_Com_Control.motor_Setting.max_Speed_u16;
// }
// uint16_t MotorCom_GetMinSpeed(void) {
//   uint16_t speed_value = motor_Com_Control.motor_Setting.min_Speed_u16;
// }

/**
********************************************************************************************************************************
* @brief   Allow an external module to update the stored motor direction
* @details - Input Parameter - Direction: 0=CCW, 1 = CW
* @param   None
* @retval  None
********************************************************************************************************************************
*/

void MotorCom_UpdateMotorDirection(uint16_t direction) {
  motor_Com_Control.motor_Setting.motor_Direction_u16 = direction ? CCW: CW;
}

/**
********************************************************************************************************************************
* @brief   Get the harmonic injection parameters as provided by MODBUS.
* @details This is used by module_motor_com to update messages sent to drive board with harmonic injection data.
* @param   None 
* @return  None
********************************************************************************************************************************
*/
#if 0
void HarmonicInjection_ModbusUpdate(uint8_t  enable_u8, 
							   int16_t  *amplitudes_s16, 
							   uint8_t  *angle_multipliers_u8, 
							   uint16_t *angle_offsets_u16, 
							   uint8_t  *is_inverted_u8, 
							   uint16_t *min_speed_u16, 
							   uint16_t *max_speed_u16)
{  
  uint8_t index_u8 = 0;
  
  hi_enable_u8 = enable_u8;
  for(index_u8 = 0; index_u8 < 8; index_u8++) {
  	hi_amplitude_s16[index_u8] 			= amplitudes_s16[index_u8]; 
  	hi_angle_multiplier_u8[index_u8]    = angle_multipliers_u8[index_u8];
  	hi_angle_offset_u16[index_u8] 		= angle_offsets_u16[index_u8];
  	hi_is_phase_inverted_u8[index_u8] 	= is_inverted_u8[index_u8];
  	hi_min_speed_u16[index_u8] 			= min_speed_u16[index_u8];
  	hi_max_speed_u16[index_u8] 			= max_speed_u16[index_u8]; 
  }
}

#endif
