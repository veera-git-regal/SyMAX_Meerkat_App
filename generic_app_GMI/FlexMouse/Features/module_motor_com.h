/**
  ***************************************************************************************************
  * @file    App_Template.h 
  * @author  Regal Pamela Lee
  * @version V1.0
  * @date    7-Jul-2020
  * @brief   Main function/s of control motor speed by 0 to 10V
  * @note    
  ***************************************************************************************************
  */
//^** Tips: APPs/Drivers adding process example step6  [refer to user manual ) 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MODULE_MOTOR_COM_H_
#define _MODULE_MOTOR_COM_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "structured_memory.h"
#include "scheduler.h"
#include "ring_buffer.h"

#ifdef __cplusplus
extern "C" {
#endif

#if HARDWARE_VERSION == HARDWARE_VERSION_4KW
// 4KW prototype needs to guarantee device slows to a stop before starting or motor could be damaged
#define MotorStopResumePeriod    45000 // after send out stop cmd to motor board and wait period 
#define MotorStopResumePeriodMax 90000 //
#elif HARDWARE_VERSION == HARDWARE_VERSION_1p3KW
#define MotorStopResumePeriod    5000 //45000 //after send out stop cmd to motor board and wait period 
#define MotorStopResumePeriodMax 6000 //90000 //
#else // running bull
#define MotorStopResumePeriod    1000  //after send out stop cmd to motor board and wait period 
#define MotorStopResumePeriodMax 2000  //
#endif
  
extern Ram_Buf sharedMemArray[STRUCT_MEM_ARRAY_SIZE];
extern ProcessInfo processInfoTable[];

enum // Motor direction
{
  CW = 6,  // TODO: StyleGuide: Descriptive names
  CCW = 9,
};

//******************* App_Template Control (inside shared memory) *******************************************************************************************************************************  
typedef struct 
{
  Ring_Buf_Handle*   InternalPipe;
  uint8_t  ErrorCode;                                                           //Error code
}App_TemplateControl;
  
struct Motor_Settings{
  uint16_t max_Speed_u16;        //Maximum allowed speed in RPM 0dp
  uint16_t min_Speed_u16;        //minumum speed in RPM 0dp
  uint16_t hysteresis_Speed_u16; //hysteresis speed in RPM 0dp  
  uint16_t max_Torque_u16;       //Maximum allowed torque
  uint16_t min_Torque_u16;       //Minmium torque
  uint16_t set_Torque_u16;       //Set torque. Changes based on user inputs  
  uint16_t control_Mode_u16;       //00 = Speed; 01 = Torque;
  uint16_t comm_Address_u16;       //address for univeral protocol
  uint16_t motor_Direction_u16;    // CCW = 9, CW = 6  
};

typedef struct
{               
  uint16_t  is_Motor_On:1;              // 1= ON, 0 = OFF
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
} Motor_Metering_Discretes;

struct Motor_Metering_Data{
  uint32_t motor_alarms_u32;                // Motor faults/alarms
  uint16_t analog_0_10v_u16;                // Analog volts;
  uint16_t analog_0_10v_Demand_Percent_u16; // Analog demand percentage 100.00%;
  uint16_t demand_Reference_Percent_u16;    // Demand reference percentage
  uint16_t demand_Reference_Speed_u16;      // Speed reference 100.00% 2dp
  uint16_t demand_Reference_Torque_u16;     // Torque reference 100.00% 2dp
  uint16_t demand_Output_Percent_u16;       // Demand output 0 - 100.00% 2dp
  uint16_t demand_Output_Speed_u16;         // Speed output RPM
  uint16_t demand_Output_Torque_u16;        // Demand output in NM  
  uint16_t user_Set_Speed_u16;              // Set speed commanded by user 0dp
  uint16_t user_Set_Torque_u16;             // Set torque. Changes based on user inputs
  uint16_t motor_Status_u16;                // Status of the motor. Run/Idle...
  uint16_t motor_Actual_Direction_u16;      // CCW = 9, CW = 6
  uint16_t errorCode_u16;                   // Error codes
  Motor_Metering_Discretes discretes_u16;	// Discrete bits
};

typedef struct{
  struct Motor_Settings motor_Setting;
  struct Motor_Metering_Data motor_Metering_Data;
}Motor_Com_Control;

// Public Functions
extern uint16_t MotorCom_GetMinSpeed(void);
extern uint16_t MotorCom_GetMaxSpeed(void);
extern uint16_t MotorCom_ConvertSpeedToDemandPercentage(uint16_t demand_percentage);
extern void MotorCom_UpdateMotorDirection(uint16_t direction);
void HarmonicInjection_ModbusUpdate(uint8_t hi_enable_u8, int16_t *hi_amplitudes_s16, uint8_t *hi_angle_multipliers_u8, uint16_t *hi_angle_offsets_u16, uint8_t* hi_is_inverted_u8, uint16_t* hi_min_speed_u16, uint16_t* hi_max_speed_u16);

extern int16_t Actual_Measured_Speed;
//******************* end of App_Template Control (inside shared memory) ******************************************************************************************************************************* 
  

#ifdef __cplusplus
}
#endif

#endif /* _MODULE_MOTOR_COM_H_ */

