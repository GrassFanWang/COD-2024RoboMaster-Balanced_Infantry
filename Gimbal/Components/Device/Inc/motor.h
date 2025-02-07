/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : motor.c
  * @brief          : motor interfaces functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : to be tested
  ******************************************************************************
  */
/* USER CODE END Header */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DEVICE_MOTOR_H
#define DEVICE_MOTOR_H


/* Includes ------------------------------------------------------------------*/
#include "config.h"
#include "stm32f4xx.h"
#include "pid.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief typedef enum that contains the usage for LK Motor.
 */
typedef enum{
	Left_Wheel,
	Right_Wheel,
  LK_MOTOR_USAGE_NUM,
}LK_MOTOR_USAGE_e;


/**
 * @brief typedef enum that contains the usage for DJI Motor.
 */
typedef enum{
	Trigger,
	Left_Shoot,
  Right_Shoot,
	Yaw,
	Pitch,
  DJI_MOTOR_USAGE_NUM,
}DJI_MOTOR_USAGE_e;


/**
 * @brief typedef enum that contains the Frame Identifier for DJI Motor Device.
 */
typedef enum
{
  DJI_TxFrame_HIGH = 0x1ffU,
  DJI_TxFrame_LOW = 0x200U,
  DJI_RxFrame_MIDDLE = 0x204U,
  DJI_MotorFrameId_NUM,
}DJI_MotorFrameId_e;


/**
 * @brief typedef enum that contains the Error status for Motor Device.
 */
typedef enum
{
  MOTOR_ERROR_NONE = 0x00U,   /*!< no error */
  MOTOR_CAN_OFFLINE = 0x01U,    /*!< CAN transfer failed */
  MOTOR_OVER_TEMPERATURE = 0x02U,   /*!< abnormal motor temperature */
}Motor_Status_e;

/**
 * @brief typedef enum that contains the type of LK Motor Device.
 */
typedef enum{
	  LK_L9025,
    LK_MOTOR_TYPE_NUM,
}LK_Motor_Type_e;

/**
 * @brief typedef enum that contains the type of DJI Motor Device.
 */
typedef enum{
    DJI_GM6020,
    DJI_M3508,
    DJI_M2006,
    DJI_MOTOR_TYPE_NUM,
}DJI_Motor_Type_e;

/**
 * @brief typedef structure that contains the information for the Motor Error handler.
 */
typedef struct 
{
  uint16_t ErrorCount;    /*!< Error status judgment count */
  Motor_Status_e Status;   /*!< Error status */
}Motor_ErrorrHandler_Typedef;

/**
 * @brief typedef structure that contains the information for the Motor CAN Transfer.
 */
typedef struct
{
  uint32_t TxStdId;   /*!< Specifies CAN transfer identifier */
  uint32_t RxStdId;   /*!< Specifies CAN transfer identifier */
  uint8_t FrameIndex;   /* index for motor transmit frame */
}Motor_CANFrameInfo_typedef;

/**
 * @brief typedef structure that contains the General information for the Motor Device.
 */
typedef struct 
{
  bool Initlized;   /*!< init flag */

  int16_t  current;   /*!< Motor electric current */
  int16_t  velocity;    /*!< Motor rotate velocity */
  int16_t  encoder;   /*!< Motor encoder angle */
  int16_t  last_encoder;   /*!< previous Motor encoder angle */
  float    angle;   /*!< Motor angle in degree */
  uint8_t  temperature;   /*!< Motor Temperature */
}Motor_GeneralInfo_Typedef;
typedef struct 
{
  bool Initlized;   /*!< init flag */
  
  int16_t  Current;   /*!< Motor electric current */
  int16_t  Velocity;    /*!< Motor rotate velocity */\
	int16_t  Last_Velocity;
	float  Accel;
	float    Rad_Velocity;
	float  Last_Angle;
  float    Angle;   /*!< Motor angle in degree */
	int16_t  Encoder; 
  uint8_t  Temperature;   /*!< Motor Temperature */

}LK_GeneralInfo_Typedef;
/**
 * @brief typedef structure that contains the information for the DJI Motor Device.
 */
typedef struct 
{
  bool Initlized;   /*!< init flag */
  int16_t  State; 	/*!< Motor ERROR Message */
  uint16_t  P_int;
	uint16_t  V_int;
	uint16_t  T_int;
	float  Position;   /*!< Motor Positon */
  float  Velocity;   /*!< Motor Velocity  */
  float  Torque;  /*!< Motor Torque */
  float  Temperature_MOS;   /*!< Motor Temperature_MOS */
	float  Temperature_Rotor;   /*!< Motor Temperature_Rotor */
  float  Angle;	
}Damiao_GeneralInfo_Typedef;
/**
 * @brief typedef structure that contains the information for the Damiao Motor Device.
 */
typedef struct
{
	DJI_Motor_Type_e Type;   /*!< Type of Motor */
  Motor_CANFrameInfo_typedef CANFrame;    /*!< information for the CAN Transfer */
	Motor_GeneralInfo_Typedef Data;   /*!< information for the Motor Device */
	Motor_ErrorrHandler_Typedef ERRORHandler;   /*!< information for the Motor Error */
}DJI_Motor_Info_Typedef;

/**
 * @brief typedef structure that contains the information for the DJI Motor Device.
 */
typedef struct
{
	bool lost;
	uint8_t ID;   /*!< Type of Motor */
	uint8_t Online_cnt;
  Motor_CANFrameInfo_typedef CANFrame;    /*!< information for the CAN Transfer */
	Damiao_GeneralInfo_Typedef Data;   /*!< information for the Motor Device */
	Motor_ErrorrHandler_Typedef ERRORHandler;   /*!< information for the Motor Error */
}Damiao_Motor_Info_Typedef;
typedef struct
{
  float Position;
	float Velocity;
	float KP;
	float KD;
	float Torque;
	float Angle;
}Damiao_Motor_Contorl_Info_Typedef;
/**
 * @brief typedef structure that contains the information for the DJI Motor Device.
 */





extern DJI_Motor_Info_Typedef DJI_Motor[DJI_MOTOR_USAGE_NUM];

extern Damiao_Motor_Info_Typedef Damiao_Pitch_Motor;

extern Damiao_Motor_Contorl_Info_Typedef  Damiao_Motor_Contorl_Info;
/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  Update the DJI motor Information
  */
extern void DJI_Motor_Info_Update(uint32_t *StdId, uint8_t *rxBuf,DJI_Motor_Info_Typedef *DJI_Motor);


extern void Damiao_Motor_Info_Update(uint8_t *rxBuf,Damiao_Motor_Info_Typedef *Damiao_Motor);

extern void Damiao_Motor_Enable(uint8_t ID);

extern void Damiao_Motor_CAN_Send(uint8_t ID,float Postion, float Velocity, float KP, float KD, float Torque);

 
extern void Damiao_Motor_DisEnable(uint8_t ID);


#endif //DEVICE_MOTOR_H
