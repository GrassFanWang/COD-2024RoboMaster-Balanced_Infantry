/**
  ******************************************************************************
  * @file           : Control_Task.c
  * @brief          : Control task
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "arm_math.h"


typedef enum 
{
  CHASSIS_WEAK,
  CHASSIS_BALANCE,
  CHASSIS_SLIP,
  Chassis_Situation_NUM,
}Chassis_Situation_e;
typedef enum 
{
  CHASSIS_FRONT,
  CHASSIS_SIDE,
  CHASSIS_SPIN,
  CHASSIS_MODE_NUM,
}Chassis_Mode_e;

typedef enum 
{
  Leg_Length_Normol,
  Leg_Length_Fly,
  Leg_Length_UpSlope,
	Leg_Length_Squat,
	Leg_Length_High,
  Leg_Length_NUM,
}Leg_Length_Mode_e;
typedef enum{
  Left_Control,
  Right_Control,
  Control_USAGE_NUM,
}Control_USAGE_e;

typedef enum{
	Remote_Ctrl_Mode,
  Key_Board_Mode,
  
  Control_Mode_NUM,
}Control_Mode_e;
typedef enum{
	Position,
  Velocity,
  
  Kalman_USAGE_NUM,
}Kalman_USAGE_e;
/**
 * @brief typedef structure that contains the information of chassis control
*/

typedef struct 
{
	Control_USAGE_e VMC_USAGE;
	float W_Velocity;

  struct
  {
  float L0,L0_dot,Last_L0,Target_L0;
	float *Phi1,*Phi4;
	float  Phi2,Phi3,Phi0,Last_Phi0;
	float X_D_Difference_X_B,Y_D_Difference_Y_B;
	float *Phi1_dot,*Phi4_dot,Phi0_dot;
	float X_B,Y_B,X_D,Y_D,X_C,Y_C;
	float A0,B0,C0,LBD_2;
	float	Gravity_Compensation;
  }VMC;
	
	struct
  {
		float Phi;
		float Phi_dot;
    float Chassis_Position;
    float Chassis_Velocity;
    float Theta;
    float Theta_dot;
  }Target;
  
  struct
  {
		float Phi;
		float Phi_dot;
		float Last_Phi;
    float Chassis_Position;
    float Chassis_Velocity;
	
		float Theta;
    float Last_Theta;
    float Theta_dot;
		float Theta_ACC;
    float Last_Theta_dot;
  }Measure;
  
	struct{
	 	float T;
		float Tp;
		float F;
		float Balance_Tp;
		float Leg_Coordinate_Tp;
		float Stand_T;
		float Last_Stand_T;
	  float Turn_T;
		float Roll_F;
		float Leg_Length_F;
  }Moment;
  
	struct
  {
    int16_t Chassis_Speed;
    int16_t Chassis_Position;
  }Limit;
  
  float LQR_K[2][6];
  float LQR_X[6];
  float LQR_Output[2][6];

  int16_t rc_tick;
	struct
  {
    float T1;
    float T2;
  }SendValue;
	
	struct{
  float FN,P,Zw,Zm;
	float Measure_F,Measure_Tp;	
	bool Fly_Flag;
	uint16_t IF_Flay_Cnt;
	}Support;
	
	struct{
		float Theta;
    float Theta_dot;
		float Velocity;
		float Position;
   	float Phi;
		float Phi_dot;
	}T_Gain;
	
	struct{
		float Theta;
    float Theta_dot;
		float Velocity;
		float Position;
   	float Phi;
		float Phi_dot;
	}Tp_Gain;
	
	float Off_Ground_FN;
	float Predict_Chassis_Velocity,Measure_Chassis_Velocity,Fusion_Chassis_Velocity,Measure_Chassis_Position;
}Leg_Info_Typedef;

 typedef struct 
{
	Chassis_Situation_e  Situation;
  Chassis_Mode_e Chassis_Mode;
	
  Control_Mode_e Control_Mode;
	Control_Mode_e Last_Control_Mode;
	Leg_Length_Mode_e  Leg_Length_Mode;
	uint16_t Chassis_Slip_Cnt;
	
		struct
  {
		
		 
		bool Begin_Init;
    
		bool Motor_Enable;
	
		struct{
		 bool Joint_Reduction_Flag[4];
		 bool IF_Joint_Init;
		}Joint_Init;
		
		struct{
		bool IF_Balance_Init;
		uint16_t Balance_Right_Cnt;
		}Balance_Init;
        
	  struct{
		bool IF_Chassis_Velocity_Init;
		uint16_t Chassis_Velocity_Right_Cnt;
		}Chassis_Velocity_Init;
	
	       
	  struct{
		bool IF_Leg_Init;
		uint16_t Leg_Right_Cnt;
		}Leg_Init;
	
	}Init;
	
	
	struct{
    bool LK_Motor;
		bool Damiao_Motor;
		bool Yaw_Motor;
	}Motor_Online;
	
	bool Chassis_Position_Clean;
	uint16_t Target_Chassis_Velocity_to_0;
  
	bool Sliping;
	float L1,L2,L3,L4,L5;
	float Target_Chassis_Velocity,Target_Chassis_Position;
	float Measure_Chassis_Velocity;
	float Chassis_Accel;
	float Center_Angle;
  float Yaw_Err;
  float Target_Yaw_Gyro;
  bool Motor_Enable;
	uint16_t LK_Stand_T_To_SendValue;
	uint16_t LK_Turn_T_To_SendValue;
	int16_t T_SendValue[2];
	float Power_Gain;
  float K;
	float Chassis_Velocity_MAX;
  bool Slip_Week;
	bool F_Week;
	float Turn_T_SendValue[2];
	float Stand_T_SendValue[2];
	uint8_t Shoot_Mode;
  bool Cover_Switch;
	bool IF_Aiming_Enable;
  bool  Week_Move;
	bool SPIN_V;
	float Vision_Lock_Angle;
	float Velocity_Power_Gain;
	Leg_Info_Typedef L_Leg_Info;
	Leg_Info_Typedef R_Leg_Info;
	
}Control_Info_Typedef;


extern Control_Info_Typedef Control_Info;
#endif //CONTROL_TASK_H
