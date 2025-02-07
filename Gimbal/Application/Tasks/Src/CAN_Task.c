/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : CAN_Task.c
  * @brief          : CAN task
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "CAN_Task.h"
#include "Control_Task.h"
#include "Vision_Task.h"
#include "Motor.h"
#include "bsp_can.h"
#include "remote_control.h"

uint8_t MotorCAN_FramInfo[4][8];
bool flag;
uint32_t CAN_Tick;
 TickType_t systick = 0;
/* USER CODE BEGIN Header_CAN_Task */
/**
* @brief Function implementing the StartCANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Task */
void CAN_Task(void const * argument)
{
  /* USER CODE BEGIN CAN_Task */
	 osDelay(600);
  		 Damiao_Motor_Enable(0x01);
		 osDelay(30);
  /* Infinite loop */
  for(;;)
  {
	 systick = osKernelSysTick();
		
		if(remote_ctrl.rc.ch[4] > 0) remote_ctrl.rc.ch[4] = 1;
		else if(remote_ctrl.rc.ch[4] < 0) remote_ctrl.rc.ch[4] = -1;
		else remote_ctrl.rc.ch[4] = 0;
  Send2Chassis_TxFrame.Data[0] =  (uint8_t) ((Control_Info.Cover_Switch<<7) | (Control_Info.Shoot.Mode)<<5 | (remote_ctrl.rc_lost)<<4 | (remote_ctrl.rc.s[0])<<2 | (remote_ctrl.rc.s[1]));
	Send2Chassis_TxFrame.Data[1] =  (uint8_t) (remote_ctrl.rc.ch[3]>> 8);
	Send2Chassis_TxFrame.Data[2] =  (uint8_t) (remote_ctrl.rc.ch[3] );
  Send2Chassis_TxFrame.Data[3] =  (uint8_t) ((uint8_t)Vision_Info.IF_Aiming_Enable);
	Send2Chassis_TxFrame.Data[4] = (uint8_t) (remote_ctrl.mouse.press_r);
	Send2Chassis_TxFrame.Data[5] = remote_ctrl.rc.ch[4]; 
	Send2Chassis_TxFrame.Data[6] = (uint8_t)(remote_ctrl.key.v >> 8);
	Send2Chassis_TxFrame.Data[7] = (uint8_t)(remote_ctrl.key.v);
  USER_CAN_TxMessage(&Send2Chassis_TxFrame);		
		

//	 Shoot_TxFrame.Data[0] = (uint8_t)(Control_Info.SendValue[0]>>8);
//   Shoot_TxFrame.Data[1] = (uint8_t)(Control_Info.SendValue[0]);
//	 Shoot_TxFrame.Data[2] = (uint8_t)(Control_Info.SendValue[Left_Shoot]>>8);
//   Shoot_TxFrame.Data[3] = (uint8_t)(Control_Info.SendValue[Left_Shoot]);
//	 Shoot_TxFrame.Data[6] = (uint8_t)(Control_Info.SendValue[Right_Shoot]>>8);
//   Shoot_TxFrame.Data[7] = (uint8_t)(Control_Info.SendValue[Right_Shoot]);
//   USER_CAN_TxMessage(&Shoot_TxFrame);	 
  
		

	Yaw_TxFrame.Data[2] = (uint8_t)(Control_Info.SendValue[Yaw]>>8);
  Yaw_TxFrame.Data[3] = (uint8_t)(Control_Info.SendValue[Yaw]);
  USER_CAN_TxMessage(&Yaw_TxFrame);

	
	Damiao_Motor_CAN_Send(0x02,0,0,0,0,0);






 	 
	


	
		
 
    osDelay(1);
  }
  /* USER CODE END CAN_Task */
}



