/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Vision_Task.c
  * @brief          : Vision task
  * @author         : Yan Yuanbin
  * @date           : 2023/07/23
  * @version        : v2.1
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "Vision_Task.h"
#include "INS_Task.h"
#include "api_trajectory.h"

/* Private variables -----------------------------------------------------------*/
/**
 * @brief structure that contains the information for the solved trajectory.
 */
SolveTrajectory_Typedef SolveTrajectory={
  .Camera_Yaw_Vertical = 0.f,//-0.1   0.025f
  .Camera_Yaw_Horizontal = 0.f,//-0.1   0.1f
  .Time_Offset = 0.1f,
	.Armor_Yaw_Limit = 0.3f,
	.Armor_Yaw_Limit_Offset = 0.f,
	.bullet_speed =27.f,//60
	.Bias_Yaw=1.f,
	.Bias_Pitch=+2.f, //+ = -
};

/**
 * @brief structure that contains the information for the Vision.
 */
Vision_Info_Typedef Vision_Info;
uint32_t Vision_Task_Cnt;

/* USER CODE BEGIN Header_Vision_Task */
/**
* @brief Function implementing the StartVisionTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Vision_Task */
void Vision_Task(void const * argument)
{
  /* USER CODE BEGIN Vision_Task */
  TickType_t systick = 0;

  /* Infinite loop */
  for(;;)
  {
    systick = osKernelSysTick();

    /* received minipc tracking , Enable the vision aiming */
    Vision_Info.IF_Aiming_Enable = (MiniPC_ReceivePacket.tracking == true);

    /* update the transmit euler angle in radians */
    MiniPC_SendPacket.pitch = -INS_Info.angle[2]+SolveTrajectory.Bias_Pitch*DegreesToRadians  ;
    MiniPC_SendPacket.yaw   = INS_Info.angle[0]+SolveTrajectory.Bias_Yaw *DegreesToRadians ;
    MiniPC_SendPacket.roll  = INS_Info.angle[1];
   MiniPC_SendPacket.detect_color=1;
    /* update the solve trajectory */
		SolveTrajectory_Update(&SolveTrajectory,-MiniPC_SendPacket.pitch,MiniPC_SendPacket.yaw,30.f);

    /* transform the solved trajetory */
    SolveTrajectory_Transform(&MiniPC_SendPacket,&MiniPC_ReceivePacket,&SolveTrajectory);

		/* update the gimbal target pose */
    Vision_Info.target_Pitch = SolveTrajectory.armorlock_pitch * RadiansToDegrees ;
    Vision_Info.target_Yaw = SolveTrajectory.armorlock_yaw * RadiansToDegrees ;
		
		/* update the fire control flag */
		Vision_Info.IF_Fire_Accept = (SolveTrajectory.control_status == 2 && Vision_Info.IF_Aiming_Enable == true);

    /* transmit the minipc frame data */
    MiniPC_SendFrameInfo(&MiniPC_SendPacket);

		Vision_Task_Cnt++;
    osDelayUntil(&systick,2);
  }
  /* USER CODE END Vision_Task */
}
//------------------------------------------------------------------------------

