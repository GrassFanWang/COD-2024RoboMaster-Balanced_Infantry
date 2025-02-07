/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : api_trajectory.h
  * @brief          : solve trajectory
  * @author         : Yan Yuanbin
  * @date           : 2023/05/21
  * @version        : v1.0
  ******************************************************************************
  * @attention      : see https://github.com/chenjunnn/rm_auto_aim
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef API_TRAJECTORY_H
#define API_TRAJECTORY_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "minipc.h"

/* Exported defines -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief typedef structure that contains the information for the solved trajectory.
 */
typedef struct
{
    float Camera_Yaw_Vertical;    /*!< the vertical distance of yaw axis to the camera(m) */
    float Camera_Yaw_Horizontal;  /*!< the horizontal distance of yaw axis to the camera(m) */

    float Time_Offset;            /*!< the bias time of communication(s) and trigger(s) etc. from user estimating */
    float Task_DWT_Timeline;      /*!< the DWT time line of EC task(s) */
    float Task_Lency;             /*!< the bias time of EC task(s) */
    float bullet_time;            /*!< ballistic time */
    float FireSystem_BiasTime;    /*!< the bias time of system(s), contains the communication delay, task delay and trigger delay etc. */

    float armor_distance;  /*!< distance to the armor plate */

    float bullet_speed;   /*!< referee bullet speed */
    
    float current_pitch;  /*!< current pitch angle */
    float current_yaw;    /*!< current yaw angle */

    float Armor_Yaw_Limit;          /* the yaw angle diff threshold */
    float Armor_Yaw_Limit_Offset;   /* the offset of yaw angle diff threshold */  

    float armorlock_yaw;      /*!< gimbal target yaw angle in radians,lock the armor */
    float armorlock_pitch;    /*!< gimbal target pitch angle in radians,lock the armor  */
	
	
	float Bias_Yaw;
	float Bias_Pitch;
	
	float Center_Lock_Yaw;


    uint8_t control_status;   /*!< vision control status */
    
}SolveTrajectory_Typedef;

/* Exported functions prototypes ---------------------------------------------*/
/**
 * @brief  Update the solve trajectory 
 */
extern void SolveTrajectory_Update(SolveTrajectory_Typedef *SolveTrajectory,float picth,float yaw,float bullet_speed);

/**
 * @brief  Transform the solve trajectory 
 */
extern void SolveTrajectory_Transform(MiniPC_SendPacket_Typedef *MiniPCTxData,MiniPC_ReceivePacket_Typedef *MiniPCRxData,SolveTrajectory_Typedef *SolveTrajectory);


#endif




