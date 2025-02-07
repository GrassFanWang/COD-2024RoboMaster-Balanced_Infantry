/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : api_trajectory.c
  * @brief          : solve trajectory
  * @author         : Yan Yuanbin
  * @date           : 2023/07/23
  * @version        : v2.1
  ******************************************************************************
  * @attention      : see https://github.com/chenjunnn/rm_auto_aim
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "api_trajectory.h"
#include "bsp_dwt.h"
#include "math.h"
#include "arm_math.h"
#include "config.h"

/* Private defines -----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/**
 * @brief  Calculate Bullet Model offset
 */
static float Trajectory_BulletModel(float ,float ,float ,float *,float );
/**
  * @brief   Update the Trajectory pitch angle
  */
static float Trajectory_Picth_Update(float ,float ,SolveTrajectory_Typedef *);

/**
 * @brief  Update the solve trajectory 
 * @param  SolveTrajectory: pointer to a SolveTrajectory_Typedef structure that
 *          contains the information of solved trajectory.
 * @param  picth: current pitch angle
 * @param  yaw: current yaw amngle
 * @param  bullet_speed: bullet speed from Referee
 * @retval none
 */
void SolveTrajectory_Update(SolveTrajectory_Typedef *SolveTrajectory,float picth,float yaw,float bullet_speed)
{
    SolveTrajectory->current_pitch = picth;
    SolveTrajectory->current_yaw = yaw;
    
    SolveTrajectory->bullet_speed = bullet_speed;
}
//------------------------------------------------------------------------------

/**
 * @brief  Transform the solve trajectory 
 * @param  bias_time: system delay
 * @param  MiniPCTxData: pointer to a MiniPC_SendPacket_Typedef structure that
  *          contains the information of transmit Data to MiniPC.
 * @param  MiniPCRxData: pointer to a MiniPC_ReceivePacket_Typedef structure that
  *          contains the information of Received Data from MiniPC.
 * @param  SolveTrajectory: pointer to a SolveTrajectory_Typedef structure that
  *          contains the information of solved trajectory.
 * @retval none
 */    float yaw_diff_offset = 0.f, yaw_diff = 0.f;

void SolveTrajectory_Transform(MiniPC_SendPacket_Typedef *MiniPCTxData,MiniPC_ReceivePacket_Typedef *MiniPCRxData,SolveTrajectory_Typedef *SolveTrajectory)
{
    /* Calculate the EC task lency */
    SolveTrajectory->Task_Lency = DWT_GetTimeline_s() - SolveTrajectory->Task_DWT_Timeline;

    /* Calculate the x/y coordinates to the armor plate */
    float x_armor = MiniPCRxData->x - MiniPCRxData->r1*arm_cos_f32(MiniPCRxData->yaw);
    float y_armor = MiniPCRxData->y - MiniPCRxData->r1*arm_sin_f32(MiniPCRxData->yaw);

    /* Calculate the distance to the armor plate */
    SolveTrajectory->armor_distance =  sqrt(x_armor*x_armor + y_armor*y_armor);

    /* Calculate the ballistic time */
    SolveTrajectory->bullet_time = SolveTrajectory->armor_distance/(SolveTrajectory->bullet_speed*arm_cos_f32(SolveTrajectory->current_pitch));

    /* Calculate the system bias time */
    SolveTrajectory->FireSystem_BiasTime = SolveTrajectory->Task_Lency + SolveTrajectory->bullet_time + SolveTrajectory->Time_Offset;

    /* calculate the predict coordinates */
    float predict_x = MiniPCRxData->x + MiniPCRxData->vx * SolveTrajectory->FireSystem_BiasTime;
    float predict_y = MiniPCRxData->y + MiniPCRxData->vy * SolveTrajectory->FireSystem_BiasTime;
    float predict_yaw = MiniPCRxData->yaw + MiniPCRxData->v_yaw * SolveTrajectory->FireSystem_BiasTime;

    SolveTrajectory->control_status = 0;




/*------------------------------------------------------------------------------------------*/
	
	SolveTrajectory->Center_Lock_Yaw= atan2(MiniPCRxData->y, MiniPCRxData->x);//+SolveTrajectory->Bias_Yaw*DegreesToRadians;


    if(fabsf(MiniPCRxData->v_yaw) < 1.5f)
    {
        MiniPCTxData->aim_x = predict_x - MiniPCRxData->r1 * arm_cos_f32(predict_yaw);
        MiniPCTxData->aim_y = predict_y - MiniPCRxData->r1 * arm_sin_f32(predict_yaw);
        MiniPCTxData->aim_z = MiniPCRxData->z;
        /* fire allowed */
        SolveTrajectory->control_status = 2;
    }
    else
    {
        bool is_armor_pair = true;
        float r = 0;
/*------------------------------------------------------------------------------------------*/

        float center_yaw = atan2(predict_y, predict_x);//+SolveTrajectory->Bias_Yaw*DegreesToRadians;
		
        /* select the aromor plate */
        for (uint8_t i = 0; i < MiniPCRxData->armors_num; i++) 
        {
            /* calculate the predict yaw angle to the all aromor plate in turn */
            float tmp_yaw = MiniPCRxData->v_yaw < 0 ? (predict_yaw - i * (2.f * PI / MiniPCRxData->armors_num)) : (predict_yaw + i * (2.f * PI / MiniPCRxData->armors_num));
        
            /* calculate the yaw angle diff */
            yaw_diff = fabsf(tmp_yaw - center_yaw) > PI ? ((tmp_yaw - center_yaw) - (tmp_yaw - center_yaw)/fabsf(tmp_yaw - center_yaw)*2*PI) : (tmp_yaw - center_yaw);
			
            /* take the threshold offset according to the direction of the yaw angular velocity */
            yaw_diff_offset = MiniPCRxData->v_yaw < 0 ? -SolveTrajectory->Armor_Yaw_Limit_Offset : SolveTrajectory->Armor_Yaw_Limit_Offset;

            if (-SolveTrajectory->Armor_Yaw_Limit + yaw_diff_offset < yaw_diff && yaw_diff < SolveTrajectory->Armor_Yaw_Limit + yaw_diff_offset) 
            {
                /* Only 4 armors has 2 radius and height */
                if (MiniPCRxData->armors_num == 4) 
                {
                    r = is_armor_pair ? MiniPCRxData->r1 :  MiniPCRxData->r2;
                    MiniPCTxData->aim_z = MiniPCRxData->z + (is_armor_pair ? 0 : MiniPCRxData->dz);
                }
                else
                {
                    r = MiniPCRxData->r1;
                    MiniPCTxData->aim_z = MiniPCRxData->z;
                }

                /* Convert to predicted armor plate coordinates */
                MiniPCTxData->aim_x = predict_x - r * arm_cos_f32(tmp_yaw);
                MiniPCTxData->aim_y = predict_y - r * arm_sin_f32(tmp_yaw);

                SolveTrajectory->control_status = 1;

                break;
            }
            is_armor_pair = !is_armor_pair;
        }
    }

    float ALLOW_ERROR_DISTANCE = 0.f, min_yaw_diff = 0.f, allow_error_angle = 0.f;

    if (SolveTrajectory->control_status != 0) 
    {
        /* Calculate gimbal command */
        SolveTrajectory->armorlock_pitch = Trajectory_Picth_Update(SolveTrajectory->armor_distance+SolveTrajectory->Camera_Yaw_Horizontal,MiniPCTxData->aim_z+SolveTrajectory->Camera_Yaw_Vertical,SolveTrajectory);
        SolveTrajectory->armorlock_yaw = atan2(MiniPCTxData->aim_y, MiniPCTxData->aim_x);

        /* Fire control */
        if (SolveTrajectory->control_status == 1) 
        {
            /* balance/hero(large armor plate) : other(small armor plate) */
            ALLOW_ERROR_DISTANCE = (MiniPCRxData->armors_num == 2 || MiniPCRxData->id == 1) ? LargeArmor_HalfWidth : LittleArmor_HalfWidth;

            /* Scale threshold based on distance */
            allow_error_angle =  atan2f(SolveTrajectory->armor_distance,ALLOW_ERROR_DISTANCE);

            /* calculate the yaw angle diff */
/*------------------------------------------------------------------------------------------*/
            yaw_diff = SolveTrajectory->armorlock_yaw-SolveTrajectory->current_yaw;//+SolveTrajectory->Bias_Yaw*DegreesToRadians;


            /* calculate the Minimum absolute yaw angle diff */
            min_yaw_diff = fabsf(yaw_diff) > PI ? ( yaw_diff/fabsf(yaw_diff)*2*PI - yaw_diff ) : (yaw_diff);

            if (fabsf(min_yaw_diff) < allow_error_angle) 
            {
                SolveTrajectory->control_status = 2;
            }
        }
    }
    /* update the task DWT timeline */
    SolveTrajectory->Task_DWT_Timeline = DWT_GetTimeline_s();
}
//------------------------------------------------------------------------------

/**
 * @brief  Calculate Bullet Model Height
 * @param  distance: distance of target armor
 * @param  bullet_k: ballistic coefficient
 * @param  bullet_speed: bullet speeed from referee
 * @param  bullet_Time： pointer to the ballistic time
 * @param  pitchangle:  target pitch angle
 * @retval Bullet Model Height in radians
 * @note   t = (e^(v*x)-1)/(k*v*cos(pitch))
 *         Height = v*sin(pitch)*t - 1/2*g*t^2
 */
static float Trajectory_BulletModel(float distance,float bullet_k,float bullet_speed,float *bullet_Time,float pitchangle)
{
    float height = 0.f;

    *bullet_Time = (float)((exp(bullet_k * distance)- 1.f) / (bullet_k * bullet_speed * cos(pitchangle)));

    height = (float)(bullet_speed * arm_sin_f32(pitchangle) * *bullet_Time - GravityAccel * *bullet_Time * *bullet_Time / 2.f);

    return height;
}
//------------------------------------------------------------------------------

/**
  * @brief   Update the Trajectory pitch angle
  * @param   distance: distance of target center
  * @param   height: height of target armor
  * @param   solvetrajectory: pointer to a SolveTrajectory_Typedef structure that
  *          contains the information of solved trajectory.
  * @retval  solved Trajectory pitch angle in radians
  */
static float Trajectory_Picth_Update(float distance,float height,SolveTrajectory_Typedef *SolveTrajectory)
{
    float z_temp = 0.f,dz = 0.f;
    float height_calc = 0.f,pitchangle = 0.f;

    /* store the height of target armor */
    z_temp = height;

    for(uint8_t i=0; i<200; i++)
    {
        /* calculate the traget pitch angle */
        /* pitchangle = actan(z,x) */
        pitchangle = (float)atan2(z_temp,distance);

        /* Calculate the Bullet Model offset */
        height_calc = Trajectory_BulletModel(distance,Bullet_Coefficient,SolveTrajectory->bullet_speed,&SolveTrajectory->bullet_time,pitchangle);

        /* calculate the difference of height */
        dz = 0.3f*(height - height_calc);
        
        /* height points compensation */
        z_temp += dz;

        /* judge the difference of height */
        if(fabs(dz) < 0.00001f)
        {
            break;
        }
    }
    return pitchangle;
}
//------------------------------------------------------------------------------
