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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CAN_TASK_H
#define CAN_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

extern uint8_t flag;
typedef  struct{
	    uint8_t   DischargeATK;
	    uint8_t SuperCapSwitch;
	    uint8_t Cap_Percent;//?????????
	    uint8_t Cap_Mode;//????     1??
	    float Cap_V;//????
	    float IN_C;
	    uint8_t P_Set_Send;//??????
}SuperCap_Info_t;
extern SuperCap_Info_t SuperCap_Info;


#endif //CAN_TASK_H
