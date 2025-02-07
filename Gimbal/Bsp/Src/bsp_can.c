/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_can.c
  * @brief          : bsp can functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to enable the can filter
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "bsp_can.h"
#include "can.h"
#include "motor.h"
#include "remote_control.h"
#include "cmsis_os.h"
#include "Control_Task.h"
/* Private variables ---------------------------------------------------------*/
/**
 * @brief the structure that contains the Information of CAN Receive.
 */
CAN_RxHeaderTypeDef USER_CAN_RxInstance;
/**
 * @brief the array that contains the Information of CAN Receive data.
 */
uint8_t USER_CAN_RxFrameData[8];

/**
 * @brief the structure that contains the Information of CAN Transmit.
 */
CAN_TxFrameTypeDef Shoot_TxFrame ={
 		.hcan = &hcan2,
		.header.StdId=0x200,
		.header.IDE=CAN_ID_STD,
		.header.RTR=CAN_RTR_DATA,
		.header.DLC=8,
};
CAN_TxFrameTypeDef Pitch_TxFrame ={
 		.hcan = &hcan2,
		.header.StdId=0x02,
		.header.IDE=CAN_ID_STD,
		.header.RTR=CAN_RTR_DATA,
		.header.DLC=8,
};
CAN_TxFrameTypeDef Yaw_TxFrame ={
 		.hcan = &hcan1,
		.header.StdId=0x1ff,
		.header.IDE=CAN_ID_STD,
		.header.RTR=CAN_RTR_DATA,
		.header.DLC=8,
};
CAN_TxFrameTypeDef Send2Chassis_TxFrame ={
 		.hcan = &hcan1,
		.header.StdId=0x302,
		.header.IDE=CAN_ID_STD,
		.header.RTR=CAN_RTR_DATA,
		.header.DLC=8,
};
/**
  * @brief  Configures the CAN Filter.
  * @param  None
  * @retval None
  */

 void BSP_CAN_Init(void)
{
  CAN_FilterTypeDef CAN1_FilterConfig = {0};

  /* Update the CAN1 filter Conifguration */
  CAN1_FilterConfig.FilterActivation = ENABLE;
  CAN1_FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  CAN1_FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  CAN1_FilterConfig.FilterIdHigh = 0x0000;
  CAN1_FilterConfig.FilterIdLow = 0x0000;
  CAN1_FilterConfig.FilterMaskIdHigh = 0x0000;
  CAN1_FilterConfig.FilterMaskIdLow = 0x0000;
  CAN1_FilterConfig.FilterBank = 0;
  CAN1_FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  CAN1_FilterConfig.SlaveStartFilterBank = 0;

  /* configures the CAN1 filter */
  if(HAL_CAN_ConfigFilter(&hcan1, &CAN1_FilterConfig) != HAL_OK)
  {	
      Error_Handler();
  }

  /* Start the CAN1 module. */
  HAL_CAN_Start(&hcan1);

  /* Enable CAN1 FIFO0 interrupts */
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  /* Update the CAN2 filter Conifguration */
	
	CAN_FilterTypeDef CAN2_FilterConfig = {0};
	CAN2_FilterConfig.FilterActivation = ENABLE;
  CAN2_FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  CAN2_FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  CAN2_FilterConfig.FilterIdHigh = 0x0000;
  CAN2_FilterConfig.FilterIdLow = 0x0000;
  CAN2_FilterConfig.FilterMaskIdHigh = 0x0000;
  CAN2_FilterConfig.FilterMaskIdLow = 0x0000;
  CAN2_FilterConfig.FilterBank = 14;
  CAN2_FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
  CAN2_FilterConfig.SlaveStartFilterBank = 14;
	


  /* configures the CAN2 filter */
  if(HAL_CAN_ConfigFilter(&hcan2, &CAN2_FilterConfig) != HAL_OK)
  {	
      Error_Handler();
  }

  /* Start the CAN2 module. */
  HAL_CAN_Start(&hcan2);

  /* Enable CAN2 FIFO0 interrupts */
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

}

//------------------------------------------------------------------------------

/**
  * @brief  USER function to transmit the Specifies message.
	* @param  Instance: pointer to the CAN Register base address
	* @param  data: pointer to the CAN transmit data
  * @retval None
  */

void USER_CAN_TxMessage(CAN_TxFrameTypeDef *TxHeader)
{
	static uint32_t TxMailbox = 0;
	HAL_CAN_AddTxMessage(TxHeader->hcan, &TxHeader->header, TxHeader->Data, &TxMailbox);
}
/**
  * @brief  USER function to converting the CAN1 received message.
	* @param  Instance: pointer to the CAN Register base address
	* @param  StdId: Specifies the standard identifier.
	* @param  data: array that contains the received massage.
  * @retval None
  */
static void CAN1_RxFifo0RxHandler(uint32_t *StdId,uint8_t data[8])
{
 if( *StdId == 0x206){
 DJI_Motor_Info_Update(StdId,data,&DJI_Motor[Yaw]);
 }else if(*StdId == 0x300){
  Control_Info.Shoot.Refree.Shooter_17mm_1_barrel_heat = ((int16_t) (data[0] << 8 | data[1]));
  Control_Info.Shoot.Refree.Shoot_initial_speed = ((int16_t) (data[2] << 8 | data[3]))/100;
  Control_Info.Shoot.Refree.Shooter_barrel_cooling_value = (data[4]);
	Control_Info.Chassis_Week_Flag = (data[5] & 0x03);
	 Control_Info.Gimbal.Gimbal_Output = (data[5] & 0x04) >> 2;
	Control_Info.Shoot.Refree.shooter_barrel_heat_limit = ((int16_t) (data[6] << 8 | data[7]));
 }
 
  

}
//------------------------------------------------------------------------------

/**
  * @brief  USER function to converting the CAN2 received message.
	* @param  Instance: pointer to the CAN Register base address
	* @param  StdId: Specifies the standard identifier.
	* @param  data: array that contains the received massage.
  * @retval None
  */
static void CAN2_RxFifo0RxHandler(uint32_t *StdId,uint8_t data[8])
{
if( *StdId == 0x243){
 Damiao_Motor_Info_Update(data,&Damiao_Pitch_Motor);
}else{
// DJI_Motor_Info_Update(StdId,data,&DJI_Motor[Left_Shoot]);
// DJI_Motor_Info_Update(StdId,data,&DJI_Motor[Right_Shoot]);
// DJI_Motor_Info_Update(StdId,data,&DJI_Motor[Trigger]);
}


}



//------------------------------------------------------------------------------

/**
  * @brief  Rx FIFO 0 message pending callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* Get an CAN frame from the Rx FIFO zone into the message RAM. */
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &USER_CAN_RxInstance, USER_CAN_RxFrameData);

  /* judge the instance of receive frame data */

    CAN1_RxFifo0RxHandler(&USER_CAN_RxInstance.StdId,USER_CAN_RxFrameData);
  

}
//------------------------------------------------------------------------------
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* Get an CAN frame from the Rx FIFO zone into the message RAM. */
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &USER_CAN_RxInstance, USER_CAN_RxFrameData);

  /* judge the instance of receive frame data */
 CAN2_RxFifo0RxHandler(&USER_CAN_RxInstance.StdId,USER_CAN_RxFrameData);
  
}