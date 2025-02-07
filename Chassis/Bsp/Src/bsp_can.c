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
#include "CAN_Task.h"
#include "Control_Task.h"
/* Private variables ---------------------------------------------------------*/
/**
 * @brief the structure that contains the Information of CAN Receive.
 */
 
CAN_RxFrameTypeDef CAN1_RxInstance;
CAN_RxFrameTypeDef CAN2_RxInstance;

/**
 * @brief the array that contains the Information of CAN Receive data.
 */

/**
 * @brief the structure that contains the Information of CAN Transmit.
 */
CAN_TxFrameTypeDef JointTxFrame[4] = {
	[L_A_Joint]={
		.hcan = &hcan2,
		.header.StdId=0x01,
		.header.IDE=CAN_ID_STD,
		.header.RTR=CAN_RTR_DATA,
		.header.DLC=8,
	},
	[L_P_Joint]={
		.hcan = &hcan2,
		.header.StdId=0x02,
		.header.IDE=CAN_ID_STD,
		.header.RTR=CAN_RTR_DATA,
		.header.DLC=8,
	}, 
	[R_A_Joint]={
		.hcan = &hcan2,
		.header.StdId=0x03,
		.header.IDE=CAN_ID_STD,
		.header.RTR=CAN_RTR_DATA,
		.header.DLC=8,
	},
	[R_P_Joint]={
		.hcan = &hcan2,
		.header.StdId=0x04,
		.header.IDE=CAN_ID_STD,
		.header.RTR=CAN_RTR_DATA,
		.header.DLC=8,
	}, 
};
CAN_TxFrameTypeDef LK_L9025_Left_TxFrame ={
    .hcan = &hcan1,
		.header.StdId=0x141,
		.header.IDE=CAN_ID_STD,
		.header.RTR=CAN_RTR_DATA,
		.header.DLC=8,
};
CAN_TxFrameTypeDef LK_L9025_Right_TxFrame ={
    .hcan = &hcan1,
		.header.StdId=0x142,
		.header.IDE=CAN_ID_STD,
		.header.RTR=CAN_RTR_DATA,
		.header.DLC=8,
};

CAN_TxFrameTypeDef SuperCapTxFrame ={
    .hcan = &hcan1,
		.header.StdId=0x100,
		.header.IDE=CAN_ID_STD,
		.header.RTR=CAN_RTR_DATA,
		.header.DLC=8,
};
CAN_TxFrameTypeDef RefreeInfoTxFrame ={
    .hcan = &hcan1,
		.header.StdId=0x300,
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

  /* Enable CAN2 FIFO1 interrupts */
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}
//------------------------------------------------------------------------------

/**
  * @brief  USER function to transmit the Specifies message.
	* @param  Instance: pointer to the CAN Register base address
	* @param  data: pointer to the CAN transmit data
  * @retval None
  */
//void USER_CAN_TxMessage(CAN_TypeDef *Instance,uint32_t StdId,uint8_t data[8],uint8_t length)
//{
//  static uint32_t TxMailbox = 0;
//	
//  /* Add a message to the first free Tx mailbox and activate the corresponding transmission request. */
//	if(Instance == CAN1)
//	{
//    CAN1_FrameTxInstance.StdId = StdId;
//    CAN1_FrameTxInstance.DLC = length;
//		HAL_CAN_AddTxMessage(&hcan1, &CAN1_FrameTxInstance, data, &TxMailbox);
//	}
//	else if(Instance == CAN2)
//	{
//    CAN2_FrameTxInstance.StdId = StdId;
//    CAN2_FrameTxInstance.DLC = length;
//		HAL_CAN_AddTxMessage(&hcan2, &CAN2_FrameTxInstance, data, &TxMailbox);
//	}
//}
//------------------------------------------------------------------------------
void USER_CAN_TxMessage(CAN_TxFrameTypeDef *TxHeader)
{
	
	static uint32_t TxMailbox = 0;

   //while( HAL_CAN_GetTxMailboxesFreeLevel( &hcan1 ) == 0 );
	//???????CAN??
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
	

	
	if(*StdId == 0x302 ){
	remote_ctrl.rc.s[0]	= ((data[0] &0x0C) >> 2); 
	remote_ctrl.rc.s[1] = (data[0] &0x03);
	remote_ctrl.rc_lost = ((data[0] &0x10)>>4); 	
  Control_Info.Shoot_Mode   =  ((data[0] & 0x60)>>5);
	Control_Info.Cover_Switch = ((data[0] & 0x80)>>7);
	remote_ctrl.rc.ch[3] = - ((int16_t) data[1] << 8  | data[2]) ;
	Control_Info.IF_Aiming_Enable   =  data[3] ;
	remote_ctrl.rc.ch[4] = data[5];
	remote_ctrl.mouse.press_r =  data[4] ;
	remote_ctrl.key.v    = (uint16_t)data[6] << 8 | data[7];
 }
  else if(*StdId == 0x142 || *StdId ==  0x141){
       LK_Motor_Info_Update(StdId,data,&LK_Motor[Left_Wheel]);  
		   LK_Motor_Info_Update(StdId,data,&LK_Motor[Right_Wheel]);
	}else if(*StdId == 0x206){
   
	    DJI_Motor_Info_Update(StdId,data,&DJI_Yaw_Motor);
  }else 	if(*StdId==0x102){
		SuperCap_Info.Cap_Percent = (uint8_t)data[0];
		SuperCap_Info.Cap_Mode   = (uint8_t)data[1];
		SuperCap_Info.Cap_V  = ((int16_t)data[2]<<8 |(int16_t)data[3])/1000.f;
		SuperCap_Info.IN_C   = ((int16_t)data[4]<<8 |(int16_t)data[5])/1000.f;
		SuperCap_Info.P_Set_Send =(uint8_t)data[7];
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
static void CAN2_RxFifo1RxHandler(uint32_t *StdId,uint8_t data[8])
{

switch(data[0]&0x0F){
	  case 0x01:
				 Damiao_Motor_Info_Update(data,&Damiao_Motor[0]);
         Damiao_Motor[0].Data.Position =  3.641593f + Damiao_Motor[0].Data.Position;
				 Damiao_Motor[0].Data.Angle =  Damiao_Motor[0].Data.Position * 57.2957732f;
			 break;
    
		case 0x02:
				Damiao_Motor_Info_Update(data,&Damiao_Motor[1]);
			  Damiao_Motor[1].Data.Position = Damiao_Motor[1].Data.Position - 0.38f;
			 break;
	  
		case 0x03:
				Damiao_Motor_Info_Update(data,&Damiao_Motor[2]);
			  Damiao_Motor[2].Data.Position = Damiao_Motor[2].Data.Position - 0.5f;
			 break;
    
		case 0x04:
				Damiao_Motor_Info_Update(data,&Damiao_Motor[3]);
			  Damiao_Motor[3].Data.Position =  3.521593f + Damiao_Motor[3].Data.Position;
			  Damiao_Motor[3].Data.Angle =  Damiao_Motor[3].Data.Position * 57.2957732f;
			 break;
	 
		default:
			 break;
}


}
//------------------------------------------------------------------------------

/**
  * @brief  Rx FIFO 0 message pending callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
//void HAL_CAN_RxFifoMsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//  /* Get an CAN frame from the Rx FIFO zone into the message RAM. */
//  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN1_RxInstance.header, CAN1_RxInstance.Data);
//  /* judge the instance of receive frame data */
//  CAN1_RxFifo0RxHandler(&CAN1_RxInstance.header.StdId,CAN1_RxInstance.Data);
//  
// 
//}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* Get an CAN frame from the Rx FIFO zone into the message RAM. */
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN1_RxInstance.header, CAN1_RxInstance.Data);
  /* judge the instance of receive frame data */
  CAN1_RxFifo0RxHandler(&CAN1_RxInstance.header.StdId,CAN1_RxInstance.Data);
}


//------------------------------------------------------------------------------
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* Get an CAN frame from the Rx FIF1 zone into the message RAM. */
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &CAN2_RxInstance.header, CAN2_RxInstance.Data);
  /* judge the instance of receive frame data */
	CAN2_RxFifo1RxHandler(&CAN2_RxInstance.header.StdId,CAN2_RxInstance.Data);
  
}