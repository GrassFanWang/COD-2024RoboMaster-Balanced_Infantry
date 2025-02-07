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

/* Includes ------------------------------------------------------------------*/
#include "motor.h"
#include "bsp_can.h"
#include "lpf.h"
#include "arm_math.h"
#include "math.h"
LK_L9025_Info_Typedef LK_Motor[LK_MOTOR_USAGE_NUM]=
{
	[Left_Wheel]=
	{
		.Type = LK_L9025,
		.CANFrame.RxStdId = 0x141,
		.lost = 1,
	},
	[Right_Wheel]=
	{
		.Type = LK_L9025,
		.CANFrame.RxStdId = 0x142, 
		.lost = 1,
	},
};

DJI_Motor_Info_Typedef DJI_Yaw_Motor =
{
	  .Type = DJI_GM6020,
		.CANFrame.RxStdId = 0x206,
    .lost = 1,

};
 
 Damiao_Motor_Info_Typedef Damiao_Motor[Damiao_MOTOR_USAGE_NUM]=
{
    [L_A_Joint]=
	  {
		  .CANFrame.TxStdId = 0x01,
			.CANFrame.RxStdId = 0x01,
			.lost = 1,
		},
    [L_P_Joint]=
		{
		   .CANFrame.TxStdId = 0x02,
			 .CANFrame.RxStdId = 0x02,
			 .lost = 1,
		},
		[R_A_Joint]=
		{
		   .CANFrame.TxStdId = 0x03,
			 .CANFrame.RxStdId = 0x03,
			 .lost = 1,
		},
		[R_P_Joint]=
		{
			 .CANFrame.RxStdId = 0x04,
		   .CANFrame.TxStdId = 0x04,
			 .lost = 1,
		},
};



//};
/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  transform the encoder(0-8192) to anglesum(3.4E38)
  */
static float encoder_to_anglesum(Motor_GeneralInfo_Typedef *,float ,uint16_t );
/**
  * @brief  transform the encoder(0-8192) to angle(-180-180)
  */
float encoder_to_angle(Motor_GeneralInfo_Typedef *,float ,uint16_t );
/** 
  * @brief  Judge the DJI Motor state
  */
static float uint_to_float(int X_int, float X_min, float X_max, int Bits);
static int float_to_uint(float x, float x_min, float x_max, int bits);

static void DJI_Motor_ErrorHandler(DJI_Motor_Info_Typedef *);

/**
  * @brief  Update the DJI motor Information
  * @param  StdId  pointer to the specifies the standard identifier.
  * @param  rxBuf  pointer to the can receive data
  * @param  DJI_Motor pointer to a DJI_Motor_Info_t structure 
  *         that contains the information of DJI motor
  * @retval None
  */
void DJI_Motor_Info_Update(uint32_t *StdId, uint8_t *rxBuf,DJI_Motor_Info_Typedef *DJI_Motor)
{
	/* check the StdId */
	if(*StdId != DJI_Motor->CANFrame.RxStdId) return;
	
	/* transforms the  general motor data */
	DJI_Motor->Data.temperature = rxBuf[6];
	DJI_Motor->Data.encoder  = ((int16_t)rxBuf[0] << 8 | (int16_t)rxBuf[1]);
	DJI_Motor->Data.velocity = ((int16_t)rxBuf[2] << 8 | (int16_t)rxBuf[3]);
	DJI_Motor->Data.current  = ((int16_t)rxBuf[4] << 8 | (int16_t)rxBuf[5]);
	/* Judge the motor error	*/
	DJI_Motor_ErrorHandler(DJI_Motor);
  DJI_Motor->Online_cnt = 250;
	DJI_Motor->lost = 0;
  /* update the txframe id and index */
  if(DJI_Motor->Data.Initlized != true)
  {
    if(DJI_Motor->CANFrame.RxStdId > DJI_RxFrame_MIDDLE)
    {
      DJI_Motor->CANFrame.TxStdId = DJI_TxFrame_HIGH;
      DJI_Motor->CANFrame.FrameIndex = 2*(DJI_Motor->CANFrame.RxStdId - DJI_RxFrame_MIDDLE - 0x01U);
    }
    else if(DJI_Motor->CANFrame.RxStdId > DJI_TxFrame_LOW)
    {
      DJI_Motor->CANFrame.TxStdId = DJI_TxFrame_LOW; 
      DJI_Motor->CANFrame.FrameIndex = 2*(DJI_Motor->CANFrame.RxStdId - DJI_TxFrame_LOW - 0x01U);
    }
  }
	
	/* transform the encoder to anglesum */
	switch(DJI_Motor->Type)
	{
		case DJI_GM6020:
	
		
		DJI_Motor->Data.angle = encoder_to_angle(&DJI_Motor->Data,1.f,8192);
		break;
	
		case DJI_M3508:
			DJI_Motor->Data.angle = encoder_to_anglesum(&DJI_Motor->Data,3591.f/187.f,8192);
		break;
		
		case DJI_M2006:
			DJI_Motor->Data.angle = encoder_to_anglesum(&DJI_Motor->Data,36.f,8192);
		break;
		
		default:break;
	}
}
//------------------------------------------------------------------------------
/**
  * @brief  Update the Damiao motor Information
  * @param  StdId  pointer to the specifies the standard identifier.
  * @param  rxBuf  pointer to the can receive data
  * @param  LK_Motor pointer to a LK_L9025_Info_Typedef structure that contains the information of LK motor
  * @retval None
  */
void Damiao_Motor_Info_Update(uint8_t *rxBuf,Damiao_Motor_Info_Typedef *Damiao_Motor)
{
	/* Judge the StdId */
  	Damiao_Motor->ID = rxBuf[0]&0x0F;
	 if(Damiao_Motor->ID != Damiao_Motor->CANFrame.RxStdId) return;
	  Damiao_Motor->Data.State = rxBuf[0]>>4;
		Damiao_Motor->Data.P_int = ((uint16_t)(rxBuf[1]) <<8) | ((uint16_t)(rxBuf[2]));
		Damiao_Motor->Data.V_int = ((uint16_t)(rxBuf[3]) <<4) | ((uint16_t)(rxBuf[4])>>4);
		Damiao_Motor->Data.T_int = ((uint16_t)(rxBuf[4]&0xF) <<8) | ((uint16_t)(rxBuf[5]));
		Damiao_Motor->Data.Torque=  uint_to_float(Damiao_Motor->Data.T_int,-45,45,12);
		Damiao_Motor->Data.Position=uint_to_float(Damiao_Motor->Data.P_int,-3.141593,3.141593,16);
    Damiao_Motor->Data.Velocity=uint_to_float(Damiao_Motor->Data.V_int,-50,50,12);

    Damiao_Motor->Data.Temperature_MOS   = (float)(rxBuf[6]);
		Damiao_Motor->Data.Temperature_Rotor = (float)(rxBuf[7]);
	  Damiao_Motor->Online_cnt = 250;
	  Damiao_Motor->lost = 0;

}



/**
  * @brief  Update the LK motor Information
  * @param  StdId  pointer to the specifies the standard identifier.
  * @param  rxBuf  pointer to the can receive data
  * @param  LK_Motor pointer to a LK_L9025_Info_Typedef structure that contains the information of LK motor
  * @retval None
  */
void LK_Motor_Info_Update(uint32_t *StdId, uint8_t *rxBuf,LK_L9025_Info_Typedef *LK_Motor)
{
	/* Judge the StdId */
	if(*StdId != LK_Motor->CANFrame.RxStdId)
  {
    return;
  }

	   LK_Motor->Data.Temperature = rxBuf[1];
	   LK_Motor->Data.Current  = ((int16_t)(rxBuf[2]) | (int16_t)(rxBuf[3]<<8));
	   LK_Motor->Data.Velocity = ((int16_t)(rxBuf[4]) | (int16_t)(rxBuf[5]<<8));
	   LK_Motor->Data.Rad_Velocity = LK_Motor->Data.Velocity*Angle_to_rad;
	   LK_Motor->Data.Encoder  =   ((int16_t)(rxBuf[6]) | (int16_t)(rxBuf[7]<<8));
     LK_Motor->Online_cnt = 250;
	   LK_Motor->lost = 0;
 
}
//------------------------------------------------------------------------------

/**
  * @brief  transform the encoder(0-8192) to anglesum(3.4E38)
  * @param  *Info        pointer to a Motor_GeneralInfo_Typedef structure that 
	*					             contains the infomation for the specified motor
  * @param  torque_ratio the specified motor torque ratio
  * @param  MAXencoder   the specified motor max encoder number
  * @retval anglesum
  */
static float encoder_to_anglesum(Motor_GeneralInfo_Typedef *Info,float torque_ratio,uint16_t MAXencoder)
{
  float res1 = 0,res2 =0;
  
  if(Info == NULL) return 0;
  
  /* Judge the motor Initlized */
  if(Info->Initlized != true)
  {
    /* update the last encoder */
    Info->last_encoder = Info->encoder;

    /* reset the angle */
    Info->angle = 0;

    /* Set the init flag */
    Info->Initlized = true;
  }
  
  /* get the possiable min encoder err */
  if(Info->encoder < Info->last_encoder)
  {
      res1 = Info->encoder - Info->last_encoder + MAXencoder;
  }
  else if(Info->encoder > Info->last_encoder)
  {
      res1 = Info->encoder - Info->last_encoder - MAXencoder;
  }
  res2 = Info->encoder - Info->last_encoder;
  
  /* update the last encoder */
  Info->last_encoder = Info->encoder;
  
  /* transforms the encoder data to tolangle */
	if(fabsf(res1) > fabsf(res2))
	{
		Info->angle += (float)res2/(MAXencoder*torque_ratio)*360.f;
	}
	else
	{
		Info->angle += (float)res1/(MAXencoder*torque_ratio)*360.f;
	}
  
  return Info->angle;
}
//------------------------------------------------------------------------------

/**
  * @brief  float loop constrain
  * @param  Input    the specified variables
  * @param  minValue minimum number of the specified variables
  * @param  maxValue maximum number of the specified variables
  * @retval variables
  */
static float f_loop_constrain(float Input, float minValue, float maxValue)
{
  if (maxValue < minValue)
  {
    return Input;
  }
  
  float len = maxValue - minValue;    

  if (Input > maxValue)
  {
      do{
          Input -= len;
      }while (Input > maxValue);
  }
  else if (Input < minValue)
  {
      do{
          Input += len;
      }while (Input < minValue);
  }
  return Input;
}
//------------------------------------------------------------------------------

/**
  * @brief  transform the encoder(0-8192) to angle(-180-180)
  * @param  *Info        pointer to a Motor_GeneralInfo_Typedef structure that 
	*					             contains the infomation for the specified motor
  * @param  torque_ratio the specified motor torque ratio
  * @param  MAXencoder   the specified motor max encoder number
  * @retval angle
  */
float encoder_to_angle(Motor_GeneralInfo_Typedef *Info,float torque_ratio,uint16_t MAXencoder)
{	
  float encoder_err = 0.f;
  
  /* check the motor init */
  if(Info->Initlized != true)
  {
    /* update the last encoder */
    Info->last_encoder = Info->encoder;

    /* reset the angle */
    Info->angle = Info->encoder/(MAXencoder*torque_ratio)*360.f;

    /* config the init flag */
    Info->Initlized = true;
  }
  
  encoder_err = Info->encoder - Info->last_encoder;
  
  /* 0 -> MAXencoder */		
  if(encoder_err > MAXencoder*0.5f)
  {
    Info->angle += (float)(encoder_err - MAXencoder)/(MAXencoder*torque_ratio)*360.f;
  }
  /* MAXencoder-> 0 */		
  else if(encoder_err < -MAXencoder*0.5f)
  {
    Info->angle += (float)(encoder_err + MAXencoder)/(MAXencoder*torque_ratio)*360.f;
  }
  else
  {
    Info->angle += (float)(encoder_err)/(MAXencoder*torque_ratio)*360.f;
  }
  
  /* update the last encoder */
  Info->last_encoder = Info->encoder;
  
  /* loop constrain */
  Info->angle = f_loop_constrain(Info->angle,-180.f,180.f);

  return Info->angle;
}
//------------------------------------------------------------------------------

/** 
  * @brief  Judge the DJI Motor state
  * @param  *DJI_Motor pointer to a DJI_Motor_Info_Typedef structure that contains
  *                    the configuration information for the specified motor.  
  * @retval None
  */
static void DJI_Motor_ErrorHandler(DJI_Motor_Info_Typedef *DJI_Motor)
{
	/* Judge the DJI motor temperature */
	if(DJI_Motor->Data.temperature > 80)
	{
    DJI_Motor->ERRORHandler.ErrorCount++;

    if(DJI_Motor->ERRORHandler.ErrorCount > 200)
    {
      DJI_Motor->ERRORHandler.Status = MOTOR_OVER_TEMPERATURE;
      DJI_Motor->ERRORHandler.ErrorCount = 0;
    }
	}
  else
	{
    DJI_Motor->ERRORHandler.ErrorCount = 0;	
	}
}

void Damiao_Motor_Enable(uint8_t ID){
	 uint8_t Motor_ID =ID-1;
   JointTxFrame[Motor_ID].Data[0] = 0xFF;
   JointTxFrame[Motor_ID].Data[1] = 0xFF;
   JointTxFrame[Motor_ID].Data[2] = 0xFF;
   JointTxFrame[Motor_ID].Data[3] = 0xFF;
   JointTxFrame[Motor_ID].Data[4] = 0xFF;
   JointTxFrame[Motor_ID].Data[5] = 0xFF;
   JointTxFrame[Motor_ID].Data[6] = 0xFF;
	 JointTxFrame[Motor_ID].Data[7] = 0xFC;
	 USER_CAN_TxMessage(&JointTxFrame[Motor_ID]);
}

 void Damiao_Motor_DisEnable(uint8_t ID){
	 uint8_t Motor_ID =ID-1;
   JointTxFrame[Motor_ID].Data[0] = 0xFF;
   JointTxFrame[Motor_ID].Data[1] = 0xFF;
   JointTxFrame[Motor_ID].Data[2] = 0xFF;
   JointTxFrame[Motor_ID].Data[3] = 0xFF;
   JointTxFrame[Motor_ID].Data[4] = 0xFF;
   JointTxFrame[Motor_ID].Data[5] = 0xFF;
   JointTxFrame[Motor_ID].Data[6] = 0xFF;
	 JointTxFrame[Motor_ID].Data[7] = 0xFD;
	 USER_CAN_TxMessage(&JointTxFrame[Motor_ID]);
}
 
void Damiao_Motor_CAN_Send(uint8_t ID,float Postion, float Velocity, float KP, float KD, float Torque){
   static uint16_t Postion_Tmp,Velocity_Tmp,Torque_Tmp,KP_Tmp,KD_Tmp;
	 uint8_t Motor_ID = ID-1;
   Postion_Tmp  =  float_to_uint(Postion,-3.141593f,3.141593f,16) ;
   Velocity_Tmp =  float_to_uint(Velocity,-50,50,12);
	 KP_Tmp = float_to_uint(KP,0,500,12);
	 KD_Tmp = float_to_uint(KD,0,5,12);
   Torque_Tmp = float_to_uint(Torque,-45,45,12);
	 JointTxFrame[Motor_ID].Data[0] = (uint8_t)(Postion_Tmp>>8);
	 JointTxFrame[Motor_ID].Data[1] = (uint8_t)(Postion_Tmp);
	 JointTxFrame[Motor_ID].Data[2] = (uint8_t)(Velocity_Tmp>>4);
	 JointTxFrame[Motor_ID].Data[3] = (uint8_t)((Velocity_Tmp&0x0F)<<4) | (uint8_t)(KP_Tmp>>8);
	 JointTxFrame[Motor_ID].Data[4] = (uint8_t)(KP_Tmp);
	 JointTxFrame[Motor_ID].Data[5] = (uint8_t)(KD_Tmp>>4);
	 JointTxFrame[Motor_ID].Data[6] = (uint8_t)((KD_Tmp&0x0F)<<4) | (uint8_t)(Torque_Tmp>>8);
	 JointTxFrame[Motor_ID].Data[7] = (uint8_t)(Torque_Tmp);
   USER_CAN_TxMessage(&JointTxFrame[Motor_ID]);
}
//------------------------------------------------------------------------------
static float uint_to_float(int X_int, float X_min, float X_max, int Bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = X_max - X_min;
    float offset = X_min;
    return ((float)X_int)*span/((float)((1<<Bits)-1)) + offset;
}

static int float_to_uint(float x, float x_min, float x_max, int bits){
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}





