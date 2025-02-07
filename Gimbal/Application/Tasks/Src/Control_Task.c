/* USER CODE BEGIN Header */
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

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "Control_Task.h"
#include "INS_Task.h"
#include "vision_task.h"
#include "remote_control.h"
#include "motor.h"
#include "referee_info.h"
#include "tim.h"

/* Private variables ---------------------------------------------------------*/

Control_Info_Typedef Control_Info={
 .Gimbal = {
   .Measure.Yaw_Angle = &INS_Info.yaw_angle,
	 .Measure.Pitch_Angle = &INS_Info.angle[2],
	 .Measure.Yaw_Gyro = &INS_Info.yaw_gyro,
   .Measure.Pitch_Gyro = &INS_Info.gyro[0],
 },
 .Shoot = {
   .Measure.ShootLeftSpeed = &DJI_Motor[Left_Shoot].Data.velocity,
   .Measure.ShootRightSpeed = &DJI_Motor[Right_Shoot].Data.velocity,
	 .Measure.TriggerSpeed = & DJI_Motor[Trigger].Data.velocity,
	 .Measure.TriggerAngle = & DJI_Motor[Trigger].Data.angle,
	 .Trigger_Buf = 8,
 },
 
};
/* Private function prototypes -----------------------------------------------*/


static void Control_Task_Init(void);
static void Control_Mode_Update(Control_Info_Typedef *Control_Info);
static void Control_Target_Update(Control_Info_Typedef *Control_Info);
static void Control_Info_Update(Control_Info_Typedef *Control_Info);
static float SpeedAdapt(float real_S , float min_S, float max_S,float up_num , float down_num);
static void Shooter_TargetSpeed_Update(Control_Info_Typedef *Control_Info);
//static uint16_t Trigger_Speed_deliver(uint16_t cooling_rate);
static float Trigger_Speed_deliver(uint16_t cooling_rate);
  static void Check_Damiao_Motor_Online();

  bool Press_L_Flag = 0;
PID_Info_TypeDef Left_Shoot_PID;
PID_Info_TypeDef Right_Shoot_PID;
PID_Info_TypeDef Yaw_PID[2];
PID_Info_TypeDef Pitch_PID[2];
PID_Info_TypeDef Trigger_PID[2];
static float Trigger_PID_Param[6] ={9.f,0.1f,0,0,2000,12000};
static float Left_Shoot_PID_Param[6] ={13.f,0.1f,0,0,5000,12000};
static float RIght_Shoot_PID_Param[6] ={13.f,0.1f,0,0,5000,12000};
static float Yaw_Angle_PID_Param[6] ={60.f,0.f,0,0,5000,15000};
static float Yaw_Velocity_PID_Param[6] ={100.f,0.1f,0,0,5000,16000};
static float Pitch_Angle_PID_Param[6] ={80.f,0.f,0,0,0,20};
static float Pitch_Velocity_PID_Param[6] ={-3.f,0.1f,0,0,5,10};
static float Trigger_Angle_PID_Param[6]={200,0,0,0,0,4000,};
static float Trigger_Velocity_PID_Param[6]={13,0.24f,0,0,1000,10000,};
/* USER CODE BEGIN Header_Control_Task */
/**
* @brief Function implementing the StartControlTas thread.
* @param argument: Not used
* @retval Noneppp
*/
/* USER CODE END Header_Control_Task */
int16_t Target1,Target2,Target3 = 0;
float Yaw_Err;
float yaw,pit,rol;
void Control_Task(void const * argument)
{
  /* USER CODE BEGIN Control_Task */
   TickType_t systick = 0;

  Control_Task_Init();
  /* Infinite loop */
  for(;;)
  {
    systick = osKernelSysTick();

	  Control_Mode_Update(&Control_Info);
		Control_Target_Update(&Control_Info);
		Control_Info_Update(&Control_Info);
		
		if(remote_ctrl.rc.s[1] == 1 ){	 
		static bool R_Flag = 0;
    if(remote_ctrl.key.set.R == 1 && R_Flag==0){
		    Control_Info.Cover_Switch = !   Control_Info.Cover_Switch;
			  R_Flag = 1;
		 }else if(remote_ctrl.key.set.R==0) R_Flag = 0;
	 }else if(remote_ctrl.rc.s[1] != 1){
	   if(SignBit(remote_ctrl.rc.ch[4]) == -1){
		  Control_Info.Cover_Switch = 1;
		 }else if(remote_ctrl.rc.ch[4] == 0){
		   Control_Info.Cover_Switch = 0;
		}
	 }
	 
//	if(Control_Info.Gimbal.Mode != GIMBAL_OFF){
//		if(Control_Info.Cover_Switch == 1){
//    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,1900);		
//		}else if (Control_Info.Cover_Switch ==0 ) __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,600);
//	}else __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,600);

 	Check_Damiao_Motor_Online();

	
  if(Damiao_Pitch_Motor.lost == 0){
		Damiao_Motor_CAN_Send(0x01,0,0,0,0,Control_Info.Pit_SendValue);
	}
		
	
	
    osDelayUntil(&systick,1);
  }
  /* USER CODE END Control_Task */
}

static void Control_Task_Init(void){

  PID_Init(&Trigger_PID[0],PID_POSITION,Trigger_Angle_PID_Param);
	PID_Init(&Trigger_PID[1],PID_POSITION,Trigger_Velocity_PID_Param);
  
	PID_Init(&Left_Shoot_PID,PID_POSITION,Left_Shoot_PID_Param);
  PID_Init(&Right_Shoot_PID,PID_POSITION,RIght_Shoot_PID_Param);
  
	PID_Init(&Yaw_PID[0],PID_POSITION,Yaw_Angle_PID_Param);
  PID_Init(&Yaw_PID[1],PID_POSITION,Yaw_Velocity_PID_Param);
  
	PID_Init(&Pitch_PID[0],PID_POSITION,Pitch_Angle_PID_Param);
  PID_Init(&Pitch_PID[1],PID_POSITION,Pitch_Velocity_PID_Param);

}

static void Control_Mode_Update(Control_Info_Typedef *Control_Info){
  
	if(remote_ctrl.rc.s[0] == 3  || remote_ctrl.rc.s[0] == 1){
		
		if(remote_ctrl.mouse.press_r != 1){  
			
			Control_Info->Gimbal.Mode = GIMBAL_IMU;
	
		}else Control_Info->Gimbal.Mode = GIMBAL_VISION;
		
  }else{
		 
			Control_Info->Gimbal.Mode = GIMBAL_OFF;

		}

		if(remote_ctrl.rc.s[0] == 1){
		 
    
			Control_Info->Shoot.Mode = SHOOTER_BURSTS;

		}else{
		
			Control_Info->Shoot.Mode = SHOOTER_OFF;

		}
if(Control_Info->Shoot.Mode != SHOOTER_OFF){		
		 
if(remote_ctrl.mouse.press_r != 1){	
	
	  static bool Q_Flag = 0;
    
	  if(remote_ctrl.key.set.Q == 1 && Q_Flag==0){
			
  			Control_Info->Shoot.Shoot_Mode_Change  = !Control_Info->Shoot.Shoot_Mode_Change;
				Q_Flag = 1;
	
		}else if(remote_ctrl.key.set.Q==0) Q_Flag = 0;
  
		 if( Control_Info->Shoot.Shoot_Mode_Change == 0)   Control_Info->Shoot.Mode = SHOOTER_BURSTS;
	
		  else  if( Control_Info->Shoot.Shoot_Mode_Change == 1)   Control_Info->Shoot.Mode = SHOOTER_SINGLE;
	 
}else {
	 
	 Control_Info->Shoot.Mode = SHOOTER_VISION;
	 
   }
  }
}
static void Control_Target_Update(Control_Info_Typedef *Control_Info){
	
if(Control_Info->Gimbal.Mode != GIMBAL_OFF){
	
 
if(Control_Info->Gimbal.Mode == GIMBAL_IMU){

	 HAL_GPIO_WritePin (GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
   
	 Control_Info->Gimbal.Target.Yaw_Angle   -= remote_ctrl.rc.ch[0]*0.0003f + remote_ctrl.mouse.x * 0.0008f;
	  
   Control_Info->Gimbal.Target.Pitch_Angle += remote_ctrl.rc.ch[1]*0.00002f - remote_ctrl.mouse.y * 0.000023f;
 
	 if( Control_Info->Gimbal.Target.Yaw_Angle > 180)   Control_Info->Gimbal.Target.Yaw_Angle -= 360.f;
	 else if( Control_Info->Gimbal.Target.Yaw_Angle < -180)  Control_Info->Gimbal.Target.Yaw_Angle+= 360.f; 
	
	
 }else if(Control_Info->Gimbal.Mode == GIMBAL_VISION){
   
   HAL_GPIO_WritePin (GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
	 
	 if(Vision_Info.IF_Aiming_Enable == 1){
	  
	  Control_Info->Gimbal.Target.Yaw_Angle = Vision_Info.target_Yaw ;
	

	 Control_Info->Gimbal.Target.Pitch_Angle = Vision_Info.target_Pitch*DegreesToRadians;
 
	 }else{
	  
	 Control_Info->Gimbal.Target.Yaw_Angle -= remote_ctrl.rc.ch[0]*0.0003f + remote_ctrl.mouse.x * 0.0008f;
	  
 	 Control_Info->Gimbal.Target.Pitch_Angle += remote_ctrl.rc.ch[1]*0.00002f - remote_ctrl.mouse.y * 0.000023f;
		 
	 } 
		
   if( Control_Info->Gimbal.Target.Yaw_Angle > 180)   Control_Info->Gimbal.Target.Yaw_Angle -= 360.f;
	 else if( Control_Info->Gimbal.Target.Yaw_Angle < -180)  Control_Info->Gimbal.Target.Yaw_Angle+= 360.f; 

 }
 
 if(Control_Info->Chassis_Week_Flag == 2){
 
 VAL_LIMIT(Control_Info->Gimbal.Target.Pitch_Angle,-0.7f,	0.18f);
 
 }
 
  if(Control_Info->Chassis_Week_Flag == 1){
 
 VAL_LIMIT(Control_Info->Gimbal.Target.Pitch_Angle,-0.15f,0.73f);
 
 }
 
 if(Control_Info->Chassis_Week_Flag == 0){
 
 VAL_LIMIT(Control_Info->Gimbal.Target.Pitch_Angle,-0.44f,0.44f);
 
 }
 
 
}else if(Control_Info->Gimbal.Mode == GIMBAL_OFF){

  Control_Info->Gimbal.Target.Yaw_Angle =  *Control_Info->Gimbal.Measure.Yaw_Angle;
	
 	Control_Info->Gimbal.Target.Pitch_Angle =  *Control_Info->Gimbal.Measure.Pitch_Angle;
	

}

 
 

 
 
 
 if(Control_Info->Shoot.Mode != SHOOTER_OFF){
  
	 Control_Info->Shoot.Target.ShootSpeed = SHOOT_SPEED_30M_S;
	
	 Shooter_TargetSpeed_Update(Control_Info);
 
	 if(Control_Info->Shoot.Mode ==SHOOTER_VISION){
		 
		  if(remote_ctrl.mouse.press_l == 1 ) {
		
		Control_Info->Shoot.Target.TriggerSpeed = Vision_Info.IF_Fire_Accept *(- 5000 * Trigger_Speed_deliver(Control_Info->Shoot.Refree.Shooter_barrel_cooling_value));
  
	 }else{
	 
	  	Control_Info->Shoot.Target.TriggerSpeed = 0;
		 
	 }
	 
		 
	 }else if(Control_Info->Shoot.Mode == SHOOTER_BURSTS){
	
		if(remote_ctrl.mouse.press_l == 1 || SignBit(remote_ctrl.rc.ch[4]) == 1) {
		 
		 Control_Info->Shoot.Target.TriggerSpeed = -4000 * Trigger_Speed_deliver(Control_Info->Shoot.Refree.Shooter_barrel_cooling_value);
    
	  }else{
	 
	  	Control_Info->Shoot.Target.TriggerSpeed = 0;
		 
	   }

	  Control_Info->Shoot.Target.TriggerAngle = *Control_Info->Shoot.Measure.TriggerAngle;
	  
  
	 }else if(Control_Info->Shoot.Mode == SHOOTER_SINGLE){

	 static bool Press_L_Flag = 0;
	 if(remote_ctrl.mouse.press_l == 1 && Press_L_Flag == 0){
	
		 Control_Info->Shoot.Target.TriggerAngle -= (float) (360.f/Control_Info->Shoot.Trigger_Buf);
     Press_L_Flag = 1;
	 
	 }else if(remote_ctrl.mouse.press_l==0) Press_L_Flag = 0;
	
	  if( Control_Info->Shoot.Target.TriggerAngle > 180.f)  Control_Info->Shoot.Target.TriggerAngle  -=   360.f;
    else if ( Control_Info->Shoot.Target.TriggerAngle < -180.f)  Control_Info->Shoot.Target.TriggerAngle +=   360.f;
	 
	 
	} 

 }else{
  Control_Info->Shoot.Target.ShootSpeed= 0;
  Control_Info->Shoot.Target.TriggerSpeed = 0;
  Control_Info->Shoot.Target.TriggerAngle = *Control_Info->Shoot.Measure.TriggerAngle;
}
 







}
static void Control_Info_Update(Control_Info_Typedef *Control_Info){
	
 if(Control_Info->Gimbal.Mode == GIMBAL_IMU || Control_Info->Gimbal.Mode == GIMBAL_VISION){
   
	 Control_Info->Gimbal.Yaw_Err = Control_Info->Gimbal.Target.Yaw_Angle -  *Control_Info->Gimbal.Measure.Yaw_Angle;
	
	 if( Control_Info->Gimbal.Yaw_Err > 180.f )         Control_Info->Gimbal.Yaw_Err -= 360.f;
	 else if( Control_Info->Gimbal.Yaw_Err < -180.f)    Control_Info->Gimbal.Yaw_Err += 360.f;
	 
	 Control_Info->Gimbal.Target.Yaw_Gyro = f_PID_Calculate(&Yaw_PID[0],Control_Info->Gimbal.Yaw_Err,0) ;
	 ;
	 Control_Info->SendValue[Yaw] = f_PID_Calculate(&Yaw_PID[1], Control_Info->Gimbal.Target.Yaw_Gyro, *Control_Info->Gimbal.Measure.Yaw_Gyro);
   
	 Control_Info->Gimbal.Target.Pitch_Gyro = f_PID_Calculate(&Pitch_PID[0],Control_Info->Gimbal.Target.Pitch_Angle,*Control_Info->Gimbal.Measure.Pitch_Angle);
	 Control_Info->Pit_SendValue = f_PID_Calculate(&Pitch_PID[1], Control_Info->Gimbal.Target.Pitch_Gyro, *Control_Info->Gimbal.Measure.Pitch_Gyro);
  
	 Control_Info->Gimbal.Target.Last_Yaw_Angle = Control_Info->Gimbal.Target.Yaw_Angle;
    	


 }else{
   Control_Info->SendValue[Yaw] = 0;
	  Control_Info->Pit_SendValue = 0;
 }


  Control_Info->SendValue[Left_Shoot] =  f_PID_Calculate(&Left_Shoot_PID,-Control_Info->Shoot.Target.ShootSpeed,*Control_Info->Shoot.Measure.ShootLeftSpeed);
  Control_Info->SendValue[Right_Shoot] =  f_PID_Calculate(&Right_Shoot_PID,Control_Info->Shoot.Target.ShootSpeed,*Control_Info->Shoot.Measure.ShootRightSpeed);
 
 
if(Control_Info->Shoot.Mode != SHOOTER_OFF){
	
  if(Control_Info->Shoot.Mode == SHOOTER_SINGLE ){	 

    Control_Info->Shoot.TriggerAngle_Err =  Control_Info->Shoot.Target.TriggerAngle - *Control_Info->Shoot.Measure.TriggerAngle;
		
		if( Control_Info->Shoot.TriggerAngle_Err > 180.f )         Control_Info->Shoot.TriggerAngle_Err -= 360.f;
  	else if( Control_Info->Shoot.TriggerAngle_Err < -180.f)    Control_Info->Shoot.TriggerAngle_Err += 360.f;
		
		Control_Info->Shoot.Target.TriggerSpeed = f_PID_Calculate(&Trigger_PID[0],  Control_Info->Shoot.TriggerAngle_Err,0);
	
	}

   Control_Info->SendValue[Trigger] = f_PID_Calculate(&Trigger_PID[1],Control_Info->Shoot.Target.TriggerSpeed,*Control_Info->Shoot.Measure.TriggerSpeed);
 }else{
    Control_Info->SendValue[Trigger] =  f_PID_Calculate(&Trigger_PID[1],0,DJI_Motor[Trigger].Data.velocity);
     Control_Info->SendValue[Left_Shoot] =  f_PID_Calculate(&Left_Shoot_PID,0,DJI_Motor[Left_Shoot].Data.velocity);
     Control_Info->SendValue[Right_Shoot] =  f_PID_Calculate(&Right_Shoot_PID,0,DJI_Motor[Right_Shoot].Data.velocity);
 }  
}

static void Shooter_TargetSpeed_Update(Control_Info_Typedef *Control_Info)
{
	if(Control_Info->Shoot.Refree.Shoot_initial_speed != Control_Info->Shoot.Last_Firespeed){ 
	
	Control_Info->Shoot.Fire_Speed_Offset += SpeedAdapt(Referee_Info.shoot_data.bullet_speed,28.3f , 28.7f , 25 , 55);
	
	Control_Info->Shoot.Last_Firespeed = Referee_Info.shoot_data.bullet_speed;

	Control_Info->Shoot.Target.ShootSpeed += 	Control_Info->Shoot.Fire_Speed_Offset;
		
	}
}
static float SpeedAdapt(float real_S , float min_S, float max_S,float up_num , float down_num)
{
	float res=0;
	static uint8_t SpeedErr_cnt=0;


  if(real_S < min_S && real_S > 8)
    
	 SpeedErr_cnt++;
  
	else if(real_S >= min_S && real_S <= max_S )
	
  	SpeedErr_cnt = 0;
	
  if(SpeedErr_cnt == 1)
  {
    SpeedErr_cnt = 0;
    res += up_num;
  }
  if(real_S > max_S)
    res -= down_num;
  
	return res;
}


static float Trigger_Speed_deliver(uint16_t cooling_rate){
	
	float Shoot_Heat_Gain = 0;
	uint16_t Shoot_Remain_Cooling_Rate = (Control_Info.Shoot.Refree.shooter_barrel_heat_limit - Control_Info.Shoot.Refree.Shooter_17mm_1_barrel_heat); 
	if( Shoot_Remain_Cooling_Rate >= 100) Shoot_Heat_Gain = 1.f; 
	else if(Shoot_Remain_Cooling_Rate >= 80) Shoot_Heat_Gain = 0.8f;
	else if(Shoot_Remain_Cooling_Rate >= 60) Shoot_Heat_Gain = 0.6f;
  else if(Shoot_Remain_Cooling_Rate >= 40) Shoot_Heat_Gain = 0.4f;
	else if(Shoot_Remain_Cooling_Rate >= 20) Shoot_Heat_Gain = 0.2f;
	else if(Shoot_Remain_Cooling_Rate >= 10) Shoot_Heat_Gain = 0.05f;
  else if(Shoot_Remain_Cooling_Rate >= 0) Shoot_Heat_Gain = 0.f;
  return  Shoot_Heat_Gain;
 }

  static void Check_Damiao_Motor_Online(){
	
  if(Damiao_Pitch_Motor.lost == 0){
	   if( Damiao_Pitch_Motor.Online_cnt>0 ){ 
		 Damiao_Pitch_Motor.Online_cnt--;
   	 if( Damiao_Pitch_Motor.Online_cnt<200){
	   Damiao_Pitch_Motor.lost = 1;
	   }
	  }
  }else if(Damiao_Pitch_Motor.lost == 1){
	
	 Damiao_Pitch_Motor.Online_cnt = 0;
	
	 } 
 
    if(Damiao_Pitch_Motor.lost == 1){
		// Damiao_Motor_Enable(0x01);
		 //osDelay(30);
	}
 
}
	//	uint16_t res = 0;
	
//	if(cooling_rate == 80)
//	{
//			if((Control_Info.Shoot.Refree.shooter_barrel_heat_limit - Control_Info.Shoot.Refree.Shooter_17mm_1_barrel_heat)>= 30)
//				res = TRIGGER_FREQ_14_HZ;
//			else 
//				res = TRIGGER_FREQ_5_HZ;							
//	}
//	else if(cooling_rate >= 50 && cooling_rate<60)
//	{
//		if((Control_Info.Shoot.Refree.shooter_barrel_heat_limit - Control_Info.Shoot.Refree.Shooter_17mm_1_barrel_heat)>= 30)
//				res = TRIGGER_FREQ_10_HZ;
//			else 
//				res = TRIGGER_FREQ_4_HZ;
//	}
//	else if(cooling_rate >= 40&& cooling_rate<50)
//	{
//			if((Control_Info.Shoot.Refree.shooter_barrel_heat_limit - Control_Info.Shoot.Refree.Shooter_17mm_1_barrel_heat) >= 30)
//				res = TRIGGER_FREQ_8_HZ;
//			else 
//				res = TRIGGER_FREQ_3_HZ;
//	}
//	else if(cooling_rate >= 30&& cooling_rate <40)
//	{
//			if((Control_Info.Shoot.Refree.shooter_barrel_heat_limit - Control_Info.Shoot.Refree.Shooter_17mm_1_barrel_heat) >= 30)
//				res = TRIGGER_FREQ_6_HZ;
//			else 
//				res = TRIGGER_FREQ_2_HZ;
//	}
//	else if (cooling_rate >= 20&& cooling_rate <30) 
//	{
//			if((Control_Info.Shoot.Refree.shooter_barrel_heat_limit - Control_Info.Shoot.Refree.Shooter_17mm_1_barrel_heat) >= 30)
//				res = TRIGGER_FREQ_5_HZ;
//			else 
//				res = TRIGGER_FREQ_1_HZ;
//	}else{
//	
//	if((Control_Info.Shoot.Refree.shooter_barrel_heat_limit - Control_Info.Shoot.Refree.Shooter_17mm_1_barrel_heat) >= 20)
//				res = TRIGGER_FREQ_4_HZ;
//			else 
//				res = 0;
//	
//	
//	
//	
//	
//	}
//	
//	return res;
