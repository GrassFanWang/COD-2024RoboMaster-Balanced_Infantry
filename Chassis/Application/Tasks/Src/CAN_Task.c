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
#include "INS_Task.h"
#include "motor.h"
#include "bsp_can.h"
#include "referee_info.h"
#include "VMC_Task.h"
#include "remote_control.h"
SuperCap_Info_t SuperCap_Info;


static void Check_Motor_Online(void);
static void Check_Damiao_Motor_Online(void);
static void	Check_Yaw_Motor_Online(void);
static void Check_LK_Motor_Online(void);
static void Motor_init();
static void Motor_Week();
static void Motor_Slip();
static void Motor_Leg_Adjust();
static void Motor_Torgue();
static void Motor_Move();
static void Referee_Info_Report(void);
static void Super_Cap_Info_Report(void);
PID_Info_TypeDef PID_Velocity;

PID_Info_TypeDef PID_Turn_Velocity1;
PID_Info_TypeDef PID_Turn_Angle1;

float PID_Velocity_Param[6] = {5,0,0,0,0,1000};
float PID_Turn_Angle_Param[6] = {5,0,0,0,0,1000};
float PID_Turn_Velocity_Param[6] = {80,0.07,0,0,2000,1000};


/* USER CODE BEGIN Header_CAN_Task */
/**
* @brief Function implementing the StartCANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Task */
TickType_t systick = 0;

void CAN_Task(void const * argument)
{
  /* USER CODE BEGIN CAN_Task */
	 
  PID_Init(&PID_Velocity,PID_POSITION,PID_Velocity_Param);
	PID_Init(&PID_Turn_Angle1,PID_POSITION,PID_Turn_Angle_Param);
  PID_Init(&PID_Turn_Velocity1,PID_POSITION,PID_Turn_Velocity_Param);
	
  /* Infinite loop */
  for(;;)
  {
		systick = osKernelSysTick();
   
		Check_Motor_Online();
		
	if(Control_Info.Init.Motor_Enable == 0){
	
	  Motor_init();
	
	}else if(Control_Info.Init.Motor_Enable == 1){
	
	if(Control_Info.Sliping == 0){	
	  
		if(Control_Info.Init.Begin_Init == 1){
		    
			if(Control_Info.Init.Joint_Init.IF_Joint_Init == 1)   Motor_Torgue();
		
			else if(Control_Info.Init.Joint_Init.IF_Joint_Init == 0) Motor_Leg_Adjust();
			
		}else if(Control_Info.Init.Begin_Init == 0) {
		
			if(remote_ctrl.rc.s[0] == 2 || remote_ctrl.rc.s[0] == 0){
			
		   Motor_Week();
			
			}else{
			
				static bool Week_Move_Flag = 0;
				
		    if(Week_Move_Flag == 0 && remote_ctrl.key.set.Z == 1){
				
				 Control_Info.Week_Move = ! Control_Info.Week_Move;
				 Week_Move_Flag = 1;
				
				}else if(remote_ctrl.key.set.Z == 0) 	 Week_Move_Flag = 0;	
				
			   if( Control_Info.Week_Move == 1){	
				
			     Motor_Move();
				
			   }else{
				 
				    Motor_Week();
				 
				 }
			
			}	
	  }
	}else{
		
			
			if(Control_Info.Slip_Week == 1){
			

				Motor_Week();
			
			
			}else   Motor_Slip();
		
		
		}
		
	}
	   osDelay(1);
}		
		
		

 

 	 
}
  /* USER CODE END CAN_Task */
uint8_t Chassis_Week_Flag = 0;
static void Referee_Info_Report(void){
	
	
if(remote_ctrl.rc.s[1] == 2 || remote_ctrl.rc.s[1] == 0){
	
	if(INS_Info.pit_angle + 1.5f> 10 ){
	
   Chassis_Week_Flag = 2;		

	}else if(INS_Info.pit_angle + 1.5f < -10){
	
   Chassis_Week_Flag = 1;

	}
		
}else{

  Chassis_Week_Flag = 0;

}
	
  
	RefreeInfoTxFrame.Data[0] = (uint8_t) (Referee_Info.power_heat_data.shooter_17mm_1_barrel_heat >> 8);
  RefreeInfoTxFrame.Data[1] = (uint8_t) (Referee_Info.power_heat_data.shooter_17mm_1_barrel_heat);
  RefreeInfoTxFrame.Data[2] = (uint8_t)((int16_t)(Referee_Info.shoot_data.initial_speed*100 ) >> 8);
  RefreeInfoTxFrame.Data[3] = (uint8_t)((int16_t)(Referee_Info.shoot_data.initial_speed*100 ));
	RefreeInfoTxFrame.Data[4] = (uint8_t) (Referee_Info.robot_status.shooter_barrel_cooling_value);
  RefreeInfoTxFrame.Data[5] = (uint8_t) ((Referee_Info.robot_status.mains_power_gimbal_output) << 2 | Chassis_Week_Flag  );
  RefreeInfoTxFrame.Data[6] = (uint8_t) (Referee_Info.robot_status.shooter_barrel_heat_limit>>8);
	RefreeInfoTxFrame.Data[7] = (uint8_t) (Referee_Info.robot_status.shooter_barrel_heat_limit);
	USER_CAN_TxMessage(&RefreeInfoTxFrame);

}
float  Send_Chassis_Power_Limit;
float  Gain;
static void Super_Cap_Info_Report(void){
	Gain =   ((float)(Referee_Info.power_heat_data.buffer_energy)/60.f);
	Send_Chassis_Power_Limit = (float)(Referee_Info.robot_status.chassis_power_limit - 30) * Gain;
  SuperCapTxFrame.Data[0] = (uint8_t)((int16_t)(Referee_Info.power_heat_data.chassis_power)*100 >>8);
	SuperCapTxFrame.Data[1] = (uint8_t)((int16_t)(Referee_Info.power_heat_data.chassis_power)*100);
	SuperCapTxFrame.Data[2] = (uint8_t)(Referee_Info.power_heat_data.chassis_voltage >>8);
	SuperCapTxFrame.Data[3] = (uint8_t)(Referee_Info.power_heat_data.chassis_voltage );
	SuperCapTxFrame.Data[4] = (uint8_t)(Referee_Info.power_heat_data.chassis_current >>8);
	SuperCapTxFrame.Data[5] = (uint8_t)(Referee_Info.power_heat_data.chassis_current);
	SuperCapTxFrame.Data[6] = (uint8_t)(Send_Chassis_Power_Limit + 30); 
	SuperCapTxFrame.Data[7] = (uint8_t)(Referee_Info.robot_status.mains_power_chassis_output);
	
	USER_CAN_TxMessage(&SuperCapTxFrame);
}
static void Check_Motor_Online(void){

     Check_LK_Motor_Online();
     Check_Damiao_Motor_Online();
    

    if(Control_Info.Motor_Online.LK_Motor +  Control_Info.Motor_Online.Damiao_Motor  == 2)	 Control_Info.Init.Motor_Enable  = 1;  
  
	  else  Control_Info.Init.Motor_Enable  = 0;  


}
static void Check_LK_Motor_Online(void){
	
	for(uint8_t i= 0; i < 2; i++){
		if(LK_Motor[i].lost == 0){
	   if( LK_Motor[i].Online_cnt>0 ){ 
		 LK_Motor[i].Online_cnt--;
   	 if( LK_Motor[i].Online_cnt<200){
	   LK_Motor[i].lost = 1;
	   }
	  }
  }else if(LK_Motor[i].lost == 1){
	
	  LK_Motor[i].Online_cnt = 0;
	
	 } 
 } 
 
 if(LK_Motor[1].lost + LK_Motor[0].lost == 0) Control_Info.Motor_Online.LK_Motor = 1;
 else  Control_Info.Motor_Online.LK_Motor = 0;

  
 
}


static void  Check_Damiao_Motor_Online(void){
	

	for(uint8_t i= 0; i <4; i++){
	
		if( Damiao_Motor[i].Online_cnt>0 ){ 
		 Damiao_Motor[i].Online_cnt--;
   	 if( Damiao_Motor[i].Online_cnt<100){
	   Damiao_Motor[i].lost = 1;
	   }
		}else if(Damiao_Motor[i].lost == 1){
	
	  Damiao_Motor[i].Online_cnt = 0;
	
	 } 
 } 
 
   if(Damiao_Motor[0].lost == 1){ 	
			Damiao_Motor_Enable(0x01);
			osDelay(10);
	}
	 if(Damiao_Motor[1].lost == 1){	
			Damiao_Motor_Enable(0x02);
			osDelay(30);
	}
	  if(Damiao_Motor[2].lost == 1){	
			Damiao_Motor_Enable(0x03);
			osDelay(10);
	}
		 if(Damiao_Motor[3].lost == 1){	
			Damiao_Motor_Enable(0x04);
			osDelay(10);
	}
 
 
	if(Damiao_Motor[0].lost + Damiao_Motor[1].lost + Damiao_Motor[2].lost +Damiao_Motor[3].lost == 0){
	 
		Control_Info.Motor_Online.Damiao_Motor = 1;
	
	}else{
		
		Control_Info.Motor_Online.Damiao_Motor = 0;
	
	}
 

}
static void	Check_Yaw_Motor_Online(void){

  if(DJI_Yaw_Motor.lost == 0 ){
	   if( DJI_Yaw_Motor.Online_cnt>0 ){ 
		     DJI_Yaw_Motor.Online_cnt--;
   	 if( DJI_Yaw_Motor.Online_cnt<200){
	       DJI_Yaw_Motor.lost = 1;
	   }
   }
 }else	if(DJI_Yaw_Motor.lost == 1){
 
    DJI_Yaw_Motor.Online_cnt = 0;
 
  }

  Control_Info.Motor_Online.Yaw_Motor =  ! DJI_Yaw_Motor.lost ;
}

static void Motor_init(void){

 if(systick% 2 == 0)
	{
			 LK_L9025_Left_TxFrame.Data[0] = 0xA1;
	     LK_L9025_Left_TxFrame.Data[4] = (uint8_t)(0);
	     LK_L9025_Left_TxFrame.Data[5] = (uint8_t)(0>>8);
	     USER_CAN_TxMessage(&LK_L9025_Left_TxFrame);
				
			 LK_L9025_Right_TxFrame.Data[0] = 0xA1;
	     LK_L9025_Right_TxFrame.Data[4] = (uint8_t)(0);
	     LK_L9025_Right_TxFrame.Data[5] = (uint8_t)(0>>8);
	     USER_CAN_TxMessage(&LK_L9025_Right_TxFrame);
			     
          if(Damiao_Motor[0].lost == 0){	
			        Damiao_Motor_CAN_Send(0x01,0,0,0,0,0);
	          }
				 
   				if(Damiao_Motor[1].lost == 0){	
			        Damiao_Motor_CAN_Send(0x02,0,0,0,0,0);
	         }
			
	

  }else 
			{
				
				
				   		 
				 Referee_Info_Report();
			 
			

				if(Damiao_Motor[2].lost == 0){	
			        Damiao_Motor_CAN_Send(0x03,0,0,0,0,0);
	          }
				 
   				if(Damiao_Motor[3].lost == 0){	
			        Damiao_Motor_CAN_Send(0x04,0,0,0,0,0);
	          }
					
					Super_Cap_Info_Report();
			
			}	 
			
			  
	
}


static void Motor_Week(void){

	if(systick% 2 == 0)
		{
	   
		 LK_L9025_Left_TxFrame.Data[0] = 0xA1;
		 LK_L9025_Left_TxFrame.Data[4] = (uint8_t)(0);
	   LK_L9025_Left_TxFrame.Data[5] = (uint8_t)(0>>8);
     USER_CAN_TxMessage(&LK_L9025_Left_TxFrame); 
			
		 LK_L9025_Right_TxFrame.Data[0] = 0xA1;
		 LK_L9025_Right_TxFrame.Data[4] = (uint8_t)(0);
	   LK_L9025_Right_TxFrame.Data[5] = (uint8_t)(0>>8);
	   USER_CAN_TxMessage(&LK_L9025_Right_TxFrame); 	
			
		 Damiao_Motor_CAN_Send(0x01,0,0,0,0,0);
		 Damiao_Motor_CAN_Send(0x02,0,0,0,0,0);
			
  	
			
    }else 
		{

        Referee_Info_Report();
			
				Damiao_Motor_CAN_Send(0x03,0,0,0,0,0);
				Damiao_Motor_CAN_Send(0x04,0,0,0,0,0);
			
			  Super_Cap_Info_Report();

		}
     
    
		


}

static void Motor_Move(){
	
     	 f_PID_Calculate(&PID_Turn_Angle1,Control_Info.Yaw_Err,0);		
	     f_PID_Calculate(&PID_Turn_Velocity1,PID_Turn_Angle1.Output,DJI_Yaw_Motor.Data.velocity);
       f_PID_Calculate(&PID_Velocity,remote_ctrl.rc.ch[3]*3 - (Key_W() - Key_S())*2000,LK_Motor[0].Data.Velocity-LK_Motor[1].Data.Velocity); 

	if(systick% 2 == 0)
		{
		
			  
			  LK_L9025_Left_TxFrame.Data[0] = 0xA1;
				LK_L9025_Left_TxFrame.Data[4] = (uint8_t)((int16_t)(PID_Velocity.Output + PID_Turn_Velocity1.Output));
			  LK_L9025_Left_TxFrame.Data[5] = (uint8_t)((int16_t)(PID_Velocity.Output + PID_Turn_Velocity1.Output)>>8);	 
	      USER_CAN_TxMessage(&LK_L9025_Left_TxFrame); 
	      
        LK_L9025_Right_TxFrame.Data[0] = 0xA1;
				LK_L9025_Right_TxFrame.Data[4] = (uint8_t)((int16_t)((-PID_Velocity.Output + PID_Turn_Velocity1.Output)));
			  LK_L9025_Right_TxFrame.Data[5] = (uint8_t)((int16_t)(-PID_Velocity.Output  + PID_Turn_Velocity1.Output)>>8);	 
	      USER_CAN_TxMessage(&LK_L9025_Right_TxFrame); 
				 
				 
			  Damiao_Motor_CAN_Send(0x01,0,0,20,1,0);
				Damiao_Motor_CAN_Send(0x02,0,0,20,1,0);
			
			

       }else{
				
        
 
				Referee_Info_Report();
			
				Damiao_Motor_CAN_Send(0x03,0,0,20,1,0);
				Damiao_Motor_CAN_Send(0x04,0,0,20,1,0);
		  
				 Super_Cap_Info_Report(); 
			 }		
		  
				
	     
				//if(systick % 2 == 0)  




}
static void Motor_Leg_Adjust(){
	
  	if(systick% 2 == 0)
		{
	   
		 LK_L9025_Left_TxFrame.Data[0] = 0xA1;
		 LK_L9025_Left_TxFrame.Data[4] = (uint8_t)(0);
	   LK_L9025_Left_TxFrame.Data[5] = (uint8_t)(0>>8);
		 
		 USER_CAN_TxMessage(&LK_L9025_Left_TxFrame); 	   
	
		 LK_L9025_Right_TxFrame.Data[0] = 0xA1;
		 LK_L9025_Right_TxFrame.Data[4] = (uint8_t)(0);
	   LK_L9025_Right_TxFrame.Data[5] = (uint8_t)(0>>8);
		  
		 USER_CAN_TxMessage(&LK_L9025_Right_TxFrame); 			
			
			
		 Damiao_Motor_CAN_Send(0x01,0,0,10,1,0);
		 Damiao_Motor_CAN_Send(0x02,0,0,10,1,0);
		
			

    }else{
			
		 	
     Referee_Info_Report();
			
		
		 Damiao_Motor_CAN_Send(0x03,0,0,10,1,0);
		 Damiao_Motor_CAN_Send(0x04,0,0,10,1,0);
			
		Super_Cap_Info_Report();	
		
		}
    

		
    
		//if(systick % 2 == 0)  

}	
static void Motor_Torgue(){


		   if(systick% 2 == 0)
		   {
				LK_L9025_Left_TxFrame.Data[0] = 0xA1;
				LK_L9025_Left_TxFrame.Data[4] = (uint8_t)(Control_Info.T_SendValue[0]);
			  LK_L9025_Left_TxFrame.Data[5] = (uint8_t)((Control_Info.T_SendValue[0])>>8);
		
			  USER_CAN_TxMessage(&LK_L9025_Left_TxFrame); 
				 
			  LK_L9025_Right_TxFrame.Data[0] = 0xA1;
				LK_L9025_Right_TxFrame.Data[4] = (uint8_t)(Control_Info.T_SendValue[1]);
			  LK_L9025_Right_TxFrame.Data[5] = (uint8_t)((Control_Info.T_SendValue[1])>>8);
		
			  USER_CAN_TxMessage(&LK_L9025_Right_TxFrame);  
				 
				 
				 
				 Damiao_Motor_CAN_Send(0x01,0,0,0,0,Control_Info.L_Leg_Info.SendValue.T1);
				Damiao_Motor_CAN_Send(0x02,0,0,0,0,Control_Info.L_Leg_Info.SendValue.T2);
			
				 
				 
       }else{
				
				 Referee_Info_Report();
	     
			  

				 
				Damiao_Motor_CAN_Send(0x03,0,0,0,0,Control_Info.R_Leg_Info.SendValue.T2);
				Damiao_Motor_CAN_Send(0x04,0,0,0,0,Control_Info.R_Leg_Info.SendValue.T1);
		    
			  Super_Cap_Info_Report();
			 }		
		
			

}
		

static void Motor_Slip(){

	f_PID_Calculate(&PID_Velocity,0,LK_Motor[0].Data.Velocity-LK_Motor[1].Data.Velocity);  
	
	if(systick% 2 == 0)
		   {
		
			  LK_L9025_Left_TxFrame.Data[0] = 0xA1;
				LK_L9025_Left_TxFrame.Data[4] = (uint8_t)((int16_t)(PID_Velocity.Output ));
			  LK_L9025_Left_TxFrame.Data[5] = (uint8_t)((int16_t)(PID_Velocity.Output )>>8);	 
	      USER_CAN_TxMessage(&LK_L9025_Left_TxFrame); 
				 
				LK_L9025_Right_TxFrame.Data[0] = 0xA1;
				LK_L9025_Right_TxFrame.Data[4] = (uint8_t)((int16_t)(-PID_Velocity.Output));
			  LK_L9025_Right_TxFrame.Data[5] = (uint8_t)((int16_t)(-PID_Velocity.Output)>>8);	 
	      USER_CAN_TxMessage(&LK_L9025_Right_TxFrame);
	      
				Damiao_Motor_CAN_Send(0x01,0,0,20,1,0);
				Damiao_Motor_CAN_Send(0x02,0,0,20,1,0);
			
 
				 

       }else{
        
	 
				   Referee_Info_Report();
				 
				Damiao_Motor_CAN_Send(0x03,0,0,20,1,0);
				Damiao_Motor_CAN_Send(0x04,0,0,20,1,0);
		    
			 
			  
				 Super_Cap_Info_Report();
			 }		
		
	      
				
			//	if(systick % 2 == 0)  

}
	

   
	



