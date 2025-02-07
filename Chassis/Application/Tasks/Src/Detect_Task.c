/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Detect_Task.c
  * @brief          : Detect task
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
#include "Detect_Task.h"
#include "remote_control.h"
#include "Control_Task.h"

#include "VMC_Task.h"
#include "INS_Task.h"
#include "motor.h"
#include "ui.h"
#include "ui_default_Left_Leg_Group_0.h"
#include "ui_default_Right_Leg_Group_0.h"
#include  "ui_default_Chassis_Mode_Group_0.h"
#include "ui_default_SuperCap_Group_1.h"
#include "ui_default_Mode_Group_0.h"
#include "ui_default_Text_Group_0.h"
#include "ui_default_Text_Group_1.h"
#include "ui_default_Text_Group_2.h"
#include "ui_default_Supercap_Per_Group_0.h"
#include "CAN_Task.h"
#include "referee_info.h"
#include "bsp_uart.h"

/* USER CODE BEGIN Header_Detect_Task */
static void Left_Leg_Coordinate_Calucate(void);
static void Right_Leg_Coordinate_Calucate(void);
static void Chassis_Mode_Calucate(void);
static void SuperCap_Calucate(void);
static void Mode_Calucate(void);

bool Init_Flag = 0;
int16_t Yaw_Err;
uint32_t Last_Systick = 0;
uint32_t Last_Systick1 = 0;
uint32_t Init_Tick = 0;
bool Updateing = 0;
/**
* @brief Function implementing the StartDetectTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Detect_Task */

void Detect_Task(void const * argument)
{
  /* USER CODE BEGIN Detect_Task */
 TickType_t systick = 0;
 
  /* Infinite loop */
  for(;;)
  {
  systick = osKernelSysTick();
		
//  if( t <= -1.5707963f && Flag1 == 0){
//	  Flag1 = 1;
//		Flag2 = 0;
//	}
//	
//	if(t>= 1.5707963f && Flag2 == 0){
//		Flag2 = 1;
//		Flag1 = 0;
//	}
//	if(Flag1 == 1) t+=0.01f;
//	if(Flag2 == 1) t-=0.01f;
//	
	
	
		
	//Length_Sin = arm_sin_f32(t);	
		
//	usart_printf("%f/n",Length_Sin);
	//usart_printf("%f,%f\n",Length_Sin,0);	
// 	   ui_self_id = Referee_Info.robot_status.robot_id;	
//	
//	if(remote_ctrl.key.set.B == 1){
//	Last_Systick1 = 0;
// 	if(systick % 50 == 0 && Last_Systick == 0){
//		 
//			_ui_init_default_Right_Leg_Group_0();
//	
//	    Last_Systick = systick;
//		}else if(systick  == Last_Systick + 50){
//		
//		   _ui_init_default_Left_Leg_Group_0();
//		
//		}else if(systick  == Last_Systick + 100){
//		
//		  _ui_init_default_Chassis_Mode_Group_0();
//			
//			
//		}else if(systick  == Last_Systick + 150){
//		  
//		  _ui_init_default_SuperCap_Group_1();
//		
//		
//		}else if(systick  == Last_Systick + 200){
//		  
//	   _ui_init_default_Mode_Group_0();
//		  
//		}else if(systick  == Last_Systick + 250){
//		  
//	    _ui_init_default_Text_Group_0();
//			
//		}else if(systick  == Last_Systick + 300){
//		  
//	    _ui_init_default_Text_Group_1();
//			
//		}else if(systick  == Last_Systick + 350){
//		  
//	    _ui_init_default_Text_Group_2();
//		 
//		}else if(systick  == Last_Systick + 400){
//		  
//	   _ui_init_default_Supercap_Per_Group_0();
//		  Last_Systick = 0;
//		}
//		
//	}else{
//	  Last_Systick = 0;
//	  if(systick % 30 == 0 && Last_Systick1==0){
//		 
//			Right_Leg_Coordinate_Calucate();
//			_ui_update_default_Right_Leg_Group_0();
//	    Last_Systick1 = systick;
//		
//		}else if(systick  == Last_Systick1 + 30){
//		
//       Left_Leg_Coordinate_Calucate();
//			 	_ui_update_default_Left_Leg_Group_0();
//		
//		}else if(systick  == Last_Systick1 + 60){
//		
//		   Chassis_Mode_Calucate();
//			 _ui_update_default_Chassis_Mode_Group_0();
//			
//		}else if(systick  == Last_Systick1 + 90){
//		  
//			 SuperCap_Calucate();
//			_ui_update_default_SuperCap_Group_1();
//	
//		}else if(systick  == Last_Systick1 + 120){
//	   
//			ui_default_Supercap_Per_Group_Supercap->number = SuperCap_Info.Cap_Percent;
//		 _ui_update_default_Supercap_Per_Group_0();  	
//	 
//		}else if(systick  == Last_Systick1 + 150){
//		 
//			Mode_Calucate();
//		  _ui_update_default_Mode_Group_0();
//	    Last_Systick1 = 0;
//	 }

//}
	
		

   osDelay(1);
	


}


  /* USER CODE END Detect_Task */
}

static void Left_Leg_Coordinate_Calucate(void){


	
	
 ui_default_Left_Leg_Group_L1->end_x =  ui_default_Left_Leg_Group_L5->end_x + 80 * (-arm_cos_f32(*Control_Info.L_Leg_Info.VMC.Phi1));
 ui_default_Left_Leg_Group_L1->end_y =  ui_default_Left_Leg_Group_L5->end_y - 80 * (arm_sin_f32(*Control_Info.L_Leg_Info.VMC.Phi1));
 
 ui_default_Left_Leg_Group_L4->start_x =  ui_default_Left_Leg_Group_L5->start_x - 80 * (arm_cos_f32(*Control_Info.L_Leg_Info.VMC.Phi4));
 ui_default_Left_Leg_Group_L4->start_y =  ui_default_Left_Leg_Group_L5->start_y - 80 * (arm_sin_f32(*Control_Info.L_Leg_Info.VMC.Phi4));

 ui_default_Left_Leg_Group_L2->start_x = 	 ui_default_Left_Leg_Group_L5->end_x  -( 146 * arm_cos_f32(Control_Info.L_Leg_Info.VMC.Phi2) + 80 * arm_cos_f32(*Control_Info.L_Leg_Info.VMC.Phi1)) ; 
 ui_default_Left_Leg_Group_L2->start_y =   ui_default_Left_Leg_Group_L5->end_y  -( 80 * arm_sin_f32(*Control_Info.L_Leg_Info.VMC.Phi1) + 146 * arm_sin_f32(Control_Info.L_Leg_Info.VMC.Phi2)); 

 ui_default_Left_Leg_Group_L2->end_x = 	  ui_default_Left_Leg_Group_L1->end_x;
 ui_default_Left_Leg_Group_L2->end_y =    ui_default_Left_Leg_Group_L1->end_y;
	
	
 ui_default_Left_Leg_Group_L3->start_x =   ui_default_Left_Leg_Group_L4->start_x;
 ui_default_Left_Leg_Group_L3->start_y =   ui_default_Left_Leg_Group_L4->start_y;

 ui_default_Left_Leg_Group_L3->end_x =   ui_default_Left_Leg_Group_L2->start_x;
 ui_default_Left_Leg_Group_L3->end_y =    ui_default_Left_Leg_Group_L2->start_y;



}
static void Right_Leg_Coordinate_Calucate(void){

 ui_default_Right_Leg_Group_L1->end_x =  ui_default_Right_Leg_Group_L5->end_x + 80 * (-arm_cos_f32(*Control_Info.R_Leg_Info.VMC.Phi1)-0.05f);
 ui_default_Right_Leg_Group_L1->end_y =  ui_default_Right_Leg_Group_L5->end_y - 80 * (arm_sin_f32(*Control_Info.R_Leg_Info.VMC.Phi1)-0.05f);
 
 ui_default_Right_Leg_Group_L4->start_x =  ui_default_Right_Leg_Group_L5->start_x - 80 * (arm_cos_f32(*Control_Info.R_Leg_Info.VMC.Phi4)+0.05f);
 ui_default_Right_Leg_Group_L4->start_y =  ui_default_Right_Leg_Group_L5->start_y - 80 * (arm_sin_f32(*Control_Info.R_Leg_Info.VMC.Phi4)+0.05f);

 ui_default_Right_Leg_Group_L2->start_x = 	 ui_default_Right_Leg_Group_L5->end_x  -( 146 * arm_cos_f32(Control_Info.R_Leg_Info.VMC.Phi2) + 80 * arm_cos_f32(*Control_Info.R_Leg_Info.VMC.Phi1)) ; 
 ui_default_Right_Leg_Group_L2->start_y =   ui_default_Right_Leg_Group_L5->end_y  -( 80 * arm_sin_f32(*Control_Info.R_Leg_Info.VMC.Phi1) + 146 * arm_sin_f32(Control_Info.R_Leg_Info.VMC.Phi2)); 

 ui_default_Right_Leg_Group_L2->end_x = 	  ui_default_Right_Leg_Group_L1->end_x;
 ui_default_Right_Leg_Group_L2->end_y =    ui_default_Right_Leg_Group_L1->end_y;
	
	
 ui_default_Right_Leg_Group_L3->start_x =   ui_default_Right_Leg_Group_L4->start_x;
 ui_default_Right_Leg_Group_L3->start_y =   ui_default_Right_Leg_Group_L4->start_y;

 ui_default_Right_Leg_Group_L3->end_x =   ui_default_Right_Leg_Group_L2->start_x;
 ui_default_Right_Leg_Group_L3->end_y =    ui_default_Right_Leg_Group_L2->start_y;	


}
static void Chassis_Mode_Calucate(void){

  if(Control_Info.Chassis_Mode == CHASSIS_FRONT){
		ui_default_Chassis_Mode_Group_CHASSIS_FRONT->start_x = 881;
    ui_default_Chassis_Mode_Group_CHASSIS_FRONT->start_y = 92;
    ui_default_Chassis_Mode_Group_CHASSIS_FRONT->end_x = 1064;
    ui_default_Chassis_Mode_Group_CHASSIS_FRONT->end_y = 211;  
			
	
    ui_default_Chassis_Mode_Group_CHASSIS_SIDE->start_y = 68 + 1080;	
    ui_default_Chassis_Mode_Group_CHASSIS_SIDE->end_y = 241 + 1080;	

    ui_default_Chassis_Mode_Group_CHASSIS_SPIN->start_x = 972;
    ui_default_Chassis_Mode_Group_CHASSIS_SPIN->start_y = 1080+80;
		
	 
	}else if(Control_Info.Chassis_Mode == CHASSIS_SIDE)	{
	
		
		
	
	
		ui_default_Chassis_Mode_Group_CHASSIS_SPIN->start_x = 972;
    ui_default_Chassis_Mode_Group_CHASSIS_SPIN->start_y = 1080+80;
		

	
    ui_default_Chassis_Mode_Group_CHASSIS_FRONT->start_y = 92 + 1080;
    ui_default_Chassis_Mode_Group_CHASSIS_FRONT->end_y = 211+1080; 
		

    ui_default_Chassis_Mode_Group_CHASSIS_SIDE->start_x = 912;
    ui_default_Chassis_Mode_Group_CHASSIS_SIDE->start_y = 68;
    ui_default_Chassis_Mode_Group_CHASSIS_SIDE->end_x = 1030;
    ui_default_Chassis_Mode_Group_CHASSIS_SIDE->end_y = 241;
		
		
	}else if(Control_Info.Chassis_Mode == CHASSIS_SPIN){
	

		
		    ui_default_Chassis_Mode_Group_CHASSIS_SPIN->start_x = 972;
    ui_default_Chassis_Mode_Group_CHASSIS_SPIN->start_y = 151;
		
	    ui_default_Chassis_Mode_Group_CHASSIS_FRONT->start_y = 92 + 1080;
    ui_default_Chassis_Mode_Group_CHASSIS_FRONT->end_y = 211+1080; 
		
		    ui_default_Chassis_Mode_Group_CHASSIS_SIDE->start_y = 68 + 1080;	
    ui_default_Chassis_Mode_Group_CHASSIS_SIDE->end_y = 241 + 1080;
	
	}

	if(Control_Info.Yaw_Err > 1.f){	

		ui_default_Chassis_Mode_Group_Shoot->end_x = ui_default_Chassis_Mode_Group_Shoot->start_x +  arm_sin_f32((float)(Control_Info.Yaw_Err)*Angle_to_rad)*93;
    ui_default_Chassis_Mode_Group_Shoot->end_y = ui_default_Chassis_Mode_Group_Shoot->start_y +  arm_cos_f32((float)(Control_Info.Yaw_Err)*Angle_to_rad)*93;
  }else if(Control_Info.Yaw_Err  < -1.f){
	  ui_default_Chassis_Mode_Group_Shoot->end_x = ui_default_Chassis_Mode_Group_Shoot->start_x + arm_sin_f32((float)(Control_Info.Yaw_Err)*Angle_to_rad)*93;
    ui_default_Chassis_Mode_Group_Shoot->end_y = ui_default_Chassis_Mode_Group_Shoot->start_y + arm_cos_f32((float)(Control_Info.Yaw_Err)*Angle_to_rad)*93;
	
	}else if(Control_Info.Yaw_Err  < 1.f && Yaw_Err >-1.f){
	  ui_default_Chassis_Mode_Group_Shoot->end_x = 972;
    ui_default_Chassis_Mode_Group_Shoot->end_y = 244;
}


}
void SuperCap_Calucate(void){

 ui_default_SuperCap_Group_SuperCap_Energy->end_x =  ui_default_SuperCap_Group_SuperCap_Energy->start_x  + 638 * SuperCap_Info.Cap_Percent * 0.01f;
 ui_default_Supercap_Per_Group_Supercap->number = SuperCap_Info.Cap_Percent;


}
static void Mode_Calucate(void){

 if(Control_Info.Shoot_Mode == 0){
   ui_default_Mode_Group_Shoot->color = 8;
 }else if(Control_Info.Shoot_Mode == 1){
   ui_default_Mode_Group_Shoot->color = 3;
 }else if(Control_Info.Shoot_Mode == 2){
   ui_default_Mode_Group_Shoot->color = 2;
 }else if(Control_Info.Shoot_Mode == 3){
    ui_default_Mode_Group_Shoot->color = 4;
 }
 if(Control_Info.Cover_Switch == 1){
 ui_default_Mode_Group_Cover->color =2;
 }else{
 ui_default_Mode_Group_Cover->color = 8;
 
 }
  if(Control_Info.Week_Move == 1){
 ui_default_Mode_Group_Cover->color =2;
 }else{
 ui_default_Mode_Group_Cover->color = 8;
 
 }
 if(Control_Info.IF_Aiming_Enable == 1){
	ui_default_Mode_Group_Vision->color = 6;
 }else ui_default_Mode_Group_Vision->color = 8;
 
 
}
