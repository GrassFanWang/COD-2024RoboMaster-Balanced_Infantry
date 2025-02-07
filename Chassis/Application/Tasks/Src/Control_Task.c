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
#include "remote_control.h"
#include "ramp.h"
#include "pid.h"
#include "motor.h"
#include "arm_math.h"
#include "referee_info.h"
#include "VMC_Task.h"
#include "bsp_uart.h"
#include "math.h"
#include "CAN_Task.h"
#include "kalman.h"
#include "api_quaternion.h"
#include "Kalman_Mix.h"
#include "Velocity_Fusion.h"
#include "api_quaternion.h"
#include "bsp_can.h"
float Turn_T_Limit;

#define fp32 float
#define printf(title, fmt, args...) usart_printf("{"#title"}"fmt"\n", ##args)
#define  Stand_T_MAX 10.f
#define  Balance_Tp_MAX 100.f
LowPassFilter1p_Info_TypeDef  LowPassFilter1p1;
LowPassFilter1p_Info_TypeDef  LowPassFilter1p2;


Control_Info_Typedef Control_Info={
   .Situation =CHASSIS_WEAK,	 
	 .L1 = 0.16f,
   .L4 = 0.16f,
	 .L2 = 0.27f,
   .L3 = 0.27f,
	 .L5 = 0.11f,
   .L_Leg_Info ={
		  .Target.Theta = -0.0f,
		  .Target.Phi =0.0f,
	    .VMC.Phi1 = &Damiao_Motor[L_A_Joint].Data.Position,
	    .VMC.Phi4 = &Damiao_Motor[L_P_Joint].Data.Position,
		  .VMC.Phi1_dot = &Damiao_Motor[L_A_Joint].Data.Velocity,
		  .VMC.Phi4_dot = &Damiao_Motor[L_P_Joint].Data.Velocity,
		  .Off_Ground_FN  = 100.f,
	 },
	 .R_Leg_Info ={
		  .Target.Theta = -0.0f,
		  .Target.Phi =0.0f,
	    .VMC.Phi1 = &Damiao_Motor[R_P_Joint].Data.Position,
	    .VMC.Phi4 = &Damiao_Motor[R_A_Joint].Data.Position,
		  .VMC.Phi1_dot = &Damiao_Motor[R_P_Joint].Data.Velocity,
		  .VMC.Phi4_dot = &Damiao_Motor[R_A_Joint].Data.Velocity,
		  .Off_Ground_FN  = 100.f,
	 },
	 .Center_Angle  = 0.f,
   .LK_Stand_T_To_SendValue = 200,
	 .LK_Turn_T_To_SendValue = 100,
	 .Chassis_Velocity_MAX = 1.5f,
	 .SPIN_V = 0,
	 .Leg_Length_Mode = Leg_Length_Normol,
};



PID_Info_TypeDef PID_Leg_length_thrust[2];
PID_Info_TypeDef PID_Left_Turn_Angle; 
PID_Info_TypeDef PID_Left_Turn_Velocity;
PID_Info_TypeDef PID_Turn_Velocity;
PID_Info_TypeDef PID_Leg_Coordinate;
PID_Info_TypeDef PID_Roll;
PID_Info_TypeDef PID_Vision_Lock_Angle;
PID_Info_TypeDef PID_Vision_Lock_Gyro;

fp32 a11[6] = {0,-94.362419f,165.233645f,-203.426872f,0.199618f};
fp32 a12[6] = {0,29.391657f,-37.257144f,-24.064116f,0.132366f};
fp32 a13[6] = {0,-221.604269f,228.547406f,-83.374201f,-10.710819f};
fp32 a14[6] = {0,-128.710115f,141.768420f,-65.956019f,-9.066364f};
fp32 a15[6] = {0,-449.752816f,581.187939f,-296.331371f,80.044275f};
fp32 a16[6] = {0,-5.831613f,16.596177f,-14.058031f,7.767911f};
fp32 a21[6] = {0,381.315597f,-364.552838f,111.958129f,14.970084f};
fp32 a22[6] = {0,39.430699f,-42.954768f,19.842893f,1.993639f};
fp32 a23[6] = {0,-285.971038f,366.186386f,-182.715407f,44.153310f};
fp32 a24[6] = {0,-236.677246f,286.179831f,-135.566936f,33.506236f};
fp32 a25[6] = {0,1715.744956f,-1772.922736f,652.649373f,50.506165f};
fp32 a26[6] = {0,137.315752f,-152.244098f,62.648877f,-2.731731f};
static float PID_Leg_length_thrust_param[6]={1000.f,0.8f,80000.f,0,20,150};
static float PID_Leg_length_thrust_param1[6]={1000.f,0.8f,80000.f,0,20,150};
static float PID_Turn_Angle_param0[6]={1.f,0.0f,0.f,0,0,100.f};
static float PID_Turn_Velocity_param0[6]={0.7f,0.01f,0.0f,0,3,10.f};
static float PID_Turn_Angle_param1[6]={3.f,0.0f,0.f,0,0,20.f};
static float PID_Turn_Velocity_param1[6]={0.7f,0.01f,.0f,0,10,10.f};
static float PID_Leg_Coordinate_param[6]={100.f,0.f,5.0f,0.f,0.f,50};
static float PID_Vision_Lock_Angle_Param[6]={30.f,0.0f,.0f,0,0,10.f};
static float PID_Vision_Lock_Gyro_Param[6]={-0.1f,0.f,.0f,0.f,0.f,10.f};

static void Jump();
static void Control_Init(Control_Info_Typedef *Control_Info);
static void VMC_Calculate(Control_Info_Typedef *Control_Info);
static void Control_LQR_K_Update(Control_Info_Typedef *Control_Info);
static void Control_Measure_Update(Control_Info_Typedef *Control_Info);
static void Control_Target_Update(Control_Info_Typedef *Control_Info);
static void Control_LQR_X_Update(Control_Info_Typedef *Control_Info);
static void Control_LQR_Stand_T_Balance_Tp_Calculate(Control_Info_Typedef *Control_Info);
static void Control_Comprehensive_F_Calculate(Control_Info_Typedef *Control_Info);
static void Control_T_Tp_F_Calculate(Control_Info_Typedef *Control_Info);
static void Control_Support_Calculate(Control_Info_Typedef *Control_Info);
static void VMC_Measure_F_Tp_Calculate(Control_Info_Typedef *Control_Info);
static void Level_Info_Update(Control_Info_Typedef *Control_Info);
static void Power_Limit(Control_Info_Typedef *Control_Info);

static void VMC_F_Tp_To_Joint_Calculate(Control_Info_Typedef *Control_Info);


/* USER CODE BEGIN Header_Control_Task */
/**
* @brief Function implementing the StartControlTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Control_Task */
float L_FN = 0;
void Control_Task(void const * argument)
{
  /* USER CODE BEGIN Control_Task */
  TickType_t systick = 0;
	PID_Init(&PID_Leg_length_thrust[0],PID_POSITION,PID_Leg_length_thrust_param);
	PID_Init(&PID_Leg_length_thrust[1],PID_POSITION,PID_Leg_length_thrust_param1);
	PID_Init(&PID_Left_Turn_Angle,PID_POSITION,PID_Turn_Angle_param0);
	PID_Init(&PID_Left_Turn_Velocity,PID_POSITION,PID_Turn_Velocity_param0);
	PID_Init(&PID_Turn_Velocity,PID_POSITION,PID_Turn_Velocity_param1);
	PID_Init(&PID_Leg_Coordinate,PID_POSITION,PID_Leg_Coordinate_param);
	PID_Init(&PID_Vision_Lock_Angle,PID_POSITION,PID_Vision_Lock_Angle_Param);
		PID_Init(&PID_Vision_Lock_Gyro,PID_POSITION,PID_Vision_Lock_Gyro_Param);
  LowPassFilter1p_Init(&LowPassFilter1p1,0.f,0.002f);
  LowPassFilter1p_Init(&LowPassFilter1p2,0.f,0.002f);
	systick = osKernelSysTick();

 /* Infinite loop */
	for(;;)
  {
		
			Control_Init(&Control_Info);
		  VMC_Calculate(&Control_Info);
			Control_LQR_K_Update(&Control_Info);
			Control_Measure_Update(&Control_Info);
			Control_Target_Update(&Control_Info);
			VMC_Measure_F_Tp_Calculate(&Control_Info);
			Control_Support_Calculate(&Control_Info);
		  Jump();
			Control_LQR_X_Update(&Control_Info);
		  Control_LQR_Stand_T_Balance_Tp_Calculate(&Control_Info);
		  Power_Limit(&Control_Info);
      Level_Info_Update(&Control_Info);

		  Control_Comprehensive_F_Calculate(&Control_Info);
			Control_T_Tp_F_Calculate(&Control_Info);
		  VMC_F_Tp_To_Joint_Calculate(&Control_Info);
	
   // printf(L0,"%f",Referee_Info.power_heat_data.chassis_power);
    

		osDelay(2);
  }
}
  /* USER CODE END Control_Task */



static void Control_Init(Control_Info_Typedef *Control_Info){

	if( remote_ctrl.rc_lost == 0  ){
	   
		
		
		 Control_Info->Control_Mode = Remote_Ctrl_Mode ;
		
		
	
  if(Control_Info->Control_Mode == Remote_Ctrl_Mode){
	  
	 if(remote_ctrl.rc.s[1] == 3 || remote_ctrl.rc.s[1] == 1){
	 
	  Control_Info->Init.Begin_Init  = 1;
		 
	 }else {
      Control_Info->Sliping = 0; 
		  Control_Info->Init.Begin_Init  = 0;
	    Control_Info->Situation = CHASSIS_WEAK;
	 
	 }
	}else if( Control_Info->Control_Mode == Key_Board_Mode){
	  
		if( Control_Info->Sliping == 1){
		
		 Control_Info->Init.Begin_Init = 1;
		
		}
	  static bool G_Flag = 0;
  
		if(remote_ctrl.key.set.G == 1 && G_Flag==0){
		    
			  if(Control_Info->Situation == CHASSIS_BALANCE) Control_Info->Situation = CHASSIS_WEAK;
			  Control_Info->Init.Begin_Init =  !Control_Info->Init.Begin_Init;
				Control_Info->Sliping = 0;
        G_Flag = 1;
		     
		}else if(remote_ctrl.key.set.G == 0) G_Flag = 0;
	  
	}
	
}else if(remote_ctrl.rc_lost == 1  ||  Referee_Info.robot_status.mains_power_chassis_output == 0){
    
	
	  Control_Info->Init.Begin_Init = 0;
	  Control_Info->Situation = CHASSIS_WEAK;
	
	  Control_Info->Init.Joint_Init.Joint_Reduction_Flag[0] = 0;
		Control_Info->Init.Joint_Init.Joint_Reduction_Flag[1] = 0;
		Control_Info->Init.Joint_Init.Joint_Reduction_Flag[2] = 0;
		Control_Info->Init.Joint_Init.Joint_Reduction_Flag[3] = 0;
		Control_Info->Init.Joint_Init.IF_Joint_Init = 0;
		
	  Control_Info->Init.Balance_Init.IF_Balance_Init = 0;
	  Control_Info->Init.Balance_Init.Balance_Right_Cnt = 0;
  		
    Control_Info->Init.Chassis_Velocity_Init.IF_Chassis_Velocity_Init = 0;
	  Control_Info->Init.Chassis_Velocity_Init.Chassis_Velocity_Right_Cnt = 0;

	  Control_Info->Init.Leg_Init.IF_Leg_Init = 0;
    Control_Info->Init.Leg_Init.Leg_Right_Cnt = 0;		
    
}
	
if(Control_Info->Sliping  == 0){
	
	if(Control_Info->Init.Begin_Init == 1 && Control_Info->Situation == CHASSIS_WEAK){
	   	 
	 if(Control_Info->Init.Motor_Enable == 1){
		
		 if(Control_Info->Init.Joint_Init.IF_Joint_Init == 0){  
	   	
			 if(Damiao_Motor[0].Data.Position > 2.94f )  
			 Control_Info->Init.Joint_Init.Joint_Reduction_Flag[0] = 1; else Control_Info->Init.Joint_Init.Joint_Reduction_Flag[0] = 0;
			 if(Damiao_Motor[1].Data.Position < 0.3f )    
		   Control_Info->Init.Joint_Init.Joint_Reduction_Flag[1] = 1; else Control_Info->Init.Joint_Init.Joint_Reduction_Flag[1] = 0; 
			 if(Damiao_Motor[2].Data.Position < 0.3f )    
			 Control_Info->Init.Joint_Init.Joint_Reduction_Flag[2] = 1; else Control_Info->Init.Joint_Init.Joint_Reduction_Flag[2] = 0;
			 if(Damiao_Motor[3].Data.Position > 2.94f )  
			 Control_Info->Init.Joint_Init.Joint_Reduction_Flag[3] = 1; else Control_Info->Init.Joint_Init.Joint_Reduction_Flag[3] = 0;
			if(Control_Info->Init.Joint_Init.Joint_Reduction_Flag[0] + Control_Info->Init.Joint_Init.Joint_Reduction_Flag[1] 
			 + Control_Info->Init.Joint_Init.Joint_Reduction_Flag[2]+Control_Info->Init.Joint_Init.Joint_Reduction_Flag[3] == 4)  

			Control_Info->Init.Joint_Init.IF_Joint_Init = 1;  else  Control_Info->Init.Joint_Init.IF_Joint_Init =0;
	    
	
		  
			
		}else if(Control_Info->Init.Joint_Init.IF_Joint_Init == 1){
		  
		  if(Control_Info->Init.Balance_Init.IF_Balance_Init == 0){
			
			   if(fabsf(INS_Info.pit_angle) < 5.f){
			     
			     Control_Info->Init.Balance_Init.Balance_Right_Cnt++;
            					 
				  if(Control_Info->Init.Balance_Init.Balance_Right_Cnt > 200){
					  Control_Info->Sliping = 0;
					 Control_Info->Init.Balance_Init.IF_Balance_Init = 1;
					 Control_Info->Init.Balance_Init.Balance_Right_Cnt = 0;	
					
					}
				}
			
			
			} else if(Control_Info->Init.Balance_Init.IF_Balance_Init == 1){
			

		
		         if(Control_Info->Init.Leg_Init.IF_Leg_Init == 0) {
		            Control_Info->Leg_Length_Mode  = Leg_Length_Normol;
  							Control_Info->L_Leg_Info.VMC.Target_L0 = 0.18f;
						    Control_Info->R_Leg_Info.VMC.Target_L0 = 0.18f;													   
						    Control_Info->L_Leg_Info.VMC.Gravity_Compensation =  f_Ramp_Calc(Control_Info->L_Leg_Info.VMC.Gravity_Compensation,90.f,0.3f);
                Control_Info->R_Leg_Info.VMC.Gravity_Compensation =  f_Ramp_Calc(Control_Info->R_Leg_Info.VMC.Gravity_Compensation,90.f,0.3f);
		
				     if(Control_Info->L_Leg_Info.VMC.Gravity_Compensation + Control_Info->R_Leg_Info.VMC.Gravity_Compensation == 180){
						     Control_Info->Init.Leg_Init.IF_Leg_Init = 1;
					       Control_Info->Situation = CHASSIS_BALANCE;
				  	  }
			  	  }
		     }
		   }
	   }	
   }
}

if(Control_Info->Situation == CHASSIS_BALANCE ){
	 
	 if( fabsf(INS_Info.pit_angle + 1.5f) > 13.f  ){
		 
		 Control_Info->Chassis_Slip_Cnt ++;
     
		 if(fabsf(Control_Info->Measure_Chassis_Velocity) > 2.f)  Control_Info->Chassis_Slip_Cnt +=2;

	 }	
	    
	   if( Control_Info->Chassis_Slip_Cnt  > 100){
			  Control_Info->Sliping = 1;
			  Control_Info->Slip_Week = 1;
				Control_Info->Init.Begin_Init = 0 ;			  
			  Control_Info->Situation = CHASSIS_WEAK;
				Control_Info->Chassis_Slip_Cnt = 0 ;
			 
		}
}

  if(Control_Info->Sliping == 1){
		Control_Info->Chassis_Slip_Cnt ++;
		if(Control_Info->Chassis_Slip_Cnt > 50){
		   Control_Info->Slip_Week = 0;
		}
	  if(Control_Info->Chassis_Slip_Cnt > 100){
		 
		Control_Info->Sliping = 0;
		Control_Info->Chassis_Slip_Cnt = 0;
			
		}
	}
 
 
 
 if(Control_Info->Init.Begin_Init == 0 && Control_Info->Situation == CHASSIS_WEAK){
	  
		
	  Control_Info->Init.Joint_Init.Joint_Reduction_Flag[0] = 0;
		Control_Info->Init.Joint_Init.Joint_Reduction_Flag[1] = 0;
		Control_Info->Init.Joint_Init.Joint_Reduction_Flag[2] = 0;
		Control_Info->Init.Joint_Init.Joint_Reduction_Flag[3] = 0;
		Control_Info->Init.Joint_Init.IF_Joint_Init = 0;
	  Control_Info->Init.Balance_Init.IF_Balance_Init = 0;
	  Control_Info->Init.Balance_Init.Balance_Right_Cnt = 0;
    Control_Info->Init.Chassis_Velocity_Init.IF_Chassis_Velocity_Init = 0;
	  Control_Info->Init.Chassis_Velocity_Init.Chassis_Velocity_Right_Cnt = 0;
	  Control_Info->Init.Leg_Init.IF_Leg_Init = 0;
    Control_Info->Init.Leg_Init.Leg_Right_Cnt = 0;	
	  Control_Info->Leg_Length_Mode = Leg_Length_Normol;
 
 }	
 
 
}

static void VMC_Calculate(Control_Info_Typedef *Control_Info){

	
Control_Info->L_Leg_Info.VMC.X_B = Control_Info->L1 * arm_cos_f32(*Control_Info->L_Leg_Info.VMC.Phi1);
Control_Info->L_Leg_Info.VMC.Y_B = Control_Info->L1 * arm_sin_f32(*Control_Info->L_Leg_Info.VMC.Phi1);
Control_Info->L_Leg_Info.VMC.X_D = Control_Info->L5 + Control_Info->L4 * arm_cos_f32(*Control_Info->L_Leg_Info.VMC.Phi4);
Control_Info->L_Leg_Info.VMC.Y_D = Control_Info->L4 * arm_sin_f32(*Control_Info->L_Leg_Info.VMC.Phi4);
Control_Info->L_Leg_Info.VMC.X_D_Difference_X_B = Control_Info->L_Leg_Info.VMC.X_D - Control_Info->L_Leg_Info.VMC.X_B;
Control_Info->L_Leg_Info.VMC.Y_D_Difference_Y_B = Control_Info->L_Leg_Info.VMC.Y_D - Control_Info->L_Leg_Info.VMC.Y_B;
Control_Info->L_Leg_Info.VMC.LBD_2 =   Control_Info->L_Leg_Info.VMC.X_D_Difference_X_B*Control_Info->L_Leg_Info.VMC.X_D_Difference_X_B +Control_Info->L_Leg_Info.VMC.Y_D_Difference_Y_B*Control_Info->L_Leg_Info.VMC.Y_D_Difference_Y_B;
Control_Info->L_Leg_Info.VMC.A0 =  2 *Control_Info->L2  * Control_Info->L_Leg_Info.VMC.X_D_Difference_X_B;
Control_Info->L_Leg_Info.VMC.B0 =  2 *Control_Info->L2  * Control_Info->L_Leg_Info.VMC.Y_D_Difference_Y_B;
arm_sqrt_f32(Control_Info->L_Leg_Info.VMC.A0*Control_Info->L_Leg_Info.VMC.A0 +  Control_Info->L_Leg_Info.VMC.B0* Control_Info->L_Leg_Info.VMC.B0 - Control_Info->L_Leg_Info.VMC.LBD_2*Control_Info->L_Leg_Info.VMC.LBD_2,&Control_Info->L_Leg_Info.VMC.C0);
Control_Info->L_Leg_Info.VMC.Phi2 =2 * atan2f(Control_Info->L_Leg_Info.VMC.B0+Control_Info->L_Leg_Info.VMC.C0 , Control_Info->L_Leg_Info.VMC.A0 + Control_Info->L_Leg_Info.VMC.LBD_2);
Control_Info->L_Leg_Info.VMC.X_C  =Control_Info->L_Leg_Info.VMC.X_B +Control_Info->L2 * arm_cos_f32(Control_Info->L_Leg_Info.VMC.Phi2);
Control_Info->L_Leg_Info.VMC.Y_C  =Control_Info->L_Leg_Info.VMC.Y_B +Control_Info->L2 * arm_sin_f32(Control_Info->L_Leg_Info.VMC.Phi2);
Control_Info->L_Leg_Info.VMC.Phi3 = atan2f(Control_Info->L_Leg_Info.VMC.Y_C-Control_Info->L_Leg_Info.VMC.Y_D,Control_Info->L_Leg_Info.VMC.X_C-Control_Info->L_Leg_Info.VMC.X_D);
arm_sqrt_f32(powf(Control_Info->L_Leg_Info.VMC.X_C -Control_Info->L5/2,2) +Control_Info->L_Leg_Info.VMC.Y_C*Control_Info->L_Leg_Info.VMC.Y_C,&Control_Info->L_Leg_Info.VMC.L0);
Control_Info->L_Leg_Info.VMC.L0_dot = (Control_Info->L_Leg_Info.VMC.L0 - Control_Info->L_Leg_Info.VMC.Last_L0)/0.002f;
Control_Info->L_Leg_Info.VMC.Last_L0 = Control_Info->L_Leg_Info.VMC.L0;
Control_Info->L_Leg_Info.VMC.Phi0 = atan2f(Control_Info->L_Leg_Info.VMC.Y_C,Control_Info->L_Leg_Info.VMC.X_C - (Control_Info->L5/2));
Control_Info->L_Leg_Info.VMC.Phi0_dot = (Control_Info->L_Leg_Info.VMC.Phi0 - Control_Info->L_Leg_Info.VMC.Last_Phi0)/0.002f;
Control_Info->L_Leg_Info.VMC.Last_Phi0 = Control_Info->L_Leg_Info.VMC.Phi0 ;


Control_Info->R_Leg_Info.VMC.X_B = Control_Info->L1 * arm_cos_f32(*Control_Info->R_Leg_Info.VMC.Phi1);
Control_Info->R_Leg_Info.VMC.Y_B = Control_Info->L1 * arm_sin_f32(*Control_Info->R_Leg_Info.VMC.Phi1);
Control_Info->R_Leg_Info.VMC.X_D = Control_Info->L5 + Control_Info->L4 * arm_cos_f32(*Control_Info->R_Leg_Info.VMC.Phi4);
Control_Info->R_Leg_Info.VMC.Y_D = Control_Info->L4 * arm_sin_f32(*Control_Info->R_Leg_Info.VMC.Phi4);
Control_Info->R_Leg_Info.VMC.X_D_Difference_X_B = Control_Info->R_Leg_Info.VMC.X_D - Control_Info->R_Leg_Info.VMC.X_B;
Control_Info->R_Leg_Info.VMC.Y_D_Difference_Y_B = Control_Info->R_Leg_Info.VMC.Y_D - Control_Info->R_Leg_Info.VMC.Y_B;
Control_Info->R_Leg_Info.VMC.LBD_2 =   Control_Info->R_Leg_Info.VMC.X_D_Difference_X_B*Control_Info->R_Leg_Info.VMC.X_D_Difference_X_B +Control_Info->R_Leg_Info.VMC.Y_D_Difference_Y_B*Control_Info->R_Leg_Info.VMC.Y_D_Difference_Y_B;
Control_Info->R_Leg_Info.VMC.A0 =  2 *Control_Info->L2  * Control_Info->R_Leg_Info.VMC.X_D_Difference_X_B;
Control_Info->R_Leg_Info.VMC.B0 =  2 *Control_Info->L2  * Control_Info->R_Leg_Info.VMC.Y_D_Difference_Y_B;
arm_sqrt_f32(Control_Info->R_Leg_Info.VMC.A0*Control_Info->R_Leg_Info.VMC.A0 +  Control_Info->R_Leg_Info.VMC.B0* Control_Info->R_Leg_Info.VMC.B0 - Control_Info->R_Leg_Info.VMC.LBD_2*Control_Info->R_Leg_Info.VMC.LBD_2,&Control_Info->R_Leg_Info.VMC.C0);
Control_Info->R_Leg_Info.VMC.Phi2 =2 * atan2f(Control_Info->R_Leg_Info.VMC.B0+Control_Info->R_Leg_Info.VMC.C0 , Control_Info->R_Leg_Info.VMC.A0 + Control_Info->R_Leg_Info.VMC.LBD_2);
Control_Info->R_Leg_Info.VMC.X_C  =Control_Info->R_Leg_Info.VMC.X_B +Control_Info->L2 * arm_cos_f32(Control_Info->R_Leg_Info.VMC.Phi2);
Control_Info->R_Leg_Info.VMC.Y_C  =Control_Info->R_Leg_Info.VMC.Y_B +Control_Info->L2 * arm_sin_f32(Control_Info->R_Leg_Info.VMC.Phi2);
Control_Info->R_Leg_Info.VMC.Phi3 = atan2f(Control_Info->R_Leg_Info.VMC.Y_C-Control_Info->R_Leg_Info.VMC.Y_D,Control_Info->R_Leg_Info.VMC.X_C-Control_Info->R_Leg_Info.VMC.X_D);
arm_sqrt_f32(powf(Control_Info->R_Leg_Info.VMC.X_C -Control_Info->L5/2,2) +Control_Info->R_Leg_Info.VMC.Y_C*Control_Info->R_Leg_Info.VMC.Y_C,&Control_Info->R_Leg_Info.VMC.L0);
Control_Info->R_Leg_Info.VMC.L0_dot = (Control_Info->R_Leg_Info.VMC.L0 - Control_Info->R_Leg_Info.VMC.Last_L0)/0.002f;
Control_Info->R_Leg_Info.VMC.Last_L0 = Control_Info->R_Leg_Info.VMC.L0;
Control_Info->R_Leg_Info.VMC.Phi0 = atan2f(Control_Info->R_Leg_Info.VMC.Y_C,Control_Info->R_Leg_Info.VMC.X_C - (Control_Info->L5/2));
Control_Info->R_Leg_Info.VMC.Phi0_dot = (Control_Info->R_Leg_Info.VMC.Phi0 - Control_Info->R_Leg_Info.VMC.Last_Phi0)/0.002f;
Control_Info->R_Leg_Info.VMC.Last_Phi0 = Control_Info->R_Leg_Info.VMC.Phi0; 

}

static void Control_LQR_K_Update(Control_Info_Typedef *Control_Info){
	float L0; 
	L0= Control_Info->L_Leg_Info.VMC.L0;
	Control_Info->L_Leg_Info.LQR_K[0][0]=  a11[1]*powf(L0,3)   + a11[2]*powf(L0,2)    +a11[3]*L0    +a11[4];      
  Control_Info->L_Leg_Info.LQR_K[0][1]=  a12[1]*powf(L0,3)   + a12[2]*powf(L0,2)    +a12[3]*L0    +a12[4];
  Control_Info->L_Leg_Info.LQR_K[0][2]=  a13[1]*powf(L0,3)   + a13[2]*powf(L0,2)    +a13[3]*L0    +a13[4];
  Control_Info->L_Leg_Info.LQR_K[0][3]=  a14[1]*powf(L0,3)   + a14[2]*powf(L0,2)    +a14[3]*L0    +a14[4];
  Control_Info->L_Leg_Info.LQR_K[0][4]=  a15[1]*powf(L0,3)   + a15[2]*powf(L0,2)    +a15[3]*L0    +a15[4];
  Control_Info->L_Leg_Info.LQR_K[0][5]=  a16[1]*powf(L0,3)   + a16[2]*powf(L0,2)    +a16[3]*L0    +a16[4];

	Control_Info->L_Leg_Info.LQR_K[1][0]=   a21[1]*powf(L0,3)   + a21[2]*powf(L0,2)    +a21[3]*L0    +a21[4];     
  Control_Info->L_Leg_Info.LQR_K[1][1]=   a22[1]*powf(L0,3)   + a22[2]*powf(L0,2)    +a22[3]*L0    +a22[4];
  Control_Info->L_Leg_Info.LQR_K[1][2]=   a23[1]*powf(L0,3)   + a23[2]*powf(L0,2)    +a23[3]*L0    +a23[4];
  Control_Info->L_Leg_Info.LQR_K[1][3]=   a24[1]*powf(L0,3)   + a24[2]*powf(L0,2)    +a24[3]*L0    +a24[4];
  Control_Info->L_Leg_Info.LQR_K[1][4]=   a25[1]*powf(L0,3)   + a25[2]*powf(L0,2)    +a25[3]*L0    +a25[4];
  Control_Info->L_Leg_Info.LQR_K[1][5]=   a26[1]*powf(L0,3)   + a26[2]*powf(L0,2)    +a26[3]*L0    +a26[4];
	
	float L1;
	L1= Control_Info->R_Leg_Info.VMC.L0;
	Control_Info->R_Leg_Info.LQR_K[0][0]=  a11[1]*powf(L1,3)   + a11[2]*powf(L1,2)    +a11[3]*L1    +a11[4];      
  Control_Info->R_Leg_Info.LQR_K[0][1]=  a12[1]*powf(L1,3)   + a12[2]*powf(L1,2)    +a12[3]*L1    +a12[4];
  Control_Info->R_Leg_Info.LQR_K[0][2]=  a13[1]*powf(L1,3)   + a13[2]*powf(L1,2)    +a13[3]*L1    +a13[4];
  Control_Info->R_Leg_Info.LQR_K[0][3]=  a14[1]*powf(L1,3)   + a14[2]*powf(L1,2)    +a14[3]*L1    +a14[4];
  Control_Info->R_Leg_Info.LQR_K[0][4]=  a15[1]*powf(L1,3)   + a15[2]*powf(L1,2)    +a15[3]*L1    +a15[4];
  Control_Info->R_Leg_Info.LQR_K[0][5]=  a16[1]*powf(L1,3)   + a16[2]*powf(L1,2)    +a16[3]*L1    +a16[4];

	Control_Info->R_Leg_Info.LQR_K[1][0]=   a21[1]*powf(L1,3)   + a21[2]*powf(L1,2)    +a21[3]*L1    +a21[4];     
  Control_Info->R_Leg_Info.LQR_K[1][1]=   a22[1]*powf(L1,3)   + a22[2]*powf(L1,2)    +a22[3]*L1    +a22[4];
  Control_Info->R_Leg_Info.LQR_K[1][2]=   a23[1]*powf(L1,3)   + a23[2]*powf(L1,2)    +a23[3]*L1    +a23[4];
  Control_Info->R_Leg_Info.LQR_K[1][3]=   a24[1]*powf(L1,3)   + a24[2]*powf(L1,2)    +a24[3]*L1    +a24[4];
  Control_Info->R_Leg_Info.LQR_K[1][4]=   a25[1]*powf(L1,3)   + a25[2]*powf(L1,2)    +a25[3]*L1    +a25[4];
  Control_Info->R_Leg_Info.LQR_K[1][5]=   a26[1]*powf(L1,3)   + a26[2]*powf(L1,2)    +a26[3]*L1    +a26[4];
}



float LK_Motor_E,Yaw_LK_Err;

static void Control_Measure_Update(Control_Info_Typedef *Control_Info){

	Control_Info->L_Leg_Info.Measure.Phi = -INS_Info.angle[2];
	Control_Info->L_Leg_Info.Measure.Phi_dot  = -INS_Info.gyro[0];
	Control_Info->L_Leg_Info.Measure.Theta =  ( 1.5707965f - Control_Info->L_Leg_Info.VMC.Phi0) - Control_Info->L_Leg_Info.Measure.Phi ;
	Control_Info->L_Leg_Info.Measure.Theta_dot = ( Control_Info->L_Leg_Info.Measure.Theta -  Control_Info->L_Leg_Info.Measure.Last_Theta)/0.002f; //+ INS_Info.gyro[2];
	Control_Info->L_Leg_Info.Measure.Last_Theta =  Control_Info->L_Leg_Info.Measure.Theta;
	
	Control_Info->R_Leg_Info.Measure.Phi = -INS_Info.angle[2];
	Control_Info->R_Leg_Info.Measure.Phi_dot  = -INS_Info.gyro[0];
	Control_Info->R_Leg_Info.Measure.Theta =  (Control_Info->R_Leg_Info.VMC.Phi0 - 1.5707965f )  - Control_Info->R_Leg_Info.Measure.Phi ;
	Control_Info->R_Leg_Info.Measure.Theta_dot = ( Control_Info->R_Leg_Info.Measure.Theta -  Control_Info->R_Leg_Info.Measure.Last_Theta)/0.002f; //+ INS_Info.gyro[2];
	Control_Info->R_Leg_Info.Measure.Last_Theta =  Control_Info->R_Leg_Info.Measure.Theta;

 if(Control_Info->Chassis_Mode == CHASSIS_SPIN){
	    
  Control_Info->K = 0.f;     
		
 }else 	 Control_Info->K = 0.8f;
	
  Control_Info->L_Leg_Info.W_Velocity =   (-LK_Motor[0].Data.Rad_Velocity + Control_Info->L_Leg_Info.VMC.Phi0_dot - INS_Info.gyro[0] )*0.09f;

	Control_Info->R_Leg_Info.W_Velocity  =  ( LK_Motor[1].Data.Rad_Velocity - Control_Info->R_Leg_Info.VMC.Phi0_dot - INS_Info.gyro[0])*0.09f;
	
	Control_Info->L_Leg_Info.Measure.Chassis_Velocity =   Control_Info->L_Leg_Info.W_Velocity
	                                                     +( Control_Info->L_Leg_Info.VMC.L0 * Control_Info->L_Leg_Info.Measure.Theta_dot * arm_cos_f32(Control_Info->L_Leg_Info.Measure.Theta))
	                                                     + Control_Info->L_Leg_Info.VMC.L0_dot * arm_sin_f32(Control_Info->L_Leg_Info.Measure.Theta);
	
	Control_Info->L_Leg_Info.Predict_Chassis_Velocity  = Control_Info->L_Leg_Info.Fusion_Chassis_Velocity +  (Control_Info->Chassis_Accel)*0.002f;  

   Control_Info->L_Leg_Info.Fusion_Chassis_Velocity  = 	Control_Info->L_Leg_Info.Measure.Chassis_Velocity + Control_Info->K*( Control_Info->L_Leg_Info.Predict_Chassis_Velocity - 	Control_Info->L_Leg_Info.Measure.Chassis_Velocity);
	
	
	
  Control_Info->R_Leg_Info.Measure.Chassis_Velocity =  Control_Info->R_Leg_Info.W_Velocity + 
	                                                     +(Control_Info->R_Leg_Info.VMC.L0 * Control_Info->R_Leg_Info.Measure.Theta_dot * arm_cos_f32(Control_Info->R_Leg_Info.Measure.Theta))                                                  
                                                       + Control_Info->R_Leg_Info.VMC.L0_dot * arm_sin_f32(Control_Info->R_Leg_Info.Measure.Theta);
	
	
	
	Control_Info->R_Leg_Info.Predict_Chassis_Velocity  = Control_Info->R_Leg_Info.Fusion_Chassis_Velocity +  (Control_Info->Chassis_Accel)*0.002f;  

  Control_Info->R_Leg_Info.Fusion_Chassis_Velocity  =   Control_Info->R_Leg_Info.Measure.Chassis_Velocity + Control_Info->K*( Control_Info->R_Leg_Info.Predict_Chassis_Velocity - 	  Control_Info->R_Leg_Info.Measure.Chassis_Velocity); 
       
 
	Control_Info->Chassis_Accel =   (float) ( -INS_Info.accel[1] - GravityAccel* arm_sin_f32 (-INS_Info.angle[2])) * arm_cos_f32 (-INS_Info.angle[2]) + 
	                                         ( INS_Info.accel[2] - GravityAccel* arm_cos_f32 (-INS_Info.angle[2])) * arm_sin_f32 (-INS_Info.angle[2]) ; 
 	
	LK_Motor_E = Control_Info->L_Leg_Info.Fusion_Chassis_Velocity - Control_Info->R_Leg_Info.Fusion_Chassis_Velocity;
	
	Yaw_LK_Err = fabsf(INS_Info.gyro[2]/1.7f - LK_Motor_E);
	
	if(Yaw_LK_Err > 0.4f){
	
	  if(Control_Info->L_Leg_Info.Fusion_Chassis_Velocity > Control_Info->R_Leg_Info.Fusion_Chassis_Velocity){
		
		   Control_Info->L_Leg_Info.Fusion_Chassis_Velocity = INS_Info.gyro[2]/1.7f + Control_Info->R_Leg_Info.Fusion_Chassis_Velocity;
		
		}
	
	    if(Control_Info->L_Leg_Info.Fusion_Chassis_Velocity < Control_Info->R_Leg_Info.Fusion_Chassis_Velocity){
		
		   Control_Info->R_Leg_Info.Fusion_Chassis_Velocity = Control_Info->L_Leg_Info.Fusion_Chassis_Velocity - INS_Info.gyro[2]/1.7f;
		
		}
		
		
	
	}
	
	Control_Info->Measure_Chassis_Velocity =(  Control_Info->L_Leg_Info.Fusion_Chassis_Velocity + Control_Info->R_Leg_Info.Fusion_Chassis_Velocity) /2.f; //- INS_Info.gyro[0]*(Control_Info->L_Leg_Info.VMC.L0+Control_Info->R_Leg_Info.VMC.L0)/2.f;

	
	
	
	
	//usart_printf("%f,%f\n",Yaw_LK_Err,0);

	
	
	
	if(Control_Info->Init.Balance_Init.IF_Balance_Init == 1){
	if(Control_Info->Target_Chassis_Velocity == 0 && Control_Info->Chassis_Position_Clean == 0){
		
  	 Control_Info->L_Leg_Info.Measure.Chassis_Position += Control_Info->Measure_Chassis_Velocity*0.002f;
		 Control_Info->R_Leg_Info.Measure.Chassis_Position  += Control_Info->Measure_Chassis_Velocity*0.002f; 
		
    VAL_LIMIT(Control_Info->L_Leg_Info.Measure.Chassis_Position,-0.7f,0.7f);
		VAL_LIMIT(Control_Info->R_Leg_Info.Measure.Chassis_Position,-0.7f,0.7f);
		
		
		}else if(Control_Info->Target_Chassis_Velocity != 0 ){
			Control_Info->L_Leg_Info.Measure.Chassis_Position =0;
		  Control_Info->R_Leg_Info.Measure.Chassis_Position = 0;
		}
 }else{

  	Control_Info->L_Leg_Info.Measure.Chassis_Position =0;
		  Control_Info->R_Leg_Info.Measure.Chassis_Position = 0;

 }
}
//float Leg_Length = 0;
float Leg_Length = 0.19f;



bool Flag1,Flag2 = 0;
static void Control_Target_Update(Control_Info_Typedef *Control_Info){

if(Control_Info->Control_Mode == Remote_Ctrl_Mode){
	
	if(remote_ctrl.rc.ch[3] != 0){
	
	Control_Info->Target_Chassis_Velocity =  f_Ramp_Calc(	Control_Info->Target_Chassis_Velocity,-remote_ctrl.rc.ch[3]*0.003f,0.003f);
		VAL_LIMIT(Control_Info->Target_Chassis_Velocity,-Control_Info->Chassis_Velocity_MAX,Control_Info->Chassis_Velocity_MAX);
	
  }else{
	
	Control_Info->Target_Chassis_Velocity =  f_Ramp_Calc(Control_Info->Target_Chassis_Velocity,0,0.003f);
	
	}
	
//	if(remote_ctrl.rc.s[1] == 1){
//	
//   	Control_Info->Leg_Length_Mode = Leg_Length_High;
//	  Control_Info->Target_Chassis_Velocity = 0;
//	}else{
//	
//	Control_Info->Leg_Length_Mode = Leg_Length_Normol;
//		
//	}

	if(remote_ctrl.rc.s[1] == 1){
	 
		//Control_Info->Chassis_Mode = CHASSIS_SPIN;
		Control_Info->Leg_Length_Mode = Leg_Length_High;
	}else{
	Control_Info->Leg_Length_Mode = Leg_Length_Normol;
	 Control_Info->Chassis_Mode = CHASSIS_FRONT;
	
	}
	
	
	if(Control_Info->Leg_Length_Mode == Leg_Length_High ){
    
	
	if(remote_ctrl.rc.s[0] == 1){
			Control_Info->Chassis_Mode = CHASSIS_SPIN;
		if( Leg_Length <= 0.19f && Flag1 == 0){
	  Flag1 = 1;
		Flag2 = 0;
	  }
	
	 if(Leg_Length >= 0.40f && Flag2 == 0){
		Flag2 = 1;
		Flag1 = 0;
	 }
	 if(Flag1 == 1) Leg_Length+=0.00035f;
	 if(Flag2 == 1) Leg_Length-=0.00035f;
		
	 if(Leg_Length > 0.23f){
	
		Control_Info->LK_Turn_T_To_SendValue = 100;
	  Control_Info->LK_Stand_T_To_SendValue = 100;
	}else{
	
		Control_Info->LK_Turn_T_To_SendValue = 100;
	  Control_Info->LK_Stand_T_To_SendValue = 150;
	}
	 
	 
  	Control_Info->R_Leg_Info.VMC.Target_L0 = f_Ramp_Calc(Control_Info->L_Leg_Info.VMC.Target_L0,Leg_Length,0.0005f);
		Control_Info->L_Leg_Info.VMC.Target_L0 = f_Ramp_Calc(Control_Info->R_Leg_Info.VMC.Target_L0,Leg_Length-0.08f,0.0005f); 
	
 }else{
	  Control_Info->Chassis_Mode = CHASSIS_FRONT;
	 
	  
	 
    Control_Info->R_Leg_Info.VMC.Target_L0 = f_Ramp_Calc(Control_Info->L_Leg_Info.VMC.Target_L0,0.35f,0.0005f);  
		Control_Info->L_Leg_Info.VMC.Target_L0 = f_Ramp_Calc(Control_Info->R_Leg_Info.VMC.Target_L0,0.35f-0.08f,0.0005f); 
   }
	}else{
	
	  Control_Info->LK_Turn_T_To_SendValue = 100;
	  Control_Info->LK_Stand_T_To_SendValue = 200;
    
  	Control_Info->R_Leg_Info.VMC.Target_L0 = f_Ramp_Calc(Control_Info->L_Leg_Info.VMC.Target_L0,0.18f,0.0005f);
		Control_Info->L_Leg_Info.VMC.Target_L0 = f_Ramp_Calc(Control_Info->R_Leg_Info.VMC.Target_L0,0.172f,0.0005f); 
	
	
	
	
	}
	
}else if(Control_Info->Control_Mode == Key_Board_Mode){
	
	static bool E_Flag = 0;
				
		    if(E_Flag == 0 && remote_ctrl.key.set.E == 1){
				
				 Control_Info->SPIN_V = ! Control_Info->SPIN_V;
				 E_Flag = 1;
				
				}else if(remote_ctrl.key.set.E == 0) 	 E_Flag = 0;	
	

if(remote_ctrl.key.set.CTRL == 1 ){

   Control_Info->Chassis_Mode = CHASSIS_SPIN;

 }else if(remote_ctrl.key.set.CTRL == 0){

   if(remote_ctrl.key.set.SHIFT == 1) Control_Info->Chassis_Mode = CHASSIS_SIDE ;
	
	 else  Control_Info->Chassis_Mode = CHASSIS_FRONT ;
} 	
	
	if(Control_Info->Chassis_Mode == CHASSIS_FRONT){
			
			if(Key_W() != 0 || Key_S() != 0){
        
			  Control_Info->Target_Chassis_Velocity = f_Ramp_Calc(Control_Info->Target_Chassis_Velocity,(Key_W()-Key_S())*Control_Info->Chassis_Velocity_MAX,0.003f);
      
			}else{
			
			 Control_Info->Target_Chassis_Velocity =  f_Ramp_Calc(Control_Info->Target_Chassis_Velocity,0,0.005f);
			
		  }
			
		}else if(Control_Info->Chassis_Mode == CHASSIS_SIDE){
			
			if(Key_A() != 0 || Key_D() != 0){
			
			   Control_Info->Target_Chassis_Velocity = f_Ramp_Calc(Control_Info->Target_Chassis_Velocity,(Key_A()-Key_D())*2.f,0.003f);
		
		  }else{
			
			  Control_Info->Target_Chassis_Velocity =  f_Ramp_Calc(Control_Info->Target_Chassis_Velocity,0,0.005f);
			
			}
			
		}else if (Control_Info->Chassis_Mode == CHASSIS_SPIN){
		
		Control_Info->Target_Chassis_Velocity = 0;
			
		}
     
 static bool Leg_Length_Flag = 0;
		
 if(remote_ctrl.key.set.X + remote_ctrl.key.set.C + remote_ctrl.key.set.V == 1 && Leg_Length_Flag == 0){
 
   if(remote_ctrl.key.set.X == 1) {
		 
	 if(Control_Info->Leg_Length_Mode == Leg_Length_UpSlope){
		   
			 Control_Info->Leg_Length_Mode  = Leg_Length_Normol;
		 
       }else Control_Info->Leg_Length_Mode = Leg_Length_UpSlope;
		
	 } else if(remote_ctrl.key.set.C == 1){
		 
		 if(Control_Info->Leg_Length_Mode == Leg_Length_Squat){
		   
			 Control_Info->Leg_Length_Mode  = Leg_Length_Normol;
		 
       }else Control_Info->Leg_Length_Mode = Leg_Length_Squat;
	 
	 }
	 else if(remote_ctrl.key.set.V == 1) {
		 
		 if(Control_Info->Leg_Length_Mode == Leg_Length_High){
		   
			 Control_Info->Leg_Length_Mode  = Leg_Length_Normol;
		 
       }else Control_Info->Leg_Length_Mode = Leg_Length_High;
 
	 }
   Leg_Length_Flag = 1;  
 
 }else if(remote_ctrl.key.set.X + remote_ctrl.key.set.C + remote_ctrl.key.set.V == 0) Leg_Length_Flag = 0;

 static bool Leg_Length_Normol_Flag = 0;
 
 if(remote_ctrl.key.set.F == 1 && Leg_Length_Normol_Flag == 0){
 
  if(Control_Info->Leg_Length_Mode != Leg_Length_Normol)  Control_Info->Leg_Length_Mode = Leg_Length_Normol;
 
  Leg_Length_Normol_Flag = 1;
 
 }else if(remote_ctrl.key.set.F == 0 )  Leg_Length_Normol_Flag = 0; 
 
if(Control_Info->R_Leg_Info.Support.Fly_Flag  +  Control_Info->L_Leg_Info.Support.Fly_Flag == 0){ 
 if(Control_Info->Leg_Length_Mode == Leg_Length_Normol ){
  
  	Control_Info->R_Leg_Info.VMC.Target_L0 = f_Ramp_Calc(Control_Info->L_Leg_Info.VMC.Target_L0,0.18f,0.0005f);
		Control_Info->L_Leg_Info.VMC.Target_L0 = f_Ramp_Calc(Control_Info->R_Leg_Info.VMC.Target_L0,0.18f,0.0005f); 
	
	}else if(Control_Info->Leg_Length_Mode == Leg_Length_UpSlope){
	
		Control_Info->R_Leg_Info.VMC.Target_L0 = f_Ramp_Calc(Control_Info->L_Leg_Info.VMC.Target_L0,0.23f,0.0005f);
		Control_Info->L_Leg_Info.VMC.Target_L0 = f_Ramp_Calc(Control_Info->R_Leg_Info.VMC.Target_L0,0.23f,0.0005f); 
		
	}else if(Control_Info->Leg_Length_Mode == Leg_Length_Squat){
	  
		Control_Info->R_Leg_Info.VMC.Target_L0 = f_Ramp_Calc(Control_Info->L_Leg_Info.VMC.Target_L0,0.13f,0.0005f);
		Control_Info->L_Leg_Info.VMC.Target_L0 = f_Ramp_Calc(Control_Info->R_Leg_Info.VMC.Target_L0,0.13f,0.0005f); 
		
	}else if(Control_Info->Leg_Length_Mode == Leg_Length_High){
	 
		Control_Info->R_Leg_Info.VMC.Target_L0 = f_Ramp_Calc(Control_Info->L_Leg_Info.VMC.Target_L0,0.32f,0.0005f);
		Control_Info->L_Leg_Info.VMC.Target_L0 = f_Ramp_Calc(Control_Info->R_Leg_Info.VMC.Target_L0,0.32f,0.0005f); 
	
	}
}else{

	Control_Info->L_Leg_Info.VMC.Target_L0 = 0.23f;
	Control_Info->R_Leg_Info.VMC.Target_L0 = 0.23f;

}
	  if(Control_Info->Target_Chassis_Velocity != 0) {
	 
	    Control_Info->Chassis_Position_Clean = 1;
	 
	    Control_Info->Target_Chassis_Velocity_to_0 = 0;
 
     }

    if(Control_Info->Target_Chassis_Velocity == 0 && Control_Info->Chassis_Position_Clean  == 1){
      Control_Info->Target_Chassis_Velocity_to_0++;
	    if(Control_Info->Target_Chassis_Velocity_to_0 > 2000){
	    Control_Info->Chassis_Position_Clean = 0;
	    Control_Info->Target_Chassis_Velocity_to_0 = 0;
		  Control_Info->L_Leg_Info.Measure.Chassis_Position = 0;
		  Control_Info->R_Leg_Info.Measure.Chassis_Position = 0; 
	  }
   }
 }
}

  
 

	
static void VMC_Measure_F_Tp_Calculate(Control_Info_Typedef *Control_Info){

 Control_Info->L_Leg_Info.Support.Measure_F = (((Damiao_Motor[1].Data.Torque * arm_cos_f32(( Control_Info->L_Leg_Info.VMC.Phi0 -  Control_Info->L_Leg_Info.VMC.Phi3)))/
                                       (0.16f *  arm_sin_f32 (( Control_Info->L_Leg_Info.VMC.Phi3 - (*Control_Info->L_Leg_Info.VMC.Phi4)))))      -
	                                    ((Damiao_Motor[0].Data.Torque * arm_cos_f32(( Control_Info->L_Leg_Info.VMC.Phi0 -  Control_Info->L_Leg_Info.VMC.Phi2)))/
                                       (0.16f * arm_sin_f32 ((*Control_Info->L_Leg_Info.VMC.Phi1 - (Control_Info->L_Leg_Info.VMC.Phi2))))));

 Control_Info->L_Leg_Info.Support.Measure_Tp= Control_Info->L_Leg_Info.VMC.L0*(((Damiao_Motor[0].Data.Torque * arm_sin_f32(( Control_Info->L_Leg_Info.VMC.Phi0 -  Control_Info->L_Leg_Info.VMC.Phi2)))/
                                       (0.16f *  arm_sin_f32 (( *Control_Info->L_Leg_Info.VMC.Phi1 - (Control_Info->L_Leg_Info.VMC.Phi2)))))      -
	                                    ((Damiao_Motor[1].Data.Torque * arm_sin_f32(( Control_Info->L_Leg_Info.VMC.Phi0 -  Control_Info->L_Leg_Info.VMC.Phi3)))/
                                       (0.16f * arm_sin_f32 ((Control_Info->L_Leg_Info.VMC.Phi3 - (*Control_Info->L_Leg_Info.VMC.Phi4))))));

	Control_Info->R_Leg_Info.Support.Measure_F = (((Damiao_Motor[2].Data.Torque * arm_cos_f32(( Control_Info->R_Leg_Info.VMC.Phi0 -  Control_Info->R_Leg_Info.VMC.Phi3)))/
                                       (0.16f *  arm_sin_f32 (( Control_Info->R_Leg_Info.VMC.Phi3 - (*Control_Info->R_Leg_Info.VMC.Phi4)))))      -
	                                    ((Damiao_Motor[3].Data.Torque * arm_cos_f32(( Control_Info->R_Leg_Info.VMC.Phi0 -  Control_Info->R_Leg_Info.VMC.Phi2)))/
                                       (0.16f * arm_sin_f32 ((*Control_Info->R_Leg_Info.VMC.Phi1 - (Control_Info->R_Leg_Info.VMC.Phi2))))));

 Control_Info->R_Leg_Info.Support.Measure_Tp= Control_Info->R_Leg_Info.VMC.L0*(((Damiao_Motor[3].Data.Torque * arm_sin_f32(( Control_Info->R_Leg_Info.VMC.Phi0 -  Control_Info->R_Leg_Info.VMC.Phi2)))/
                                       (0.16f *  arm_sin_f32 (( *Control_Info->R_Leg_Info.VMC.Phi1 - (Control_Info->R_Leg_Info.VMC.Phi2)))))      -
	                                    ((Damiao_Motor[2].Data.Torque * arm_sin_f32(( Control_Info->R_Leg_Info.VMC.Phi0 -  Control_Info->R_Leg_Info.VMC.Phi3)))/
                                       (0.16f * arm_sin_f32 ((Control_Info->R_Leg_Info.VMC.Phi3 - (*Control_Info->R_Leg_Info.VMC.Phi4))))));
}
static void Control_Support_Calculate(Control_Info_Typedef *Control_Info){
if(Control_Info->Init.Leg_Init.IF_Leg_Init == 1 ){

	
 Control_Info->L_Leg_Info.Support.P =  Control_Info->L_Leg_Info.Support.Measure_F * arm_cos_f32(Control_Info->L_Leg_Info.Measure.Theta)
                                      +(-(Control_Info->L_Leg_Info.Support.Measure_Tp * arm_sin_f32(Control_Info->L_Leg_Info.Measure.Theta))/Control_Info->L_Leg_Info.VMC.L0);
 
	                                    
 Control_Info->L_Leg_Info.Support.FN =    Control_Info->L_Leg_Info.Support.P + 7.f*GravityAccel;

 Control_Info->R_Leg_Info.Support.P =  Control_Info->R_Leg_Info.Support.Measure_F*arm_cos_f32(Control_Info->R_Leg_Info.Measure.Theta)
                                      +((Control_Info->R_Leg_Info.Support.Measure_Tp*arm_sin_f32(Control_Info->R_Leg_Info.Measure.Theta))/Control_Info->R_Leg_Info.VMC.L0);
                        
Control_Info->R_Leg_Info.Support.FN =    Control_Info->R_Leg_Info.Support.P + 7.f*GravityAccel ;
	

	
}else{

  Control_Info->L_Leg_Info.Support.FN =150;
  Control_Info->R_Leg_Info.Support.FN =150;
}
if(Control_Info->L_Leg_Info.Support.FN < Control_Info->L_Leg_Info.Off_Ground_FN) Control_Info->L_Leg_Info.Support.Fly_Flag = 1;
else  Control_Info->L_Leg_Info.Support.Fly_Flag = 0;

if(Control_Info->R_Leg_Info.Support.FN < Control_Info->R_Leg_Info.Off_Ground_FN) Control_Info->R_Leg_Info.Support.Fly_Flag = 1;
else  Control_Info->R_Leg_Info.Support.Fly_Flag = 0;
}
bool Begin_Jump = 0;
bool Jump1 = 0;
bool Jump2 = 0;

bool One_Flag = 0;
uint16_t Begin_Jump_Cnt = 0;
uint16_t Jump_Cnt1 = 0;
uint16_t Jump_Cnt2 = 0;
uint16_t Jump_Cnt3 = 0;


static void Jump(){

    if(remote_ctrl.rc.ch[4] == 255 && One_Flag == 0){
		   
			 One_Flag = 1;
		   Begin_Jump = 1;
		
		}
	  
	  if(One_Flag == 1){
		
		if(remote_ctrl.rc.ch[4] == 255){
		
		  One_Flag = 1;
			
		}else {
		
			One_Flag = 0; 
		
		
		  }
		}
	
	if(Begin_Jump == 1){
	if(Begin_Jump_Cnt < 200){
	 Control_Info.L_Leg_Info.VMC.Target_L0 = 0.13f;
	 Control_Info.R_Leg_Info.VMC.Target_L0 = 0.13f;		
	 Begin_Jump_Cnt++;
	}else if(Begin_Jump_Cnt >= 200){
	  
	if(Jump_Cnt1 < 100 )	
	Jump1 = 1;
	 Control_Info.L_Leg_Info.VMC.Target_L0 = 0.35f;
	 Control_Info.R_Leg_Info.VMC.Target_L0 = 0.35f;
		 PID_Leg_length_thrust[0].param.kp = 70000;
		  PID_Leg_length_thrust[1].param.kp = 70000;
		  Jump_Cnt1++;
		  
	
		
		  if(Jump_Cnt1 >= 100 ){
			
			  Control_Info.L_Leg_Info.VMC.Target_L0 = 0.13f;
	      Control_Info.R_Leg_Info.VMC.Target_L0 = 0.13f;
			  Jump_Cnt2++;
				if(Jump_Cnt2 >100){
				  Control_Info.L_Leg_Info.VMC.Target_L0 = 0.18f;
	        Control_Info.R_Leg_Info.VMC.Target_L0 = 0.18f;
				  PID_Leg_length_thrust[0].param.kp = 5000;
		      PID_Leg_length_thrust[1].param.kp = 5000;
					PID_Leg_length_thrust[0].param.kd = 200000;
		  PID_Leg_length_thrust[1].param.kd = 200000;
					if( Control_Info.L_Leg_Info.Support.FN > 80 && Control_Info.R_Leg_Info.Support.FN >80){
					   Jump_Cnt2 = 0;
						Begin_Jump_Cnt = 0;
				     Jump1 = 0;
			      Jump_Cnt1 = 0;
				   Begin_Jump = 0;
					  
					 } 
						 
					
				}
			
			}
	 }
	
	}else{
	 
	 PID_Leg_length_thrust[0].param.kp = 1000;
		  PID_Leg_length_thrust[1].param.kp = 1000;
		
	PID_Leg_length_thrust[0].param.kd = 100000;
		  PID_Leg_length_thrust[1].param.kd = 100000;
	}

}

   


static void Control_LQR_X_Update(Control_Info_Typedef *Control_Info){

	
	if(Control_Info->L_Leg_Info.Support.Fly_Flag == 1 ||Jump1 == 1){
	  
	Control_Info->L_Leg_Info.Target.Theta = 0.03f;	 
	
 }else{
 
  Control_Info->L_Leg_Info.Target.Theta = 0.f;	
	 
 }	
	
	
	
Control_Info->L_Leg_Info.LQR_X[0] = (Control_Info->L_Leg_Info.Target.Theta - Control_Info->L_Leg_Info.Measure.Theta);
Control_Info->L_Leg_Info.LQR_X[1] = (Control_Info->L_Leg_Info.Target.Theta_dot - Control_Info->L_Leg_Info.Measure.Theta_dot);
Control_Info->L_Leg_Info.LQR_X[2] = (Control_Info->Target_Chassis_Position  - Control_Info->L_Leg_Info.Measure.Chassis_Position) ;//
Control_Info->L_Leg_Info.LQR_X[3] = (Control_Info->Target_Chassis_Velocity  -    Control_Info->Measure_Chassis_Velocity   );
Control_Info->L_Leg_Info.LQR_X[4] = (Control_Info->L_Leg_Info.Target.Phi - Control_Info->L_Leg_Info.Measure.Phi);
Control_Info->L_Leg_Info.LQR_X[5] = (Control_Info->L_Leg_Info.Target.Phi_dot - Control_Info->L_Leg_Info.Measure.Phi_dot);

if(Control_Info->R_Leg_Info.Support.Fly_Flag == 1 ||Jump1 == 1){
	  
	Control_Info->R_Leg_Info.Target.Theta = -0.f;	 
	
 }else{
 
  Control_Info->R_Leg_Info.Target.Theta = -0.075f;	
	 
 }		
	
	
	
	
Control_Info->R_Leg_Info.LQR_X[0] = (Control_Info->R_Leg_Info.Target.Theta - Control_Info->R_Leg_Info.Measure.Theta);
Control_Info->R_Leg_Info.LQR_X[1] = (Control_Info->R_Leg_Info.Target.Theta_dot - Control_Info->R_Leg_Info.Measure.Theta_dot);
Control_Info->R_Leg_Info.LQR_X[2] = (Control_Info->Target_Chassis_Position - Control_Info->R_Leg_Info.Measure.Chassis_Position) ;//
Control_Info->R_Leg_Info.LQR_X[3] = (Control_Info->Target_Chassis_Velocity   -     Control_Info->Measure_Chassis_Velocity  );
Control_Info->R_Leg_Info.LQR_X[4] = (Control_Info->R_Leg_Info.Target.Phi - Control_Info->R_Leg_Info.Measure.Phi);
Control_Info->R_Leg_Info.LQR_X[5] = (Control_Info->R_Leg_Info.Target.Phi_dot - Control_Info->R_Leg_Info.Measure.Phi_dot);


if(Control_Info->Init.Balance_Init.IF_Balance_Init == 0){

Control_Info->L_Leg_Info.LQR_X[2] = 0;
Control_Info->R_Leg_Info.LQR_X[2] = 0;

Control_Info->L_Leg_Info.LQR_X[3] = 0;
Control_Info->R_Leg_Info.LQR_X[3] = 0;
	
	
}





}


static void Control_LQR_Stand_T_Balance_Tp_Calculate(Control_Info_Typedef *Control_Info){

	if(Control_Info->Init.Leg_Init.IF_Leg_Init == 0){
	
		Control_Info->L_Leg_Info.T_Gain.Phi = 3.f;
	Control_Info->L_Leg_Info.T_Gain.Phi_dot = 1.f;
  Control_Info->L_Leg_Info.Tp_Gain.Theta = 3.f;
	Control_Info->L_Leg_Info.Tp_Gain.Theta_dot = 1.f;
	
		Control_Info->L_Leg_Info.Tp_Gain.Position = 1.f;
	Control_Info->L_Leg_Info.Tp_Gain.Velocity = 1.f;
	
	Control_Info->L_Leg_Info.Tp_Gain.Phi = 2.f;
	Control_Info->L_Leg_Info.Tp_Gain.Phi_dot = 2.f;
	
	
	Control_Info->R_Leg_Info.T_Gain.Phi = 3.f;
	Control_Info->R_Leg_Info.T_Gain.Phi_dot = 1.f;
	
  Control_Info->R_Leg_Info.Tp_Gain.Theta = 5.f;
	Control_Info->R_Leg_Info.Tp_Gain.Theta_dot = 5.f;
	
		Control_Info->R_Leg_Info.Tp_Gain.Position = 1.f;
	Control_Info->R_Leg_Info.Tp_Gain.Velocity = 1.f;
	
	Control_Info->R_Leg_Info.Tp_Gain.Phi = 2.f;
	Control_Info->R_Leg_Info.Tp_Gain.Phi_dot = 2.f;
		
	
	}else{
	

		Control_Info->L_Leg_Info.T_Gain.Phi =1.f;
	Control_Info->L_Leg_Info.T_Gain.Phi_dot = 1.f;
	
  Control_Info->L_Leg_Info.Tp_Gain.Theta = 2.f;
	Control_Info->L_Leg_Info.Tp_Gain.Theta_dot = 1.f;
	
		Control_Info->L_Leg_Info.Tp_Gain.Position = 1.f;
	Control_Info->L_Leg_Info.Tp_Gain.Velocity = 1.f;
	
	Control_Info->L_Leg_Info.Tp_Gain.Phi = 2.f;
	Control_Info->L_Leg_Info.Tp_Gain.Phi_dot = 1.f;
	
	
	Control_Info->R_Leg_Info.T_Gain.Phi = 1.f;
	Control_Info->R_Leg_Info.T_Gain.Phi_dot = 1.f;
	
  Control_Info->R_Leg_Info.Tp_Gain.Theta = 2.f;
	Control_Info->R_Leg_Info.Tp_Gain.Theta_dot = 1.f;
	
		Control_Info->R_Leg_Info.Tp_Gain.Position = 1.f;
	Control_Info->R_Leg_Info.Tp_Gain.Velocity = 1.f;
	
	Control_Info->R_Leg_Info.Tp_Gain.Phi = 2.f;
	Control_Info->R_Leg_Info.Tp_Gain.Phi_dot = 1.f;

	}
	
	 
if(Control_Info->L_Leg_Info.Support.Fly_Flag == 1 || Jump1 == 1){
	  
		 Control_Info->L_Leg_Info.Moment.Stand_T = 0;
	   Control_Info->L_Leg_Info.Moment.Balance_Tp = ( Control_Info->L_Leg_Info.LQR_X[0]*Control_Info->L_Leg_Info.LQR_K[1][0]*Control_Info->L_Leg_Info.Tp_Gain.Theta 
	                                                  +Control_Info->L_Leg_Info.LQR_X[1]*Control_Info->L_Leg_Info.LQR_K[1][1]*Control_Info->L_Leg_Info.Tp_Gain.Theta_dot);

}else{

	
 Control_Info->L_Leg_Info.LQR_Output[0][0] = Control_Info->L_Leg_Info.LQR_X[0]*Control_Info->L_Leg_Info.LQR_K[0][0];
 Control_Info->L_Leg_Info.LQR_Output[0][1] = Control_Info->L_Leg_Info.LQR_X[1]*Control_Info->L_Leg_Info.LQR_K[0][1];
 Control_Info->L_Leg_Info.LQR_Output[0][2] = Control_Info->L_Leg_Info.LQR_X[2]*Control_Info->L_Leg_Info.LQR_K[0][2]*2.f;
 
 Control_Info->L_Leg_Info.LQR_Output[0][3] = Control_Info->L_Leg_Info.LQR_X[3]*Control_Info->L_Leg_Info.LQR_K[0][3]*1.2f;
 Control_Info->L_Leg_Info.LQR_Output[0][4] = Control_Info->L_Leg_Info.LQR_X[4]*Control_Info->L_Leg_Info.LQR_K[0][4]*Control_Info->L_Leg_Info.T_Gain.Phi;
 Control_Info->L_Leg_Info.LQR_Output[0][5] = Control_Info->L_Leg_Info.LQR_X[5]*Control_Info->L_Leg_Info.LQR_K[0][5]*Control_Info->L_Leg_Info.T_Gain.Phi_dot;

 Control_Info->L_Leg_Info.Moment.Stand_T = (Control_Info->L_Leg_Info.LQR_Output[0][0]+Control_Info->L_Leg_Info.LQR_Output[0][1]+Control_Info->L_Leg_Info.LQR_Output[0][2]+
	                                         Control_Info->L_Leg_Info.LQR_Output[0][3]+Control_Info->L_Leg_Info.LQR_Output[0][4]+Control_Info->L_Leg_Info.LQR_Output[0][5]) ;
	
 VAL_LIMIT(Control_Info->L_Leg_Info.Moment.Stand_T,-Stand_T_MAX,Stand_T_MAX);

 Control_Info->L_Leg_Info.LQR_Output[1][0]  = Control_Info->L_Leg_Info.LQR_X[0]*Control_Info->L_Leg_Info.LQR_K[1][0]*Control_Info->L_Leg_Info.Tp_Gain.Theta;
 Control_Info->L_Leg_Info.LQR_Output[1][1] = Control_Info->L_Leg_Info.LQR_X[1]*Control_Info->L_Leg_Info.LQR_K[1][1]*Control_Info->L_Leg_Info.Tp_Gain.Theta_dot;
 Control_Info->L_Leg_Info.LQR_Output[1][2] = Control_Info->L_Leg_Info.LQR_X[2]*Control_Info->L_Leg_Info.LQR_K[1][2]*Control_Info->L_Leg_Info.Tp_Gain.Position;
 Control_Info->L_Leg_Info.LQR_Output[1][3] = Control_Info->L_Leg_Info.LQR_X[3]*Control_Info->L_Leg_Info.LQR_K[1][3]*Control_Info->L_Leg_Info.Tp_Gain.Velocity;
 Control_Info->L_Leg_Info.LQR_Output[1][4] = Control_Info->L_Leg_Info.LQR_X[4]*Control_Info->L_Leg_Info.LQR_K[1][4]*Control_Info->L_Leg_Info.Tp_Gain.Phi; 
 Control_Info->L_Leg_Info.LQR_Output[1][5] = Control_Info->L_Leg_Info.LQR_X[5]*Control_Info->L_Leg_Info.LQR_K[1][5]*Control_Info->L_Leg_Info.Tp_Gain.Phi_dot;

	Control_Info->L_Leg_Info.Moment.Balance_Tp = (Control_Info->L_Leg_Info.LQR_Output[1][0]+Control_Info->L_Leg_Info.LQR_Output[1][1]+Control_Info->L_Leg_Info.LQR_Output[1][2]+
	                                              Control_Info->L_Leg_Info.LQR_Output[1][3]+Control_Info->L_Leg_Info.LQR_Output[1][4]+Control_Info->L_Leg_Info.LQR_Output[1][5]);

 VAL_LIMIT(Control_Info->L_Leg_Info.Moment.Balance_Tp,-Balance_Tp_MAX,Balance_Tp_MAX);
 
}
	
if(Control_Info->R_Leg_Info.Support.Fly_Flag  == 1 || Jump1 == 1){
	  
		Control_Info->R_Leg_Info.Moment.Stand_T = 0;

	  Control_Info->R_Leg_Info.Moment.Balance_Tp = ( Control_Info->R_Leg_Info.LQR_X[0]*Control_Info->R_Leg_Info.LQR_K[1][0]*Control_Info->R_Leg_Info.Tp_Gain.Theta 
	                                                  +Control_Info->R_Leg_Info.LQR_X[1]*Control_Info->R_Leg_Info.LQR_K[1][1]*Control_Info->R_Leg_Info.Tp_Gain.Theta_dot);
 }else{
 
 Control_Info->R_Leg_Info.LQR_Output[0][0] = Control_Info->R_Leg_Info.LQR_X[0]*Control_Info->R_Leg_Info.LQR_K[0][0];
 Control_Info->R_Leg_Info.LQR_Output[0][1] = Control_Info->R_Leg_Info.LQR_X[1]*Control_Info->R_Leg_Info.LQR_K[0][1];
 Control_Info->R_Leg_Info.LQR_Output[0][2] = Control_Info->R_Leg_Info.LQR_X[2]*Control_Info->R_Leg_Info.LQR_K[0][2]*2.f;
 Control_Info->R_Leg_Info.LQR_Output[0][3] = Control_Info->R_Leg_Info.LQR_X[3]*Control_Info->R_Leg_Info.LQR_K[0][3]*1.2f;
 Control_Info->R_Leg_Info.LQR_Output[0][4] = Control_Info->R_Leg_Info.LQR_X[4]*Control_Info->R_Leg_Info.LQR_K[0][4]*Control_Info->R_Leg_Info.T_Gain.Phi;
 Control_Info->R_Leg_Info.LQR_Output[0][5] = Control_Info->R_Leg_Info.LQR_X[5]*Control_Info->R_Leg_Info.LQR_K[0][5]*Control_Info->R_Leg_Info.T_Gain.Phi_dot;

Control_Info->R_Leg_Info.Moment.Stand_T = (Control_Info->R_Leg_Info.LQR_Output[0][0]+Control_Info->R_Leg_Info.LQR_Output[0][1]+Control_Info->R_Leg_Info.LQR_Output[0][2]+
	                         Control_Info->R_Leg_Info.LQR_Output[0][3]+Control_Info->R_Leg_Info.LQR_Output[0][4]+Control_Info->R_Leg_Info.LQR_Output[0][5]);

VAL_LIMIT(Control_Info->R_Leg_Info.Moment.Stand_T,-Stand_T_MAX,Stand_T_MAX);

 Control_Info->R_Leg_Info.LQR_Output[1][0] = Control_Info->R_Leg_Info.LQR_X[0]*Control_Info->R_Leg_Info.LQR_K[1][0]*Control_Info->R_Leg_Info.Tp_Gain.Theta;
 Control_Info->R_Leg_Info.LQR_Output[1][1] = Control_Info->R_Leg_Info.LQR_X[1]*Control_Info->R_Leg_Info.LQR_K[1][1]*Control_Info->R_Leg_Info.Tp_Gain.Theta_dot;
 Control_Info->R_Leg_Info.LQR_Output[1][2] = Control_Info->R_Leg_Info.LQR_X[2]*Control_Info->R_Leg_Info.LQR_K[1][2]*Control_Info->R_Leg_Info.Tp_Gain.Position;
 Control_Info->R_Leg_Info.LQR_Output[1][3] = Control_Info->R_Leg_Info.LQR_X[3]*Control_Info->R_Leg_Info.LQR_K[1][3]*Control_Info->R_Leg_Info.Tp_Gain.Velocity;
 Control_Info->R_Leg_Info.LQR_Output[1][4] = Control_Info->R_Leg_Info.LQR_X[4]*Control_Info->R_Leg_Info.LQR_K[1][4]*Control_Info->R_Leg_Info.Tp_Gain.Phi;
 Control_Info->R_Leg_Info.LQR_Output[1][5] = Control_Info->R_Leg_Info.LQR_X[5]*Control_Info->R_Leg_Info.LQR_K[1][5]*Control_Info->R_Leg_Info.Tp_Gain.Phi_dot;

	Control_Info->R_Leg_Info.Moment.Balance_Tp = (Control_Info->R_Leg_Info.LQR_Output[1][0]+Control_Info->R_Leg_Info.LQR_Output[1][1]+Control_Info->R_Leg_Info.LQR_Output[1][2]+
	                                              Control_Info->R_Leg_Info.LQR_Output[1][3]+Control_Info->R_Leg_Info.LQR_Output[1][4]+Control_Info->R_Leg_Info.LQR_Output[1][5]);

 VAL_LIMIT(Control_Info->R_Leg_Info.Moment.Balance_Tp,-Balance_Tp_MAX,Balance_Tp_MAX);
 }


}
	
static void Control_Comprehensive_F_Calculate(Control_Info_Typedef *Control_Info){
 
 Control_Info->L_Leg_Info.Moment.Leg_Length_F = Control_Info->L_Leg_Info.VMC.Gravity_Compensation + f_PID_Calculate(&PID_Leg_length_thrust[0],Control_Info->L_Leg_Info.VMC.Target_L0,Control_Info->L_Leg_Info.VMC.L0);
 Control_Info->R_Leg_Info.Moment.Leg_Length_F = Control_Info->R_Leg_Info.VMC.Gravity_Compensation + f_PID_Calculate(&PID_Leg_length_thrust[1],Control_Info->R_Leg_Info.VMC.Target_L0,Control_Info->R_Leg_Info.VMC.L0);
  
if(Control_Info->Chassis_Mode == CHASSIS_FRONT ){
		 
	  Control_Info->Center_Angle = -120.f;
		
	 }else if(Control_Info->Chassis_Mode == CHASSIS_SIDE ){
	
	  Control_Info->Center_Angle = 150.f;
	
	 }	
    
	Control_Info->Yaw_Err = Control_Info->Center_Angle - DJI_Yaw_Motor.Data.angle;
	
   if(Control_Info->Yaw_Err > 180 ) Control_Info->Yaw_Err -= 360.f;
   else if (Control_Info->Yaw_Err < -180.f) Control_Info->Yaw_Err += 360.f;	

	
if(Control_Info->Init.Joint_Init.IF_Joint_Init ==1 ){
	
	static bool Vision_Flag = 0;
	
	if(remote_ctrl.mouse.press_r == 0){
		
		Vision_Flag = 0;
   
		if(Control_Info->Chassis_Mode != CHASSIS_SPIN ){
	

   Control_Info->L_Leg_Info.Moment.Turn_T  = f_PID_Calculate(&PID_Left_Turn_Velocity,f_PID_Calculate(&PID_Left_Turn_Angle,Control_Info->Yaw_Err,0),DJI_Yaw_Motor.Data.velocity);
	 Control_Info->R_Leg_Info.Moment.Turn_T  = 	Control_Info->L_Leg_Info.Moment.Turn_T;
	 
	}else if(Control_Info->Chassis_Mode == CHASSIS_SPIN){
	    
		  Control_Info->L_Leg_Info.Moment.Turn_T    = f_PID_Calculate(&PID_Turn_Velocity,400.f,-INS_Info.yaw_gyro);
	    Control_Info->R_Leg_Info.Moment.Turn_T  = 	Control_Info->L_Leg_Info.Moment.Turn_T;
	
	}
	
	 VAL_LIMIT(Control_Info->L_Leg_Info.Moment.Turn_T,-Stand_T_MAX,Stand_T_MAX); 
   VAL_LIMIT(Control_Info->R_Leg_Info.Moment.Turn_T,-Stand_T_MAX,Stand_T_MAX);

	}else if(remote_ctrl.mouse.press_r == 1){
	
	if( Control_Info->Chassis_Mode != CHASSIS_SPIN){
  
	 if( Control_Info->IF_Aiming_Enable == 1){
		
		 if(Vision_Flag == 0 ){
	     Control_Info->Vision_Lock_Angle = INS_Info.yaw_angle;
	     Vision_Flag  = 1;
	   }
		 
		 Control_Info->L_Leg_Info.Moment.Turn_T  = f_PID_Calculate(&PID_Vision_Lock_Gyro,f_PID_Calculate(&PID_Vision_Lock_Angle,Control_Info->Vision_Lock_Angle,INS_Info.yaw_angle),INS_Info.yaw_gyro);
	   Control_Info->R_Leg_Info.Moment.Turn_T  = 	Control_Info->L_Leg_Info.Moment.Turn_T;
	 
	 }else{
	   Vision_Flag = 0;
	   Control_Info->L_Leg_Info.Moment.Turn_T  = f_PID_Calculate(&PID_Left_Turn_Velocity,f_PID_Calculate(&PID_Left_Turn_Angle,Control_Info->Yaw_Err,0),DJI_Yaw_Motor.Data.velocity);
	   Control_Info->R_Leg_Info.Moment.Turn_T  = 	Control_Info->L_Leg_Info.Moment.Turn_T;
	 }
 }else if(Control_Info->Chassis_Mode == CHASSIS_SPIN){
	    

			
			Control_Info->L_Leg_Info.Moment.Turn_T    = f_PID_Calculate(&PID_Turn_Velocity,550,-INS_Info.yaw_gyro);
	    Control_Info->R_Leg_Info.Moment.Turn_T  = 	Control_Info->L_Leg_Info.Moment.Turn_T;
				
			
	
	 
		 
	
	}
 }
}else{
  Control_Info->L_Leg_Info.Moment.Turn_T=0;
  Control_Info->R_Leg_Info.Moment.Turn_T=0;
}

  Control_Info->L_Leg_Info.Moment.Roll_F = -20*(INS_Info.rol_angle+0.7f);
  Control_Info->R_Leg_Info.Moment.Roll_F =  20*(INS_Info.rol_angle+0.7f);

if(Control_Info->L_Leg_Info.Support.Fly_Flag == 1 ||Jump1 == 1){
  Control_Info->L_Leg_Info.Moment.Roll_F = 0;
  Control_Info->L_Leg_Info.Moment.Turn_T = 0;
}
if(Control_Info->R_Leg_Info.Support.Fly_Flag == 1 ||Jump1 == 1){
  Control_Info->R_Leg_Info.Moment.Roll_F = 0;
  Control_Info->R_Leg_Info.Moment.Turn_T = 0;
}

Control_Info->L_Leg_Info.Moment.Leg_Coordinate_Tp = f_PID_Calculate(&PID_Leg_Coordinate,0,Control_Info->L_Leg_Info.Measure.Theta-(Control_Info->R_Leg_Info.Measure.Theta + 0.075f));
Control_Info->R_Leg_Info.Moment.Leg_Coordinate_Tp = Control_Info->L_Leg_Info.Moment.Leg_Coordinate_Tp ;

}

static void Power_Limit(Control_Info_Typedef *Control_Info){
Control_Info->Power_Gain = 1.f;
//	if(Referee_Info.power_heat_data.buffer_energy >= 60) Control_Info->Power_Gain = 1.f;
//	else if(Referee_Info.power_heat_data.buffer_energy >= 50) Control_Info->Power_Gain = 0.8f;
//	else if(Referee_Info.power_heat_data.buffer_energy >= 40) Control_Info->Power_Gain = 0.6f;
//  else if(Referee_Info.power_heat_data.buffer_energy >= 30) Control_Info->Power_Gain = 0.4f;
//	else if(Referee_Info.power_heat_data.buffer_energy >= 20) Control_Info->Power_Gain = 0.2f;
//	else if(Referee_Info.power_heat_data.buffer_energy >= 10) Control_Info->Power_Gain = 0.05f;
//	else if(Referee_Info.power_heat_data.buffer_energy >= 0) Control_Info->Power_Gain = 0.f;
	Control_Info->Velocity_Power_Gain = 1.f;
//	if(Referee_Info.power_heat_data.buffer_energy >= 50) Control_Info->Velocity_Power_Gain = 1.f;
//	else if(Referee_Info.power_heat_data.buffer_energy >= 40) Control_Info->Velocity_Power_Gain = 0.9f;
//	else if(Referee_Info.power_heat_data.buffer_energy >= 30) Control_Info->Velocity_Power_Gain = 0.8f;
//  else if(Referee_Info.power_heat_data.buffer_energy >= 20) Control_Info->Velocity_Power_Gain = 0.7f;
//	else if(Referee_Info.power_heat_data.buffer_energy >= 10) Control_Info->Velocity_Power_Gain = 0.6f;
//	else if(Referee_Info.power_heat_data.buffer_energy >= 0) Control_Info->Velocity_Power_Gain = 0.5f;
	
}
static void Level_Info_Update(Control_Info_Typedef *Control_Info){

//if(Referee_Info.robot_status.chassis_power_limit <=100){	
//	
//	if(Referee_Info.robot_status.chassis_power_limit <= 70){
//  Control_Info->LK_Turn_T_To_SendValue = 130 - (uint16_t) ((70 - Referee_Info.robot_status.chassis_power_limit)/10)*10;
//	}else if(Referee_Info.robot_status.chassis_power_limit == 80){
//	 Control_Info->LK_Turn_T_To_SendValue = 140;
//	}else if(Referee_Info.robot_status.chassis_power_limit >= 90){
//	
//	 Control_Info->LK_Turn_T_To_SendValue = 150;		
//	
//	}

//  Control_Info->LK_Stand_T_To_SendValue = 200;
//	
//}else if(Referee_Info.robot_status.chassis_power_limit > 100){

// Control_Info->LK_Turn_T_To_SendValue  = 200;

//Control_Info->LK_Stand_T_To_SendValue = 200;

//}

 Control_Info->Chassis_Velocity_MAX = 3.f * Control_Info->Velocity_Power_Gain;

}

static void Control_T_Tp_F_Calculate(Control_Info_Typedef *Control_Info){
	
	
 Control_Info->L_Leg_Info.Moment.Tp = -Control_Info->L_Leg_Info.Moment.Balance_Tp  -  Control_Info->L_Leg_Info.Moment.Leg_Coordinate_Tp;
 Control_Info->R_Leg_Info.Moment.Tp =  Control_Info->R_Leg_Info.Moment.Balance_Tp  -  Control_Info->R_Leg_Info.Moment.Leg_Coordinate_Tp;

 VAL_LIMIT( Control_Info->L_Leg_Info.Moment.Tp,-Balance_Tp_MAX,Balance_Tp_MAX);
 VAL_LIMIT( Control_Info->R_Leg_Info.Moment.Tp,-Balance_Tp_MAX,Balance_Tp_MAX);

	
	
	
	Control_Info ->Turn_T_SendValue[0] =   (Control_Info->L_Leg_Info.Moment.Turn_T*Control_Info->LK_Turn_T_To_SendValue)*Control_Info->Power_Gain;
	Control_Info ->Stand_T_SendValue[0] =  (-Control_Info->L_Leg_Info.Moment.Stand_T * Control_Info->LK_Stand_T_To_SendValue);
	
	VAL_LIMIT(Control_Info ->Turn_T_SendValue[0],-(2000-fabs(Control_Info ->Stand_T_SendValue[0])),(2000-fabs(Control_Info ->Stand_T_SendValue[0])));
	
	Control_Info->T_SendValue[0] = (int16_t)  (Control_Info ->Turn_T_SendValue[0] + Control_Info ->Stand_T_SendValue[0]);
	
	Control_Info ->Turn_T_SendValue[1] =   (Control_Info->R_Leg_Info.Moment.Turn_T*Control_Info->LK_Turn_T_To_SendValue)*Control_Info->Power_Gain;
	Control_Info ->Stand_T_SendValue[1] =  (Control_Info->R_Leg_Info.Moment.Stand_T * Control_Info->LK_Stand_T_To_SendValue);
	
	VAL_LIMIT(Control_Info ->Turn_T_SendValue[1],-(2000-fabs(Control_Info ->Stand_T_SendValue[1])),(2000-fabs(Control_Info ->Stand_T_SendValue[1])));
	
	Control_Info->T_SendValue[1] = 	(int16_t) ( Control_Info ->Turn_T_SendValue[1] + Control_Info ->Stand_T_SendValue[1] );
 
 VAL_LIMIT(Control_Info->T_SendValue[0],-2000,2000);
 VAL_LIMIT(Control_Info->T_SendValue[1],-2000,2000);
 
Control_Info->L_Leg_Info.Moment.F =  Control_Info->L_Leg_Info.Moment.Leg_Length_F +  Control_Info->L_Leg_Info.Moment.Roll_F;
Control_Info->R_Leg_Info.Moment.F =  Control_Info->R_Leg_Info.Moment.Leg_Length_F +  Control_Info->R_Leg_Info.Moment.Roll_F;

 VAL_LIMIT(Control_Info->L_Leg_Info.Moment.F,-250.f,250.f);
 VAL_LIMIT(Control_Info->R_Leg_Info.Moment.F,-250.f,250.f);
 

}

static void VMC_F_Tp_To_Joint_Calculate(Control_Info_Typedef *Control_Info){
 
	 float Phi2_3,Phi0_3,Phi1_2,Phi0_2,Phi3_4;
   
	 Phi2_3=  ( Control_Info->L_Leg_Info.VMC.Phi2 -Control_Info->L_Leg_Info.VMC.Phi3);
   Phi0_3=  ( Control_Info->L_Leg_Info.VMC.Phi0 -  Control_Info->L_Leg_Info.VMC.Phi3);
   Phi1_2=  ((*Control_Info->L_Leg_Info.VMC.Phi1) -  Control_Info->L_Leg_Info.VMC.Phi2);
  Control_Info->L_Leg_Info.SendValue.T1 = -(((Control_Info->L1 * arm_sin_f32(Phi1_2))*(Control_Info->L_Leg_Info.Moment.F 
	                                     *Control_Info->L_Leg_Info.VMC.L0 * arm_sin_f32(Phi0_3)+ 
	                                      Control_Info->L_Leg_Info.Moment.Tp * arm_cos_f32(Phi0_3)))
	                                    /(Control_Info->L_Leg_Info.VMC.L0*arm_sin_f32(Phi2_3)));
   Phi0_2 =  ( Control_Info->L_Leg_Info.VMC.Phi0 -  Control_Info->L_Leg_Info.VMC.Phi2);
   Phi3_4 =  ( Control_Info->L_Leg_Info.VMC.Phi3 - (*Control_Info->L_Leg_Info.VMC.Phi4));
  Control_Info->L_Leg_Info.SendValue.T2=  -(((Control_Info->L4 * arm_sin_f32(Phi3_4))*(Control_Info->L_Leg_Info.Moment.F  
	                                     *Control_Info->L_Leg_Info.VMC.L0 * arm_sin_f32(Phi0_2)+ 
	                                      Control_Info->L_Leg_Info.Moment.Tp  * arm_cos_f32(Phi0_2)))
	                                    /(Control_Info->L_Leg_Info.VMC.L0*arm_sin_f32(Phi2_3)));
	 
	 Phi2_3=  ( Control_Info->R_Leg_Info.VMC.Phi2 -Control_Info->R_Leg_Info.VMC.Phi3);
   Phi0_3=  ( Control_Info->R_Leg_Info.VMC.Phi0 -  Control_Info->R_Leg_Info.VMC.Phi3);
   Phi1_2=  ((*Control_Info->R_Leg_Info.VMC.Phi1) -  Control_Info->R_Leg_Info.VMC.Phi2);
   Control_Info->R_Leg_Info.SendValue.T1 = -(((Control_Info->L1 * arm_sin_f32(Phi1_2))*(Control_Info->R_Leg_Info.Moment.F 
	                                      *Control_Info->R_Leg_Info.VMC.L0 * arm_sin_f32(Phi0_3)+ 
	                                      Control_Info->R_Leg_Info.Moment.Tp * arm_cos_f32(Phi0_3)))
	                                    /(Control_Info->R_Leg_Info.VMC.L0*arm_sin_f32(Phi2_3)));
   Phi0_2 =  ( Control_Info->R_Leg_Info.VMC.Phi0 -  Control_Info->R_Leg_Info.VMC.Phi2);
   Phi3_4 =  ( Control_Info->R_Leg_Info.VMC.Phi3 - (*Control_Info->R_Leg_Info.VMC.Phi4));
  Control_Info->R_Leg_Info.SendValue.T2=  -(((Control_Info->L4 * arm_sin_f32(Phi3_4))*(Control_Info->R_Leg_Info.Moment.F  
	                                     *Control_Info->R_Leg_Info.VMC.L0 * arm_sin_f32(Phi0_2)+ 
	                                      Control_Info->R_Leg_Info.Moment.Tp  * arm_cos_f32(Phi0_2)))
	                                    /(Control_Info->R_Leg_Info.VMC.L0*arm_sin_f32(Phi2_3)));

  if(  Control_Info->Init.Balance_Init.IF_Balance_Init == 0   ){
	   Control_Info->L_Leg_Info.SendValue.T1 = 0;
		 Control_Info->L_Leg_Info.SendValue.T2 = 0;
		 Control_Info->R_Leg_Info.SendValue.T1 = 0;
		 Control_Info->R_Leg_Info.SendValue.T2 = 0;
  }
}
