#ifndef __VELOCITY_FUNSION_H
#define __VELOCITY_FUNSION_H

#include "kalman.h"

typedef struct 
{
   matrix r_matirx,accel_matrix;
   matrix res_accel;
   KalmanFilter_Info_TypeDef Funsion_kalman;
   float accel[3];
   float v;
}Velocity_Fusion_Typedef;



void Velocity_Fusion_Init(Velocity_Fusion_Typedef *p,float Q,float R,float dt);
void Velocity_Fusion_Update(Velocity_Fusion_Typedef *p,float q[4],float v,float a[3]);

#endif

