#include "Velocity_Fusion.h"
#include "config.h"

void Velocity_Fusion_Init(Velocity_Fusion_Typedef *p,float Q,float R,float dt)
{
    Kalman_Filter_Init(&p->Funsion_kalman,1,1,1);

    p->Funsion_kalman.Data.A[0]=1.f;
    p->Funsion_kalman.Data.B[0]=dt;
    p->Funsion_kalman.Data.H[0]=1.f;
    p->Funsion_kalman.Data.Q[0]=Q;
    p->Funsion_kalman.Data.R[0]=R;

    p->r_matirx.numRows = 3;
    p->r_matirx.numCols = 3;
    p->r_matirx.pData=(float *)user_malloc(p->Funsion_kalman.sizeof_float * p->r_matirx.numRows * p->r_matirx.numCols);
    Matrix_Init(&p->r_matirx,p->r_matirx.numRows,p->r_matirx.numCols,p->r_matirx.pData);

    p->accel_matrix.numRows = 3;
    p->accel_matrix.numCols = 1;
    p->accel_matrix.pData=(float *)user_malloc(p->Funsion_kalman.sizeof_float * p->accel_matrix.numRows * p->accel_matrix.numCols);
    Matrix_Init(&p->accel_matrix,p->accel_matrix.numRows,p->accel_matrix.numCols,p->accel_matrix.pData);
    
    Matrix_Init(&p->res_accel,p->accel_matrix.numRows,p->accel_matrix.numCols,p->accel);
}

void Velocity_Fusion_Update(Velocity_Fusion_Typedef *p,float q[4],float v,float a[3])
{
    p->r_matirx.pData[0] = 1-2.f*q[2]*q[2]-2.f*q[3]*q[3];
    p->r_matirx.pData[1] = 2.f*q[1]*q[2]-2.f*q[0]*q[3];
    p->r_matirx.pData[2] = 2.f*q[1]*q[3]+2.f*q[0]*q[2];

    p->r_matirx.pData[3] = 2.f*q[1]*q[2]+2.f*q[0]*q[3];
    p->r_matirx.pData[4] = 1-2.f*q[1]*q[1]-2.f*q[3]*q[3];
    p->r_matirx.pData[5] = 2.f*q[2]*q[3]-2.f*q[0]*q[1];

    p->r_matirx.pData[6] = 2.f*q[1]*q[3]-2.f*q[0]*q[2];
    p->r_matirx.pData[7] = 2.f*q[2]*q[3]+2.f*q[0]*q[1];
    p->r_matirx.pData[8] = 1-2.f*q[1]*q[1]+2.f*q[2]*q[2];

    p->accel_matrix.pData[0] = a[0];
    p->accel_matrix.pData[1] = a[1];
    p->accel_matrix.pData[2] = a[2];

    Matrix_Multiply(&p->r_matirx,&p->accel_matrix,&p->res_accel);

    p->Funsion_kalman.MeasuredVector[0]=v;	
    p->Funsion_kalman.ControlVector[0]=p->accel[1];

    Kalman_Filter_Update(&p->Funsion_kalman);
    p->v = p->Funsion_kalman.Data.xhat[0];
}
