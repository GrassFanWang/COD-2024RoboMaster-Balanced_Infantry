#include "Kalman_Mix.h"

/* ��ǰ-�ͺ󲹳��� */
/* ��ǰ�ͺ󲹳�����C++ʵ�� */
/*csdn : https://blog.csdn.net/zmhzmhzm/article/details/106228738*/

//��ʼ��������
void lag_comp_init(Lead_Lag_t *lag_comp,float T1,float T2,float delta_t)
{
	lag_comp->T1 = T1;
	lag_comp->T2 = T2;
	lag_comp->dT = delta_t;
	lag_comp->x = 0;
	lag_comp->y = 0;
}

//���������ݸ���
float lag_comp_update(Lead_Lag_t *lag_comp,float error)
{  
	float out;
	lag_comp->x = (1 - lag_comp->dT / lag_comp->T2) * lag_comp->x
					+ (lag_comp->dT / lag_comp->T2) * error; //error��ֵ���������ڸ�������    
	lag_comp->y = (1 - lag_comp->T1 / lag_comp->T2) * lag_comp->x
					+ (lag_comp->T1 / lag_comp->T2) * error;
	out = lag_comp->y/(lag_comp->T1/lag_comp->T2);
	return out;
}

//��ʼ�����׿������۲���
void kalman_second_order_init(kalman2_state *state)
{
	  state->x[0]    = 0;
	  state->x[1]    = 0;
    state->p[0][0] = 1;
    state->p[0][1] = 0;
    state->p[1][0] = 0;
    state->p[1][1] = 1;
//	  state->A       = {{1, 0.002}, {0, 1}};
      state->A[0][0] = 1;
      state->A[0][1] = 0.002;	//����dt ԭ����2ms ��ΪLQR����û���û���3ms����������ٶȶ�һ��� �����ٶȻ���һ�����
      state->A[1][0] = 0;
      state->A[1][1] = 1;
//	  state->H       = {1,0};
      state->H[0][0] = 1;
      state->H[0][1] = 0;//1
	  state->H[1][0] = 0;
	  state->H[1][1] = 0;
//    state->q       = {{10e-6,0}, {0,10e-6}};  /* measure noise convariance */		
	    state->q[0]    = 0.01f;
      state->q[1]    = 0.01f;
//    state->r									/* estimated error convariance */
    state->r[0][0] = 5.0f;	//�����Ƕԡ�Ԥ�⡱��Ȩ�� ���������˵����������챵�Ԥ��
	  state->r[0][1] = 0;
	  state->r[1][0] = 0;
	  state->r[1][1] = 1.0f;
//    state->B		
	  state->B[0]    = state->A[0][1]* state->A[0][1]/2.0;
	  state->B[1]    = state->A[0][1];
}

//���¶��׿������۲�������
float kalman_second_order_update(kalman2_state *state, float x_weiyi,float x_speed,float a)
{
	float temp0 = 0.0f;
	float temp1 = 0.0f;
	float temp0_0 = 0.0f;
	float temp0_1 = 0.0f;
	float temp1_0 = 0.0f;
	float temp1_1 = 0.0f;
	float temp00 = 0.0f;
	float temp01 = 0.0f;
	float temp10 = 0.0f;
	float temp11 = 0.0f;

/* Step1: Predict X(k+1)= A*X(k) +B*U(k)*/
	state->x[0] = state->A[0][0] * state->x[0] + state->A[0][1] * state->x[1]+state->B[0]*a;
	state->x[1] = state->A[1][0] * state->x[0] + state->A[1][1] * state->x[1]+state->B[1]*a;
/* Step2: Covariance Predict P(k+1)=A*P(k)*(A^T)+Q;*/
	state->p[0][0] = (state->p[0][0]+state->p[1][0]*state->A[0][1])+(state->p[0][1]+state->p[1][1]*state->A[0][1])*state->A[0][1]+state->q[0];
	state->p[0][1] = state->p[0][1]+state->p[1][1]*state->A[0][1];//+state->q[0];
	state->p[1][0] = state->p[1][0]+state->p[1][1]*state->A[0][1];//+state->q[1];
	state->p[1][1] = state->p[1][1]+state->q[1];
/* Step3: Gain Measurement : gain = p * H^T * [r + H * p * H^T]^(-1), H^T means transpose.*/
	temp0_0  = (state->p[0][0] +state->r[0][0])*(state->p[1][1] +state->r[1][1])-(state->p[0][1] +state->r[0][1])*(state->p[1][0] +state->r[1][0]);
	temp0_1  = (state->p[0][0] +state->r[0][0])*(state->p[1][1] +state->r[1][1])-(state->p[0][1] +state->r[0][1])*(state->p[1][0] +state->r[1][0]);
	temp1_0  = (state->p[0][0] +state->r[0][0])*(state->p[1][1] +state->r[1][1])-(state->p[0][1] +state->r[0][1])*(state->p[1][0] +state->r[1][0]);
	temp1_1  = (state->p[0][0] +state->r[0][0])*(state->p[1][1] +state->r[1][1])-(state->p[0][1] +state->r[0][1])*(state->p[1][0] +state->r[1][0]);		
	temp00  =   state->p[1][1] / temp0_0;
	temp01  =  -state->p[0][1] / temp0_1;
	temp10  =  -state->p[1][0] / temp1_0;
	temp11  =   state->p[0][0] / temp1_1;	
	state->gain[0][0] = state->p[0][0]* temp00 + state->p[0][1]* temp10;
	state->gain[0][1] = state->p[0][0]* temp01 + state->p[0][1]* temp11;
	state->gain[1][0] = state->p[1][0]* temp00 + state->p[1][1]* temp10;
	state->gain[1][1] = state->p[1][0]* temp01 + state->p[1][1]* temp11;
/* Step4: Status Update : x(n|n) = x(n|n-1) + gain(n) * [z_measure - H(n)*x(n|n-1)]*/
	state->x[0] = state->x[0] + state->gain[0][0] * (x_weiyi - state->x[0])
					+ state->gain[0][1] * (x_speed - state->x[1]);
	state->x[1] = state->x[1] + state->gain[1][0] * (x_weiyi - state->x[0])
					+ state->gain[1][1] * (x_speed - state->x[1]); 
/* Step5: Covariance Update p: p(n|n) = [I - gain * H] * p(n|n-1) */
	temp0=state->p[0][0];
	temp1=state->p[0][1];
	state->p[0][0] = (1 - state->gain[0][0] ) * state->p[0][0]-(state->gain[0][1]* state->p[1][0]);
	state->p[0][1] = (1 - state->gain[0][0] ) * state->p[0][1]-(state->gain[0][1]* state->p[1][1]);
	state->p[1][0] = (1 - state->gain[1][1] ) * state->p[1][0]-state->gain[1][0]* temp0;//state->p[0][0]
	state->p[1][1] = (1 - state->gain[1][1] ) * state->p[1][1]-state->gain[1][0]* temp1;//state->p[0][1]
	
	return state->x[1];
}
