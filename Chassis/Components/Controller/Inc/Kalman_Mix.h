#ifndef _KALMAN_MIX_H_
#define _KALMAN_MIX_H_

#include "config.h"

typedef struct {
    float T1; //上一时刻的最优结果  X(k-|k-1)
    float T2;  //当前时刻的预测结果  X(k|k-1)
    float dT;  //当前时刻的最优结果  X(k|k)
	float x;
	float y;
}Lead_Lag_t;

typedef struct {
      float x[2];     /* state: [0]-angle [1]-diffrence of angle, 2x1 */
      float A[2][2];  /* X(n)=A*X(n-1)+U(n),U(n)~N(0,q), 2x2 */
      float H[2][2];     /* Z(n)=H*X(n)+W(n),W(n)~N(0,r), 1x2   */
      float q[2];     /* process(predict) noise convariance,2x1 [q0,0; 0,q1] */
      float r[2][2];        /* measure noise convariance */
      float p[2][2];  /* estimated error convariance,2x2 [p0 p1; p2 p3] */
      float gain[2][2];  /* 2x1 */
	    float B[2];
} kalman2_state;   

void lag_comp_init(Lead_Lag_t *Lead_Lag,float T1,float T2,float delta_t);
float lag_comp_update(Lead_Lag_t *Lead_Lag,float error);
extern void kalman_second_order_init(kalman2_state *state);
extern float kalman_second_order_update(kalman2_state *state, float x_weiyi,float x_speed,float a);

#endif



