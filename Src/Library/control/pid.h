#ifndef __PID_H__
#define __PID_H__
#include <stdint.h>

#ifdef  __cplusplus
extern "C" {
#endif

typedef struct
{
    //Constants
    float Kp;	//比例系数
    float Ki;	//积分系数
    float Kd;	//微分系数
    //PID control values
    float u;	//这次输出的值
    float u1;	//上次输出的值
    float ek;	//当次误差
    float ek1;	//上一次误差
    float ek2;	//上两次误差
}pid_t;

void pid_init(pid_t *pid, float Kp, float Ki, float Kd);

float pid_process(pid_t *pid, float target_value, float cur_value);
	
#ifdef  __cplusplus
}  
#endif

#endif //__PID_H__
