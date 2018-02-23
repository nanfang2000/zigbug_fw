#include "pid.h"
#include <string.h>
// #include "app_util_platform.h"
// #include "nrf_delay.h"
// #define NRF_LOG_MODULE_NAME "APP"
// #include "nrf_log.h"
// #include "nrf_log_ctrl.h"
// #include "app_error.h"

void pid_init(pid_t *pid, float Kp, float Ki, float Kd)
{
    memset(pid, 0, sizeof(pid_t));
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

float pid_process(pid_t *pid, float target_value, float cur_value)
{
    float pError = 0, iError = 0, dError = 0;
    pid->ek = target_value - cur_value;
    pError = pid->ek - pid->ek1;                //比例误差(等于当前误差减去前一次的误差)
    iError = pid->ek;                           //积分误差(等于当前误差值)
    dError = pid->ek - pid->ek1 * 2 + pid->ek2; //微分误差(等于当前误差减去前一次2倍误差再加上前两次的误差)

    pid->ek2 = pid->ek1; //储存前两次的误差值
    pid->ek1 = pid->ek;  //储存前一次的误差值

    pid->u = pid->Kp * pError + pid->Ki * iError + pid->Kd * dError + pid->u1; //获取PID调节的误差值
    pid->u1 = pid->u;  
    return pid->u;                                                        //储存前一次的输出值
}
	