#ifndef __MORTOR_H
#define __MORTOR_H
#include <stdint.h>

#ifdef  __cplusplus
extern "C" {
#endif

void motor_init(void);
void motor_start(int16_t left_speed, int16_t right_speed);
void motor_stop(void);

#ifdef  __cplusplus
}  
#endif

#endif
