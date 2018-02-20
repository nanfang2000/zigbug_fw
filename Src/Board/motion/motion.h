#ifndef __MOTION_H__
#define __MOTION_H__
#include <stdint.h>
#include "vector3.h"
#include "motion_data.h"

#ifdef  __cplusplus
extern "C" {
#endif

uint32_t motion_init(void);
void motion_start(void);
void motion_stop(void);

uint32_t motion_read_raw(motion_raw_t *data);

uint32_t motion_update(motion_raw_t *data);
motion_data_t* motion_get_data(void);

#ifdef  __cplusplus
}  
#endif

#endif
