#ifndef MOTION_PROCESSOR_H
#define MOTION_PROCESSOR_H

#include "motion_data.h"

typedef enum
{
    DOF9,
    DOF6
}Fusion;

typedef struct
{
    Fusion DOF69;
    int integrateRotation;
}motion_processor_config_t;
    
void motion_processor_init(motion_processor_config_t *);

void motion_processor_reset();

void motion_processor_initBaseline();

void motion_processor_process(motion_raw_t* raw);
motion_data_t* motion_processor_get_data(void);

#endif // MOTION_PROCESSOR_H
