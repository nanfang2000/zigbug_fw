#ifndef __ORGAN_H__
#define __ORGAN_H__
#include <stdint.h>
#include <stddef.h>
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

class Organ
{
public:
    Organ(void);
    virtual void init(void) const = 0;
    virtual void start(void) const = 0;
    virtual void stop(void) const = 0;
    //virtual void sleep(void)=0;
};
#endif
