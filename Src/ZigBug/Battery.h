#ifndef __BATTERY_H__
#define __BATTERY_H__
#include "Organ.h"

class Battery:public Organ
{
public:
    Battery(int32_t interval_ms);
    virtual void init(void);
    virtual void start(void);
    virtual void stop(void);
    //virtual void sleep(void)=0;
private:
    int32_t m_interval_ms;
};
#endif