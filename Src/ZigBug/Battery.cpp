#include "Battery.h"
#include "batt_meas.h"

Battery::Battery(int32_t interval_ms)
{
    m_interval_ms = interval_ms;
}

void Battery::init(void)
{
    batt_meas_init(NULL);
}

void Battery::start(void)
{
    batt_meas_enable(m_interval_ms);
}

void Battery::stop(void)
{
    batt_meas_disable();
}