#include "vision.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_gpiote.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "app_error.h"
#include "vl53l0x_api.h"
#define LOCAL_DEBUG
#include "debug.h"

static nrf_drv_twi_t m_i2c_instance = NRF_DRV_TWI_INSTANCE(1);
static nrf_drv_twi_config_t m_i2c_config =
{                                                                              
    .frequency          = TWI_FREQUENCY_FREQUENCY_K400,   
    .scl                = 4,                                                  
    .sda                = 6,                                                  
    .interrupt_priority = TWI_DEFAULT_CONFIG_IRQ_PRIORITY,                     
    .clear_bus_init     = TWI_DEFAULT_CONFIG_CLR_BUS_INIT,                     
    .hold_bus_uninit    = TWI_DEFAULT_CONFIG_HOLD_BUS_UNINIT,                  
};
VL53L0X_Dev_t m_vl53l0x_dev = 
{
    .I2cDevAddr = 0x52 >> 1,  //0x29 in 7 bits
    .comms_type = COM_I2C,
    .comms_speed_khz = 400
};

#define RANGER_SHDN_PIN (10)


static void i2c_event_handler(uint32_t const *p_data_received,
                              uint32_t *p_data_to_send,
                              uint16_t number_of_words)
{
    
}

int32_t _I2CWrite(VL53L0X_DEV Dev, uint8_t *pdata, uint32_t count) 
{
    ret_code_t ret;

    ret = nrf_drv_twi_tx(&m_i2c_instance, Dev->I2cDevAddr, pdata, count, false);
    if (ret != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("I2C error 0x%x %d len", Dev->I2cDevAddr, count);
        DEBUG_PRINTF(0, "I2C error 0x%x %d len", Dev->I2cDevAddr, count);
        return ret;
    }
    return 0;
}

int32_t _I2CRead(VL53L0X_DEV Dev, uint8_t *pdata, uint32_t count) 
{
    ret_code_t ret;

    ret = nrf_drv_twi_rx(&m_i2c_instance, Dev->I2cDevAddr, pdata, count);
    if (ret != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("I2C error 0x%x %d len", Dev->I2cDevAddr, count);
        DEBUG_PRINTF(0, "I2C error 0x%x %d len", Dev->I2cDevAddr, count);
        return ret;
    }
    return 0;
}

void vision_init(void)
{
    /* If not initialized gpiote model, init is */
    if (!nrf_drv_gpiote_is_init())
    {
        APP_ERROR_CHECK(nrf_drv_gpiote_init());
    }

    nrf_drv_gpiote_out_config_t shdn_pin_config = GPIOTE_CONFIG_OUT_SIMPLE(true);

    APP_ERROR_CHECK(nrf_drv_gpiote_out_init(RANGER_SHDN_PIN, &shdn_pin_config));

    APP_ERROR_CHECK(nrf_drv_twi_init(&m_i2c_instance, &m_i2c_config, NULL, NULL));

    vision_enable();

    VL53L0X_Error error;
    VL53L0X_Version_t vl53l0x_version;
    error = VL53L0X_GetVersion(&vl53l0x_version);
    DEBUG_PRINTF(0, "VL53L0X PAL Version:%d.%d.%d Rev %d\n", vl53l0x_version.major, vl53l0x_version.minor, vl53l0x_version.revision, vl53l0x_version.revision);
    /* Init VL53L0X */
    error = VL53L0X_DataInit(&m_vl53l0x_dev);
    if (error != VL53L0X_ERROR_NONE)
    {
        DEBUG_PRINTF(0, "VL53L0X_DataInit error:%d \n", error);
        return;
    }

    error = VL53L0X_StaticInit(&m_vl53l0x_dev);
    if (error != VL53L0X_ERROR_NONE)
    {
        DEBUG_PRINTF(0, "VL53L0X_StaticInit error:%d \n", error);
        return;
    }

    // error = VL53L0X_SetDeviceMode(&m_vl53l0x_dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    // if (error != VL53L0X_ERROR_NONE)
    // {
    //     DEBUG_PRINTF(0, "VL53L0X_SetDeviceMode error:%d \n", error);
    //     return;
    // }
}

void vision_enable(void)
{
    nrf_drv_gpiote_out_set(RANGER_SHDN_PIN);
    nrf_drv_twi_enable(&m_i2c_instance);
}

void vision_disable(void)
{
    nrf_drv_gpiote_out_clear(RANGER_SHDN_PIN);
    nrf_drv_twi_disable(&m_i2c_instance);
}

void vision_start()
{
    VL53L0X_Error error;
    VL53L0X_RangingMeasurementData_t range_data;
    VL53L0X_PowerModes power_mode;
    VL53L0X_GetPowerMode(&m_vl53l0x_dev, &power_mode);
    DEBUG_PRINTF(0, "VL53L0X_GetPowerMode Mode:%d \n", power_mode);
    error = VL53L0X_PerformSingleRangingMeasurement(&m_vl53l0x_dev, &range_data);
    if (error != VL53L0X_ERROR_NONE)
    {
        DEBUG_PRINTF(0, "VL53L0X_PerformSingleMeasurement error:%d \n", error);
        return;
    }
    else
    {
        DEBUG_PRINTF(0, "VL53L0X_PerformSingleMeasurement Range:%d \n", range_data.RangeMilliMeter);
    }
}

void vision_stop(void)
{

}
