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
#include "vl6180x_api.h"
#define LOCAL_DEBUG
#include "debug.h"
#include "FreeRTOS.h"
#include "task.h"

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
struct MyDev_t myDev = 
{
    .i2c_dev_addr = 0x52 >> 1,  //0x29 in 7 bits
};

#define FRONT_RANGER_SHDN_PIN       (10)
#define BODY_RANGER_SHDN_PIN        (17)
#define LEFT_EYE_RANGER_SHDN_PIN    (1)
#define RIGHT_EYE_RANGER_SHDN_PIN   (31)
#define REAR_RANGER_SHDN_PIN        (18)
#define LEFT_SIDE_RANGER_SHDN_PIN   (9)
#define RIGHT_SIDE_RANGER_SHDN_PIN  (5)

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
        NRF_LOG_ERROR("I2C error 0x%x %d len\n", Dev->I2cDevAddr, count);
        DEBUG_PRINTF(0, "I2C error 0x%x %d len\n", Dev->I2cDevAddr, count);
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
        NRF_LOG_ERROR("I2C error 0x%x %d len\n", Dev->I2cDevAddr, count);
        DEBUG_PRINTF(0, "I2C error 0x%x %d len\n", Dev->I2cDevAddr, count);
        return ret;
    }
    return 0;
}

int32_t VL6180x_I2CWrite(VL6180xDev_t Dev, uint8_t *pdata, uint8_t count) 
{
    ret_code_t ret;

    ret = nrf_drv_twi_tx(&m_i2c_instance, Dev->i2c_dev_addr, pdata, count, true);
    if (ret != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("I2C error 0x%x %d len\n", Dev->i2c_dev_addr, count);
        DEBUG_PRINTF(0, "I2C error 0x%x %d len\n", Dev->i2c_dev_addr, count);
        return ret;
    }
    return 0;
}

int32_t VL6180x_I2CRead(VL6180xDev_t Dev, uint8_t *pdata, uint8_t count) 
{
    ret_code_t ret;

    ret = nrf_drv_twi_rx(&m_i2c_instance, Dev->i2c_dev_addr, pdata, count);
    if (ret != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("I2C error 0x%x %d len\n", Dev->i2c_dev_addr, count);
        DEBUG_PRINTF(0, "I2C error 0x%x %d len\n", Dev->i2c_dev_addr, count);
        return ret;
    }
    return 0;
}

void VL6180x_PollDelay(VL6180xDev_t dev)
{
    vTaskDelay(10);
}
void vision_init(void)
{
    uint32_t  refSpadCount;
    uint8_t   isApertureSpads;
    uint8_t   VhvSettings;
    uint8_t   PhaseCal;
    
    /* If not initialized gpiote model, init is */
    if (!nrf_drv_gpiote_is_init())
    {
        APP_ERROR_CHECK(nrf_drv_gpiote_init());
    }

    nrf_drv_gpiote_out_config_t shdn_pin_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

    APP_ERROR_CHECK(nrf_drv_gpiote_out_init(FRONT_RANGER_SHDN_PIN, &shdn_pin_config));
    APP_ERROR_CHECK(nrf_drv_gpiote_out_init(BODY_RANGER_SHDN_PIN, &shdn_pin_config));
    APP_ERROR_CHECK(nrf_drv_gpiote_out_init(LEFT_EYE_RANGER_SHDN_PIN, &shdn_pin_config));
    APP_ERROR_CHECK(nrf_drv_gpiote_out_init(RIGHT_EYE_RANGER_SHDN_PIN, &shdn_pin_config));
    APP_ERROR_CHECK(nrf_drv_gpiote_out_init(REAR_RANGER_SHDN_PIN, &shdn_pin_config));
    APP_ERROR_CHECK(nrf_drv_gpiote_out_init(LEFT_SIDE_RANGER_SHDN_PIN, &shdn_pin_config));
    APP_ERROR_CHECK(nrf_drv_gpiote_out_init(RIGHT_SIDE_RANGER_SHDN_PIN, &shdn_pin_config));

    APP_ERROR_CHECK(nrf_drv_twi_init(&m_i2c_instance, &m_i2c_config, NULL, NULL));

    vision_enable();
    nrf_delay_ms(200);

    // VL53L0X_Error error;
    // VL53L0X_Version_t vl53l0x_version;
    // error = VL53L0X_GetVersion(&vl53l0x_version);
    // DEBUG_PRINTF(0, "VL53L0X PAL Version:%d.%d.%d Rev %d\n", vl53l0x_version.major, vl53l0x_version.minor, vl53l0x_version.revision, vl53l0x_version.revision);
    // /* Init VL53L0X */
    // error = VL53L0X_DataInit(&m_vl53l0x_dev);
    // if (error != VL53L0X_ERROR_NONE)
    // {
    //     DEBUG_PRINTF(0, "VL53L0X_DataInit error:%d \n", error);
    //     return;
    // }


    // error = VL53L0X_StaticInit(&m_vl53l0x_dev);
    // if (error != VL53L0X_ERROR_NONE)
    // {
    //     DEBUG_PRINTF(0, "VL53L0X_StaticInit error:%d \n", error);
    //     return;
    // }
    // VL53L0X_PerformRefSpadManagement( &m_vl53l0x_dev, &refSpadCount, &isApertureSpads );
    // VL53L0X_PerformRefCalibration( &m_vl53l0x_dev, &VhvSettings, &PhaseCal );
    // VL53L0X_SetLimitCheckValue(&m_vl53l0x_dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 0.1*(1<<7));
    // VL53L0X_SetVcselPulsePeriod(&m_vl53l0x_dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
    // VL53L0X_SetVcselPulsePeriod(&m_vl53l0x_dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
    // VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&m_vl53l0x_dev,200);

    // VL53L0X_SetLimitCheckEnable( &m_vl53l0x_dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1 );
    // VL53L0X_SetLimitCheckEnable( &m_vl53l0x_dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1 );
    // VL53L0X_SetLimitCheckEnable( &m_vl53l0x_dev, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1 );
    // VL53L0X_SetLimitCheckValue( &m_vl53l0x_dev, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, (FixPoint1616_t)( 1.5 * 0.023 * 65536 ) );
    // nrf_drv_gpiote_out_clear(BODY_RANGER_SHDN_PIN);
    // nrf_delay_ms(50);
    // nrf_drv_gpiote_out_set(BODY_RANGER_SHDN_PIN);
    // nrf_delay_ms(50);
    //VL53L0X_ResetDevice(&m_vl53l0x_dev);
    // error = VL53L0X_SetDeviceMode(&m_vl53l0x_dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    // if (error != VL53L0X_ERROR_NONE)
    // {
    //     DEBUG_PRINTF(0, "VL53L0X_SetDeviceMode error:%d \n", error);
    //     return;
    // }
    //VL6180x_SetOffsetCalibrationData(myDev, 0);
    VL6180x_SetI2CAddress(&myDev, 0x54);
    myDev.i2c_dev_addr = 0x54>>1;
    int32_t status = VL6180x_InitData(&myDev);
    if (status)
    {
        DEBUG_PRINTF(0, "VL6180x_InitData error:%d \n", status);
        // return;
    }
    status = VL6180x_Prepare(&myDev);
    if (status)
    {
        DEBUG_PRINTF(0, "VL6180x_Prepare error:%d \n", status);
        return;
    }

    // VL6180x_StaticInit(&myDev);
}

void vision_enable(void)
{
    nrf_drv_gpiote_out_clear(REAR_RANGER_SHDN_PIN);
    nrf_delay_ms(50);
    nrf_drv_gpiote_out_set(REAR_RANGER_SHDN_PIN);
    nrf_delay_ms(50);
    nrf_drv_twi_enable(&m_i2c_instance);
}

void vision_disable(void)
{
    nrf_drv_gpiote_out_clear(REAR_RANGER_SHDN_PIN);
    nrf_drv_twi_disable(&m_i2c_instance);
}

void vision_start()
{
    // VL53L0X_Error error;
    // VL53L0X_RangingMeasurementData_t range_data;
    // error = VL53L0X_PerformSingleRangingMeasurement(&m_vl53l0x_dev, &range_data);
    // if (error != VL53L0X_ERROR_NONE)
    // {
    //     DEBUG_PRINTF(0, "VL53L0X_PerformSingleMeasurement error:%d \n", error);
    //     return;
    // }
    // else
    // {
    //     if(range_data.RangeStatus != 0)
    //     {
    //     char errorstring[50];
    //     VL53L0X_GetRangeStatusString(range_data.RangeStatus, errorstring);
    //       DEBUG_PRINTF(0, "VL53L0X_PerformSingleMeasurement Range Error:%d-%s\n", range_data.RangeStatus, errorstring);
    //       return;
    //     }
    //     DEBUG_PRINTF(0, "VL53L0X_PerformSingleMeasurement Range:%d \n", range_data.RangeMilliMeter);
    // }
    VL6180x_RangeData_t range_data;
    VL6180x_RangePollMeasurement(&myDev, &range_data);
    if(range_data.errorStatus != 0)
    {
        DEBUG_PRINTF(0, "VL6180x_RangePollMeasurement error:%d-%s \n",range_data.errorStatus, VL6180x_RangeGetStatusErrString(range_data.errorStatus));
    }
    else
    {
        DEBUG_PRINTF(0, "VL6180x_RangePollMeasurement Range:%d \n", range_data.range_mm);
    }
//    VL6180x_AlsData_t als_data;
//    VL6180x_AlsPollMeasurement(myDev, &als_data);
//    if(als_data.errorStatus != 0)
//    {
//        DEBUG_PRINTF(0, "VL6180x_AlsPollMeasurement error:%d \n", als_data.errorStatus);
//    }
//    else
//    {
//        DEBUG_PRINTF(0, "VL6180x_AlsPollMeasurement Lux:%d \n", als_data.lux);
//    }
}

int16_t vision_get_rear_dist(void)
{
    VL6180x_RangeData_t range_data;
    VL6180x_RangePollMeasurement(&myDev, &range_data);
    if(range_data.errorStatus != 0)
    {
        DEBUG_PRINTF(0, "VL6180x_RangePollMeasurement error:%d-%s \n",range_data.errorStatus, VL6180x_RangeGetStatusErrString(range_data.errorStatus));
        return 1000;
    }
    else
    {
        DEBUG_PRINTF(0, "VL6180x_RangePollMeasurement Range:%d \n", range_data.range_mm);
        return range_data.range_mm;
    }
}

void vision_stop(void)
{

}
