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
//#define LOCAL_DEBUG
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
VL53L0X_Dev_t m_vl53l0x_dev_front = 
{
    .I2cDevAddr = 0x52 >> 1,  //0x29 in 7 bits
    .comms_type = COM_I2C,
    .comms_speed_khz = 400
};
struct MyDev_t m_vl6180_dev_rear = 
{
    .i2c_dev_addr = 0x52 >> 1,  //0x29 in 7 bits
};

struct MyDev_t m_vl6180_dev_body = 
{
    .i2c_dev_addr = 0x52 >> 1,  //0x29 in 7 bits
};

struct MyDev_t m_vl6180_dev_eye_left = 
{
    .i2c_dev_addr = 0x52 >> 1,  //0x29 in 7 bits
};

struct MyDev_t m_vl6180_dev_eye_right = 
{
    .i2c_dev_addr = 0x52 >> 1,  //0x29 in 7 bits
};

struct MyDev_t m_vl6180_dev_left = 
{
    .i2c_dev_addr = 0x52 >> 1,  //0x29 in 7 bits
};

struct MyDev_t m_vl6180_dev_right = 
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

int16_t vision_get_front_dist(void);
int16_t vision_get_vl6180x_dist(VL6180xDev_t dev);

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
    vTaskDelay(5);
}

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev) {
    VL53L0X_Error status = VL53L0X_ERROR_NONE;

    //nrf_delay_ms(2);
    vTaskDelay(5);
    return status;
}
void vision_vl_enable(uint32_t pin)
{
    nrf_drv_gpiote_out_clear(pin);
    nrf_delay_ms(5);
    nrf_drv_gpiote_out_set(pin);
    nrf_delay_ms(5);
}

void vision_vl_disable(uint32_t pin)
{
    nrf_drv_gpiote_out_clear(pin);
    nrf_delay_ms(5);
}

void vision_init_vl53l0x(VL53L0X_Dev_t* dev, uint8_t shdn_pin, uint8_t i2c_addr)
{
    uint32_t  refSpadCount;
    uint8_t   isApertureSpads;
    uint8_t   VhvSettings;
    uint8_t   PhaseCal;
    
    vision_vl_enable(shdn_pin);
    VL53L0X_Error error;
    VL53L0X_Version_t vl53l0x_version;
    error = VL53L0X_GetVersion(dev);
    DEBUG_PRINTF(0, "VL53L0X PAL Version:%d.%d.%d Rev %d\n", vl53l0x_version.major, vl53l0x_version.minor, vl53l0x_version.revision, vl53l0x_version.revision);
    
    /* Init VL53L0X */
    error = VL53L0X_DataInit(dev);
    if (error != VL53L0X_ERROR_NONE)
    {
        DEBUG_PRINTF(0, "VL53L0X_DataInit error:%d \n", error);
        return;
    }

    error = VL53L0X_StaticInit(dev);
    if (error != VL53L0X_ERROR_NONE)
    {
        DEBUG_PRINTF(0, "VL53L0X_StaticInit error:%d \n", error);
        return;
    }
    VL53L0X_PerformRefSpadManagement( dev, &refSpadCount, &isApertureSpads );
    VL53L0X_PerformRefCalibration( dev, &VhvSettings, &PhaseCal );
    VL53L0X_SetLimitCheckValue(dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 0.1*(1<<7));
    VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
    VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
    VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev,200);

    // VL53L0X_SetLimitCheckEnable( dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1 );
    // VL53L0X_SetLimitCheckEnable( dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1 );
    // VL53L0X_SetLimitCheckEnable( dev, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1 );
    // VL53L0X_SetLimitCheckValue( dev, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, (FixPoint1616_t)( 1.5 * 0.023 * 65536 ) );

    VL53L0X_SetDeviceAddress(dev, i2c_addr);
    dev->I2cDevAddr = i2c_addr>>1;
    DEBUG_PRINTF(0, "VL53L0X Init \n");
}

void vision_init_vl6180x(VL6180xDev_t dev, uint8_t shdn_pin, uint8_t i2c_addr)
{
    vision_vl_enable(shdn_pin);
    VL6180x_SetI2CAddress(dev, i2c_addr);
    dev->i2c_dev_addr = i2c_addr>>1;

    int32_t status = VL6180x_InitData(dev);
    if (status)
    {
        DEBUG_PRINTF(0, "VL6180x_InitData error:%d \n", status);
        // return;
    }
    status = VL6180x_Prepare(dev);
    if (status)
    {
        DEBUG_PRINTF(0, "VL6180x_Prepare error:%d \n", status);
        return;
    }

    // VL6180x_StaticInit(&m_vl6180_dev_rear);
    DEBUG_PRINTF(0, "VL6180x Init \n");
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
    nrf_drv_twi_enable(&m_i2c_instance);

    vision_init_vl53l0x(&m_vl53l0x_dev_front,       FRONT_RANGER_SHDN_PIN,      0x54);
    vision_init_vl6180x(&m_vl6180_dev_rear,         REAR_RANGER_SHDN_PIN,       0x56);
    vision_init_vl6180x(&m_vl6180_dev_body,         BODY_RANGER_SHDN_PIN,       0x58);
    vision_init_vl6180x(&m_vl6180_dev_eye_left,     LEFT_EYE_RANGER_SHDN_PIN,   0x5A);
    vision_init_vl6180x(&m_vl6180_dev_eye_right,    RIGHT_EYE_RANGER_SHDN_PIN,  0x5C);
    vision_init_vl6180x(&m_vl6180_dev_left,         LEFT_SIDE_RANGER_SHDN_PIN,  0x5E);
    vision_init_vl6180x(&m_vl6180_dev_right,        RIGHT_SIDE_RANGER_SHDN_PIN, 0x60);
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
    // error = VL53L0X_PerformSingleRangingMeasurement(&m_vl6180_dev_rear, &range_data);
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
    VL6180x_RangePollMeasurement(&m_vl6180_dev_rear, &range_data);
    if(range_data.errorStatus != 0)
    {
        DEBUG_PRINTF(0, "VL6180x_RangePollMeasurement error:%d-%s \n",range_data.errorStatus, VL6180x_RangeGetStatusErrString(range_data.errorStatus));
    }
    else
    {
        DEBUG_PRINTF(0, "VL6180x_RangePollMeasurement Range:%d \n", range_data.range_mm);
    }
//    VL6180x_AlsData_t als_data;
//    VL6180x_AlsPollMeasurement(m_vl6180_dev_rear, &als_data);
//    if(als_data.errorStatus != 0)
//    {
//        DEBUG_PRINTF(0, "VL6180x_AlsPollMeasurement error:%d \n", als_data.errorStatus);
//    }
//    else
//    {
//        DEBUG_PRINTF(0, "VL6180x_AlsPollMeasurement Lux:%d \n", als_data.lux);
//    }
}

void vision_start_measure(int32_t orientations)
{
    if(orientations & VISION_FRONT)
    {    
        VL53L0X_Error Status;
        Status = VL53L0X_ERROR_NONE;   
        /* This function will do a complete single ranging
         * Here we fix the mode! */
        Status = VL53L0X_SetDeviceMode(&m_vl53l0x_dev_front, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    
        if (Status == VL53L0X_ERROR_NONE)
            Status = VL53L0X_StartMeasurement(&m_vl53l0x_dev_front);
    }
        
    if(orientations & VISION_UPSIDE)
    {
        VL6180x_RangeStartSingleShot(&m_vl6180_dev_body);
    }

    if(orientations & VISION_LEFT_EYE)
    {
        VL6180x_RangeStartSingleShot(&m_vl6180_dev_eye_left);
    }

    if(orientations & VISION_RIGHT_EYE)
    {
        VL6180x_RangeStartSingleShot(&m_vl6180_dev_eye_right);
    }

    if(orientations & VISION_LEFT_SIDE)
    {
        VL6180x_RangeStartSingleShot(&m_vl6180_dev_left);
    }

    if(orientations & VISION_RIGHT_SIDE)
    {
        VL6180x_RangeStartSingleShot(&m_vl6180_dev_right);
    }

    if(orientations & VISION_BACK)
    {
        VL6180x_RangeStartSingleShot(&m_vl6180_dev_rear);
    }
}

void vision_get_measure(int32_t orientations, vision_t *vision)
{
    if(orientations & VISION_FRONT)
    {    
        vision->dist_front = vision_get_front_dist();
    }
        
    if(orientations & VISION_UPSIDE)
    {
        vision->dist_upside= vision_get_vl6180x_dist(&m_vl6180_dev_body);
    }

    if(orientations & VISION_LEFT_EYE)
    {
        vision->dist_left_eye= vision_get_vl6180x_dist(&m_vl6180_dev_eye_left);
    }

    if(orientations & VISION_RIGHT_EYE)
    {
        vision->dist_right_eye= vision_get_vl6180x_dist(&m_vl6180_dev_eye_right);
    }

    if(orientations & VISION_LEFT_SIDE)
    {
        vision->dist_left_side= vision_get_vl6180x_dist(&m_vl6180_dev_left);
    }

    if(orientations & VISION_RIGHT_SIDE)
    {
        vision->dist_right_side = vision_get_vl6180x_dist(&m_vl6180_dev_right);
    }

    if(orientations & VISION_BACK)
    {
        vision->dist_back= vision_get_vl6180x_dist(&m_vl6180_dev_rear);
    }
}

int16_t vision_get_front_dist(void)
{
    VL53L0X_RangingMeasurementData_t range_data;
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;       
    // error = VL53L0X_PerformSingleRangingMeasurement(&m_vl53l0x_dev_front, &range_data);

    //if (Status == VL53L0X_ERROR_NONE)
    //Status = VL53L0X_measurement_poll_for_completion(&m_vl53l0x_dev_front);

    /* Change PAL State in case of single ranging or single histogram */
    if (Status == VL53L0X_ERROR_NONE)
        PALDevDataSet((&m_vl53l0x_dev_front), PalState, VL53L0X_STATE_IDLE);

    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_GetRangingMeasurementData(&m_vl53l0x_dev_front, &range_data);

    if (Status == VL53L0X_ERROR_NONE)
        Status = VL53L0X_ClearInterruptMask(&m_vl53l0x_dev_front, 0);

    if (Status != VL53L0X_ERROR_NONE)
    {
        DEBUG_PRINTF(0, "VL53L0X_PerformSingleMeasurement error:%d \n", Status);
        return 2000;
    }
    else
    {
        if(range_data.RangeStatus != 0)
        {
            char errorstring[50];
            VL53L0X_GetRangeStatusString(range_data.RangeStatus, errorstring);
            DEBUG_PRINTF(0, "VL53L0X_PerformSingleMeasurement Range Error:%d-%s\n", range_data.RangeStatus, errorstring);
            return 2000;
        }
        else
        {
            DEBUG_PRINTF(0, "VL53L0X_PerformSingleMeasurement Range:%d \n", range_data.RangeMilliMeter);
            return range_data.RangeMilliMeter;            
        }
    }
}

int16_t vision_get_vl6180x_dist(VL6180xDev_t dev)
{
    int i = 10;
    VL6180x_RangeData_t range_data;
    //VL6180x_RangePollMeasurement(&m_vl6180_dev_rear, &range_data);
    //VL6180x_RangeGetMeasurementIfReady(dev, &range_data);
    //VL6180x_RangeGetMeasurementPoll(dev, &range_data);
    while(VL6180x_RangeGetMeasurementIfReady(dev, &range_data))
    {
        i--;
        vTaskDelay(2);
        if(i == 0)
        {
            break;
        }
    }

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
