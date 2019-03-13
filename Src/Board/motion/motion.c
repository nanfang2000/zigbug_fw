#include "motion.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_gpiote.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "app_error.h"
#include "drv_motion.h"
#include "drv_mpu9250.h"
#define MOTION_DEBUG
#include "debug.h"
#include "FreeRTOS.h"
#include "task.h"
#include "inv_mpu.h"
#include "motion_processor.h"

#ifdef MOTION_DEBUG
    #define LOCAL_DEBUG
#endif

#define RAW_PARAM_NUM                 9     // Number of raw parameters (3 * acc + 3 * gyro + 3 * compass).
#define RAW_Q_FORMAT_ACC_INTEGER_BITS 6     // Number of bits used for integer part of raw data.
#define RAW_Q_FORMAT_GYR_INTEGER_BITS 11    // Number of bits used for integer part of raw data.
#define RAW_Q_FORMAT_CMP_INTEGER_BITS 12    // Number of bits used for integer part of raw data.

static nrf_drv_twi_t m_i2c_instance = NRF_DRV_TWI_INSTANCE(1);
#define MPU_TWI_SCL (4)
#define MPU_TWI_SDA (6)

static const nrf_drv_twi_config_t twi_config_mpu9250 =
{
    .scl                = MPU_TWI_SCL,
    .sda                = MPU_TWI_SDA,
    .frequency          = NRF_TWI_FREQ_400K,
    .interrupt_priority = APP_IRQ_PRIORITY_LOW
};
#if 0
// static const nrf_drv_twi_config_t twi_config_lis2dh12 =
// {
//     #if defined(THINGY_HW_v0_7_0)
//         .scl = TWI_SCL,
//         .sda = TWI_SDA,
//     #elif  defined(THINGY_HW_v0_8_0)
//         .scl = TWI_SCL,
//         .sda = TWI_SDA,
//     #elif  defined(THINGY_HW_v0_9_0)
//         .scl = TWI_SCL,
//         .sda = TWI_SDA,
//     #else
//         .scl = TWI_SCL_EXT,
//         .sda = TWI_SDA_EXT,
//     #endif
//     .frequency          = NRF_TWI_FREQ_400K,
//     .interrupt_priority = APP_IRQ_PRIORITY_LOW
// };

static void i2c_event_handler(uint32_t const *p_data_received,
                              uint32_t *p_data_to_send,
                              uint16_t number_of_words)
{
    
}

static void drv_motion_evt_handler(drv_motion_evt_t const * p_evt, void * p_data, uint32_t size)
{
    switch (*p_evt)
    {
        case DRV_MOTION_EVT_RAW:
        {
            APP_ERROR_CHECK_BOOL(size == sizeof(int32_t) * RAW_PARAM_NUM);

            motion_raw_t data;
            int32_t     * p_raw = (int32_t *)p_data;

            /* p_raw is in 16Q16 format. This is compressed for BLE transfer */

            // Set upper and lower overflow limits.
            static const int16_t overflow_limit_upper[RAW_PARAM_NUM] = {
                                                    (1 << (RAW_Q_FORMAT_ACC_INTEGER_BITS - 1)) - 1,
                                                    (1 << (RAW_Q_FORMAT_ACC_INTEGER_BITS - 1)) - 1,
                                                    (1 << (RAW_Q_FORMAT_ACC_INTEGER_BITS - 1)) - 1,
                                                    (1 << (RAW_Q_FORMAT_GYR_INTEGER_BITS - 1)) - 1,
                                                    (1 << (RAW_Q_FORMAT_GYR_INTEGER_BITS - 1)) - 1,
                                                    (1 << (RAW_Q_FORMAT_GYR_INTEGER_BITS - 1)) - 1,
                                                    (1 << (RAW_Q_FORMAT_CMP_INTEGER_BITS - 1)) - 1,
                                                    (1 << (RAW_Q_FORMAT_CMP_INTEGER_BITS - 1)) - 1,
                                                    (1 << (RAW_Q_FORMAT_CMP_INTEGER_BITS - 1)) - 1};

            static const int16_t overflow_limit_lower[RAW_PARAM_NUM] = {
                                                    -(1 << (RAW_Q_FORMAT_ACC_INTEGER_BITS - 1)),
                                                    -(1 << (RAW_Q_FORMAT_ACC_INTEGER_BITS - 1)),
                                                    -(1 << (RAW_Q_FORMAT_ACC_INTEGER_BITS - 1)),
                                                    -(1 << (RAW_Q_FORMAT_GYR_INTEGER_BITS - 1)),
                                                    -(1 << (RAW_Q_FORMAT_GYR_INTEGER_BITS - 1)),
                                                    -(1 << (RAW_Q_FORMAT_GYR_INTEGER_BITS - 1)),
                                                    -(1 << (RAW_Q_FORMAT_CMP_INTEGER_BITS - 1)),
                                                    -(1 << (RAW_Q_FORMAT_CMP_INTEGER_BITS - 1)),
                                                    -(1 << (RAW_Q_FORMAT_CMP_INTEGER_BITS - 1))};

            int16_t overflow_check;

            for (uint8_t i = 0; i < RAW_PARAM_NUM; i++)
            {
                overflow_check = p_raw[i] >> 16;    // Right shift 16 to remove decimal part.

                if (overflow_check >= overflow_limit_upper[i])
                {
                    DEBUG_PRINTF(0, RTT_CTRL_TEXT_BRIGHT_YELLOW"motion: p_raw[%d] over limit. Val: %d limit: %d \r\n"RTT_CTRL_RESET, i, overflow_check, overflow_limit_upper[i]);
                    p_raw[i] = overflow_limit_upper[i] << 16;
                }
                else if (overflow_check < overflow_limit_lower[i])
                {
                    DEBUG_PRINTF(0, RTT_CTRL_TEXT_BRIGHT_YELLOW"motion: p_raw[%d] below limit. Val: %d limit: %d \r\n"RTT_CTRL_RESET, i, overflow_check, overflow_limit_lower[i]);
                    p_raw[i] = overflow_limit_lower[i] << 16;
                }
                else
                {
                    // No overflow has occured.
                }
            }

            data.accel.x = (int16_t)(p_raw[0] >> RAW_Q_FORMAT_ACC_INTEGER_BITS);
            data.accel.y = (int16_t)(p_raw[1] >> RAW_Q_FORMAT_ACC_INTEGER_BITS);
            data.accel.z = (int16_t)(p_raw[2] >> RAW_Q_FORMAT_ACC_INTEGER_BITS);

            data.gyro.x = (int16_t)(p_raw[3] >> RAW_Q_FORMAT_GYR_INTEGER_BITS);
            data.gyro.y = (int16_t)(p_raw[4] >> RAW_Q_FORMAT_GYR_INTEGER_BITS);
            data.gyro.z = (int16_t)(p_raw[5] >> RAW_Q_FORMAT_GYR_INTEGER_BITS);

            data.compass.x = (int16_t)(p_raw[6] >> RAW_Q_FORMAT_CMP_INTEGER_BITS);
            data.compass.y = (int16_t)(p_raw[7] >> RAW_Q_FORMAT_CMP_INTEGER_BITS);
            data.compass.z = (int16_t)(p_raw[8] >> RAW_Q_FORMAT_CMP_INTEGER_BITS);

            #ifdef MOTION_DEBUG
                DEBUG_PRINTF(0, "DRV_MOTION_EVT_RAW:\r\n");

                double f_buf;
                char buffer[8];

                f_buf = (double)p_raw[0];
                f_buf = f_buf/(1<<16);
                sprintf(buffer, "%.2f", f_buf);
                DEBUG_PRINTF(0, " accel.x [G's] = %s:\r\n", buffer);

                f_buf = (double)p_raw[1];
                f_buf = f_buf/(1<<16);
                sprintf(buffer, "%.2f", f_buf);
                DEBUG_PRINTF(0, " accel.y [G's] = %s:\r\n", buffer);

                f_buf = (double)p_raw[2];
                f_buf = f_buf/(1<<16);
                sprintf(buffer, "%.2f", f_buf);
                DEBUG_PRINTF(0, " accel.z [G's] = %s:\r\n", buffer);


                f_buf = (double)p_raw[3];
                f_buf = f_buf/(1<<16);
                sprintf(buffer, "%.2f", f_buf);
                DEBUG_PRINTF(0, " gyro.x [deg/s] = %s:\r\n", buffer);

                f_buf = (double)p_raw[4];
                f_buf = f_buf/(1<<16);
                sprintf(buffer, "%.2f", f_buf);
                DEBUG_PRINTF(0, " gyro.y [deg/s] = %s:\r\n", buffer);

                f_buf = (double)p_raw[5];
                f_buf = f_buf/(1<<16);
                sprintf(buffer, "%.2f", f_buf);
                DEBUG_PRINTF(0, " gyro.z [deg/s] = %s:\r\n", buffer);


                f_buf = (double)p_raw[6];
                f_buf = f_buf/(1<<16);
                sprintf(buffer, "%.2f", f_buf);
                DEBUG_PRINTF(0, " mag.x [uT] = %s:\r\n", buffer);

                f_buf = (double)p_raw[7];
                f_buf = f_buf/(1<<16);
                sprintf(buffer, "%.2f", f_buf);
                DEBUG_PRINTF(0, " mag.y [uT] = %s:\r\n", buffer);

                f_buf = (double)p_raw[8];
                f_buf = f_buf/(1<<16);
                sprintf(buffer, "%.2f", f_buf);
                DEBUG_PRINTF(0, " mag.z [uT] = %s:\r\n", buffer);
            #endif

            //(void)motion_raw_set(&m_tms, &data);
        }
        break;

        case DRV_MOTION_EVT_QUAT:
        {
            APP_ERROR_CHECK_BOOL(size == sizeof(int32_t) * 4);

            motion_quat_t data;
            int32_t      * p_quat = (int32_t *)p_data;

            data.w = p_quat[0];
            data.x = p_quat[1];
            data.y = p_quat[2];
            data.z = p_quat[3];

            #ifdef MOTION_DEBUG
                static const uint8_t QUAT_ELEMENTS = 4;
                double f_buf;
                char buffer[QUAT_ELEMENTS][7];

                for (uint8_t i = 0; i < QUAT_ELEMENTS; i++)
                {
                    f_buf = (double)p_quat[i];
                    f_buf = f_buf/(1<<30);
                    sprintf(buffer[i], "% 1.3f", f_buf);
                }

                DEBUG_PRINTF(0, "DRV_MOTION_EVT_QUAT: \n w:%s x:%s y:%s z:%s\r\n", buffer[0], buffer[1], buffer[2], buffer[3]);
            #endif

            //(void)motion_quat_set(&m_tms, &data);
        }
        break;

        case DRV_MOTION_EVT_EULER:
        {
            APP_ERROR_CHECK_BOOL(size == sizeof(long) * 3);

            motion_euler_t data;
            int32_t      * p_euler = (int32_t *)p_data;

            data.roll   = p_euler[0];
            data.pitch  = p_euler[1];
            data.yaw    = p_euler[2];

            #ifdef MOTION_DEBUG
                DEBUG_PRINTF(0, "DRV_MOTION_EVT_EULER, [deg]:  roll(x):%3d   pitch(y):%3d   yaw(z):%3d  \r\n", data.roll/(1<<16), data.pitch/(1<<16), data.yaw/(1<<16));
            #endif

            //(void)motion_euler_set(&m_tms, &data);
        }
        break;

        case DRV_MOTION_EVT_ROT_MAT:
        {
            APP_ERROR_CHECK_BOOL(size == sizeof(int32_t) * 9);

            motion_rot_mat_t data;
            int32_t         * p_matrix = (int32_t *)p_data;

            data.matrix[0] = (int16_t)(p_matrix[0] >> 16);
            data.matrix[1] = (int16_t)(p_matrix[1] >> 16);
            data.matrix[2] = (int16_t)(p_matrix[2] >> 16);
            data.matrix[3] = (int16_t)(p_matrix[3] >> 16);
            data.matrix[4] = (int16_t)(p_matrix[4] >> 16);
            data.matrix[5] = (int16_t)(p_matrix[5] >> 16);
            data.matrix[6] = (int16_t)(p_matrix[6] >> 16);
            data.matrix[7] = (int16_t)(p_matrix[7] >> 16);
            data.matrix[8] = (int16_t)(p_matrix[8] >> 16);

            #ifdef MOTION_DEBUG
                static const uint8_t ROT_MAT_ELEMENTS = 9;
                char buffer[ROT_MAT_ELEMENTS][6];
                double tmp;
                for(uint8_t i = 0; i<ROT_MAT_ELEMENTS; i++)
                {
                    tmp = p_matrix[i]/(double)(1<<30);
                    sprintf(buffer[i], "% 1.2f", tmp);
                }
                    DEBUG_PRINTF(0, "DRV_MOTION_EVT_ROT_MAT:\r\n[%s %s %s]\r\n[%s %s %s]\r\n[%s %s %s]\r\n",
                                buffer[0],
                                buffer[1],
                                buffer[2],
                                buffer[3],
                                buffer[4],
                                buffer[5],
                                buffer[6],
                                buffer[7],
                                buffer[8]);
            #endif

            //(void)motion_rot_mat_set(&m_tms, &data);
        }
        break;

        case DRV_MOTION_EVT_HEADING:
        {
            APP_ERROR_CHECK_BOOL(size == sizeof(long));

            #ifdef MOTION_DEBUG
                int32_t heading = *(int32_t *)p_data;
                DEBUG_PRINTF(0, "DRV_MOTION_EVT_HEADING [deg]:  h: %d\r\n", heading/(1<<16));
            #endif

            //(void)motion_heading_set(&m_tms, (motion_heading_t *)p_data);
        }
        break;

        case DRV_MOTION_EVT_GRAVITY:
        {
            APP_ERROR_CHECK_BOOL(size == sizeof(float) * 3);

            motion_gravity_t data;
            float           * p_gravity = (float *)p_data;

            data.x = p_gravity[0];
            data.y = p_gravity[1];
            data.z = p_gravity[2];

            #ifdef MOTION_DEBUG
                static const uint8_t GRAVITY_ELEMENTS = 3;
                char buffer[GRAVITY_ELEMENTS][8];

                for (uint8_t i = 0; i<GRAVITY_ELEMENTS; i++)
                {
                    sprintf(buffer[i], "% 2.3f", p_gravity[i]);
                }

                DEBUG_PRINTF(0, "DRV_MOTION_EVT_GRAVITY [m/s^2]:  [%s, %s, %s]\r\n", buffer[0], buffer[1], buffer[2]);
            #endif

            //(void)motion_gravity_set(&m_tms, &data);
        }
        break;

        case DRV_MOTION_EVT_TAP:
        {
            APP_ERROR_CHECK_BOOL(size == 2);

            motion_tap_t data;
            uint8_t * p_tap = (uint8_t *)p_data;

            data.dir = p_tap[0];
            data.cnt = p_tap[1];

            #ifdef MOTION_DEBUG
                DEBUG_PRINTF(0, "%sDRV_MOTION_EVT_TAP:%s [%d %d]\r\n", RTT_CTRL_TEXT_BRIGHT_YELLOW,
                                                                       RTT_CTRL_RESET,
                                                                       data.cnt,
                                                                       data.dir);
            #endif

            //(void)motion_tap_set(&m_tms, &data);
        }
        break;

        case DRV_MOTION_EVT_ORIENTATION:
        {
            APP_ERROR_CHECK_BOOL(size == sizeof(uint8_t));

            #ifdef MOTION_DEBUG
                DEBUG_PRINTF(0, "%sDRV_MOTION_EVT_ORIENTATION:%s %d\r\n", RTT_CTRL_TEXT_BRIGHT_YELLOW,
                                                                          RTT_CTRL_RESET,
                                                                          *(motion_orientation_t *)p_data);
            #endif

            //(void)motion_orientation_set(&m_tms, (motion_orientation_t *)p_data);
        }
        break;

        case DRV_MOTION_EVT_PEDOMETER:
        {
            APP_ERROR_CHECK_BOOL(size == sizeof(unsigned long) * 2);

            motion_pedo_t  data;
            unsigned long * p_pedo = (unsigned long *)p_data;

            data.steps = p_pedo[0];
            data.time_ms  = p_pedo[1];

            #ifdef MOTION_DEBUG
                DEBUG_PRINTF(0, "%sDRV_MOTION_EVT_PEDOMETER:%s %d steps %d ms\r\n", RTT_CTRL_TEXT_BRIGHT_YELLOW,
                                                                                   RTT_CTRL_RESET,
                                                                                   p_pedo[0],
                                                                                   p_pedo[1]);
            #endif

            //(void)motion_pedo_set(&m_tms, &data);
        }
        break;

        default:
            DEBUG_PRINTF(0, "m_motion: drv_motion_evt_handler: Unknown data!\r\n");
            break;
    }
}

uint32_t motion_init(void)
{
    uint32_t err_code;
    drv_motion_twi_init_t motion_params_mpu9250;
    drv_motion_twi_init_t motion_params_lis2dh12;

    //NULL_PARAM_CHECK(p_handle);
    //NULL_PARAM_CHECK(p_params);

    DEBUG_PRINTF(0, "m_motion_init: \r\n");

    //p_handle->ble_evt_cb = motion_on_ble_evt;
    //p_handle->init_cb    = motion_service_init;

    motion_params_mpu9250.p_twi_instance = &m_i2c_instance;
    motion_params_mpu9250.p_twi_cfg      = &twi_config_mpu9250;

    // motion_params_lis2dh12.p_twi_instance = p_params->p_twi_instance;
    // motion_params_lis2dh12.p_twi_cfg      = &twi_config_lis2dh12;

    err_code = drv_motion_init(drv_motion_evt_handler, &motion_params_mpu9250, &motion_params_lis2dh12);
    RETURN_IF_ERROR(err_code);
    drv_motion_enable(DRV_MOTION_FEATURE_DMP_MASK);

    return NRF_SUCCESS;
}

#endif

uint32_t motion_init(void)
{
    uint32_t err_code;
    drv_mpu9250_init_t motion_params_mpu9250;

    DEBUG_PRINTF(0, "m_motion_init: \r\n");
    motion_params_mpu9250.p_twi_instance = &m_i2c_instance;
    motion_params_mpu9250.p_twi_cfg      = &twi_config_mpu9250;

    err_code = drv_mpu9250_init(&motion_params_mpu9250);
    RETURN_IF_ERROR(err_code);

    mpu_init(NULL);
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL/* | INV_XYZ_COMPASS*/);
    mpu_set_sample_rate(100);
    //mpu_set_compass_sample_rate(100);

    motion_processor_config_t config = {.DOF69 = DOF6, .integrateRotation = false};
    motion_processor_init(&config);

    return NRF_SUCCESS;
}

void motion_reset(void)
{
    motion_processor_reset();
}

uint32_t motion_read_raw(motion_raw_t *data)
{
    short tmp[3] = {0};
    unsigned long timestamp = 0;
    unsigned char fsr = 0;
    long temp = 0;

    mpu_get_accel_fsr(&fsr);
    mpu_get_accel_reg(tmp, &timestamp);
    data->accel.x = tmp[0]*fsr/32768.f;
    data->accel.y = tmp[1]*fsr/32768.f;
    data->accel.z = tmp[2]*fsr/32768.f;

    mpu_get_gyro_fsr(&fsr);
    mpu_get_gyro_reg(tmp, &timestamp);
    data->gyro.x = tmp[0]*fsr/32768.f;
    data->gyro.y = tmp[1]*fsr/32768.f;
    data->gyro.z = tmp[2]*fsr/32768.f;

    mpu_get_compass_fsr(&fsr);
    mpu_get_compass_reg(&tmp, &timestamp);
    data->compass.x = tmp[0]*fsr/32768.f;
    data->compass.y = tmp[1]*fsr/32768.f;
    data->compass.z = tmp[2]*fsr/32768.f;

    mpu_get_temperature(&temp, &timestamp);
    data->temp = temp/65536.f;

    data->timestamp = timestamp/1000.f;
    return 0;
}

uint32_t motion_update(motion_raw_t *data)
{
    motion_processor_process(data);
    return 0;
}

motion_data_t* motion_get_data(void)
{
    return motion_processor_get_data();
}
