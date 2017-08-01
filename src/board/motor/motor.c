#include "motor.h"
#include "app_util_platform.h"
#include "nrf_drv_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "app_error.h"
#include "nrf_drv_pwm.h"

#define TC214B_LEFT_IA  10 // Motor LEFT Input A --> MOTOR LEFT OA+
#define TC214B_LEFT_IB  11 // Motor LEFT Input B --> MOTOR LEFT OB-
#define TC214B_RIGHT_IA 10 // Motor RIGHT Input A --> MOTOR RIGHT OA+
#define TC214B_RIGHT_IB 11 // Motor RIGHT Input B --> MOTOR RIGHT OB-

#define CONFIG_MOTOR_LEFT_PWM			(TC214B_LEFT_IA)
#define CONFIG_MOTOR_LEFT_DIR			(TC214B_LEFT_IB)
#define CONFIG_MOTOR_RIGHT_PWM			(TC214B_RIGHT_IA)
#define CONFIG_MOTOR_RIGHT_DIR			(TC214B_RIGHT_IB)

const nrf_drv_pwm_t m_pwm_instance_left = NRF_DRV_PWM_INSTANCE(0);
const nrf_drv_pwm_t m_pwm_instance_right = NRF_DRV_PWM_INSTANCE(1);

const nrf_drv_pwm_config_t m_pwm_left_config = 
{                                                                             \
    .output_pins  = {CONFIG_MOTOR_LEFT_PWM,                                   \
                     NRF_DRV_PWM_PIN_NOT_USED,                                \
                     NRF_DRV_PWM_PIN_NOT_USED,                                \
                     NRF_DRV_PWM_PIN_NOT_USED },                              \
    .irq_priority = PWM_DEFAULT_CONFIG_IRQ_PRIORITY,                          \
    .base_clock   = (nrf_pwm_clk_t)PWM_DEFAULT_CONFIG_BASE_CLOCK,             \
    .count_mode   = (nrf_pwm_mode_t)PWM_DEFAULT_CONFIG_COUNT_MODE,            \
    .top_value    = PWM_DEFAULT_CONFIG_TOP_VALUE,                             \
    .load_mode    = (nrf_pwm_dec_load_t)PWM_DEFAULT_CONFIG_LOAD_MODE,         \
    .step_mode    = (nrf_pwm_dec_step_t)PWM_DEFAULT_CONFIG_STEP_MODE,         \
};

const nrf_drv_pwm_config_t m_pwm_right_config = 
{                                                                             \
    .output_pins  = {CONFIG_MOTOR_RIGHT_PWM,                                  \
                     NRF_DRV_PWM_PIN_NOT_USED,                                \
                     NRF_DRV_PWM_PIN_NOT_USED,                                \
                     NRF_DRV_PWM_PIN_NOT_USED },                              \
    .irq_priority = PWM_DEFAULT_CONFIG_IRQ_PRIORITY,                          \
    .base_clock   = (nrf_pwm_clk_t)PWM_DEFAULT_CONFIG_BASE_CLOCK,             \
    .count_mode   = (nrf_pwm_mode_t)PWM_DEFAULT_CONFIG_COUNT_MODE,            \
    .top_value    = PWM_DEFAULT_CONFIG_TOP_VALUE,                             \
    .load_mode    = (nrf_pwm_dec_load_t)PWM_DEFAULT_CONFIG_LOAD_MODE,         \
    .step_mode    = (nrf_pwm_dec_step_t)PWM_DEFAULT_CONFIG_STEP_MODE,         \
};

void motor_init(void)
{
    /* If not initialized gpiote model, init is */
    if(!nrf_drv_gpiote_is_init())
    {
        APP_ERROR_CHECK(nrf_drv_gpiote_init());
    }

    nrf_drv_gpiote_out_config_t motor_left_dir_pin_config = GPIOTE_CONFIG_OUT_SIMPLE(false);
    nrf_drv_gpiote_out_config_t motor_right_dir_pin_config = GPIOTE_CONFIG_OUT_SIMPLE(false);
    APP_ERROR_CHECK(nrf_drv_gpiote_out_init(CONFIG_MOTOR_LEFT_DIR, &audio_ctrl_pin_config));
    APP_ERROR_CHECK(nrf_drv_gpiote_out_init(CONFIG_MOTOR_RIGHT_DIR, &audio_ctrl_pin_config));

    APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm_instance_left, &m_pwm_left_config, NULL));
    APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm_instance_right, &m_pwm_right_config, NULL));
}

void motor_start(int16_t left_speed, int16_t right_speed)
{
    //Left
    if(left_speed < 0)
    {
        nrf_drv_gpiote_out_set(CONFIG_MOTOR_LEFT_DIR);
        left_speed = MAX(-255, left_speed);
    }
    else
    {
        nrf_drv_gpiote_out_clear(CONFIG_MOTOR_LEFT_DIR);
        left_speed = MIN(255, left_speed);
    }

    //Right
    if(right_speed < 0)
    {
        nrf_drv_gpiote_out_set(CONFIG_MOTOR_RIGHT_DIR);
        right_speed = MAX(-255, right_speed);
    }
    else
    {
        nrf_drv_gpiote_out_clear(CONFIG_MOTOR_LEFT_DIR);
        right_speed = MIN(255, right_speed);
    }
}

void motor_stop(void)
{
}