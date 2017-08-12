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

#define TC214B_LEFT_IA  29 // Motor LEFT Input A --> MOTOR LEFT OA+
#define TC214B_LEFT_IB  27 // Motor LEFT Input B --> MOTOR LEFT OB-
#define TC214B_RIGHT_IA 25 // Motor RIGHT Input A --> MOTOR RIGHT OA+
#define TC214B_RIGHT_IB 00 // Motor RIGHT Input B --> MOTOR RIGHT OB-

#define CONFIG_MOTOR_LEFT_PWM			(TC214B_LEFT_IA)
#define CONFIG_MOTOR_LEFT_DIR			(TC214B_LEFT_IB)
#define CONFIG_MOTOR_RIGHT_PWM			(TC214B_RIGHT_IA)
#define CONFIG_MOTOR_RIGHT_DIR			(TC214B_RIGHT_IB)

static const nrf_drv_pwm_t m_pwm_motor_instance = NRF_DRV_PWM_INSTANCE(0);
nrf_pwm_values_individual_t m_pwm_seq_values[] = {0, 0, 0, 0};
nrf_pwm_sequence_t const m_pwm_seq =
{
    .values.p_individual = m_pwm_seq_values,
    .length          = NRF_PWM_VALUES_LENGTH(m_pwm_seq_values),
    .repeats         = 0,
    .end_delay       = 0
};

const nrf_drv_pwm_config_t m_pwm_motor_config = 
{                                                                             
    .output_pins  = {CONFIG_MOTOR_LEFT_PWM,                                   
                     CONFIG_MOTOR_RIGHT_PWM,                                  
                     NRF_DRV_PWM_PIN_NOT_USED,                                
                     NRF_DRV_PWM_PIN_NOT_USED },                              
    .irq_priority = PWM_DEFAULT_CONFIG_IRQ_PRIORITY,                          
    .base_clock   = NRF_PWM_CLK_1MHz,             
    .count_mode   = NRF_PWM_MODE_UP,            
    .top_value    = 100,                             
    .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,         
    .step_mode    = NRF_PWM_STEP_AUTO,         
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
    APP_ERROR_CHECK(nrf_drv_gpiote_out_init(CONFIG_MOTOR_LEFT_DIR, &motor_left_dir_pin_config));
    APP_ERROR_CHECK(nrf_drv_gpiote_out_init(CONFIG_MOTOR_RIGHT_DIR, &motor_right_dir_pin_config));

    APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm_motor_instance, &m_pwm_motor_config, NULL));
}

static void motor_pwm_update_duty_cycle(uint8_t duty_cycle0, uint8_t duty_cycle1)
{
    
    // Check if value is outside of range. If so, set to 100%
    if(duty_cycle0 >= 100)
    {
        m_pwm_seq_values->channel_0 = 0;
    }
    else
    {
        m_pwm_seq_values->channel_0 = 100 - duty_cycle0;
    }

    if(duty_cycle1 >= 100)
    {
        m_pwm_seq_values->channel_1 = 0;
    }
    else
    {
        m_pwm_seq_values->channel_1 = 100 - duty_cycle1;
    }
    
    nrf_drv_pwm_simple_playback(&m_pwm_motor_instance, &m_pwm_seq, 1, NRF_DRV_PWM_FLAG_LOOP);
}

void motor_start(int8_t left_speed, int8_t right_speed)
{
    uint8_t duty_left;
    uint8_t duty_right;
    //Left
    if(left_speed < 0)
    {
        nrf_drv_gpiote_out_set(CONFIG_MOTOR_LEFT_DIR);
        duty_left = 100 + MAX(-100, left_speed);
    }
    else
    {
        nrf_drv_gpiote_out_clear(CONFIG_MOTOR_LEFT_DIR);
        duty_left = MIN(100, left_speed);
    }

    //Right
    if(right_speed < 0)
    {
        nrf_drv_gpiote_out_set(CONFIG_MOTOR_RIGHT_DIR);
        duty_right = 100 + MAX(-100, right_speed);
    }
    else
    {
        nrf_drv_gpiote_out_clear(CONFIG_MOTOR_RIGHT_DIR);
        duty_right = MIN(100, right_speed);
    }
    motor_pwm_update_duty_cycle(duty_left, duty_right);
}

void motor_stop(void)
{
}