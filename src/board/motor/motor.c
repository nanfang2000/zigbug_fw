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
#include "nrf_drv_timer.h"

#define TC214B_LEFT_IA  29 // Motor LEFT Input A --> MOTOR LEFT OA+
#define TC214B_LEFT_IB  27 // Motor LEFT Input B --> MOTOR LEFT OB-
#define TC214B_RIGHT_IA 25 // Motor RIGHT Input A --> MOTOR RIGHT OA+
#define TC214B_RIGHT_IB 00 // Motor RIGHT Input B --> MOTOR RIGHT OB-

#define CONFIG_MOTOR_LEFT_PWM			(TC214B_LEFT_IA)
#define CONFIG_MOTOR_LEFT_DIR			(TC214B_LEFT_IB)
#define CONFIG_MOTOR_RIGHT_PWM			(TC214B_RIGHT_IA)
#define CONFIG_MOTOR_RIGHT_DIR			(TC214B_RIGHT_IB)

#define MOTION_USE_PWM   0
#define MOTION_USE_PFM   1
#if MOTION_USE_PWM
#define MOTOR_PWM_FREQ  (10000) // Hz max 10khz
#define PWM_FREQ_SCALE  (1000000/100/MOTOR_PWM_FREQ) //PWM CLK = 1Mhz

static const nrf_drv_pwm_t m_pwm_motor_instance = NRF_DRV_PWM_INSTANCE(0);
nrf_pwm_values_individual_t m_pwm_seq_values[] = {0, 0, 0, 0};
nrf_pwm_sequence_t const m_pwm_seq =
{
    .values.p_individual = m_pwm_seq_values,
    .length          = NRF_PWM_VALUES_LENGTH(m_pwm_seq_values),
    .repeats         = 0,
    .end_delay       = 0
};

nrf_drv_pwm_config_t m_pwm_motor_config = 
{                                                                             
    .output_pins  = {CONFIG_MOTOR_LEFT_PWM,                                   
                     CONFIG_MOTOR_RIGHT_PWM,                                  
                     NRF_DRV_PWM_PIN_NOT_USED,                                
                     NRF_DRV_PWM_PIN_NOT_USED },                              
    .irq_priority = PWM_DEFAULT_CONFIG_IRQ_PRIORITY,                          
    .base_clock   = NRF_PWM_CLK_1MHz,             
    .count_mode   = NRF_PWM_MODE_UP,            
    .top_value    = 100*PWM_FREQ_SCALE,                             
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

//    nrf_drv_gpiote_out_config_t motor_left_dir_pin_config = GPIOTE_CONFIG_OUT_SIMPLE(false);
//    nrf_drv_gpiote_out_config_t motor_right_dir_pin_config = GPIOTE_CONFIG_OUT_SIMPLE(false);
//    APP_ERROR_CHECK(nrf_drv_gpiote_out_init(CONFIG_MOTOR_LEFT_DIR, &motor_left_dir_pin_config));
//    APP_ERROR_CHECK(nrf_drv_gpiote_out_init(CONFIG_MOTOR_RIGHT_DIR, &motor_right_dir_pin_config));
//
//    APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm_motor_instance, &m_pwm_motor_config, NULL));
}

static void motor_pwm_update_duty_cycle(uint16_t duty_cycle0, uint16_t duty_cycle1)
{
    
    // Check if value is outside of range. If so, set to 100%
    if(duty_cycle0 >= 100)
    {
        m_pwm_seq_values->channel_0 = 0;
    }
    else
    {
        m_pwm_seq_values->channel_0 = (100 - duty_cycle0)*PWM_FREQ_SCALE;
    }

    if(duty_cycle1 >= 100)
    {
        m_pwm_seq_values->channel_1 = 0;
    }
    else
    {
        m_pwm_seq_values->channel_1 = (100 - duty_cycle1)*PWM_FREQ_SCALE;
    }
    
    nrf_drv_pwm_simple_playback(&m_pwm_motor_instance, &m_pwm_seq, 1, NRF_DRV_PWM_FLAG_LOOP);
}

void motor_start(int16_t left_speed, int16_t right_speed)
{
    uint8_t duty_left;
    uint8_t duty_right;
    nrf_drv_pwm_uninit(&m_pwm_motor_instance);
    nrf_drv_gpiote_out_uninit(TC214B_LEFT_IA);
    nrf_drv_gpiote_out_uninit(TC214B_LEFT_IB);
    nrf_drv_gpiote_out_uninit(TC214B_RIGHT_IA);
    nrf_drv_gpiote_out_uninit(TC214B_RIGHT_IB);
    //Left
    if(left_speed < 0)
    {
        nrf_drv_gpiote_out_uninit(TC214B_LEFT_IA);
        nrf_drv_gpiote_out_config_t motor_left_dir_pin_config = GPIOTE_CONFIG_OUT_SIMPLE(false);
        APP_ERROR_CHECK(nrf_drv_gpiote_out_init(TC214B_LEFT_IA, &motor_left_dir_pin_config));
        m_pwm_motor_config.output_pins[0] = TC214B_LEFT_IB;
        duty_left = -MAX(-100, left_speed);
    }
    else
    {
        nrf_drv_gpiote_out_uninit(TC214B_LEFT_IB);
        nrf_drv_gpiote_out_config_t motor_left_dir_pin_config = GPIOTE_CONFIG_OUT_SIMPLE(false);
        APP_ERROR_CHECK(nrf_drv_gpiote_out_init(TC214B_LEFT_IB, &motor_left_dir_pin_config));
        m_pwm_motor_config.output_pins[0] = TC214B_LEFT_IA;
        duty_left = MIN(100, left_speed);
    }

    //Right
    if(right_speed < 0)
    {
        nrf_drv_gpiote_out_uninit(TC214B_RIGHT_IA);
        nrf_drv_gpiote_out_config_t motor_right_dir_pin_config = GPIOTE_CONFIG_OUT_SIMPLE(false);
        APP_ERROR_CHECK(nrf_drv_gpiote_out_init(TC214B_RIGHT_IA, &motor_right_dir_pin_config));
        m_pwm_motor_config.output_pins[1] = TC214B_RIGHT_IB;
        duty_right = -MAX(-100, right_speed);
    }
    else
    {
        nrf_drv_gpiote_out_uninit(TC214B_RIGHT_IB);
        nrf_drv_gpiote_out_config_t motor_right_dir_pin_config = GPIOTE_CONFIG_OUT_SIMPLE(false);
        APP_ERROR_CHECK(nrf_drv_gpiote_out_init(TC214B_RIGHT_IB, &motor_right_dir_pin_config));
        m_pwm_motor_config.output_pins[1] = TC214B_RIGHT_IA;
        duty_right = MIN(100, right_speed);
    }
    APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm_motor_instance, &m_pwm_motor_config, NULL));
    motor_pwm_update_duty_cycle(duty_left, duty_right);
}

void motor_stop(void)
{
    nrf_drv_pwm_stop(&m_pwm_motor_instance, false);
}

#else

#define MaxPFM 100
volatile int g_left_dir_pin = TC214B_LEFT_IA;
volatile int g_left_pfm_pin = TC214B_LEFT_IB;
volatile int g_right_dir_pin = TC214B_RIGHT_IA;
volatile int g_right_pfm_pin = TC214B_RIGHT_IB;
volatile int g_pfm_left = 0;
volatile int g_pfm_right = 0;
volatile int g_pfm_dir_left = 0;
volatile int g_pfm_dir_right = 0;

void DoPfmLeft(int pp, int dir)
{
    static int pfmCnt = 0;

    pfmCnt += pp;
    if (pfmCnt > MaxPFM)
    {
        pfmCnt -= MaxPFM;
        nrf_drv_gpiote_out_set(g_left_pfm_pin);
    }
    else
    {
        nrf_drv_gpiote_out_clear(g_left_pfm_pin);
    }
}

void DoPfmRight(int pp, int dir)
{
    static int pfmCnt = 0;

    pfmCnt += pp;
    if (pfmCnt > MaxPFM)
    {
        pfmCnt -= MaxPFM;
        nrf_drv_gpiote_out_set(g_right_pfm_pin);
    }
    else
    {
        nrf_drv_gpiote_out_clear(g_right_pfm_pin);
    }
}

nrf_drv_timer_t pfm_timer = NRF_DRV_TIMER_INSTANCE(0);
nrf_drv_timer_config_t pfm_timer_config = NRF_DRV_TIMER_DEFAULT_CONFIG;

void pfm_timer_handle(nrf_timer_event_t event_type, void * p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            DoPfmLeft(g_pfm_left, g_pfm_dir_left);
            DoPfmRight(g_pfm_right, g_pfm_dir_right);
            break;

        default:
            //Do nothing.
            break;
    }
}

void motor_init(void)
{
    /* If not initialized gpiote model, init is */
    if(!nrf_drv_gpiote_is_init())
    {
        APP_ERROR_CHECK(nrf_drv_gpiote_init());
    }

    //Motor pins
    nrf_drv_gpiote_out_config_t motor_left_dir_pin_config = GPIOTE_CONFIG_OUT_SIMPLE(false);
    nrf_drv_gpiote_out_config_t motor_right_dir_pin_config = GPIOTE_CONFIG_OUT_SIMPLE(false);
    APP_ERROR_CHECK(nrf_drv_gpiote_out_init(CONFIG_MOTOR_LEFT_DIR, &motor_left_dir_pin_config));
    APP_ERROR_CHECK(nrf_drv_gpiote_out_init(CONFIG_MOTOR_LEFT_PWM, &motor_left_dir_pin_config));
    APP_ERROR_CHECK(nrf_drv_gpiote_out_init(CONFIG_MOTOR_RIGHT_DIR, &motor_left_dir_pin_config));
    APP_ERROR_CHECK(nrf_drv_gpiote_out_init(CONFIG_MOTOR_RIGHT_PWM, &motor_left_dir_pin_config));
    
    //Timer init
    uint32_t time_us = 2000; //Time(in miliseconds) between consecutive compare events.
    uint32_t time_ticks;
    nrf_drv_timer_init(&pfm_timer, &pfm_timer_config, pfm_timer_handle);
    time_ticks = nrf_drv_timer_us_to_ticks(&pfm_timer, time_us);
    nrf_drv_timer_extended_compare(
         &pfm_timer, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
    nrf_drv_timer_enable(&pfm_timer);
}

void motor_start(int16_t left_speed, int16_t right_speed)
{
    nrf_drv_timer_resume(&pfm_timer);
    if(left_speed < 0)
    {
        g_left_dir_pin = TC214B_LEFT_IA;
        g_left_pfm_pin = TC214B_LEFT_IB;
        g_pfm_dir_left = 1;
        g_pfm_left = -MAX(-100, left_speed);
    }
    else
    {
        g_left_dir_pin = TC214B_LEFT_IB;
        g_left_pfm_pin = TC214B_LEFT_IA;
        g_pfm_dir_left = 0;
        g_pfm_left = MIN(100, left_speed);
    }
    nrf_drv_gpiote_out_clear(g_left_dir_pin);
    nrf_drv_gpiote_out_clear(g_left_pfm_pin);

    if(right_speed < 0)
    {
        g_right_dir_pin = TC214B_RIGHT_IA;
        g_right_pfm_pin = TC214B_RIGHT_IB;
        g_pfm_dir_right = 1;
        g_pfm_right = -MAX(-100, right_speed);
    }
    else
    {
        g_right_dir_pin = TC214B_RIGHT_IB;
        g_right_pfm_pin = TC214B_RIGHT_IA;
        g_pfm_dir_right = 0;
        g_pfm_right = MIN(100, right_speed);
    }
    nrf_drv_gpiote_out_clear(g_right_dir_pin);
    nrf_drv_gpiote_out_clear(g_right_pfm_pin);
}

void motor_stop(void)
{
    nrf_drv_timer_pause(&pfm_timer);
}

#endif