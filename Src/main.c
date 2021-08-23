/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
 *
 * @defgroup blinky_example_main main.c
 * @{
 * @ingroup blinky_example
 * @brief Blinky Example Application main file.
 *
 * This file contains the source code for a sample application to blink LEDs.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
//#include "boards.h"
#include "neopixels.h"
#include "audio.h"
#include "mic.h"
#include "batt_meas.h"
#include "app_error.h"
#include "app_timer.h"
#include "app_scheduler.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_timer.h"
#include "vision.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "motion.h"
#include "pid.h"
#include "uart.h"

// #define LOCAL_DEBUG
#include "debug.h"

#define DEAD_BEEF   0xDEADBEEF          /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define SCHED_MAX_EVENT_DATA_SIZE   MAX(APP_TIMER_SCHED_EVENT_DATA_SIZE, 0) 
#define SCHED_QUEUE_SIZE            60  /**< Maximum number of events in the scheduler queue. */

static rgb_t       m_led_rgb_body[4];
const led_group_t m_body_leds = 
{
	.out_pin = 14,
	.led_num = 4,
	.rgb_buf = (rgb_t*)&m_led_rgb_body,
};
rgb_t red = {255, 0, 0};
rgb_t green = {0, 255, 0};
rgb_t blue = {0, 0, 255};
rgb_t white = {255, 255, 255};
rgb_t black = {0, 0, 0};

static const nrf_drv_spi_t neopixels_spi_instance = NRF_DRV_SPI_INSTANCE(0);
#include "wav_1.h"
static uint16_t listen_buffer[26000];

#define SPEED 15

uint32_t error_code;
void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    #ifdef DEBUG
    app_error_print(id, pc, info);
    #endif
    error_code = ((error_info_t *)(info))->err_code;
    UNUSED_VARIABLE(error_code);
    while (1)
    {
       neopixels_write_rgb(&m_body_leds, 0, &red);
       nrf_delay_ms(200);
       neopixels_write_rgb(&m_body_leds, 0, &green);
       nrf_delay_ms(200);
    };
}

static void lfclk_config(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}
#define TASK_DELAY        200           /**< Task delay. Delays a LED0 task for 200 ms */
#define TIMER_PERIOD      1000          /**< Timer period. LED1 timer will expire after 1000 ms */

TaskHandle_t  led_toggle_task_handle;   /**< Reference to LED0 toggling FreeRTOS task. */
TaskHandle_t  vision_task_handle;   /**< Reference to LED0 toggling FreeRTOS task. */
TaskHandle_t  motion_task_handle;   /**< Reference to LED0 toggling FreeRTOS task. */
TaskHandle_t  batt_meas_task_handle;   /**< Reference to LED0 toggling FreeRTOS task. */
TaskHandle_t  zigbug_task_handle;   /**< Reference to LED0 toggling FreeRTOS task. */
TaskHandle_t  scheduler_task_handle;   /**< Reference to LED0 toggling FreeRTOS task. */
TimerHandle_t led_toggle_timer_handle;  /**< Reference to LED1 toggling FreeRTOS timer. */
TaskHandle_t motor_pfm_task_handle;  /**< Reference to LED1 toggling FreeRTOS timer. */
/**@brief LED0 task entry function.
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the task.
 */
static void led_toggle_task_function (void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    while (true)
    {
        neopixels_write_rgb(&m_body_leds, 0, &red);

        /* Delay a task for a given number of ticks */
        vTaskDelay(50);
        neopixels_write_rgb(&m_body_leds, 0, &black);
        vTaskDelay(1500);

        /* Tasks must be implemented to never return... */
    }
}

APP_TIMER_DEF(breath_led_app_timer_id);
bool led_breath_toggle = false;
static void app_timer_breath_led_handler(void * unused)
{
    if(led_breath_toggle)
    {
        neopixels_write_rgb(&m_body_leds, 0, &red);
        //motor_start(-SPEED, -SPEED);
    }
    else
    {
        neopixels_write_rgb(&m_body_leds, 0, &black);
        //motor_start(SPEED, SPEED);
    }
    led_breath_toggle = !led_breath_toggle;
}
void enable_breath_led(uint32_t meas_interval_ms)
{
    uint32_t err_code = app_timer_create(&breath_led_app_timer_id,
        APP_TIMER_MODE_REPEATED,
        app_timer_breath_led_handler);
    RETURN_IF_ERROR(err_code);

    err_code = app_timer_start(breath_led_app_timer_id,
                                APP_TIMER_TICKS(meas_interval_ms), NULL);
    RETURN_IF_ERROR(err_code);
}
vision_t vision;
motion_raw_t motion_raw;
volatile bool motion_init_done = false;
//#include <string.h>
/**@brief LED0 task entry function.
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the task.
 */
static void vision_task_function(void *pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    vision_init();
    motion_init();
    motion_init_done = true;
    memset(&vision, 0, sizeof(vision));
    while (true)
    {
        vision_start_measure(VISION_ALL);
        vTaskDelay(5);
        motion_read_raw(&motion_raw);
        motion_update(&motion_raw);
        vTaskDelay(5);
        motion_read_raw(&motion_raw);
        motion_update(&motion_raw);
        vTaskDelay(5);
        motion_read_raw(&motion_raw);
        motion_update(&motion_raw);
        vTaskDelay(5);
        motion_read_raw(&motion_raw);
        motion_update(&motion_raw);
        vTaskDelay(5);
        motion_read_raw(&motion_raw);
        motion_update(&motion_raw);
        vTaskDelay(5);
        vision_get_measure(VISION_ALL, &vision);
        
    }
}

#define TEST_NO  3
static void motion_update_task_function(void *pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    pid_t pid,pid_barrier;

    motion_init();
    pid_init(&pid, 0.6, 0.005, 5.5);
    vTaskDelay(20);

    //pid_init(&pid_barrier, 0.6, 0.005, 0.5);
    while (true)
    {
        motion_read_raw(&motion_raw);
        motion_update(&motion_raw);
//        DEBUG_PRINTF(0, "motion:%dms, yaw=%d\r\n", (int32_t)(motion_raw.timestamp*1000),
//                                                                    (int32_t)(motion_get_data()->euler.yaw*10));
        #if (TEST_NO == 0)
        float cur_degree = motion_get_data()->euler.yaw*10;
        int32_t delta = cur_degree - 60;
        float next_motor = pid_process(&pid, 90.f, cur_degree);
        DEBUG_PRINTF(0, "motion:%dms, yaw=%d, next_pwm=%d\r\n", (int32_t)(motion_raw.timestamp*1000),
                                                                    (int32_t)(cur_degree), (int32_t)(next_motor));
        motor_start((int16_t)next_motor, -(int16_t)next_motor);
        #elif (TEST_NO == 1)
        float barrier_force = abs(vision.dist_right_side);
        if(vision.dist_right_side == 0)
        {
            barrier_force = 0;
        }
        else if(vision.dist_right_side > 100)
        {
            barrier_force = 0;
        }
        else
        {
            barrier_force = 100 - vision.dist_right_side;
        }
        float next_motor = pid_process(&pid_barrier, 0, barrier_force);
        if(barrier_force == 0)
        {
          motor_start(0, 0);
        }
        else
        {
          motor_start(SPEED+(int16_t)next_motor, SPEED-(int16_t)next_motor);
        }
//       int32_t delta = motion_get_data()->euler.yaw*10 - 0;
//       if(delta > 40)
//       {
//         delta = 40;
//       }
//       else if(delta < -40)
//       {
//           delta = -40;
//       }
//       motor_start(SPEED-delta/5, SPEED+delta/5);
        #elif (TEST_NO == 2)
        float cur_degree = motion_get_data()->euler.yaw*10;
        float next_motor = pid_process(&pid, 0, cur_degree);
        motor_start(SPEED+(int16_t)next_motor, SPEED-(int16_t)next_motor);
        #endif
        vTaskDelay(5);
    }
}
float dist_to_force(float dist)
{
    float force;
    if (dist == 0)
    {
        force = 0;
    }
    else if (dist > 100)
    {
        force = 0;
    }
    else
    {
        force = 100 - dist;
    }
    return force;
}

void run_with_barrier_detect(float target_angle, pid_t *pid, pid_t *pid_barrier_left, pid_t *pid_barrier_right, pid_t *pid_front, pid_t *pid_back)
{
    float barrier_force_left = dist_to_force(vision.dist_left_side);
    float barrier_force_right = dist_to_force(vision.dist_right_side);
    float barrier_force_front = dist_to_force(vision.dist_front);
    float barrier_force_back = dist_to_force(vision.dist_back);

    if (barrier_force_left == 0 && barrier_force_right == 0 && barrier_force_front == 0 && barrier_force_back == 0)
    {
        target_angle = motion_get_data()->euler.yaw;
        pid_init(pid, 0.3, 0.001, 1.0);
        pid_init(pid_barrier_left, 0.2, 0.005, 7.0);
        pid_init(pid_barrier_right, 0.2, 0.005, 7.0);
        pid_init(pid_front, 0.5, 0.01, 7.0);
        pid_init(pid_back, 0.5, 0.01, 7.0);
        motor_start(0, 0);
    }
    else
    {
        int32_t speed = 20;
        float cur_degree = motion_get_data()->euler.yaw;
        target_angle = cur_degree;
        if (barrier_force_left > 0)
        {
            target_angle += 50;
            speed += SPEED;
        }
        else if (barrier_force_right > 0)
        {
            target_angle -= 50;
            speed += SPEED;
        }
        if (barrier_force_front > 0)
        {
            target_angle += 50;
            speed += SPEED;
        }
        else if (barrier_force_back > 0)
        {
            speed += SPEED;
        }
        for (int32_t i = 0; i < 50; i++)
        {
            cur_degree = motion_get_data()->euler.yaw;
            float next_motor1 = pid_process(pid, target_angle, cur_degree);
            float next_motor2 = 0;//pid_process(pid_barrier_right, 0, barrier_force_right);
            float next_motor3 = 0;//-pid_process(pid_barrier_left, 0, barrier_force_left);
            float next_motor4 = 0;//pid_process(pid_front, 0, barrier_force_front);
            float next_motor5 = 0;//-pid_process(pid_back, 0, barrier_force_back);
            motor_start(speed + (int16_t)(next_motor1 + next_motor2 + next_motor3 + next_motor4 + next_motor5),
                        speed - (int16_t)(next_motor1 + next_motor2 + next_motor3 - next_motor4 - next_motor5));
            vTaskDelay(5);
        }
    }
}

void run_with_fixed_orientation(pid_t *pid, float target_angle)
{
    float cur_degree = motion_get_data()->euler.yaw;
    float next_motor = pid_process(pid, target_angle, cur_degree);
    // DEBUG_PRINTF(0, "deg:%d\n", (int32_t)cur_degree);
    motor_start(SPEED + (int16_t)next_motor, SPEED - (int16_t)next_motor);
}

void run_with_openmv(pid_t *pid)
{
    uint8_t tmp = uart_get();
    uint16_t x=160,y;
    if (tmp == 0xAA) {
      tmp = uart_get();
      if(tmp == 0x55) {
        uint8_t tm1 = uart_get();
        uint8_t tm2 = uart_get();
        x = (uint16_t)tm1+(((uint16_t)tm1)<<16);
        y = uart_get() + ((uint16_t)uart_get()<<16);
      }
    }

    float next_motor = pid_process(pid, 160, 320-x);
    motor_start(SPEED + (int16_t)next_motor, SPEED - (int16_t)next_motor);
}

void zigbug_stop(void)
{
    /* Wait for zigbug to stop, checking ACC */
    motor_start(0, 0);
    int32_t static_cnt = 0;
    for (int32_t i = 0; i < 100; i++)
    {
        vector3_t norm_acc = motion_get_data()->normAcc;
        if (abs(norm_acc.x) + abs(norm_acc.y) < 0.1)
        {
            static_cnt++;
            if (static_cnt > 10)
            {
                break;
            }
        }
        vTaskDelay(10);
    }
    motion_reset();
}

void zigbug_rotate(float degree)
{
    pid_t pid_turn;

    int32_t degree_cnt = 0;
    float target_angle = motion_get_data()->euler.yaw;
    target_angle += degree;
    pid_init(&pid_turn, 0.5, 0.005, 7.0);
    for (int32_t i = 0; i < 200; i++)
    {
        float cur_degree = motion_get_data()->euler.yaw;
        float next_motor1 = pid_process(&pid_turn, target_angle, cur_degree);

        if(next_motor1 > 30)
        {
            next_motor1 = 30;
        }
        else if(next_motor1 < -30)
        {
            next_motor1 = -30;
        }
        motor_start(+ (int16_t)(next_motor1),
                    - (int16_t)(next_motor1));
        if (abs(target_angle - cur_degree) < 5)
        {
            degree_cnt++;
            if (degree_cnt > 3)
            {
                break;
            }
        }
        vTaskDelay(5);
    }
}

#define TEST_NO1  2
static void zigbug_run_task_function(void *pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    float target_angle = 0;
    pid_t pid,pid_barrier_right, pid_barrier_left, pid_front, pid_back;
    pid_init(&pid, 0.5, 0.01, 1.0);
    while (true)
    {
        if(!motion_init_done)
        {
            vTaskDelay(20);
            continue;
        }
        // if (uart_get() != 's')
        // {
        //     vTaskDelay(20);
        //     continue;
        // }
#if (TEST_NO1 == 0)
        float cur_degree = motion_get_data()->euler.yaw*10;
        float next_motor = pid_process(&pid, 90.f, cur_degree);
        DEBUG_PRINTF(0, "motion:%dms, yaw=%d, next_pwm=%d\r\n", (int32_t)(motion_raw.timestamp*1000),
                                                                    (int32_t)(cur_degree), (int32_t)(next_motor));
        motor_start((int16_t)next_motor, -(int16_t)next_motor);
#elif (TEST_NO1 == 1)
        run_with_barrier_detect(target_angle, &pid, &pid_barrier_left, &pid_barrier_right, &pid_front, &pid_back);
#elif (TEST_NO1 == 2)
//        run_with_fixed_orientation(&pid, target_angle);
        run_with_openmv(&pid);
#else
        // my_printf("eye:%d,%d, front:%d\r\n", vision.dist_left_eye, vision.dist_right_eye, vision.dist_front );
        if(vision.dist_left_eye > 150 || vision.dist_right_eye > 150)
        {
            
            /* Wait for zigbug to stop */
            zigbug_stop();
            vTaskDelay(500);
            /* Wait for zigbug to rotate, checking degree */
            zigbug_rotate(90);
            target_angle = motion_get_data()->euler.yaw;
            // my_printf("target_angle1:%f\r\n", target_angle);
        }
        else
        {
            if(vision.dist_left_eye < 40)
            {
                zigbug_stop();
                vTaskDelay(100);
                /* Wait for zigbug to rotate, checking degree */
                zigbug_rotate(90);
                zigbug_stop();
                vTaskDelay(100);
                target_angle = motion_get_data()->euler.yaw;
            }
            else if(vision.dist_right_eye < 40)
            {
                zigbug_stop();
                vTaskDelay(100);
                /* Wait for zigbug to rotate, checking degree */
                zigbug_rotate(-90);
                zigbug_stop();
                vTaskDelay(100);
                target_angle = motion_get_data()->euler.yaw;
            }
            else if (vision.dist_front < 100)
            {
                /* Wait for zigbug to stop */
                zigbug_stop();
                vTaskDelay(500);
                if(vision.dist_front < 100)
                {
                    /* Wait for zigbug to rotate, checking degree */
                    zigbug_rotate(90);
                    zigbug_stop();
                    vTaskDelay(100);
                    target_angle = motion_get_data()->euler.yaw;
                }
                run_with_fixed_orientation(&pid, target_angle);
                // my_printf("target_angle2:%f\r\n", target_angle);

            }
            else
            {
                run_with_fixed_orientation(&pid, target_angle);
                // my_printf("target_angle3:%f\r\n", target_angle);
            }
            
            // run_with_barrier_detect(target_angle, &pid, &pid_barrier_left, &pid_barrier_right, &pid_front, &pid_back);
        //    motor_start(SPEED, SPEED);
        //    DEBUG_PRINTF(0, "deg:%d\n", (int32_t)vision.dist_front);
        //    if (vision.dist_front < 200)
        //    {
        //        zigbug_stop();
        //        vTaskDelay(500);
        //        zigbug_rotate(60);
        //        motor_start(SPEED, SPEED);
        //    }
        }
        // DEBUG_PRINTF(0, "LEFT_EYE:%d\n", vision.dist_left_eye);
#endif
        vTaskDelay(20);
    }
}
static void battery_meas_task_function(void *pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    batt_meas_init(NULL);
    while (true)
    {
        batt_meas_start();
        vTaskDelay(5000);
    }
}

static void scheduler_task_function(void *pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    while (true)
    {
        app_sched_execute();
        vTaskDelay(100);
    }
}

/**
 * @brief Function for application main entry.
 */
int main(void)
 {
    lfclk_config();
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
    // app_timer_init();
    /* Configure board. */
    //bsp_board_leds_init();
    nrf_drv_gpiote_init();
    uart_init();
    audio_init();
    mic_init();
    //vision_init();
    motor_init();
    //motor_start(-10, 10);
//    my_printf("ZigBug Starts!\r\n");
    // while(1)
    // {
    // motor_start(100, 100);
    // nrf_delay_ms(2);
    // motor_start(0, 0);
    // nrf_delay_ms(8);
    // }
    //motion_init();
    //batt_meas_init(NULL);
    //batt_meas_enable(5000);
    //motor_start(-SPEED, -SPEED);
    //neopixel_init(&m_body_leds, 1, &neopixels_spi_instance);
    // enable_breath_led(1000);
    /* Toggle LEDs. */
    /* Create task for LED0 blinking with priority set to 2 */
    //UNUSED_VARIABLE(xTaskCreate(led_toggle_task_function, "LED0", configMINIMAL_STACK_SIZE + 100, NULL, 1, &led_toggle_task_handle));

    UNUSED_VARIABLE(xTaskCreate(vision_task_function, "VISION", configMINIMAL_STACK_SIZE + 200, NULL, 4, &vision_task_handle));
    //UNUSED_VARIABLE(xTaskCreate(motion_update_task_function, "Motion", configMINIMAL_STACK_SIZE + 200, NULL, 3, &motion_task_handle));
    //UNUSED_VARIABLE(xTaskCreate(scheduler_task_function, "Scheduler", configMINIMAL_STACK_SIZE + 100, NULL, 3, &scheduler_task_handle));
    //UNUSED_VARIABLE(xTaskCreate(battery_meas_task_function, "Battery", configMINIMAL_STACK_SIZE + 100, NULL, 2, &batt_meas_task_handle));
    UNUSED_VARIABLE(xTaskCreate(zigbug_run_task_function, "Zigbug", configMINIMAL_STACK_SIZE + 200, NULL, 3, &zigbug_task_handle));
    /* Activate deep sleep mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* Start FreeRTOS scheduler. */
    vTaskStartScheduler();

    while (true)
    {
        //audio_play(DATA, SOUND_LENGTH);
        //DEBUG_PRINTF(0, "Battery:%d%%\n", batt_meas_get_level());
       //mic_listen(0, listen_buffer, 2000);
       app_sched_execute();
    }
}

/**
 *@}
 **/
