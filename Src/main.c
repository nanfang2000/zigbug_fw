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
#include "app_error.h"
#include "nrf_drv_gpiote.h"
#include "SEGGER_RTT_Conf.h"
#include "SEGGER_RTT.h"

static rgb_t       m_led_rgb_body[4];
const led_group_t m_body_leds = 
{
	.out_pin = 14,
	.led_num = 4,
	.rgb_buf = (rgb_t*)&m_led_rgb_body,
};
rgb_t red = {255, 20, 20};
rgb_t green = {20, 255, 20};
rgb_t blue = {20, 255, 20};
rgb_t white = {255, 255, 255};

static const nrf_drv_spi_t neopixels_spi_instance = NRF_DRV_SPI_INSTANCE(SPI0_INSTANCE_INDEX);
#include "wav_1.h"
static uint16_t listen_buffer[16000];

uint32_t error_code;
void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    #ifdef DEBUG
    app_error_print(id, pc, info);
    #endif
    error_code = ((error_info_t *)(info))->err_code;
    UNUSED_VARIABLE(error_code);
    while (1);
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    /* Configure board. */
    //bsp_board_leds_init();
    nrf_drv_gpiote_init();
    //neopixel_init(&m_body_leds, 1, &neopixels_spi_instance);
    audio_init();
    mic_init();
 	
    /* Toggle LEDs. */
    while (true)
    {
        /*neopixels_write_rgb(&m_body_leds, 0, &white);
        //bsp_board_led_invert(i);
        nrf_delay_ms(500);
        neopixels_write_rgb(&m_body_leds, 0, &white);
        nrf_delay_ms(500);
        neopixels_write_rgb(&m_body_leds, 0, &white);*/
        nrf_delay_ms(1000);
        //audio_play(DATA, SOUND_LENGTH);
        SEGGER_RTT_printf(0, "replay\n");
        mic_listen(0, listen_buffer, 200);
    }
}

/**
 *@}
 **/
