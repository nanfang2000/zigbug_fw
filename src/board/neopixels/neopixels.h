#ifndef __NEOPIXELS_H
#define __NEOPIXELS_H
#include <stdint.h>
#include "nrf_drv_spi.h"

#ifdef  __cplusplus
extern "C" {
#endif
typedef struct
{
	uint8_t green;
	uint8_t red;
	uint8_t blue;
}rgb_t;

typedef struct
{
	uint8_t out_pin;
	uint8_t led_num;
	rgb_t *rgb_buf;
}led_group_t;

void neopixel_init(const led_group_t * groups, uint8_t group_size, const nrf_drv_spi_t *spi_instance);
void neopixels_write_rgb(const led_group_t * led_group, int32_t index, const rgb_t *rgb);

#ifdef  __cplusplus
}  
#endif

#endif
