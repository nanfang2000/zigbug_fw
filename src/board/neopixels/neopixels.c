#include "neopixels.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "app_error.h"

static const nrf_drv_spi_t *m_spi_instance;// = NRF_DRV_SPI_INSTANCE(SPI0_INSTANCE_INDEX);
static nrf_drv_spi_config_t m_spi_config = 
{                                                            \
    .sck_pin      = 22,                \
    .mosi_pin     = 22,                \
    .miso_pin     = NRF_DRV_SPI_PIN_NOT_USED,                \
    .ss_pin       = NRF_DRV_SPI_PIN_NOT_USED,                \
    .irq_priority = SPI_DEFAULT_CONFIG_IRQ_PRIORITY,         \
    .orc          = 0xFF,                                    \
    .frequency    = NRF_DRV_SPI_FREQ_8M,                     \
    .mode         = NRF_DRV_SPI_MODE_0,                      \
    .bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,         \
};

typedef struct
{
	uint8_t green[8];
	uint8_t red[8];
	uint8_t blue[8];
}led_spi_buf_t;

#define MAX_LEDS	(6)
static led_spi_buf_t    m_spi_tx_buf[MAX_LEDS];

#define CODE_0	(0x70)
#define CODE_1	(0x7C)

static const led_group_t * m_last_select_group = NULL;

static void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    NRF_LOG_INFO("Transfer completed.\r\n");
}

void neopixel_init(const led_group_t * groups, uint8_t group_size, const nrf_drv_spi_t *spi_instance)
{
	m_last_select_group = NULL;
	m_spi_instance = spi_instance;
	memset(m_spi_tx_buf, 0, sizeof(m_spi_tx_buf));
	for(int8_t i = 0; i < group_size; i++)
	{
		memset(groups[i].rgb_buf, 0, groups[i].led_num*3);
		//spi_config.mosi_pin = dat_pin;
		//APP_ERROR_CHECK(nrf_drv_spi_init(&spi_instance, &spi_config, spi_event_handler, NULL));
	}
}

static void neopixel_select_group(const led_group_t * led_group)
{
	if(m_last_select_group != led_group)
	{
		m_spi_config.mosi_pin = led_group->out_pin;
		if(m_last_select_group != NULL)
		{
			nrf_drv_spi_uninit(m_spi_instance);
		}
		m_last_select_group = led_group;
		APP_ERROR_CHECK(nrf_drv_spi_init(m_spi_instance, &m_spi_config, spi_event_handler, NULL));
	}
}

static void neopixels_fill_spi_buf(uint8_t color, uint8_t *spi_buffer)
{
    for(uint8_t i_rgb = 0; i_rgb < 8; i_rgb++)
    {
        switch(color & 0x80 )
        {
            case ( 0x00 ):
                *spi_buffer = CODE_0;
                break;
            case ( 0x80 ):
                *spi_buffer = CODE_1;
                break;
        }
        spi_buffer++;
        color <<= 1;
    }
}

void neopixels_write_rgb(const led_group_t * led_group, int32_t index, const rgb_t *rgb)
{
	neopixel_select_group(led_group);
	led_group->rgb_buf[index].green = rgb->green;
	led_group->rgb_buf[index].red = rgb->red;
	led_group->rgb_buf[index].blue = rgb->blue;
	for(int8_t i = 0; i < led_group->led_num; i++)
	{
		neopixels_fill_spi_buf(led_group->rgb_buf[i].green, m_spi_tx_buf[i].green);
		neopixels_fill_spi_buf(led_group->rgb_buf[i].red, m_spi_tx_buf[i].red);
		neopixels_fill_spi_buf(led_group->rgb_buf[i].blue, m_spi_tx_buf[i].blue);
	}
	
	APP_ERROR_CHECK(nrf_drv_spi_transfer(m_spi_instance, (const uint8_t *)m_spi_tx_buf, led_group->led_num*sizeof(led_spi_buf_t), NULL, 0));
}
