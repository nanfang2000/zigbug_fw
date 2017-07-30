#include "mic.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "app_error.h"
#include "nrf_drv_pdm.h"

#define CONFIG_PDM_PIN_CLK			(30)
#define CONFIG_PDM_PIN_DIN			(20)

#define CONFIG_PDM_BUFFER_SIZE_SAMPLES		(64)
#define CONFIG_PDM_TRANSIENT_STATE_LEN 		(500)

STATIC_ASSERT(PDM_CONFIG_CLOCK_FREQ == NRF_PDM_FREQ_1032K);
#define SAMPLING_RATE   (1032 * 1000 / 64)

int16_t m_pdm_buffer[2][CONFIG_PDM_BUFFER_SIZE_SAMPLES];
const nrf_drv_pdm_config_t m_pdm_config = NRF_DRV_PDM_DEFAULT_CONFIG(CONFIG_PDM_PIN_CLK,
																	 CONFIG_PDM_PIN_DIN,
																	 m_pdm_buffer[0],
																	 m_pdm_buffer[1],
																	 CONFIG_PDM_BUFFER_SIZE_SAMPLES);

static uint16_t *m_listen_data;
static uint32_t m_listen_size;
static uint32_t m_listen_channel;
static volatile uint16_t m_listen_index;
//static volatile audio_state_t m_audio_state = AUDIO_STATE_IDLE;
static uint16_t                      m_skip_buffers;

static void pdm_event_handler(uint32_t * buffer, uint16_t length)
{
	int32_t i;
	ASSERT(length == CONFIG_PDM_BUFFER_SIZE_SAMPLES);
/*    length /= 2;
	for(i = 0; i < length; i++)
	{
		m_listen_data[m_listen_index++] = (uint16_t)(buffer[i] & 0xFFFF);
	}*/
	if (m_skip_buffers)
    {
        m_skip_buffers -= 1;
    }
    else
	{
		memcpy(&m_listen_data[m_listen_index], buffer, CONFIG_PDM_BUFFER_SIZE_SAMPLES);
		m_listen_index += CONFIG_PDM_BUFFER_SIZE_SAMPLES/2;
	}
}

void mic_init(void)
{
    memset(m_pdm_buffer, 0, sizeof(m_pdm_buffer));
	APP_ERROR_CHECK(nrf_drv_pdm_init(&m_pdm_config, pdm_event_handler));
}

void mic_start(void)
{
	    // Skip buffers with invalid data.
    m_skip_buffers = MAX(1, ROUNDED_DIV((CONFIG_PDM_TRANSIENT_STATE_LEN * SAMPLING_RATE),
                                        (1000 * CONFIG_PDM_BUFFER_SIZE_SAMPLES)));
	APP_ERROR_CHECK(nrf_drv_pdm_start());
}

void mic_stop(void)
{
	APP_ERROR_CHECK(nrf_drv_pdm_stop());
}

void mic_listen(uint8_t channel,uint16_t *rx_buffer, uint32_t micro_seconds)
{
	m_listen_data = (uint16_t *)rx_buffer;
	m_listen_index = 0;
	m_listen_channel = channel;
	m_listen_size = MAX(1, ROUNDED_DIV((micro_seconds * SAMPLING_RATE),
									   (1000 * CONFIG_PDM_BUFFER_SIZE_SAMPLES))) *
					CONFIG_PDM_BUFFER_SIZE_SAMPLES/2;
	mic_start();
	while(m_listen_index < m_listen_size);
	mic_stop();
}
