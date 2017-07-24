#include "mic.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "app_error.h"
#include "nrf_drv_pdm.h"

#define PDM_PIN_CLK			(0)
#define PDM_PIN_DIN			(20)
#define PDM_BUFFER_SIZE		(1000)

int16_t m_pdm_buffer[PDM_BUFFER_SIZE*2];
const nrf_drv_pdm_config_t m_pdm_config = NRF_DRV_PDM_DEFAULT_CONFIG(PDM_PIN_CLK,
																	 PDM_PIN_DIN,
																	 m_pdm_buffer,
																	 m_pdm_buffer + PDM_BUFFER_SIZE,
																	 PDM_BUFFER_SIZE);

static uint16_t *m_listen_data;
static uint32_t m_listen_msec;
static volatile uint16_t m_listen_index;
//static volatile audio_state_t m_audio_state = AUDIO_STATE_IDLE;

static void pdm_event_handler(uint32_t * buffer, uint16_t length)
{
	memcpy(&m_listen_data[m_listen_index], buffer, length*sizeof(uint16_t));
    m_listen_index += length;
}

void mic_init(void)
{
    memset(m_pdm_buffer, 0, sizeof(m_pdm_buffer));
	APP_ERROR_CHECK(nrf_drv_pdm_init(&m_pdm_config, pdm_event_handler));
}

void mic_start(void)
{
	APP_ERROR_CHECK(nrf_drv_pdm_start());
}

void mic_stop(void)
{
	APP_ERROR_CHECK(nrf_drv_pdm_stop());
}

void mic_listen(uint32_t *rx_buffer, uint32_t micro_seconds)
{
	m_listen_data = (uint16_t *)rx_buffer;
	m_listen_index = 0;
	mic_start();
	nrf_delay_ms(micro_seconds);
	mic_stop();
}
