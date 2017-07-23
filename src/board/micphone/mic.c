#include "mic.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "app_error.h"
#include "nrf_drv_pdm.h"

#define PDM_PIN_CLK			(21)
#define PDM_PIN_DIN			(22)
#define PDM_BUFFER_SIZE		(1000)

const int16_t m_pdm_buffer[PDM_BUFFER_SIZE*2];
const nrf_drv_pdm_config_t m_pdm_config = NRF_DRV_PDM_DEFAULT_CONFIG(PDM_PIN_CLK,
																	 PDM_PIN_DIN,
																	 m_pdm_buffer,
																	 m_pdm_buffer + PDM_BUFFER_SIZE,
																	 PDM_BUFFER_SIZE);

static void pdm_event_handler(uint32_t * buffer, uint16_t length)
{
}

void mic_init(void)
{
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