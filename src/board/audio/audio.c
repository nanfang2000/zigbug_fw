#include "audio.h"
#include "nrf_drv_i2s.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "app_error.h"

// In Master mode the MCK frequency and the MCK/LRCK ratio should be
// set properly in order to achieve desired audio sample rate (which
// is equivalent to the LRCK frequency).
// For the following settings we'll get the LRCK frequency equal to
// 15873 Hz (the closest one to 16 kHz that is possible to achieve).
static nrf_drv_i2s_config_t m_i2s_config = 
{                                            \
    .sck_pin      = I2S_CONFIG_SCK_PIN,      \
    .lrck_pin     = I2S_CONFIG_LRCK_PIN,     \
    .mck_pin      = I2S_CONFIG_MCK_PIN,      \
    .sdout_pin    = I2S_CONFIG_SDOUT_PIN,    \
    .sdin_pin     = I2S_CONFIG_SDIN_PIN,     \
    .irq_priority = I2S_CONFIG_IRQ_PRIORITY, \
    .mode         = NRF_I2S_MODE_MASTER,       \
    .format       = NRF_I2S_FORMAT_I2S,       \
    .alignment    = NRF_I2S_ALIGN_LEFT,        \
    .sample_width = NRF_I2S_SWIDTH_16BIT,       \
    .channels     = NRF_I2S_CHANNELS_LEFT,     \
    .mck_setup    = NRF_I2S_MCK_32MDIV21,   \
    .ratio        = NRF_I2S_RATIO_96X,        \
};

#define I2S_BUFFER_SIZE     1000
static uint32_t m_buffer_tx[I2S_BUFFER_SIZE];

static volatile uint8_t  m_blocks_transferred     = 0;
static          uint16_t m_sample_value_to_send;

static const uint16_t *m_audio_data;
static uint16_t m_audio_length;
static uint16_t m_audio_index;

static void i2s_event_handler(uint32_t const * p_data_received,
															uint32_t       * p_data_to_send,
															uint16_t         number_of_words)
{
	  if (m_blocks_transferred == 0)
    {
        m_sample_value_to_send   = 0xCAFE;
    }
	  uint16_t i;
    for (i = 0; i < number_of_words; ++i)
    {
        ((uint16_t *)p_data_to_send)[i]     = m_sample_value_to_send;
        ++m_sample_value_to_send;
    }
		m_blocks_transferred++;
    NRF_LOG_INFO("Transfer completed.\r\n");
}

void audio_init(void)
{
	APP_ERROR_CHECK(nrf_drv_i2s_init(&m_i2s_config, i2s_event_handler));
}

void audio_play(const uint16_t * p_audio_data, uint16_t audio_length)
{
	if(audio_length == 0)
		return;
	m_audio_data = p_audio_data;
	m_audio_length = audio_length;
	m_audio_index = 0;
	
	int32_t blocks_need_transfer = (audio_length + I2S_BUFFER_SIZE - 1)/I2S_BUFFER_SIZE;
	blocks_need_transfer += 1;/* ???, don't know when the last block transfer finished, so transfer more block */
	m_blocks_transferred = 0;
	APP_ERROR_CHECK(nrf_drv_i2s_start(NULL, m_buffer_tx, I2S_BUFFER_SIZE, 0));
	while(m_blocks_transferred < blocks_need_transfer);
	nrf_drv_i2s_stop();
}


void audio_stop(void)
{
	nrf_drv_i2s_stop();
}
