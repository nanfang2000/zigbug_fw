#include "audio.h"
#include "nrf_drv_i2s.h"
#include "nrf_drv_gpiote.h"
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

#define AUDIO_STATE_IDLE			(0)
#define AUDIO_STATE_PLAYING		(1)
#define AUDIO_STATE_FINISH		(2)

#define AUDIO_CTRL_PIN				(23)

static uint32_t m_buffer_tx[I2S_BUFFER_SIZE];

static const uint8_t *m_audio_data;
static uint16_t m_audio_length;
static volatile uint16_t m_audio_index;
static volatile uint16_t m_audio_state = AUDIO_STATE_IDLE;

static void i2s_event_handler(uint32_t const * p_data_received,
							  uint32_t       * p_data_to_send,
							  uint16_t         number_of_words)
{
    uint16_t i;
    if(m_audio_index < m_audio_length)
    {
        m_audio_state = AUDIO_STATE_PLAYING;
        for (i = 0; i < number_of_words; ++i)
        {
            if(m_audio_index < m_audio_length)
            {
                ((uint16_t *)p_data_to_send)[2*i]     = (uint16_t)m_audio_data[m_audio_index++]<<8;
								((uint16_t *)p_data_to_send)[2*i+1]     = (uint16_t)m_audio_data[m_audio_index++]<<8;
            }
            else
            {
                ((uint16_t *)p_data_to_send)[2*i]     = 0x5555;
								((uint16_t *)p_data_to_send)[2*i+1]     = 0x5555;
            }
						//m_audio_index++;
        }
    }
    else
    {
        m_audio_state = AUDIO_STATE_FINISH;
    }

    NRF_LOG_INFO("Transfer completed:%d\r\n", m_audio_index);
}

void audio_init(void)
{
    nrf_drv_gpiote_out_config_t audio_ctrl_pin_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

    APP_ERROR_CHECK(nrf_drv_gpiote_out_init(AUDIO_CTRL_PIN, &audio_ctrl_pin_config));

    if(!nrf_drv_gpiote_is_init())
    {
        APP_ERROR_CHECK(nrf_drv_gpiote_init());
    }
    APP_ERROR_CHECK(nrf_drv_i2s_init(&m_i2s_config, i2s_event_handler));
    m_audio_state = AUDIO_STATE_IDLE;
}

void audio_enable(void)
{
	nrf_drv_gpiote_out_set(AUDIO_CTRL_PIN);
}

void audio_disable(void)
{
	nrf_drv_gpiote_out_clear(AUDIO_CTRL_PIN);
}

void audio_play(const uint8_t * p_audio_data, uint16_t audio_length)
{
    if(audio_length == 0)
		return;
	
	m_audio_data = p_audio_data;
	m_audio_length = audio_length;
	m_audio_index = 0;
	
	/* Enable Audio chip */
	nrf_drv_gpiote_out_set(AUDIO_CTRL_PIN);
		
	APP_ERROR_CHECK(nrf_drv_i2s_start(NULL, m_buffer_tx, I2S_BUFFER_SIZE, 0));
	/* Wait for finish */
	while(m_audio_index < m_audio_length);
	
	nrf_drv_i2s_stop();
		
	/* Disable Audio chip */
	nrf_drv_gpiote_out_clear(AUDIO_CTRL_PIN);
	
	m_audio_state = AUDIO_STATE_IDLE;
}


void audio_stop(void)
{
	nrf_drv_i2s_stop();
	/* Disable Audio chip */
	nrf_drv_gpiote_out_clear(AUDIO_CTRL_PIN);
}
