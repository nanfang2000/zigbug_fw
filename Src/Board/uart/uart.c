#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_uart.h"
#include "nrf_uart.h"
#include "app_util_platform.h"
#include "nrf_drv_gpiote.h"

#define RX_PIN_NUMBER  22
#define TX_PIN_NUMBER  24
#define CTS_PIN_NUMBER NRF_UART_PSEL_DISCONNECTED
#define RTS_PIN_NUMBER NRF_UART_PSEL_DISCONNECTED

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[100];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
//            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;
            index = 0;
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}

void gpio_event(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action){
}

void uart_init(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
//nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
//nrf_drv_gpiote_in_init(22, &config,gpio_event);
}

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "nrf_delay.h"
#include <stdint.h>

void vprint(const char *fmt, va_list argp)
{
    char string[200];
    char *p_char = string;
    if(0 < vsprintf(string,fmt,argp)) // build string
    {
        while(*p_char != 0)
        {
            UNUSED_VARIABLE(app_uart_put(*p_char++));
        }
    }
}

void my_printf(const char *fmt, ...) // custom printf() function
{
    va_list argp;
    va_start(argp, fmt);
    vprint(fmt, argp);
    va_end(argp);
}

uint8_t uart_get(void)
{
    uint8_t byte;
    uint32_t error=NRF_ERROR_NOT_FOUND;
    while (error == NRF_ERROR_NOT_FOUND) {
      error = app_uart_get(&byte);
    }
    return byte;
}

uint8_t gpio_read(void)
{
  return nrf_drv_gpiote_in_is_set(22);
}