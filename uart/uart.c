#include "uart.h"
#include "common/shared.h"
#include "hardware/uart.h"
#include "pico/printf.h"
#include "cobs/cobslib.h"
#include "string.h"
#include "../service_comm/service_comm.h"

#define UART_INST uart0
#define UART_TX 0
#define UART_RX 1

extern uint8_t buffer_sent[];
extern uint8_t buffer_recv[];

void uart_service_init(void)
{
    if (!mutex_is_initialized(&config_data.command_mutex)) mutex_init(&config_data.command_mutex);
    if (!mutex_is_initialized(&config_data.response_mutex)) mutex_init(&config_data.response_mutex);

    if (uart_is_enabled(UART_INST)) return;
    gpio_init(UART_TX);
    gpio_init(UART_RX);
    gpio_set_function(UART_TX, GPIO_FUNC_UART);
    gpio_set_function(UART_RX, GPIO_FUNC_UART);

    uart_init(UART_INST, 115200);
}

void uart_service_read_command(void)
{
    uint32_t ptr = 0;
    if (uart_is_readable_within_us(UART_INST, MUTEX_TIMEOUT_MS))
    {
        do
        {
            buffer_recv[ptr++] = uart_getc(UART_INST);
            if (ptr >= (360 - 1)) break;
        } while (uart_is_readable_within_us(UART_INST, MUTEX_TIMEOUT_MS));
        buffer_recv[ptr] = '\0';
        if (mutex_enter_timeout_ms(&config_data.command_mutex, MUTEX_TIMEOUT_MS))
        {
            uint32_t len = ptr;
            decodeCOBS(buffer_recv, config_data.command, &len);
            mutex_exit(&config_data.command_mutex);
            memset(buffer_recv, 0x00, ptr);
            config_data.command_len = len;
            uint16_t data_len = service_comm_parse_message();
            if (config_data.command_len > CMD_PADDING + CMD_MAX_LEN || data_len > CMD_MAX_LEN) return; // Check message too long
            if (config_data.command_len != data_len + CMD_PADDING) return; // Check message too short
                
            config_data.command_rdy = true;
            config_data.response_sent = false;
        }
    }
    return;
}

void uart_service_send_response(void)
{
    uint16_t response_len;
    if (mutex_enter_timeout_ms(&config_data.response_mutex, MUTEX_TIMEOUT_MS))
    {
        encodeCOBS(config_data.response, buffer_sent, &config_data.response_len);
        response_len = config_data.response_len;
        mutex_exit(&config_data.response_mutex);
        config_data.response_rdy = false;
        for (int i = 0; i < response_len; i++)
        {
            uart_putc_raw(UART_INST, buffer_sent[i]);
        }
        memset(buffer_sent, 0x00, response_len);
        config_data.response_sent = true;
    }
}