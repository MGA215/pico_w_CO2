/**
 * @file ee895.c
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Implements communication with E+E EE895 sensor
 * @version 0.1
 * @date 2024-06-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "ee872.h"
#include <string.h>
#include "common/debug.h"
#include "error_codes.h"
#include "common/functions.h"
#include "../power/power.h"

#include "uart/uart.h"
#include "hardware/watchdog.h"

#define EE872_ADDR              237
#define EE872_UART_TIMEOUT_MS   100
#define EE872_CO2_ADDR          0x0424
#define EE872_T_ADDR            0x03EA
#define EE872_P_ADDR            0x04B0

// CO2 range
#define CO2_MIN_RANGE           0.0
#define CO2_MAX_RANGE           10000.0

// temperature range
#define T_MIN_RANGE            -40.0
#define T_MAX_RANGE             60.0

sensor_functions_t ee872_functions_uart = {
    .sensor_get_value = ee872_get_value,
    .sensor_init = ee872_init,
    .sensor_read_config = NULL
};

static int32_t ee872_read(uint16_t addr, uint16_t nreg, uint8_t* buffer);

/**
 * @brief Computes Modbus CRC for specified buffer
 * 
 * @param buf Buffer the CRC is computed from
 * @param len Length of the buffer
 * @return uint16_t CRC value
 */
static inline uint16_t ee_modbus_crc(uint8_t* buf, uint32_t len);

static inline uint16_t ee_modbus_crc(uint8_t* buf, uint32_t len)
{
    uint16_t crc = 0xFFFF;
    uint32_t i;

    for (i = 0; i < len; i++) 
    {
        crc ^= (uint16_t)buf[i];

        for (int i = 8; i != 0; i--) 
        {
            if ((crc & 0x0001) != 0) 
            {
                crc >>= 1;
                crc ^= 0xA001;
            } 
            else 
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

static inline void ee_power(sensor_t* ee895, bool on)
{
    if (!ee895->config.power_global_control && !ee895->config.power_continuous) // If power not controlled globally
    {
        power_en_set_index(ee895->index, on);
    }
}



void ee872_get_value(sensor_t* sensor)
{
    // ee_power(sensor, true);
    uint8_t buffer[4];
    sensor->internal_error_state = ee872_read(EE872_CO2_ADDR, 2, buffer);
    if (sensor->internal_error_state) return;
    uint32_t co2_hex = 0;
    co2_hex |= buffer[0] << 0;
    co2_hex |= buffer[1] << 8;
    co2_hex |= buffer[2] << 16;
    co2_hex |= buffer[3] << 24;
    sensor->co2 = byte2float(ntoh32(co2_hex));

    sensor->internal_error_state = ee872_read(EE872_T_ADDR, 2, buffer);
    if (sensor->internal_error_state) return;
    uint32_t t_hex = 0;
    t_hex |= buffer[0] << 0;
    t_hex |= buffer[1] << 8;
    t_hex |= buffer[2] << 16;
    t_hex |= buffer[3] << 24;
    sensor->temperature = byte2float(ntoh32(t_hex));

    sensor->internal_error_state = ee872_read(EE872_P_ADDR, 2, buffer);
    if (sensor->internal_error_state) return;
    uint32_t p_hex = 0;
    p_hex |= buffer[0] << 0;
    p_hex |= buffer[1] << 8;
    p_hex |= buffer[2] << 16;
    p_hex |= buffer[3] << 24;
    sensor->pressure = byte2float(ntoh32(p_hex));

    sensor->internal_error_state = STATE_OK;
    sensor->meas_state = MEAS_FINISHED;
    sensor->wake_time = at_the_end_of_time;
    // ee_power(sensor, false);

    return;
}


void ee872_init(sensor_t* sensor)
{
    sensor->internal_error_state = STATE_OK;
    return;
}

static int32_t ee872_read(uint16_t addr, uint16_t nreg, uint8_t* buffer)
{
    uint8_t send_buffer[8];
    send_buffer[0] = EE872_ADDR;
    send_buffer[1] = 0x03;
    send_buffer[2] = (addr & 0xFF00) >> 8; // Convert reg address to big endian
    send_buffer[3] = (addr & 0x00FF) >> 0;
    send_buffer[4] = (nreg & 0xFF00) >> 8; // Convert number of registers to big endian
    send_buffer[5] = (nreg & 0x00FF) >> 0;
    uint16_t crc = ee_modbus_crc(send_buffer, 6); // CRC computation
    send_buffer[6] = (crc & 0x00FF) >> 0;
    send_buffer[7] = (crc & 0xFF00) >> 8;
    
    uint8_t recv_data_len = 0;
    uint8_t recv_buffer[nreg * 2 + 8];
    memset(recv_buffer, 0x00, nreg * 2 + 8);
    for (int i = 0; i < 2; i++)
    {
        uart_sensor_empty_buffer(); // Make sure the UART buffer is empty
        uart_sensor_send(send_buffer, 8); // Send data
        absolute_time_t time_recv_timeout = make_timeout_time_ms(EE872_UART_TIMEOUT_MS);
        while (recv_data_len == 0 && !time_reached(time_recv_timeout))
        {
            recv_data_len = uart_sensor_recv(recv_buffer, nreg * 2 + 8); // Wait for data receive
            // watchdog_update();
        }
        if (time_reached(time_recv_timeout)) continue;
        break;
    }
    if (recv_buffer[1] != 0x03 || recv_buffer[2] != 2 * nreg) // Check valid command & number of registers
    {
        print_ser_output(SEVERITY_DEBUG, SOURCE_SENSORS, SOURCE_EE895, "Addr: 0x%X; nreg: %i, recv_len: %i", addr, nreg, recv_data_len);
        print_ser_output(SEVERITY_DEBUG, SOURCE_SENSORS, SOURCE_EE895, "FnCode: 0x%X, data_read_len: %i, expected %i", recv_buffer[1], recv_buffer[2], 2*nreg);
        return EE895_ERROR_READ_RESP; 
    }
    if (ee_modbus_crc(recv_buffer, nreg * 2 + 5) != 0) return EE895_ERROR_INVALID_CRC; // Check CRC valid

    memcpy(buffer, &recv_buffer[3], nreg * 2);

    return SUCCESS;
}
