/**
 * @file mux.c
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Mux control implementation
 * @version 0.1
 * @date 2024-07-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "mux.h"

int32_t mux_enable_sensor(uint8_t sensor_index)
{
    int32_t ret;
    if (sensor_index >= 8) return ERROR_MUX_INVALID_INDEX;
    uint8_t mux_addr = sensor_index < 4 ? MUX0_ADDR : MUX1_ADDR;
    uint8_t sensor_mux_index = sensor_index % 4;
    uint8_t mux_vector = 0b1 << sensor_mux_index;
    if ((ret = i2c_write_timeout_us(I2C_SENSOR, mux_addr, &mux_vector, 1, false, I2C_TIMEOUT_US)) < 0) return ret;
    return SUCCESS;
}



