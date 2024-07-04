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
    if ((ret = i2c_write_timeout_us(I2C_SENSOR, mux_addr, &mux_vector, 1, false, I2C_TIMEOUT_US * 3)) < 0) return ret;
    return SUCCESS;
}

void mux_reset(void)
{
    gpio_put(MUX_RST, 0);
    sleep_us(100);
    gpio_put(MUX_RST, 1);
    sleep_us(10);
    return;
}

void mux_init(void)
{
    gpio_init(MUX_RST);
    gpio_set_function(MUX_RST, GPIO_FUNC_SIO);
    gpio_set_drive_strength(MUX_RST, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_dir(MUX_RST, GPIO_OUT);
    gpio_put(MUX_RST, 1);
    return;
}

