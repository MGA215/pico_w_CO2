/**
 * @file functions.c
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Implements useful functions
 * @version 0.1
 * @date 2024-06-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "functions.h"

float byte2float(uint32_t byte_value)
{
    uint8_t* bytes = (uint8_t*)&byte_value;
    float output;

    *((uint8_t*)(&output) + 3) = bytes[0];
    *((uint8_t*)(&output) + 2) = bytes[1];
    *((uint8_t*)(&output) + 1) = bytes[2];
    *((uint8_t*)(&output) + 0) = bytes[3];

    return output;
}

uint16_t ntoh16(uint16_t network)
{
    return (network >> 8) | ((network & 0x00FF) << 8);
}

uint32_t ntoh32(uint32_t network)
{
    return (network >> 16) | ((network & 0x0000FFFF) << 16);
}

void common_init_struct(sensor_t* sensor, uint8_t input_index)
{
    sensor->co2 = 0.0f;
    sensor->humidity = 0.0f;
    sensor->pressure = 0.0f;
    sensor->temperature = 0.0f;
    sensor->meas_state = MEAS_FINISHED;
    sensor->measurement_iterator = 0;
    sensor->state = ERROR_SENSOR_NOT_INITIALIZED;
    sensor->timeout_iterator = 0;
    sensor->wake_time = at_the_end_of_time;
    sensor->input_index = input_index;
    memset(sensor->state_reg, 0x00, 26);
}