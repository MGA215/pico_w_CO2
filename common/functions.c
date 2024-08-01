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

#include "common_include.h"

#include "pico/stdio.h"
#include "pico/printf.h"
#include "malloc.h"
#include "hardware/watchdog.h"
#include "shared.h"

#include "stdlib.h"
#include "math.h"
#include <string.h>
#include <stdarg.h>

/**
 * @brief Get the input and power index from sensor index
 * 
 * @param internal_index Index of the sensor in programm (generated from loop iteration)
 * @param input_index Index of the input (should be the same as the input index)
 * @param power_index Index in the power vector
 */
void get_input_power_index(uint8_t internal_index, uint8_t* input_index, uint8_t* power_index);

float byte2float(uint32_t byte_value)
{
    // uint8_t* bytes = (uint8_t*)&byte_value;
    // float output;

    // *( (uint8_t*)(&output) + 3) = bytes[0];
    // *( (uint8_t*)(&output) + 2) = bytes[1];
    // *( (uint8_t*)(&output) + 1) = bytes[2];
    // *( (uint8_t*)(&output) + 0) = bytes[3];

    // union {
    //     uint8_t bytes[8];
    //     float output;
    // } b2f;

    // b2f.bytes[0] = (byte_value & 0x000000FF) >> 0;
    // b2f.bytes[0] = (byte_value & 0x0000FF00) >> 8;
    // b2f.bytes[0] = (byte_value & 0x00FF0000) >> 16;
    // b2f.bytes[0] = (byte_value & 0xFF000000) >> 24;

    float output;
    uint32_t bytes = 0;
    bytes |= (byte_value & 0xFF000000) >> 24;
    bytes |= (byte_value & 0x00FF0000) >> 8;
    bytes |= (byte_value & 0x0000FF00) << 8;
    bytes |= (byte_value & 0x000000FF) << 24;
    memcpy(&output, &bytes, 4);

    return output;
}

uint32_t float2byte(float float_value)
{
//     uint8_t bytes[4];
//     bytes[0] = *( (uint8_t*)(&float_value) + 3);
//     bytes[1] = *( (uint8_t*)(&float_value) + 2);
//     bytes[2] = *( (uint8_t*)(&float_value) + 1);
//     bytes[3] = *( (uint8_t*)(&float_value) + 0);

//     return *( (uint32_t*)&bytes[0]);

    uint32_t output;
    uint32_t bytes = 0;
    memcpy(&bytes, &float_value, 4);
    output |= (bytes & 0xFF000000) >> 24;
    output |= (bytes & 0x00FF0000) >> 8;
    output |= (bytes & 0x0000FF00) << 8;
    output |= (bytes & 0x000000FF) << 24;

    return output;
}

uint16_t ntoh16(uint16_t network)
{
    return ((network & 0xFF00) >> 8) | ((network & 0x00FF) << 8);
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
    sensor->config.sensor_active = false;
    sensor->index = input_index;
    sensor->sensor_number = 0;
    get_input_power_index(input_index, &(sensor->input_index), &(sensor->power_index));
    memset(sensor->state_reg, 0x00, 26);
}

void get_input_power_index(uint8_t internal_index, uint8_t* input_index, uint8_t* power_index)
{
    *input_index = (internal_index + 4) % 8; // for 1to8 mux on board
    *power_index = internal_index;
}

float round_precision(float value, uint8_t precision)
{
    int charsNeeded = 1 + snprintf(NULL, 0, "%.*f", precision, value);
    char *buffer = malloc(charsNeeded);
    snprintf(buffer, charsNeeded, "%.*f", precision, value);
    float result = atof(buffer);
    free(buffer);
    return result;
}

uint32_t get_error(uint8_t error_byte)
{
    return ((uint32_t)error_byte << 24) | 0x000080FF;
}

volatile uint8_t hex2dec(uint8_t hex_val)
{
    uint8_t out;
    uint8_t val1 = hex_val & 0xF0;
    uint8_t val2 = val1 >> 4;
    uint8_t val3 = val2 * 10;
    uint8_t val4 = hex_val & 0x0F;
    out = val3 + val4;
    return out;
}

uint8_t dec2hex(uint8_t dec_val)
{
    uint8_t out = 0;
    out |= dec_val % 10;
    out |= (dec_val / 10) << 4;
    return out;
}
