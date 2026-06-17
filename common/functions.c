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

    uint32_t output = 0;
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
    sensor->sensor_state = NOT_INITIALIZED;
    sensor->error_state = ERROR_SENSOR_NOT_INITIALIZED;
    sensor->internal_error_state = STATE_OK;
    sensor->timeout_iterator = 0;
    sensor->wake_time = get_absolute_time();
    sensor->config.sensor_active = false;
    sensor->index = input_index;
    sensor->sensor_number = 0;
    sensor->err_iter_counter = 0;
    sensor->start_new_measurement = false;
    get_input_power_index(input_index, &(sensor->input_index), &(sensor->power_index)); // For HYT271 and MS5607 not needed
    memset(sensor->state_reg, 0x00, 26);
    memset(sensor->pressure_raw, 0x00, 3);
    memset(sensor->temperature_raw, 0x00, 3);
    memset(sensor->humidity_raw, 0x00, 3);
    memset(sensor->prom_buffer, 0x00, 16 * sizeof(uint16_t));
}

void get_input_power_index(uint8_t internal_index, uint8_t* input_index, uint8_t* power_index)
{
    *input_index = (internal_index + 4) % 8; // for 1to8 mux on board
    *power_index = internal_index;
}

float round_precision(float value, uint8_t precision)
{
    int32_t charsNeeded = 1 + snprintf(NULL, 0, "%.*f", precision, value);
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

uint8_t hex2dec(uint8_t hex_val)
{
    // uint8_t out = (((hex_val & 0xF0) >> 4) * 10) + (hex_val & 0x0F);
    return (((hex_val & 0xF0) >> 4) * 10) + (hex_val & 0x0F);
}

uint8_t dec2hex(uint8_t dec_val)
{
    uint8_t out = 0;
    out |= dec_val % 10;
    out |= (dec_val / 10) << 4;
    return out;
}

uint8_t reverse_bits_in_byte(uint8_t b) 
{
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

bool common_is_measurement_running(sensor_t* sensor)
{
    return sensor->meas_state != MEAS_FINISHED && is_at_the_end_of_time(sensor->wake_time);
}

void common_measurement_force_stop(sensor_t* sensor)
{
    sensor->meas_state = MEAS_FINISHED;
    sensor->wake_time = at_the_end_of_time;
}

void common_measurement_start(sensor_t* sensor)
{
    sensor->meas_state = MEAS_STARTED;
    sensor->wake_time = get_absolute_time();
}

bool common_should_sensor_operate(sensor_t* sensor)
{
    return time_reached(sensor->wake_time) && sensor->config.sensor_active;
}

void common_disable_sensor_for_ms(sensor_t* sensor, uint32_t time_ms)
{
    sensor->wake_time = make_timeout_time_us((uint64_t)time_ms * 1000);
}