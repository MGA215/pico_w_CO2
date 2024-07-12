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

#include "../common_include.h"

#include "pico/stdio.h"
#include "pico/printf.h"
#include "malloc.h"

#include "stdlib.h"
#include "math.h"
#include <string.h>
#include <stdarg.h>

#if DEBUG && ENABLE_COLORED_DEBUG
    #define RED_BOLD    "\033[1;31;40m"
    #define RED         "\033[0;31m"
    #define GRN         "\033[0;32m"
    #define YEL         "\033[0;33m"
    #define BLU         "\033[0;34m"
    #define MAG         "\033[0;35m"
    #define CYN         "\033[0;36m"
    #define WHT         "\033[0;37m"
    #define RESET       "\033[0;0m"
    #define COLOR_FATAL "\033[1;37;41m"
    #define COLOR_ERROR "\033[1;31m"
    #define COLOR_WARN  "\033[0;33m"
    #define COLOR_INFO  "\033[1m"
    #define COLOR_DEBUG "\033[1;90m"
    #define COLOR_TRACE "\033[0;90m"
#else
    #define RED_BOLD    ""
    #define RED         ""
    #define GRN         ""
    #define YEL         ""
    #define BLU         ""
    #define MAG         ""
    #define CYN         ""
    #define WHT         ""
    #define RESET       ""
    #define COLOR_FATAL ""
    #define COLOR_ERROR ""
    #define COLOR_WARN  ""
    #define COLOR_INFO  ""
    #define COLOR_DEBUG ""
    #define COLOR_TRACE ""
#endif

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
    uint8_t* bytes = (uint8_t*)&byte_value;
    float output;

    *((uint8_t*)(&output) + 3) = bytes[0];
    *((uint8_t*)(&output) + 2) = bytes[1];
    *((uint8_t*)(&output) + 1) = bytes[2];
    *((uint8_t*)(&output) + 0) = bytes[3];

    return output;
}

uint32_t float2byte(float float_value)
{
    uint8_t bytes[4];
    bytes[0] = *((uint8_t*)(&float_value) + 3);
    bytes[1] = *((uint8_t*)(&float_value) + 2);
    bytes[2] = *((uint8_t*)(&float_value) + 1);
    bytes[3] = *((uint8_t*)(&float_value) + 0);

    return *((uint32_t*)&bytes[0]);
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
    get_input_power_index(input_index, &(sensor->input_index), &(sensor->power_index));
    memset(sensor->state_reg, 0x00, 26);
}

void get_input_power_index(uint8_t internal_index, uint8_t* input_index, uint8_t* power_index)
{
    *input_index = (internal_index + 4) % 8;
    *power_index = internal_index;
}

void print_ser_output(debug_severity_e severity, const uint8_t* source, const uint8_t* message, ...)
{
    if (DEBUG >= severity)
    {
        int32_t message_len = strlen(message);
        message_len += 64;
        uint8_t buf[message_len];
        uint8_t severity_str[8];
        uint8_t severity_color[12];
        switch (severity)
        {
            case SEVERITY_TRACE:
                snprintf(severity_str, 8, "[trace]");
                snprintf(severity_color, 11, COLOR_TRACE);
                break;
            case SEVERITY_DEBUG:
                snprintf(severity_str, 8, "[debug]");
                snprintf(severity_color, 11, COLOR_DEBUG);
                break;
            case SEVERITY_INFO:
                snprintf(severity_str, 8, "[info] ");
                snprintf(severity_color, 11, COLOR_INFO);
                break;
            case SEVERITY_WARN:
                snprintf(severity_str, 8, "[Warn] ");
                snprintf(severity_color, 11, COLOR_WARN);
                break;
            case SEVERITY_ERROR:
                snprintf(severity_str, 8, "[ERROR]");
                snprintf(severity_color, 11, COLOR_ERROR);
                break;
            case SEVERITY_FATAL:
                snprintf(severity_str, 8, "[FATAL]");
                snprintf(severity_color, 11, COLOR_FATAL);
                break;
            default:
                break;
        }
        
        va_list va;
        va_start(va, message);
        vsnprintf(buf, message_len, message, va);
        va_end(va);

        float time_sec = (float)(to_us_since_boot(get_absolute_time()) / 1000) / 1000.0f;
        printf("%s[%12.3f] %s [%s] %s\n"RESET"", severity_color, time_sec, severity_str, source, buf);
    }
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
