/**
 * @file debug.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Debugging module
 * @version 0.1
 * @date 2024-07-17
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __DEBUG_H__
#define __DEBUG_H__

#include "pico/stdlib.h"

#define DEBUG_TIME false

typedef enum debug_severity
{
    SEVERITY_TRACE = 6,
    SEVERITY_DEBUG = 5,
    SEVERITY_INFO = 4,
    SEVERITY_WARN = 3,
    SEVERITY_ERROR = 2,
    SEVERITY_FATAL = 1
} debug_severity_e;

typedef enum debug_source
{
    SOURCE_NO_SOURCE = 0,
    SOURCE_MAIN_INIT = 1,
    SOURCE_MAIN_LOOP = 2,
    SOURCE_SENSORS = 3,
    SOURCE_SOAP = 4,
    SOURCE_RAM = 5,
    SOURCE_DISPLAY = 6,
    SOURCE_RTC = 7,
    SOURCE_GFX = 8,
    SOURCE_SERVICE_COMM = 9,
    SOURCE_EEPROM = 10,
    SOURCE_EE895 = 20,
    SOURCE_CDM7162 = 21,
    SOURCE_SUNRISE = 22,
    SOURCE_SUNLIGHT = 23,
    SOURCE_SCD30 = 24,
    SOURCE_SCD41 = 25,
    SOURCE_COZIR_LP3 = 26,
    SOURCE_CM1107N = 27,
    SOURCE_POWER = 30,
    SOURCE_MUX = 31,
    SOURCE_WIFI = 100,
    SOURCE_TCP_CLIENT = 101,
    SOURCE_TCP_SERVER = 102,
    SOURCE_TCP_DNS = 103
} debug_source_e;

extern uint8_t debug;

extern uint8_t debug_wifi;
extern uint8_t debug_tcp_client;
extern uint8_t debug_tcp_server;
extern uint8_t debug_tcp_dns;

/**
 * @brief Prints a debug message if corresponding DEBUG severity is enabled in constants.h
 * 
 * @param severity Severity of the message
 * @param source Source of the message
 * @param message Message
 * @param ... Message additional arguments
 */
extern void print_ser_output(debug_severity_e severity, debug_source_e source, debug_source_e subsource, const uint8_t* message, ...);

#endif