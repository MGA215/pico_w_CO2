/**
 * @file constants.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Constants
 * @version 0.1
 * @date 2024-06-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef CONSTANTS
#define CONSTANTS

#include "structs.h"
#include "sensor_config.h"

// Set DEBUG to 0 to disable logging into console
#ifndef DEBUG
    #define DEBUG 1
    #ifndef DEBUG_WARN
        #define DEBUG_WARN 1
        #ifndef DEBUG_INFO
            #define DEBUG_INFO 1
        #endif
    #endif
#endif

#define CONNECTED_SENSORS 8

#define I2C_BAUDRATE 40000
#define I2C_SENSOR i2c0
#define I2C_SDA 4
#define I2C_SCL 5
#define I2C_TIMEOUT_US 12000

#define DS3231_I2C_PORT i2c1
#define DS3231_I2C_SDA_PIN 6
#define DS3231_I2C_SCL_PIN 7

#define UART_SENSOR_TXD 8
#define UART_SENSOR_RXD 9

#define VOLTAGE_12V_EN 11

#define GFX_PACK_BUTTON_A 12
#define GFX_PACK_BUTTON_B 13
#define GFX_PACK_BUTTON_C 14
#define GFX_PACK_BUTTON_D 15
#define GFX_PACK_BUTTON_E 22

#define MUX_RST 16

#define GFX_PACK_SPI spi0
#define GFX_PACK_SPI_BAUDRATE 10000000
#define GFX_PACK_BACKLIGHT_PIN 9
#define GFX_PACK_CHIP_SELECT_PIN 17
#define GFX_PACK_SERIAL_CLOCK_PIN 18
#define GFX_PACK_MASTER_OUT_SLAVE_IN_PIN 19
#define GFX_PACK_DATA_COMMAND_PIN 20
#define GFX_PACK_RESET_PIN 21
#define GFX_PACK_MASTER_IN_SLAVE_OUT_PIN 2147483647

#define EN_1 26
#define EN_2 27

#define SERIALIZE_BUFFER_LEN 32

#define MUX0_ADDR 0x70
#define MUX1_ADDR 0x71


// value representing the interval between display draws in ms
static uint16_t display_interval = 33;

// value representing the interval between sensor readings in ms
static uint32_t sensor_read_interval_ms = 15000;

#endif