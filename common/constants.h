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
#ifndef __CONSTANTS_H__
#define __CONSTANTS_H__

#include "pico/stdlib.h"

// Software version
#define FW_VERSION_MAJOR 1
#define FW_VERSION_MINOR 1
#define FW_VERSION_PATCH 3
#define FW_VERSION_BUILD 0

// Number of max sensors connected to the board
#define CONNECTED_SENSORS 8

// Sensor I2C bus
#define I2C_FREQ 40000
#define I2C_TIMEOUT_US 12000
#define I2C_SENSOR i2c0
#define I2C_SENSOR_SDA 4
#define I2C_SENSOR_SCL 5

// Other devices I2C bus
#define I2C_DEVICE i2c1
#define I2C_DEVICE_FERQ 100000
#define I2C_DEVICE_SDA 6
#define I2C_DEVICE_SCL 7

// Sensor UART
#define UART_SENSOR_TXD 8
#define UART_SENSOR_RXD 9

// 12 V power for sensors enable pin
#define VOLTAGE_12V_EN 11

// Buttons
#define GFX_PACK_BUTTON_A 12
#define GFX_PACK_BUTTON_B 13
#define GFX_PACK_BUTTON_C 14
#define GFX_PACK_BUTTON_D 15
#define GFX_PACK_BUTTON_E 22

// Multiplexer reset pin
#define MUX_RST 16

// Display
#define GFX_PACK_SPI spi0
#define GFX_PACK_SPI_BAUDRATE 10000000
#define GFX_PACK_BACKLIGHT_PIN 10
#define GFX_PACK_CHIP_SELECT_PIN 17
#define GFX_PACK_SERIAL_CLOCK_PIN 18
#define GFX_PACK_MASTER_OUT_SLAVE_IN_PIN 19
#define GFX_PACK_DATA_COMMAND_PIN 20
#define GFX_PACK_RESET_PIN 21
#define GFX_PACK_MASTER_IN_SLAVE_OUT_PIN 2147483647

// Sensor EN pins
#define EN_1 26
#define EN_2 27

// UART service mode pin
#define SVC_MODE 28

// Mutex default timeout value
#define MUTEX_TIMEOUT_MS 1000

// Buffer sizes
#define MAX_SOAP_SIZE 4096
#define CONFIG_SEND_BUFFER_SIZE 4096
#define CONFIG_RECVD_BUFFER_SIZE 300 + 60


// value representing the interval between display draws in ms
static uint16_t display_interval = 33;

// value representing the interval between attempts to connect to wifi in ms
static uint32_t wifi_wait_next_connect_ms = 30000;

// value representing the timeout for DNS server to resolve host IP in ms
static uint32_t wifi_wait_for_dns = 30000;


#endif