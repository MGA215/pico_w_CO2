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

#define CONNECTED_SENSORS 8

#define I2C_BAUDRATE 40000
#define I2C_SENSOR i2c0
#define I2C_SDA 4
#define I2C_SCL 5
#define I2C_TIMEOUT_US 12000

#define I2C_DEVICE i2c1

#define I2C_DEVICE_SDA 6
#define I2C_DEVICE_SCL 7

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
#define GFX_PACK_BACKLIGHT_PIN 10
#define GFX_PACK_CHIP_SELECT_PIN 17
#define GFX_PACK_SERIAL_CLOCK_PIN 18
#define GFX_PACK_MASTER_OUT_SLAVE_IN_PIN 19
#define GFX_PACK_DATA_COMMAND_PIN 20
#define GFX_PACK_RESET_PIN 21
#define GFX_PACK_MASTER_IN_SLAVE_OUT_PIN 2147483647

#define EN_1 26
#define EN_2 27

#define SERIALIZE_BUFFER_LEN 32

#define MUX_ADDR 0x70
#define POWER_EN_ADDR 0x39
#define POWER_5V_ADDR 0x38

#define MUTEX_TIMEOUT_MS 1000

#define MAX_SOAP_SIZE 4096
#define CONFIG_SEND_BUFFER_SIZE 4096
#define CONFIG_RECVD_BUFFER_SIZE 300 + 60

#define SOAP_TESTER_NAME_1 "Tester_01"
#define SOAP_TESTER_NAME_2 "Tester_02"

#define SOAP_TESTER_SN_1 0x24069001


// value representing the interval between display draws in ms
static uint16_t display_interval = 33;

// value representing the interval between sensor readings in ms
static uint32_t sensor_read_interval_ms = 15000;

// value representing the interval between attempts to connect to wifi in ms
static uint32_t wifi_wait_next_connect_ms = 300000;

// value representing the timeout for DNS server to resolve host IP
static uint32_t wifi_wait_for_dns = 30000;

// value representing the interval between SOAP messages in s
static uint16_t soap_write_message_s = 15;

// value representing the delay to add to the main value before the first SOAP message is written in s
static uint16_t soap_write_message_initial_delay_s = 30;

#endif