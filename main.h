/**
 * @file main.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief PICO and peripherals control
 * @version 0.1
 * @date 2024-06-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __MAIN_H__
#define __MAIN_H__

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "hardware/gpio.h"
#include "error_codes.h"
#include "string.h"
#include "common/shared.h"

#include "wifi/wifi.h"
#include "common/constants.h"
#include "common/serialize.h"
#include "sensor_config.h"
#include "config_map.h"
#include "soap/soap.h"
#include "soap/soap_channels.h"
#include "service_comm/service_comm.h"

#include "gfx_pack/gfx_pack.h"
#include "sensors/sensors.h"

/**
 * @brief Initializes pins, buses, modules; runs only once
 * 
 * @return int Value describing encountered error during initialization
 */
int init(void);

/**
 * @brief Main program loop
 * 
 * @return int Value describing encountered error during execution
 */
int loop(void);

/**
 * @brief Reads button inputs and reacts to them
 * 
 */
void read_inputs();

/**
 * @brief Updates the display
 * 
 */
void update_display();

/**
 * @brief Updates all components
 * 
 */
void update();

/**
 * @brief Prepares data to be displayed, writes data to the diaplay framebuffer
 * 
 */
void write_display(void);

/**
 * @brief Displays information from sensor
 * 
 * @param sensor_name String containing name of sensor
 * @param state State of sensor
 * @param co2 Should CO2 be displayed
 * @param co2_value CO2 value
 * @param temp Should temperature be displayed
 * @param temp_value Temperature value
 * @param pressure Should pressure be displayed
 * @param pressure_value Pressure value
 * @param humidity Should humidity be displayed
 * @param humidity_value Humidity value
 */
void write_display_sensor(uint8_t* sensor_name, int state, 
        bool co2, float co2_value, 
        bool temp, float temp_value, 
        bool pressure, float pressure_value,
        bool humidity, float humidity_value);

/**
 * @brief Creates a sensor name string according to the sensor type
 * 
 * @param sensor Sensor structure
 * @param buf String buffer
 * @param len Max length of the buffer
 */
void get_sensor_name_string(sensor_t* sensor, uint8_t* buf, uint8_t len);

/**
 * @brief Assigns SOAP channels from configuration map
 * 
 */
void assign_soap_channels(void);

/**
 * @brief Creates SOAP messages and saves them to their buffers
 * 
 */
void create_soap_messages(void);





#endif