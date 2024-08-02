/**
 * @file scd30.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Header file for communication with Sensirion SCD30 sensor
 * @version 0.1
 * @date 2024-06-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __SCD30_H__
#define __SCD30_H__

#include "pico/stdlib.h"
#include "common/structs.h"

/**
 * @brief Reads data from the SCD30 sensor
 * 
 * @param command Command to execute (get specific data)
 * @param buf Buffer to save read data
 * @param len Length of the buffer
 * @return int32_t Return code
 */
extern int32_t scd30_read(uint16_t command, uint16_t* buf, uint32_t len);

/**
 * @brief Writes a value to the SCD30 sensor
 * 
 * @param command Command to write specific data
 * @param value Data to write
 * @return int32_t Return code
 */
extern int32_t scd30_write_value(uint16_t command, uint16_t value);

/**
 * @brief Executes a command on the SCD30 sensor
 * 
 * @param command Command to send
 * @return int32_t Return code
 */
extern int32_t scd30_write_command(uint16_t command);

/**
 * @brief Retrieves measured values from the SCD30 sensor
 * 
 * @param scd30 Sensor structure; return code saved to the state variable
 */
extern void scd30_get_value(sensor_t* scd30);

/**
 * @brief Initializes the sensor and writes configuration to it
 * 
 * @param scd30 Sensor structure
 * @param config Configuration to write
 * @return int32_t Return code
 */
extern int32_t scd30_init(sensor_t* scd30, sensor_config_t* config);

/**
 * @brief Reads configuration from the SCD30 sensor
 * 
 * @param config Config structure the read configuration will be saved to
 * @return int32_t Return code
 */
extern int32_t scd30_read_config(sensor_config_t* config);

#endif