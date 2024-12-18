/**
 * @file cdm7162.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Header file for communication with Figaro CDM7162 sensor
 * @version 0.1
 * @date 2024-06-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __CDM7162_H__
#define __CDM7162_H__

#include "pico/stdlib.h"
#include "common/structs.h"

/**
 * @brief Reads data from cdm7162 sensor
 * 
 * @param addr register address to read from
 * @param buf output buffer
 * @param num_bytes number of bytes to read
 * @return int32_t error code
 */
extern int32_t cdm7162_read(uint8_t addr, uint8_t* buf, uint8_t num_bytes);

/**
 * @brief Writes data to cdm7162 sensor
 * 
 * @param addr Register address to write to
 * @param value Value to write
 * @return int32_t Return code
 */
extern int32_t cdm7162_write(uint8_t addr, uint8_t value);

/**
 * @brief Reads measured values from the sensor
 * 
 * @param cdm7162 CDM7162 sensor structure
 */
extern void cdm7162_get_value(sensor_t* cdm7162);

/**
 * @brief Initializes the cdm7162 sensor
 * 
 * @param cdm7162 output CDM7162 sensor structure
 * @param config Configuration of the CDM7162 sensor to be written
 * @return int32_t Return code
 */
extern int32_t cdm7162_init(sensor_t* cdm7162, sensor_config_t* config);

/**
 * @brief Reads CDM7162 sensor config
 * 
 * @param config CDM7162 config structure the read configuration will be saved to
 * @return int32_t Return code
 */
extern int32_t cdm7162_read_config(sensor_config_t* config);

/**
 * @brief Resets the sensor
 * 
 * @return int32_t Return code
 */
extern int32_t cdm7162_reset(void);

#endif