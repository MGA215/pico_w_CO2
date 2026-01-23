/**
 * @file sunlight.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Header file for communication with Senseair SUNLIGHT sensor
 * @version 0.1
 * @date 2024-06-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __SUNLIGHT_H__
#define __SUNLIGHT_H__

#include "pico/stdlib.h"
#include "common/structs.h"

/**
 * @brief Reads data from the SUNLIGHT sensor
 * 
 * @param addr Register address to be read from
 * @param buf Data buffer
 * @param num_bytes Number of bytes to read
 * @return int Return code
 */
extern int32_t sunlight_read(uint8_t addr, uint8_t* buf, uint16_t num_bytes);

/**
 * @brief Writes data to the SUNLIGHT sensor to specified address
 * 
 * @param addr Register address
 * @param buf Data to be sent
 * @param len Length of the data
 * @return int Return code
 */
extern int32_t sunlight_write(uint8_t addr, uint8_t* buf, uint16_t len);

/**
 * @brief Reads measured values from the sensor
 * 
 * @param sunlight SUNLIGHT sensor structure
 */
extern void sunlight_get_value(sensor_t* sunlight);

/**
 * @brief Initializes SUNLIGHT sensor
 * 
 * @param sensor SUNLIGHT sensor structure
 */
extern void sunlight_init(sensor_t* sensor);

/**
 * @brief Reads SUNLIGHT sensor configuration
 * 
 * @param config SUNLIGHT config structure the read configuration will be saved to
 * @param single_measurement_mode unused variable
 * @return int Return code
 */
extern int32_t sunlight_read_config(sensor_config_t* config, bool single_measurement_mode);

/**
 * @brief Resets the sensor (soft reset)
 * 
 * @return int Return code
 */
extern int32_t sunlight_reset(void);

extern sensor_functions_t sunlight_functions;

#endif