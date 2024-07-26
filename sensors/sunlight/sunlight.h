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

#include "../../common/common_include.h"

/**
 * @brief Reads data from the SUNLIGHT sensor
 * 
 * @param addr Register address to be read from
 * @param buf Data buffer
 * @param num_bytes Number of bytes to read
 * @return int Return code
 */
extern int sunlight_read(uint8_t addr, uint8_t* buf, uint16_t num_bytes);

/**
 * @brief Writes data to the SUNLIGHT sensor to specified address
 * 
 * @param addr Register address
 * @param buf Data to be sent
 * @param len Length of the data
 * @return int Return code
 */
extern int sunlight_write(uint8_t addr, uint8_t* buf, uint16_t len);

/**
 * @brief Reads measured values from the sensor
 * 
 * @param sunlight SUNLIGHT sensor structure
 */
extern void sunlight_get_value(sensor_t* sunlight);

/**
 * @brief Initializes SUNLIGHT sensor
 * 
 * @param sunlight Output SUNLIGHT sensor structure
 * @param config Configuration of the SUNLIGHT sensor to be written
 * @return int Return code
 */
extern int sunlight_init(sensor_t* sunlight, sensor_config_t* config);

/**
 * @brief Reads SUNLIGHT sensor configuration
 * 
 * @param config SUNLIGHT config structure the read configuration will be saved to
 * @return int Return code
 */
extern int sunlight_read_config(sensor_config_t* config);

/**
 * @brief Resets the sensor (soft reset)
 * 
 * @return int Return code
 */
extern int sunlight_reset(void);

#endif