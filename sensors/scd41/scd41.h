/**
 * @file scd41.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Header file for communication with Sensirion SCD41 sensor
 * @version 0.1
 * @date 2024-06-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __SCD41_H__
#define __SCD41_H__

#include "../../common/common_include.h"

/**
 * @brief Retrieves measured values from the SCD41 sensor
 * 
 * @param scd41 sensor structure; return code saved to the state variable
 */
extern void scd41_get_value(sensor_t* scd41);

/**
 * @brief Initializes the sensor and writes configuration to it
 * 
 * @param scd41 Sensor structure
 * @param config Configuration to write
 * @return int32_t Return code
 */
extern int32_t scd41_init(sensor_t* scd41, sensor_config_t* config);

/**
 * @brief Reads configuration from the SCD41 sensor
 * 
 * @param config Config structure the read configuration will be saved to
 * @return int32_t Return code
 */
extern int32_t scd41_read_config(sensor_config_t* config, bool single_meas_mode);

/**
 * @brief Soft resets the sensor
 * 
 */
extern void scd41_reset(void);

#endif