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

#ifndef SCD30_MODULE
#define SCD30_MODULE

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "../error_codes.h"
#include "../common/functions.h"
#include "string.h"
#include <math.h>
#include "../common/constants.h"

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