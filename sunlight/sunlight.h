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

#ifndef SUNLIGHT_MODULE
#define SUNLIGHT_MODULE

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "string.h"
#include "../error_codes.h"
#include "../common/functions.h"
#include "math.h"
#include "../common/constants.h"


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