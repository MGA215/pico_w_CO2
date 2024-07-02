/**
 * @file sunrise.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Header file for communication with Senseair SUNRISE sensor
 * @version 0.1
 * @date 2024-06-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __SUNRISE_H__
#define __SUNRISE_H__

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "string.h"
#include "../error_codes.h"
#include "../common/functions.h"
#include "math.h"
#include "../common/constants.h"

/**
 * @brief Resets the sensor (soft reset)
 * 
 * @return int Return code
 */
extern int sunrise_reset(void);

/**
 * @brief Initializes SUNRISE sensor
 * 
 * @param sunrise Output SUNRISE sensor structure
 * @param config Configuration of the SUNRISE sensor to be written
 * @return int Return code
 */
extern int sunrise_init(sensor_t* sunrise, sensor_config_t* config);

/**
 * @brief Reads measured values from the sensor
 * 
 * @param sunrise SUNRISE sensor structure
 */
extern void sunrise_get_value(sensor_t* sunrise);

/**
 * @brief Reads SUNRISE sensor configuration
 * 
 * @param config SUNRISE config structure the read configuration will be saved to
 * @return int Return code
 */
extern int sunrise_read_config(sensor_config_t* config);

#endif