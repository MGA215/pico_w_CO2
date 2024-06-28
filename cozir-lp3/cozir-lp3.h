/**
 * @file cozir-lp3.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Header file for communication with GSS CozIR-LP3 sensor
 * @version 0.1
 * @date 2024-06-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef COZIR_LP3_MODULE
#define COZIR_LP3_MODULE

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "../error_codes.h"
#include "../common/functions.h"
#include "string.h"
#include <math.h>
#include "../common/constants.h"

/**
 * @brief Retrieves measured values from the CozIR-LP3 sensor
 * 
 * @param cozir_lp3 Sensor structure; return code saved to the state variable
 */
extern void cozir_lp3_get_value(sensor_t* cozir_lp3);

/**
 * @brief Initializes the CozIR-LP3 sensor and writes configuration to it
 * 
 * @param cozir_lp3 Sensor structure
 * @param config Configuration to write
 * @return int32_t Return code
 */
extern int32_t cozir_lp3_init(sensor_t* cozir_lp3, sensor_config_t* config);

/**
 * @brief Reads configuration from the CozIR-LP3 sensor
 * 
 * @param config Configuration structure the read configuration will be saved to
 * @return int32_t Return code
 */
extern int32_t cozir_lp3_read_config(sensor_config_t* config);

#endif