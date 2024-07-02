/**
 * @file cm1107n.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief 
 * @version 0.1
 * @date 2024-07-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef __CM1107N_H__
#define __CM1107N_H__

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "string.h"
#include "../error_codes.h"
#include "../common/functions.h"
#include "math.h"
#include "../common/constants.h"

/**
 * @brief Retrieves measured values from the CM1107N sensor
 * 
 * @param cm1107n Sensor structure; return code saved to the state variable
 */
extern void cm1107n_get_value(sensor_t* cm1107n);

/**
 * @brief Initializes the CM1107N sensor and writes configuration to it
 * 
 * @param cm1107n Sensor structure
 * @param config Configuration to write
 * @return int32_t Return code
 */
extern int32_t cm1107n_init(sensor_t* cm1107n, sensor_config_t* config);

/**
 * @brief Reads configuration from the CM1107N sensor
 * 
 * @param config Configuration structure the read configuration will be saved to
 * @return int32_t Return code
 */
extern int32_t cm1107n_read_config(sensor_config_t* config);

#endif
