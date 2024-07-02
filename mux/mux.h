/**
 * @file mux.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Library for mux control
 * @version 0.1
 * @date 2024-07-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __MUX_H__
#define __MUX_H__

#include "pico/stdlib.h"
#include "error_codes.h"
#include "common/constants.h"
#include "hardware/i2c.h"

/**
 * @brief Enables sensor in the MUX
 * 
 * @param sensor_index Index of the sensor
 * @return int32_t Return code
 */
int32_t mux_enable_sensor(uint8_t sensor_index);



#endif