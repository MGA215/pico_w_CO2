/**
 * @file power.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Power control
 * @version 0.1
 * @date 2024-07-22
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __POWER_H__
#define __POWER_H__

#include "pico/stdlib.h"

/**
 * @brief Sets power to several sensors at once
 * 
 * @param affected_sensors_vector Vector of sensors that should be affected
 * @param on turn power [on]
 * @return int32_t Return code
 */
extern int32_t power_en_set_vector(uint8_t affected_sensors_vector, bool on);

/**
 * @brief Sets power to single sensor
 * 
 * @param sensor_index Index of sensor to affect
 * @param on turn power [on]
 * @return int32_t Return code
 */
extern int32_t power_en_set_index(uint8_t sensor_index, bool on);

/**
 * @brief Sets 5V power to the vector of sensors (according to sensor power indices)
 * 
 * @param power_vector vector of 5V enabled sensors
 * @return int32_t Return code
 */
extern int32_t power_5v_set_vector(uint8_t power_vector);

/**
 * @brief Turns off power and sets 5V to disable on all sensors
 * 
 */
extern void power_reset_all(void);

#endif