/**
 * @file sensors.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Control module for sensor communication & data
 * @version 0.1
 * @date 2024-07-18
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __SENSORS_H__
#define __SENSORS_H__

#include "pico/stdlib.h"
#include "../common/structs.h"

// Sensors structure
extern sensor_t sensors[8];

// Measurement interval
extern uint32_t sensor_measurement_interval_s;

/**
 * @brief Initializes sensors up to min(config_map_length, 8)
 * 
 * @param configuration_map Map of configurations for the sensors
 * @param config_map_length Length of the map, should be 8
 */
extern void sensors_init_all(sensor_config_t** configuration_map, uint8_t config_map_length);

/**
 * @brief Initializes single sensor
 * 
 * @param sensor_index Index of the sensor
 * @param configuration Configuration for the sensor
 * @param is_first_init true if first init, false if reinit
 * @return true if init swas successful
 * @return false if init failed
 */
extern bool sensors_init(uint8_t sensor_index, sensor_config_t* configuration, bool is_first_init);

/**
 * @brief Whether measurement has finished
 * 
 * @return true if measurement finished
 * @return false if measurement not finished
 */
extern bool sensors_is_measurement_finished(void);

/**
 * @brief Tries to read sensors, should be called in a loop
 * 
 */
extern void sensors_read_sensors(void);

#endif