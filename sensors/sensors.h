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

sensor_t sensors[8] = {NULL};


extern void sensors_init_all(sensor_config_t** configuration_map, uint8_t config_map_length);


extern void sensors_init(uint8_t sensor_index, sensor_config_t* configuration, bool is_first_init);


extern void sensors_read_all(void);


extern void sensors_read(uint8_t sensor_index);

extern void sensors_start_measurement(void);





#endif