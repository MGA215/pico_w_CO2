/**
 * @file sensor_config.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Sensors configuration
 * @version 0.1
 * @date 2024-06-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __SENSOR_CONFIG_H__
#define __SENSOR_CONFIG_H__

#include "common/structs.h"

static sensor_config_t sensor_config_default = {
    .sensor_type = UNKNOWN,
    .co2_en = false,
    .temp_en = false, 
    .RH_en = false,
    .pressure_en = false,
    .power_5V = false,
    .power_global_control = false,
    .sensor_active = false,
};
#endif