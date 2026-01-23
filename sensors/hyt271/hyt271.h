/**
 * @file hyt271.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Module for communication with the HYT271 sensor
 * @version 0.1
 * @date 2024-08-02
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __HYT271_H__
#define __HYT271_H__

#include "common/structs.h"

/**
 * @brief Reads values from the HYT271 sensor
 * 
 */
extern void hyt271_get_value(sensor_t* sensor);

extern sensor_functions_t hyt271_functions;

#endif