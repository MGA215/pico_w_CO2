/**
 * @file ee895.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Header file for communication with E+E EE895 sensor
 * @version 0.1
 * @date 2024-06-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __EE872_H__
#define __EE872_H__

#include "pico/stdlib.h"
#include "common/structs.h"


/* EE872 via UART */

extern void ee872_get_value(sensor_t* sensor);
extern void ee872_init(sensor_t* sensor);

extern sensor_functions_t ee872_functions_uart;

#endif