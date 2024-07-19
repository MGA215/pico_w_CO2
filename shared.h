/**
 * @file shared.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Shared resources
 * @version 0.1
 * @date 2024-07-19
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __SHARED_H__
#define __SHARED_H__

#include "common/structs.h"
#include "pico/stdlib.h"

extern sensor_t sensors[8];
// extern soap_data_t soap_data1;
// extern soap_data_t soap_data2;

extern sensor_config_mutex_t sensors_config_all[8]; // for TCP server to copy from/to, for now this is stored under wifi module


#endif
