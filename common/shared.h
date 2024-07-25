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
extern soap_data_t soap_data1;
extern soap_data_t soap_data2;


extern config_data_t config_data;

extern bool service_mode;

extern message_channel* channels1[16];
extern uint8_t channels1_len;
extern message_channel* channels2[16];
extern uint8_t channels2_len;



#endif
