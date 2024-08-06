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

#include "structs.h"
#include "pico/stdlib.h"

extern sensor_t sensors[8];
extern ms5607_t ms5607;
extern hyt271_t hyt271;

extern soap_data_t soap_data[2];

extern service_comm_data_t config_data;
extern bool service_mode;

extern message_channel* channels1[16];
extern uint8_t channels1_len;
extern message_channel* channels2[16];
extern uint8_t channels2_len;

extern datetime_t datetime;
extern uint8_t datetime_str[30];

#endif
