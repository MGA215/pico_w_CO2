/**
 * @file tcp_client.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief TCP client control
 * @version 0.1
 * @date 2024-07-09
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __TCP_CLIENT_H__
#define __TCP_CLIENT_H__

#include "pico/stdlib.h"

void tcp_client_init(void);

void run_tcp_client(uint8_t* data, uint16_t data_len, bool close_tcp);

#endif
