/**
 * @file tcp_server.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief TCP server implementation for setting up PICO & sensor configuration
 * @version 0.1
 * @date 2024-07-24
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef __TCP_SERVER_H__
#define __TCP_SERVER_H__

#include "pico/stdlib.h"
#include "lwip/err.h"

#define BUF_SIZE 4096 + 1024



/**
 * @brief Initializes TCP server structure
 * 
 * @return err_t Return error code
 */
extern err_t tcp_server_init();

/**
 * @brief Runs the TCP server to listen
 * 
 */
extern void tcp_server_run(void);









#endif