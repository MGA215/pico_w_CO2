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

/**
 * @brief Stops the TCP server if currently running
 * 
 */
extern void tcp_server_stop(void);

/**
 * @brief Checks whether TCP server is running
 * 
 * @return uint8_t value representing server state
 */
extern uint8_t tcp_server_is_running(void);







#endif