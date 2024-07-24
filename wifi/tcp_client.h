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
#include "pico/mutex.h"
#include "lwip/err.h"



/**
 * @brief Initializes the TCP structures and sets IP address
 * 
 * @return err_t Error code
 */
err_t tcp_client_init(void);

/**
 * @brief Runs the TCP client, opens a socket, sends data and closes the socket
 * 
 * @param close_tcp If TCP socket should be closed after send
 * @return true if the client should be run immediately again
 * @return false if connection was closed or communication ended
 */
bool run_tcp_client(bool close_tcp);

#endif
