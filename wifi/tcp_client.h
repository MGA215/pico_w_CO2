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
#include "lwip/err.h"



/**
 * @brief Initializes the TCP structures and sets IP address
 * 
 * @param retry_send Pointer to bool if message should be tried to be sent again
 * @return err_t Error code
 */
err_t tcp_client_init(bool* retry_send);

/**
 * @brief Runs the TCP client, opens a socket, sends data and closes the socket
 * 
 * @param soap_index Index of message that should be sent
 * @return true if the client should be run immediately again
 * @return false if connection was closed or communication ended
 */
bool run_tcp_client(uint8_t soap_index);

/**
 * @brief Stops the TCP client
 * 
 */
extern void tcp_client_stop(void);

/**
 * @brief Checks whether TCP client is running
 * 
 * @return true if client is running
 * @return false if client has stopped
 */
extern bool tcp_client_is_running(void);

#endif
