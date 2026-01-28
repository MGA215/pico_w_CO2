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

// Last message timestamp
extern uint8_t last_message_time[32];

// Last error code
extern uint8_t last_message_error;



/**
 * @brief Initializes the TCP structures and sets IP address
 * 
 * @param retry_send Pointer to bool if message should be tried to be sent again
 * @return err_t Error code
 */
err_t tcp_client_init(bool* retry_send);

/**
 * @brief Runs the TCP client state machine
 * 
 */
void tcp_state_machine(void);

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

/**
 * @brief Instructs the TCP client to send messages
 * 
 */
extern void tcp_run_client(void);

#endif
