/**
 * @file uart.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief UART service communication processing
 * @version 0.1
 * @date 2024-08-19
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __UART_H__
#define __UART_H__

/**
 * @brief Initializes service comm uart
 * 
 */
extern void uart_service_init(void);

/**
 * @brief Reads service command from UART
 * 
 */
extern void uart_service_read_command(void);

/**
 * @brief Sends service response via UART
 * 
 */
extern void uart_service_send_response(void);

#endif