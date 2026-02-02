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

#include "pico/stdlib.h"

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


extern void uart_sensor_send(uint8_t* data, uint8_t data_len);

extern void uart_sensor_init(void);

extern uint8_t uart_sensor_recv(uint8_t* data, uint8_t max_data_len);

extern void uart_sensor_empty_buffer(void);

#endif