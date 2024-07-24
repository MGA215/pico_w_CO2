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

#define BUF_SIZE 4096 + 1024

bool data_recved;
uint8_t buffer_sent[BUF_SIZE];
uint8_t buffer_recv[BUF_SIZE];











#endif