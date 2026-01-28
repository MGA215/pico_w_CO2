/**
 * @file http.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Defines functions for the HTTP protocol
 * @version 0.1
 * @date 2024-07-09
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __HTTP_H__
#define __HTTP_H__

#include "pico/mutex.h"
/**
 * @brief Create a http header for a SOAP message
 * 
 * @param ip_url_addr IP or URL address
 * @param is_url true if is URL address
 * @param soap_http_addr HTTP address
 * @param soap_action SOAP action (http://tempuri.org/InsertMSxSample)
 * @param data Data to send
 * @param data_len Length of the data to send
 * @param out_buf Output buffer the message will be saved to
 * @param out_buf_size Size of the output buffer
 */
void create_http_header(uint8_t* ip_url_addr, bool is_cloud, uint8_t* path, uint16_t port, uint8_t* soap_action, uint8_t* data, uint16_t data_len, mutex_t* data_mutex, uint8_t* out_buf, uint16_t out_buf_size);

#endif