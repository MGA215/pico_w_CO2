/**
 * @file http.c
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Defines function for creating the HTTP header
 * @version 0.1
 * @date 2024-07-09
 * 
 * @copyright Copyright (c) 2024
 * 
 */


#ifndef __HTTP_C__
#define __HTTP_C__

#include "http.h"
#include "../common_include.h"
#include <string.h>
#include "pico/stdio.h"
#include "pico/printf.h"
#include <malloc.h>

static uint16_t header_size = 512;

#define MAX_SOAP_HTTP_ADDR_LEN 64
#define MAX_SOAP_ACTION_LEN 64
#define MAX_URL_LEN 64

uint8_t* create_http_header(uint8_t* ip_url_addr, bool is_url, uint8_t* path, uint16_t port, uint8_t* soap_action, uint8_t* data, uint16_t data_len, mutex_t* data_mutex)
{
    uint8_t header[header_size]; // Create header buffer
    memset(header, 0x00, header_size);

    if (strlen(soap_action) > MAX_SOAP_ACTION_LEN || is_url && strlen(ip_url_addr) > MAX_URL_LEN) return NULL; // Check max lengths
    snprintf(header, header_size, "POST "/*%s%s*/"%s HTTP/1.1\r\nHost: %s:%u\r\nContent-Type: text/xml; charset=utf-8\r\nContent-Length: %u\r\nSOAPAction: \"%s\"\r\n\r\n",
        /*is_url ? "" : "http://", ip_url_addr, */path, ip_url_addr, port, data_len, soap_action); // Create header

    uint8_t* message = malloc(strlen(header) + data_len + 1 * sizeof(uint8_t)); // allocate message buffer
    strncpy(message, header, strlen(header)); // Copy header
    mutex_enter_timeout_ms(data_mutex, MUTEX_TIMEOUT_MS); // Safe copy SOAP message as body
    strncpy(&message[strlen(header)], data, data_len);
    mutex_exit(data_mutex);
    return message;
}   

#endif