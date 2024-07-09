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

static uint16_t header_size = 512;
#include <string.h>
#include <stdio.h>
#include <malloc.h>

#define MAX_SOAP_HTTP_ADDR_LEN 64
#define MAX_SOAP_ACTION_LEN 64
#define MAX_URL_LEN 64

uint8_t* create_http_header(uint8_t* ip_url_addr, bool is_url, uint8_t* soap_http_addr, uint8_t* soap_action, uint8_t* data, uint16_t data_len)
{
    uint8_t header[header_size];
    uint16_t index = 0;
    memset(header, 0x00, header_size);

    if (strlen(soap_http_addr) > MAX_SOAP_HTTP_ADDR_LEN || strlen(soap_action) > MAX_SOAP_ACTION_LEN || is_url && strlen(ip_url_addr) > MAX_URL_LEN) return NULL;
    snprintf(header, header_size, "POST %s%s HTTP/1.1\r\n\r\nHost: %s\r\nContent-Type: text/xml; charset=utf-8\r\nContent-Length: %u\r\nSOAPAction: \"%s\"\r\n\r\n",
        is_url ? "" : "http://", ip_url_addr, soap_http_addr, data_len, soap_action);

    uint8_t* message = malloc(strlen(header) + data_len + 1 * sizeof(uint8_t));
    strncpy(message, header, strlen(header));
    strncpy(&message[strlen(header)], data, data_len);
    return message;
}   

#endif