/**
 * @file wifi.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Main functionality of the second (wifi) core
 * @version 0.1
 * @date 2024-07-09
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __WIFI_H__
#define __WIFI_H__

#include "lwipopts.h"
#include "tcp_client.h"
#include "../common/functions.h"
#include "../common/constants.h"
#include "../common/structs.h"
#include "../soap/soap.h"
#include "pico/mutex.h"
#include "../soap/soap.h"


/**
 * @brief Wifi & network entry point
 * 
 * @param soap_1 
 * @param soap_2 
 */
void wifi_main(soap_data_t* soap_1, soap_data_t* soap_2);

#endif