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

#include "../common/structs.h"


/**
 * @brief Wifi & network entry point
 * 
 * @param soap_1 
 * @param soap_2 
 */
extern void wifi_main(soap_data_t* soap_1, soap_data_t* soap_2);

#endif