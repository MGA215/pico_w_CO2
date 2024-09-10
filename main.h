/**
 * @file main.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief PICO and peripherals control
 * @version 0.1
 * @date 2024-06-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __MAIN_H__
#define __MAIN_H__

#include "pico/stdlib.h"
#include "common/structs.h"


/**
 * @brief Main function of core 1
 * 
 */
void core1_main(void);

/**
 * @brief Initializes pins, buses, modules; runs only once
 * 
 * @return int Value describing encountered error during initialization
 */
int init(void);

/**
 * @brief Main program loop
 * 
 * @return int Value describing encountered error during execution
 */
int loop(void);

/**
 * @brief Updates all components
 * 
 */
void update();

/**
 * @brief Creates SOAP messages and saves them to their buffers
 * 
 */
void create_soap_messages(void);

/**
 * @brief Checks for service mode pin change
 * 
 */
void check_svc_mode(void);

/**
 * @brief Initializes the service mode pin
 * 
 */
void svc_pin_init(void);




#endif