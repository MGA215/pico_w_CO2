/**
 * @file display.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Module for displaying stuff on display
 * @version 0.1
 * @date 2024-08-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#include "pico/stdlib.h"

/**
 * @brief Initializes the display structures
 * 
 */
void display_init();

/**
 * @brief Updates the display
 * 
 * @param force_update If update should be forced
 */
void display_update(bool force_update);




#endif