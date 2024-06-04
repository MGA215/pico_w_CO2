/**
 * @file gfx_pack.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Header file for a GFX Pack for Raspberry Pi Pico W
 * @version 0.1
 * @date 2024-06-04
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "hardware/i2c.h"
#include "pico/stdlib.h"

/**
 * @brief Initializes the GFX Pack
 * 
 */
void gfx_pack_init(void);

/**
 * @brief Switches on/off the backlight of the GFX Pack
 * 
 * @param enable If the backlight should be turned on or off
 */
void gfx_pack_enable_backlight(bool enable);