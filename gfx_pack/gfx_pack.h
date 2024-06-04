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
#include "hardware/spi.h"
#include "hardware/pwm.h"
#include "math.h"

#define GFX_PACK_BUTTON_A 12
#define GFX_PACK_BUTTON_B 13
#define GFX_PACK_BUTTON_C 14
#define GFX_PACK_BUTTON_D 15
#define GFX_PACK_BUTTON_E 22

/**
 * @brief Initializes the GFX Pack
 * 
 */
void gfx_pack_init(void);

/**
 * @brief Controls the brightness of display backlight
 * 
 * @param brightness Brightness of the backlight
 */
void gfx_pack_set_backlight(uint8_t brightness);

void command(uint8_t command, size_t len, const char* data);

/**
 * @brief Returns button status
 * 
 * @param GFX_PACK_BUTTON Button
 * @return true when specified button is pressed
 * @return false when specified button is not pressed
 */
bool gfx_pack_read_button(uint8_t GFX_PACK_BUTTON);

/**
 * @brief Resets the GFX Pack
 * 
 */
void gfx_pack_reset(void);