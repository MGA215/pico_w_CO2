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

typedef struct {
    uint8_t x;
    uint8_t y;
} point_t;

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

/**
 * @brief Updates the display pixels
 * 
 */
void gfx_pack_update(void);

/**
 * @brief Writes a character on the display
 * 
 * @param position Position the character should be written at
 * @param c Character to be written
 * @return true if character successfully written
 * @return false if character outside of the display
 */
bool gfx_pack_write_char(point_t* position, char c);

/**
 * @brief Writes text on the display
 * 
 * @param position Position of the first letter of the text
 * @param text text to be written
 * @param text_len length of the text to write
 * @return true if text successfully written
 * @return false if part of the text could not be written
 */
bool gfx_pack_write_text(point_t* position, char* text, uint8_t text_len);