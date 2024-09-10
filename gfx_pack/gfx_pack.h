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

#ifndef __GFX_PACK_H__
#define __GFX_PACK_H__

#include "pico/stdlib.h"

typedef struct {
    uint8_t x;
    uint8_t y;
} point_t;

/**
 * @brief Initializes the GFX Pack
 * @param brightness Brightness of the display backlight
 * 
 */
extern void gfx_pack_init(uint8_t brightness);

/**
 * @brief Controls the brightness of display backlight
 * 
 * @param brightness Brightness of the backlight
 */
extern void gfx_pack_set_backlight(uint8_t brightness);

/**
 * @brief Returns button status
 * 
 * @param GFX_PACK_BUTTON Button
 * @return true when specified button is pressed
 * @return false when specified button is not pressed
 */
extern bool gfx_pack_read_button(uint8_t GFX_PACK_BUTTON);

/**
 * @brief Resets the GFX Pack
 * 
 */
extern void gfx_pack_reset(void);

/**
 * @brief Updates the display pixels
 * 
 */
extern void gfx_pack_update(void);

/**
 * @brief Writes a character on the display
 * 
 * @param position Position the character should be written at
 * @param c Character to be written
 * @return true if character successfully written
 * @return false if character outside of the display
 */
extern bool gfx_pack_write_char(point_t* position, char c);

/**
 * @brief Writes text on the display
 * 
 * @param position Position of the first letter of the text
 * @param text text to be written
 * @return true if text successfully written
 * @return false if part of the text could not be written
 */
extern bool gfx_pack_write_text(point_t* position, char* text);

/**
 * @brief Clears the display
 * 
 */
extern void gfx_pack_clear_display(void);

/**
 * @brief Inverts colors in a row
 * 
 * @param row Row to invert the colors in
 */
extern void gfx_pack_invert_row_color(uint8_t row);

#endif