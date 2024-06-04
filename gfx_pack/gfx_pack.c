#include "hardware/i2c.h"
#include "pico/stdlib.h"


#include "gfx_pack.h"

#define GFX_PACK_BACKLIGHT_PIN 9

/**
 * @brief Function to perform the initialization sequence
 * 
 */
void gfx_pack_init_sequence(void);

void gfx_pack_init(void)
{
    gpio_init(GFX_PACK_BACKLIGHT_PIN); // Initialize GPIO pin for the backlight
    gpio_set_dir(GFX_PACK_BACKLIGHT_PIN, true); // Set out direction for the GPIO pin for the backlight
    gfx_pack_init_sequence(); // Perform initialization sequence
}

void gfx_pack_init_sequence(void)
{
    gpio_put(GFX_PACK_BACKLIGHT_PIN, 1); // Turn backlight on
    sleep_ms(300);
    gpio_put(GFX_PACK_BACKLIGHT_PIN, 0); // Turn backlight off
}

void gfx_pack_enable_backlight(bool enable)
{
    gpio_put(GFX_PACK_BACKLIGHT_PIN, enable); // Switch on/off the backlight
}
