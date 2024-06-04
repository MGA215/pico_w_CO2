#include "hardware/i2c.h"
#include "pico/stdlib.h"


#include "gfx_pack.h"

#define GFX_PACK_BACKLIGHT_PIN 9

void gfx_pack_init_sequence(void);

void gfx_pack_init(void)
{
    gpio_init(GFX_PACK_BACKLIGHT_PIN);
    gpio_set_dir(GFX_PACK_BACKLIGHT_PIN, true);
    gfx_pack_init_sequence();
}

void gfx_pack_init_sequence(void)
{
    gpio_put(GFX_PACK_BACKLIGHT_PIN, 1);
    sleep_ms(300);
    gpio_put(GFX_PACK_BACKLIGHT_PIN, 0);
}

void gfx_pack_enable_backlight(bool enable)
{
    gpio_put(GFX_PACK_BACKLIGHT_PIN, enable);
}
