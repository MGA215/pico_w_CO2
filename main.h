#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/divider.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"

#define DS3231_I2C_PORT i2c1
#define DS3231_I2C_SDA_PIN 6
#define DS3231_I2C_SCL_PIN 7

#include "blink.pio.h"
#include "ds3231/include/ds3231.h"
#include "gfx_pack/gfx_pack.h"



int init(void);

int loop(void);

int ds3231_datetime2str(char *buf, uint8_t buf_size, const ds3231_datetime_t *dt);