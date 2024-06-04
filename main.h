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
 * @brief Converts datetime format to a string YYYY-MM-DD hh-mm-ss
 * 
 * @param buf output buffer containing the datetime string
 * @param buf_size max size of the buffer
 * @param dt datetime struct read from RTC module
 * @return int negative if conversion failed, otherwise number of written characters (except null byte)
 */
int ds3231_datetime2str(char *buf, uint8_t buf_size, const ds3231_datetime_t *dt);