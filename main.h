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
 * @brief Get current datetime string value
 * 
 * @param datetime_str string the datetime is written to
 * @param datetime_len length of the string
 */
void get_datetime(uint8_t* datetime_str, uint8_t datetime_len);

/**
 * @brief Converts datetime format to a string YYYY-MM-DD hh-mm-ss
 * 
 * @param buf output buffer containing the datetime string
 * @param buf_size max size of the buffer
 * @param dt datetime struct read from RTC module
 * @return int negative if conversion failed, otherwise number of written characters (except null byte)
 */
int ds3231_datetime2str(char *buf, uint8_t buf_size, const ds3231_datetime_t *dt);

/**
 * @brief Reads button inputs and reacts to them
 * 
 * @param rt timer structure
 * @return true if the timer should continue
 * @return false if the timer should stop
 */
bool read_inputs(repeating_timer_t *rt);

/**
 * @brief Updates the display
 * 
 * @param rt timer structure
 * @return true if the timer should continue
 * @return false if the timer should stop
 */
bool update_display();


