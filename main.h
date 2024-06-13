#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "error_codes.h"
#include <stdlib.h>
#include <string.h>

#include "ds3231/include/ds3231.h"
#include "gfx_pack/gfx_pack.h"
#include "ee895/ee895.h"
#include "cdm7162/cdm7162.h"
#include "sunrise/sunrise.h"
#include "sunlight/sunlight.h"

#include "sensor_config.h"

typedef struct sensors
{
    ee895_t ee895;
    cdm7162_t cdm7162;
    sunrise_t sunrise;
    sunlight_t sunlight;
} sensors_t;

/**
 * @brief Initializes pins, buses, modules; runs only once
 * 
 * @return int Value describing encountered error during initialization
 */
int init(void);

/**
 * @brief Initializes I2C for sensor communication
 * 
 */
void init_sensor_i2c(void);

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
 */
void read_inputs();

/**
 * @brief Updates the display
 * 
 */
void update_display();

/**
 * @brief Updates datetime from RTC
 * 
 */
void update_RTC();

/**
 * @brief Updates all components
 * 
 */
void update();
//bool update(repeating_timer_t* rt);

/**
 * @brief Initializes the sensors structure
 * 
 */
void init_sensors(void);

/**
 * @brief Prepares data to be displayed, writes data to the diaplay framebuffer
 * 
 */
void write_display(void);

/**
 * @brief Reads data from sensors - timer callback
 * 
 * @param rt Timer structure
 * @return true if the timer should continue
 */
void read_sensors();

/**
 * @brief Displays information from sensor
 * 
 * @param sensor_name String containing name of sensor
 * @param state State of sensor
 * @param co2 Should CO2 be displayed
 * @param co2_value CO2 value
 * @param temp Should temperature be displayed
 * @param temp_value Temperature value
 * @param pressure Should pressure be displayed
 * @param pressure_value Pressure value
 */
void write_display_sensor(uint8_t* sensor_name, int state, 
        bool co2, float co2_value, 
        bool temp, float temp_value, 
        bool pressure, float pressure_value);


void sensor_timer_vector_update(void);

void read_sensors_start();