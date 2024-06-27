#ifndef SUNRISE_MODULE
#define SUNRISE_MODULE

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "string.h"
#include "../error_codes.h"
#include "../common/functions.h"
#include "math.h"
#include "../common/constants.h"

/**
 * @brief Writes data to the SUNRISE sensor to specified address
 * 
 * @param addr Register address
 * @param buf Data to be sent
 * @param len Length of the data
 * @return int Return code
 */
int sunrise_write(uint8_t addr, uint8_t* buf, uint16_t len);

/**
 * @brief Reads data from the SUNRISE sensor
 * 
 * @param addr Register address to be read from
 * @param buf Data buffer
 * @param num_bytes Number of bytes to read
 * @return int Return code
 */
int sunrise_read(uint8_t addr, uint8_t* buf, uint16_t num_bytes);

/**
 * @brief Resets the sensor (soft reset)
 * 
 * @return int Return code
 */
int sunrise_reset(void);

/**
 * @brief Initializes SUNRISE sensor
 * 
 * @param sunrise Output SUNRISE sensor structure
 * @param config Configuration of the SUNRISE sensor to be written
 * @return int Return code
 */
int sunrise_init(sensor_t* sunrise, sensor_config_t* config);

/**
 * @brief Reads measured values from the sensor
 * 
 * @param sunrise SUNRISE sensor structure
 */
void sunrise_get_value(sensor_t* sunrise);

/**
 * @brief Reads SUNRISE sensor configuration
 * 
 * @param config SUNRISE config structure the read configuration will be saved to
 * @return int Return code
 */
int sunrise_read_config(sensor_config_t* config);

/**
 * @brief Switches sensor power [on] if not controlled globally
 * 
 * @param sunrise sensor structure
 * @param on if the power should be switched on (true) or off (false)
 */
void sunrise_power(sensor_t* sunrise, bool on);



#endif