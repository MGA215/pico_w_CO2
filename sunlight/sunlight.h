#ifndef SUNLIGHT_MODULE
#define SUNLIGHT_MODULE

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "string.h"
#include "../error_codes.h"
#include "../common/functions.h"
#include "math.h"
#include "../common/constants.h"

/**
 * @brief Writes data to the SUNLIGHT sensor to specified address
 * 
 * @param addr Register address
 * @param buf Data to be sent
 * @param len Length of the data
 * @return int Return code
 */
int sunlight_write(uint8_t addr, uint8_t* buf, uint16_t len);

/**
 * @brief Reads data from the SUNLIGHT sensor
 * 
 * @param addr Register address to be read from
 * @param buf Data buffer
 * @param num_bytes Number of bytes to read
 * @return int Return code
 */
int sunlight_read(uint8_t addr, uint8_t* buf, uint16_t num_bytes);

/**
 * @brief Resets the sensor (soft reset)
 * 
 * @return int Return code
 */
int sunlight_reset(void);

/**
 * @brief Initializes SUNLIGHT sensor
 * 
 * @param sunlight Output SUNLIGHT sensor structure
 * @param config Configuration of the SUNLIGHT sensor to be written
 * @return int Return code
 */
int sunlight_init(sensor_t* sunlight, sensor_config_t* config);

/**
 * @brief Reads measured values from the sensor
 * 
 * @param sunlight SUNLIGHT sensor structure
 */
void sunlight_get_value(sensor_t* sunlight);

/**
 * @brief Reads SUNLIGHT sensor configuration
 * 
 * @param config SUNLIGHT config structure the read configuration will be saved to
 * @return int Return code
 */
int sunlight_read_config(sensor_config_t* config);

/**
 * @brief Switches sensor power [on] if not controlled globally
 * 
 * @param sunlight Sensor structure
 * @param on if the power should be switched on (true) or off (false)
 */
void sunlight_power(sensor_t* sunlight, bool on);

#endif