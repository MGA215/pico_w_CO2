#ifndef SCD30_MODULE
#define SCD30_MODULE

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "../error_codes.h"
#include "../common/functions.h"
#include "string.h"
#include <math.h>
#include "../common/constants.h"

/**
 * @brief Reads data from the SCD30 sensor
 * 
 * @param command Command to execute (get specific data)
 * @param buf Buffer to save read data
 * @param len Length of the buffer
 * @return int32_t Return code
 */
int32_t scd30_read(uint16_t command, uint16_t* buf, uint32_t len);

/**
 * @brief Writes a value to the SCD30 sensor
 * 
 * @param command Command to write specific data
 * @param value Data to write
 * @return int32_t Return code
 */
int32_t scd30_write_value(uint16_t command, uint16_t value);

/**
 * @brief Executes a command on the SCD30 sensor
 * 
 * @param command Command to send
 * @return int32_t Return code
 */
int32_t scd30_write_command(uint16_t command);

/**
 * @brief Retrieves measured values from the SCD30 sensor
 * 
 * @param scd30 Sensor structure; return code saved to the state variable
 */
void scd30_get_value(sensor_t* scd30);

/**
 * @brief Turns the sensor power [on]
 * 
 * @param scd30 Sensor structure
 * @param on Whether the power should be turned on
 */
void scd30_power(sensor_t* scd30, bool on);

/**
 * @brief Initializes the sensor and writes configuration to it
 * 
 * @param scd30 Sensor structure
 * @param config Configuration to write
 * @return int32_t Return code
 */
int32_t scd30_init(sensor_t* scd30, sensor_config_t* config);

/**
 * @brief Reads configuration from the SCD30 sensor
 * 
 * @param config Config structure the read configuration will be saved to
 * @return int32_t Return code
 */
int32_t scd30_read_config(sensor_config_t* config);

#endif