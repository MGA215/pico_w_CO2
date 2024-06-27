#ifndef SCD41_MODULE
#define SCD41_MODULE

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "../error_codes.h"
#include "../common/functions.h"
#include "string.h"
#include <math.h>
#include "../common/constants.h"

/**
 * @brief Reads data from the SCD41 sensor
 * 
 * @param command Command to execute (get specific data)
 * @param buf Buffer to save read data
 * @param len Length of the buffer
 * @return int32_t Return code
 */
int32_t scd41_read(uint16_t command, uint16_t* buf, uint32_t len);

/**
 * @brief Writes a value to the SCD41 sensor
 * 
 * @param command Command to write specific data
 * @param value Data to write
 * @return int32_t Return code
 */
int32_t scd41_write_value(uint16_t command, uint16_t value);

/**
 * @brief Executes a command on the SCD41 sensor
 * 
 * @param command Command to send
 * @return int32_t Return code
 */
int32_t scd41_write_command(uint16_t command);

/**
 * @brief Retrieves measured values from the SCD41 sensor
 * 
 * @param scd41 sensor structure; return code saved to the state variable
 */
void scd41_get_value(sensor_t* scd41);

/**
 * @brief Turns the sensor power [on]
 * 
 * @param scd41 Sensor structure
 * @param on Whether the power should be turned on
 */
void scd41_power(sensor_t* scd41, bool on);

/**
 * @brief Initializes the sensor and writes configuration to it
 * 
 * @param scd41 Sensor structure
 * @param config Configuration to write
 * @return int32_t Return code
 */
int32_t scd41_init(sensor_t* scd41, sensor_config_t* config);

/**
 * @brief Reads configuration from the SCD41 sensor
 * 
 * @param config Config structure the read configuration will be saved to
 * @return int32_t Return code
 */
int32_t scd41_read_config(sensor_config_t* config, bool single_meas_mode);

/**
 * @brief Soft resets the sensor
 * 
 */
void scd41_reset(void);

#endif