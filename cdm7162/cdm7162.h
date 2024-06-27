#ifndef CDM7162_MODULE
#define CDM7162_MODULE
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "error_codes.h"
#include "common/functions.h"
#include "math.h"
#include "common/constants.h"

/**
 * @brief Reads data from cdm7162 sensor
 * 
 * @param addr register address to read from
 * @param buf output buffer
 * @param num_bytes number of bytes to read
 * @return int32_t error code
 */
int32_t cdm7162_read(uint8_t addr, uint8_t* buf, uint8_t num_bytes);

/**
 * @brief Writes data to cdm7162 sensor
 * 
 * @param addr Register address to write to
 * @param value Value to write
 * @return int32_t Return code
 */
int32_t cdm7162_write(uint8_t addr, uint8_t value);

/**
 * @brief Resets the sensor
 * 
 * @return int32_t Return code
 */
int32_t cdm7162_reset(void);

/**
 * @brief Initializes the cdm7162 sensor
 * 
 * @param cdm7162 output CDM7162 sensor structure
 * @param config Configuration of the CDM7162 sensor to be written
 * @return int32_t Return code
 */
int32_t cdm7162_init(sensor_t* cdm7162, sensor_config_t* config);

/**
 * @brief Sets atmospheric pressure for pressure correction
 * 
 * @param pressure Pressure in hPa
 * @return int32_t Return code
 */
int32_t cdm7162_set_atm_pressure(uint16_t pressure);

/**
 * @brief Sets default atmospheric pressure 1013 hPa
 * 
 * @return int32_t Return code
 */
int32_t cdm7162_set_default_atm_pressure(void);

/**
 * @brief Reads measured values from the sensor
 * 
 * @param cdm7162 CDM7162 sensor structure
 */
void cdm7162_get_value(sensor_t* cdm7162);

/**
 * @brief Reads CDM7162 sensor config
 * 
 * @param config CDM7162 config structure the read configuration will be saved to
 * @return int32_t Return code
 */
int32_t cdm7162_read_config(sensor_config_t* config);

/**
 * @brief Switches sensor power [on] if not controlled globally
 * 
 * @param cdm7162 sensor structure
 * @param on if the power should be switched on (true) or off (false)
 */
void cdm7162_power(sensor_t* cdm7162, bool on);


#endif