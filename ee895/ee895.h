#ifndef EE895_MODULE
#define EE895_MODULE

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "string.h"
#include "error_codes.h"
#include "math.h"
#include "common/functions.h"
#include "../common/constants.h"

/**
 * @brief Reads number of registers from the EE895
 * 
 * @param addr Address of the register to be read from
 * @param nreg Number of registers to read
 * @param buf Output buffer of values
 * @return int32_t Return code
 */
int32_t ee895_read(uint16_t addr, uint16_t nreg, uint8_t* buf);

/**
 * @brief Writes a value to the EE895
 * 
 * @param addr Address of the register to be written to
 * @param value Value to be written
 * @return int32_t Return code
 */
int32_t ee895_write(uint16_t addr, uint16_t value);

/**
 * @brief Gets CO2, temperature and pressure from the sensor
 * 
 * @param ee895 Sensor structure
 */
void ee895_get_value(sensor_t* ee895);

/**
 * @brief Reads number of registers from the EE895 with timing; must manually turn power on/off
 * 
 * @param addr Address of the register to be read from
 * @param nreg Number of registers to read
 * @param buf Output buffer of values
 * @return int32_t Return code
 */
int32_t ee895_read_reg(uint16_t addr, uint16_t nreg, uint8_t* buf);

/**
 * @brief Writes a value to the EE895 with timing; must manually turn power on/off
 * 
 * @param addr Address of the register to be written to
 * @param value Value to be written
 * @return int32_t Return code
 */
int32_t ee895_write_reg(uint16_t addr, uint16_t value);

/**
 * @brief Initializes the EE895 sensor
 * 
 * @param ee895 output EE895 sensor structure
 * @param config Configuration of the EE895 sensor to be written
 * @return int32_t Return code
 */
int32_t ee895_init(sensor_t* ee895, sensor_config_t* config);

/**
 * @brief Reads EE895 sensor configuration
 * 
 * @param config EE895 config structure the read configuration will be saved to
 * @return int32_t Return code
 */
int32_t ee895_read_config(sensor_config_t* config);

/**
 * @brief Switches sensor power to [on] state if not controlled globally
 * 
 * @param ee895 Sensor structure
 * @param on if the power should be switched on (true) or off (false)
 */
void ee895_power(sensor_t* ee895, bool on);

#endif