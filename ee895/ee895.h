/**
 * @file ee895.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Header file for communication with E+E EE895 sensor
 * @version 0.1
 * @date 2024-06-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __EE895_H__
#define __EE895_H__

#include "../common_include.h"

/**
 * @brief Reads number of registers from the EE895 with timing; must manually turn power on/off
 * 
 * @param addr Address of the register to be read from
 * @param nreg Number of registers to read
 * @param buf Output buffer of values
 * @return int32_t Return code
 */
extern int32_t ee895_read_reg(uint16_t addr, uint16_t nreg, uint8_t* buf);

/**
 * @brief Writes a value to the EE895 with timing; must manually turn power on/off
 * 
 * @param addr Address of the register to be written to
 * @param value Value to be written
 * @return int32_t Return code
 */
extern int32_t ee895_write_reg(uint16_t addr, uint16_t value);

/**
 * @brief Gets CO2, temperature and pressure from the sensor
 * 
 * @param ee895 Sensor structure
 */
extern void ee895_get_value(sensor_t* ee895);

/**
 * @brief Initializes the EE895 sensor
 * 
 * @param ee895 output EE895 sensor structure
 * @param config Configuration of the EE895 sensor to be written
 * @return int32_t Return code
 */
extern int32_t ee895_init(sensor_t* ee895, sensor_config_t* config);

/**
 * @brief Reads EE895 sensor configuration
 * 
 * @param config EE895 config structure the read configuration will be saved to
 * @return int32_t Return code
 */
extern int32_t ee895_read_config(sensor_config_t* config);

#endif