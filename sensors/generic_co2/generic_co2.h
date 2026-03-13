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

#ifndef __GENERIC_CO2_H__
#define __GENERIC_CO2_H__

#include "pico/stdlib.h"
#include "common/structs.h"



/**
 * @brief Reads number of registers from the EE895 with timing; must manually turn power on/off
 * 
 * @param addr Address of the register to be read from
 * @param nreg Number of registers to read
 * @param buf Output buffer of values
 * @return int32_t Return code
 */
extern int32_t generic_co2_read_reg(uint16_t addr, uint16_t nreg, uint8_t* buf);

/**
 * @brief Writes a value to the EE895 with timing; must manually turn power on/off
 * 
 * @param addr Address of the register to be written to
 * @param value Value to be written
 * @return int32_t Return code
 */
// extern int32_t generic_co2_write_reg(uint16_t addr, uint16_t value);

// /**
//  * @brief Gets CO2, temperature and pressure from the sensor
//  * 
//  * @param ee895 Sensor structure
//  */
// extern void ee895_get_value(sensor_t* ee895);

// /**
//  * @brief Initializes the EE895 sensor
//  * 
//  * @param sensor EE895 sensor structure
//  */
// extern void ee895_init(sensor_t* sensor);

// /**
//  * @brief Reads EE895 sensor configuration
//  * 
//  * @param config EE895 config structure the read configuration will be saved to
//  * @param single_measurement_mode if single measurement mode is active
//  * @return int32_t Return code
//  */
// extern int32_t ee895_read_config(sensor_config_t* config, bool single_measurement_mode);

extern int32_t generic_co2_write_float(uint16_t addr, float value);

extern int32_t generic_co2_read_float(uint16_t addr, float* value);

extern int32_t generic_co2_write(uint16_t addr, uint16_t nreg, uint16_t* buffer);

extern int32_t generic_co2_read(uint16_t addr, uint16_t nreg, uint16_t* buf);

extern void generic_co2_init(sensor_t* sensor);

extern void generic_co2_get_value(sensor_t* sensor);

extern int32_t generic_co2_read32(uint16_t addr, uint32_t* value);

extern int32_t generic_co2_write32(uint16_t addr, uint32_t value);

extern int32_t generic_co2_read16(uint16_t addr, uint16_t* value);

extern int32_t generic_co2_write16(uint16_t addr, uint16_t value);


extern sensor_functions_t generic_co2_functions;


#endif