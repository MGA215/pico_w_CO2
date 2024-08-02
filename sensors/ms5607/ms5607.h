/**
 * @file ms5607.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Module for communication with the MS5607 pressure sensor
 * @version 0.1
 * @date 2024-08-02
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __MS5607_H__
#define __MS5607_H__

#include "pico/stdlib.h"

// pressure & temperature range from the sensor
#define MS_PRESS_MIN          10.0
#define MS_PRESS_MAX          1200.0
#define MS_TEMP_MIN           -40.0
#define MS_TEMP_MAX           85.0
// pressure range at sea level
#define MS_PRESS_SEA_MIN      5.0
#define MS_PRESS_SEA_MAX      1300.0

/**
 * @brief Reads PROM registers from the sensor
 * 
 * @param prom_buffer Buffer to save the registers
 * @param buffer_len Length of the buffer
 * @return int32_t Return code
 */
extern int32_t ms5607_get_prom_const(uint16_t* prom_buffer, uint8_t buffer_len);

/**
 * @brief Reads the ADC value from the sensor
 * 
 * @param channel Channel to read - 0 for pressure, 1 for temperature
 * @param buffer Buffer the read value will be saved to
 * @param buffer_length Length of the output buffer
 * @return int32_t Return code
 */
extern int32_t ms5607_get_adc_val(int32_t channel, uint8_t* buffer, uint8_t buffer_length);

/**
 * @brief Reads and compensates temperature and pressure values from the sensor and saves them to the ms5607 struct
 * Exit code is saved to ms5607.state
 * 
 */
extern void ms5607_get_value(void);

#endif