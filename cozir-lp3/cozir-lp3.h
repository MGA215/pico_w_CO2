#ifndef COZIR_LP3_MODULE
#define COZIR_LP3_MODULE

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "../error_codes.h"
#include "../common/functions.h"
#include "string.h"
#include <math.h>
#include "../common/constants.h"

/**
 * @brief Writes data to the CozIR-LP3 sensor
 * 
 * @param addr Register address the data should be written to
 * @param data Data to write - MSB first
 * @param len Length of the data (in bytes)
 * @return int32_t Return code
 */
int32_t cozir_lp3_write(uint8_t addr, uint8_t* data, uint8_t len);

/**
 * @brief Reads data from the CozIR-LP3 sensor
 * 
 * @param addr Register address to read from
 * @param buf Read data - MSB first
 * @param len Length of the data to read (in bytes)
 * @return int32_t Return code
 */
int32_t cozir_lp3_read(uint8_t addr, uint8_t* buf, uint8_t len);

/**
 * @brief Retrieves measured values from the CozIR-LP3 sensor
 * 
 * @param cozir_lp3 Sensor structure; return code saved to the state variable
 */
void cozir_lp3_get_value(sensor_t* cozir_lp3);

/**
 * @brief Turns the sensor power [on]
 * 
 * @param cozir_lp3 Sensor structure
 * @param on Whether the power should be turned on
 */
void cozir_lp3_power(sensor_t* cozir_lp3, bool on);

/**
 * @brief Reads configuration from the CozIR-LP3 sensor
 * 
 * @param config Configuration structure the read configuration will be saved to
 * @return int32_t Return code
 */
int32_t cozir_lp3_read_config(sensor_config_t* config);

/**
 * @brief Initializes the CozIR-LP3 sensor and writes configuration to it
 * 
 * @param cozir_lp3 Sensor structure
 * @param config Configuration to write
 * @return int32_t Return code
 */
int32_t cozir_lp3_init(sensor_t* cozir_lp3, sensor_config_t* config);

#endif