/**
 * @file cozir-lp3.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Header file for communication with GSS CozIR-LP3 sensor
 * @version 0.1
 * @date 2024-06-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __COZIR_LP3_H__
#define __COZIR_LP3_H__

#include "pico/stdlib.h"
#include "common/structs.h"

/**
 * @brief Reads data from the CozIR-LP3 sensor
 * 
 * @param addr Register address to read from
 * @param buf Read data - MSB first
 * @param len Length of the data to read (in bytes)
 * @return int32_t Return code
 */
extern int32_t cozir_lp3_read(uint8_t addr, uint8_t* buf, uint8_t len);

/**
 * @brief Writes data to the CozIR-LP3 sensor
 * 
 * @param addr Register address the data should be written to
 * @param data Data to write - MSB first
 * @param len Length of the data (in bytes)
 * @return int32_t Return code
 */
extern int32_t cozir_lp3_write(uint8_t addr, uint8_t* data, uint8_t len);

/**
 * @brief Retrieves measured values from the CozIR-LP3 sensor
 * 
 * @param cozir_lp3 Sensor structure; return code saved to the state variable
 */
extern void cozir_lp3_get_value(sensor_t* cozir_lp3);

/**
 * @brief Initializes the CozIR-LP3 sensor and writes configuration to it
 * 
 * @param cozir_lp3 Sensor structure
 * @param config Configuration to write
 * @return int32_t Return code
 */
extern int32_t cozir_lp3_init(sensor_t* cozir_lp3, sensor_config_t* config);

/**
 * @brief Reads configuration from the CozIR-LP3 sensor
 * 
 * @param config Configuration structure the read configuration will be saved to
 * @return int32_t Return code
 */
extern int32_t cozir_lp3_read_config(sensor_config_t* config);

#endif