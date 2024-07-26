/**
 * @file cm1107n.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief 
 * @version 0.1
 * @date 2024-07-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef __CM1107N_H__
#define __CM1107N_H__

#include "../../common/common_include.h"

/**
 * @brief Reads data from the CM1107N sensor
 * 
 * @param addr Command (register to read from)
 * @param buf Read data (MSB first)
 * @param len Length of the data to read (in bytes)
 * @return int32_t Return code
 */
extern int32_t cm1107n_read(uint8_t addr, uint8_t* buf, uint8_t len);

/**
 * @brief Writes data to the CM1107N sensor
 * 
 * @param addr Command (register to write to)
 * @param data Data to write (MSB first)
 * @param len Length of the data (in bytes)
 * @return int32_t Return code
 */
extern int32_t cm1107n_write(uint8_t addr, uint8_t* data, uint8_t len);

/**
 * @brief Sends a command to the sensor
 * 
 * @param addr Command to send
 * @return int32_t Return code
 */
extern int32_t cm1107n_write_command(uint8_t addr);

/**
 * @brief Retrieves measured values from the CM1107N sensor
 * 
 * @param cm1107n Sensor structure; return code saved to the state variable
 */
extern void cm1107n_get_value(sensor_t* cm1107n);

/**
 * @brief Initializes the CM1107N sensor and writes configuration to it
 * 
 * @param cm1107n Sensor structure
 * @param config Configuration to write
 * @return int32_t Return code
 */
extern int32_t cm1107n_init(sensor_t* cm1107n, sensor_config_t* config);

/**
 * @brief Reads configuration from the CM1107N sensor
 * 
 * @param config Configuration structure the read configuration will be saved to
 * @return int32_t Return code
 */
extern int32_t cm1107n_read_config(sensor_config_t* config);

#endif
