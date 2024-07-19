/**
 * @file i2c_extras.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Extra functions for I2C
 * @version 0.1
 * @date 2024-07-19
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __I2C_EXTRAS_H__
#define __I2C_EXTRAS_H__


/**
 * @brief Resets I2C bus by sending several clock pulses
 * 
 */
extern void reset_i2c(void);

/**
 * @brief Initializes sensor I2C bus
 * 
 */
extern void init_sensor_i2c(void);

#endif