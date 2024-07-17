/**
 * @file functions.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Defines useful functions
 * @version 0.1
 * @date 2024-06-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __FUNCTIONS_H__
#define __FUNCTIONS_H__

#include "structs.h"
#include "pico/stdlib.h"

/**
 * @brief Converts 16-bit value to the other endian
 * 
 * @param network Value to be converted
 * @return uint16_t Converted value
 */
extern uint16_t ntoh16(uint16_t network);

/**
 * @brief Swaps halves of a 32-bit value
 * 
 * @param network Value to be converted
 * @return uint16_t Converted value
 */
extern uint32_t ntoh32(uint32_t network);

/**
 * @brief converts byte representation of a float to float
 * 
 * @param byte_value bytes of the float
 * @return float output value
 */
extern float byte2float(uint32_t byte_value);

/**
 * @brief Converts float into its byte representation
 * 
 * @param float_value float value
 * @return uint32_t bytes of the float
 */
extern uint32_t float2byte(float float_value);

/**
 * @brief Initializes the sensor structure
 * 
 * @param sensor Sensor structure
 * @param input_index Sensor index
 */
extern void common_init_struct(sensor_t* sensor, uint8_t input_index);

/**
 * @brief Rounds a float number to specified precision
 * 
 * @param value Value to round
 * @param precision Number of decimal places
 * @return float rounded value
 */
extern float round_precision(float value, uint8_t precision);

#endif