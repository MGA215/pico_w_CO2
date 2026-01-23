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

#include "pico/stdlib.h"
#include "structs.h"

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

/**
 * @brief Converts error byte to float representation, outputs the float in bytes
 * 
 * @param error_byte Error value
 * @return uint32_t Float bytes error value
 */
extern uint32_t get_error(uint8_t error_byte);

/**
 * @brief Returns decimal value from hexadecimal representation (i.e. 0x10 -> 10)
 * 
 * @param hex_val Hexadecimal value
 * @return uint8_t Decimal value
 */
extern uint8_t hex2dec(uint8_t hex_val);

/**
 * @brief Returns hexadecimal value from decimal representation (i.e. 10 -> 0x10)
 * 
 * @param dec_val Decimal value
 * @return uint8_t Hexadecimal value
 */
extern uint8_t dec2hex(uint8_t dec_val);

/**
 * @brief Reverts bits in a byte (last bit becomes first one)
 * 
 * @param b Byte to reverse
 * @return uint8_t Reversed byte
 */
extern uint8_t reverse_bits_in_byte(uint8_t b);

/**
 * @brief Checks whether is measurement in progress on specified sensor
 * 
 * @param sensor Sensor to check
 * @return true if measurement is still in progress
 * @return false if measurement is finished
 */
extern bool common_is_measurement_running(sensor_t* sensor);

/**
 * @brief Force stops measurement on specified sensor
 * 
 * @param sensor Sensor to stop measurement on
 */
extern void common_measurement_force_stop(sensor_t* sensor);

/**
 * @brief Starts a new measurement on specified sensor
 * 
 * @param sensor Sensor to start measurement on
 */
extern void common_measurement_start(sensor_t* sensor);

/**
 * @brief Whether sensor should operate (is active and should be awake)
 * 
 * @param sensor sensor
 * @return true if sensor should operate
 * @return false if sensor should not operate
 */
extern bool common_should_sensor_operate(sensor_t* sensor);

/**
 * @brief Disables sensor for specified time
 * 
 * @param sensor Sensor to disable
 * @param time_ms Time to disable the sensor for, in ms
 */
extern void common_disable_sensor_for_ms(sensor_t* sensor, uint32_t time_ms);

#endif