#include "pico/stdlib.h"
/**
 * @brief Converts 16-bit value to the other endian
 * 
 * @param network Value to be converted
 * @return uint16_t Converted value
 */
uint16_t ntoh16(uint16_t network);

/**
 * @brief Swaps halves of a 32-bit value
 * 
 * @param network Value to be converted
 * @return uint16_t Converted value
 */
uint32_t ntoh32(uint32_t network);

/**
 * @brief converts byte representation of a float to float
 * 
 * @param byte_value bytes of the float
 * @return float output value
 */
float byte2float(uint32_t byte_value);

