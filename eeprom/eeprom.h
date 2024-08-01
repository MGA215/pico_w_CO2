/**
 * @file eeprom.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Module for EEPROM communication
 * @version 0.1
 * @date 2024-08-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __EEPROM_H__
#define __EEPROM_H__

#include "pico/stdlib.h"

/**
 * @brief Reads bytes from EEPROM
 * 
 * @param addr Address in the EEPROM to read from
 * @param buffer Output buffer the read data will be saved to
 * @param num_bytes Number of bytes to read
 * @return int32_t Return code
 */
extern int32_t eeprom_read(uint32_t addr, uint8_t* buffer, uint16_t num_bytes);

/**
 * @brief Writes bytes to EEPROM
 * 
 * @param addr Address in the EEPROM to write to
 * @param buffer Data to write to EEPROM
 * @param num_bytes Number of bytes to write
 * @return int32_t Return code
 */
extern int32_t eeprom_write(uint32_t addr, uint8_t* buffer, uint16_t num_bytes);


#endif