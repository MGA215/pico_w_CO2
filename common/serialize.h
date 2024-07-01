/**
 * @file serialize.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Module for sensor configuration serialization & deserialization
 * @version 0.1
 * @date 2024-07-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "structs.h"
#include "constants.h"
#include "../error_codes.h"
#include "functions.h"

/**
 * @brief Serializes configuration
 * 
 * @param config Configuration to serialize
 * @param serialized Serialized buffer
 * @param len length of serialized buffer
 * @returns int32_t Return code
 */
int32_t serializer_serialize(sensor_config_t* config, uint8_t* serialized, uint8_t len);

/**
 * @brief Deserializes configuration
 * 
 * @param config Output configuration
 * @param serialized Serialized configuration
 * @param len Length of serialized configuration buffer
 * @returns int32_t Return code
 */
int32_t serializer_deserialize(sensor_config_t* config, uint8_t* serialized, uint8_t len);

