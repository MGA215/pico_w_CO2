/**
 * @file error_codes.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Error codes used in the project
 * @version 0.1
 * @date 2024-06-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __ERROR_CODES_H__
#define __ERROR_CODES_H__

#define SUCCESS 0

// PICO Errors
#define PICO_ERROR_TIMEOUT -1 // Error PICO - timeout limit has been reached
#define PICO_ERROR_GENERIC -2 // Error PICO - Generic error (might be sensor disconnected)
#define PICO_ERROR_NO_DATA -3 // Error PICO - No data
#define PICO_ERROR_NOT_PERMITTED -4 // Error PICO - Permission denied
#define PICO_ERROR_INVALID_ARG -5 // Error PICO - Invalid argument
#define PICO_ERROR_IO -6 // Error PICO - IO error
#define PICO_ERROR_BADAUTH -7 // Error PICO - Authorization failed
#define PICO_ERROR_CONNECT_FAILED -8 // Error PICO - Failed to establish connection
#define PICO_ERROR_INSUFFICIENT_RESOURCES -9 // Error PICO - Insufficient resources (memory, storage)

// Main Errors
#define ERROR_STDIO_INIT -11 // Error initializing STDIO
#define ERROR_TIMER_SENSORS_INIT -12 // Error initializing sensor timer
#define ERROR_SENSOR_NOT_INITIALIZED -13 // Error sensor is not initialized
#define ERROR_SENSOR_INIT_FAILED -14 // Error sensor initialization failed
#define ERROR_NO_MEAS -15 // Error No measurement has been performed yet
#define ERROR_UNKNOWN_SENSOR -16 // Error Sensor identification failed - unknown sensor

// Serialization errors
#define ERROR_SERIALIZATION_BUFFER_LEN -21 // Error - serialization buffer is too short
#define ERROR_DESERIALIZATION_VALUE_OUT_OF_RAGE -22 // Error - deserialized value is out of range for specified sensor 
#define ERROR_DESERIALIZATION_FAILURE -23 // Error - deserialization failed

// MUX errors
#define ERROR_MUX_INVALID_INDEX -31 // Error - invalid MUX index
#define ERROR_SENSOR_MUX_FAILED -32 // Error - MUX failed

// WiFi errors
#define ERROR_NETWORK_NOT_FOUND -41 // Error - WiFi network not visible

// TCP errors
#define ERROR_TCP_OPEN_FAILED -51 // Error - could not open TCP socket

// EE895 Errors
#define EE895_ERROR_NREG_REG 111 // Error ee895 - specified number of registers is invalid
#define EE895_ERROR_READ_RESP 112 // Error ee895 - read response is not valid
#define EE895_ERROR_INVALID_CRC 113 // Error ee895 - response CRC is not valid
#define EE895_ERROR_WRITE_RESP 114 // Error ee895 - write response is not valid
#define EE895_ERROR_DATA_READY_TIMEOUT 115 // Error ee895 - Data was not ready for more than 1.25 s
#define EE895_ERROR_RANGE 116 // Error ee895 - read data is out of range

// CDM7162 Errors
#define CDM7162_ERROR_WRITE_RESP 214 // Error CDM7162 - write response is not valid
#define CDM7162_ERROR_DATA_READY_TIMEOUT 215 // Error cdm7162 - data was not ready for more than 1 s
#define CDM7162_ERROR_RANGE 216 // Error cdm7162 - value is out of range

// SUNRISE Errors
#define SUNRISE_ERROR_WRITE_RESP 314 // Error SUNRISE - write response is not valid
#define SUNRISE_ERROR_DATA_READY_TIMEOUT 315 // Error SUNRISE - data not ready
#define SUNRISE_ERROR_FATAL 317 // Error SUNRISE - failed initialization
#define SUNRISE_ERROR_I2C 318 // Error SUNRISE - invalid address access
#define SUNRISE_ERROR_ALGORITHM 319 // Error SUNRISE - corrupted parameters detected
#define SUNRISE_ERROR_CAL 320 // Error SUNRISE - calibration failed
#define SUNRISE_ERROR_SELF_DIAG 321 // Error SUNRISE - internal failure
#define SUNRISE_ERROR_OUT_OF_RANGE 322 // Error SUNRISE - CO2, temperature or set pressure out of range
#define SUNRISE_ERROR_MEMORY 323 // Error SUNRISE - memory operation error
#define SUNRISE_ERROR_LOW_INTERNAL_VOLTAGE 324 // Error SUNRISE - low internal regulated voltage
#define SUNRISE_ERROR_MEASUREMENT_TIMEOUT 325 // Error SUNRISE - unable to complete measurement in time
#define SUNRISE_ERROR_ABNORMAL_SIGNAL_LEVEL 326 // Error SUNRISE - invalid measurement sample detected
#define SUNRISE_ERROR_SENSOR_GENERAL 327 // Error SUNRISE - general sensor error
#define SUNRISE_ERROR_WRONG_STATE 328 // Error SUNRISE - invalid buffer for reading/writing current state in single meas mode
#define SUNRISE_ERROR_WRONG_MODE 329 // Error SUNRISE - sensor is in a wrong mode

// SUNLIGHT Errors
#define SUNLIGHT_ERROR_WRITE_RESP 314 // Error SUNLIGHT - write response is not valid
#define SUNLIGHT_ERROR_DATA_READY_TIMEOUT 315 // Error SUNLIGHT - data not ready
#define SUNLIGHT_ERROR_FATAL 317 // Error SUNLIGHT - failed initialization
#define SUNLIGHT_ERROR_I2C 318 // Error SUNLIGHT - invalid address access
#define SUNLIGHT_ERROR_ALGORITHM 319 // Error SUNLIGHT - corrupted parameters detected
#define SUNLIGHT_ERROR_CAL 320 // Error SUNLIGHT - calibration failed
#define SUNLIGHT_ERROR_SELF_DIAG 321 // Error SUNLIGHT - internal failure
#define SUNLIGHT_ERROR_OUT_OF_RANGE 322 // Error SUNLIGHT - CO2, temperature or set pressure out of range
#define SUNLIGHT_ERROR_MEMORY 323 // Error SUNLIGHT - memory operation error
#define SUNLIGHT_ERROR_LOW_INTERNAL_VOLTAGE 324 // Error SUNLIGHT - low internal regulated voltage
#define SUNLIGHT_ERROR_MEASUREMENT_TIMEOUT 325 // Error SUNLIGHT - unable to complete measurement in time
#define SUNLIGHT_ERROR_ABNORMAL_SIGNAL_LEVEL 326 // Error SUNLIGHT - invalid measurement sample detected
#define SUNLIGHT_ERROR_SENSOR_GENERAL 327 // Error SUNLIGHT - general sensor error
#define SUNLIGHT_ERROR_WRONG_STATE 328 // Error SUNLIGHT - invalid buffer for reading/writing current state in single meas mode
#define SUNLIGHT_ERROR_WRONG_MODE 329 // Error SUNLIGHT - sensor is in a wrong mode

// SCD30 Errors
#define SCD30_ERROR_CRC 413 // Error SCD30 - Invalid CRC
#define SCD30_ERROR_DATA_READY_TIMEOUT 415 // Error SCD30 - Data not ready

// SCD41 Errors
#define SCD41_ERROR_CRC 513 // Error SCD41 - Invalid CRC
#define SCD41_ERROR_DATA_READY_TIMEOUT 515 // Error SCD41 - Data not ready

// CozIR-LP3 Errors
#define COZIR_LP3_ERROR_NREG_REG 611 // Error CozIR-LP3 - specified number of registers is invalid
#define COZIR_LP3_ERROR_DATA_READY_TIMEOUT 615 // Error CozIR-LP3 - Data not ready

// CUBIC CM1107N
#define CM1107N_ERROR_CRC 713 // Error CM1107N - Invalid CRC
#define CM1107N_ERROR_FATAL 717 // Error CM1107N - Sensor error
#define CM1107N_ERROR_CAL 720 // Error CM1107N - Sensor not calibrated
#define CM1107N_ERROR_OUT_OF_RANGE 722 // Error CM1107N - Measurement out of range
#define CM1107N_ERROR_LIGHT_AGING 730 // Error CM1107N - Sensor light aging
#define CM1107N_ERROR_DRIFT 731 // Error CM1107N - Sensor drifting
#define CM1107N_ERROR_PREHEATING 732 // Error CM1107N - Sensor preheating


#endif