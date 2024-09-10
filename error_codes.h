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
#define ERROR_CONFIG_INIT -17 // Error Failed to read configuration from EEPROM

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

// Service comm errors
#define ERROR_SERVICE_COMM_SHORT_COMMAND -61 // Error - service comm command is too short
#define ERROR_SERVICE_COMM_UNKNOWN_VALUE -62 // Error - service comm could not retrieve value
#define ERROR_SERVICE_COMM_WRONG_OFFSET -63 // Error - data offset is too small
#define ERROR_SERVICE_COMM_SHORT_BUFFER -64 // Error - send buffer is not big enough

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
#define SUNRISE_ERROR_OUT_OF_RANGE 316 // Error SUNRISE - CO2, temperature or set pressure out of range
#define SUNRISE_ERROR_FATAL 317 // Error SUNRISE - failed initialization
#define SUNRISE_ERROR_I2C 318 // Error SUNRISE - invalid address access
#define SUNRISE_ERROR_ALGORITHM 319 // Error SUNRISE - corrupted parameters detected
#define SUNRISE_ERROR_CAL 320 // Error SUNRISE - calibration failed
#define SUNRISE_ERROR_SELF_DIAG 321 // Error SUNRISE - internal failure
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
#define SUNLIGHT_ERROR_OUT_OF_RANGE 316 // Error SUNLIGHT - CO2, temperature or set pressure out of range
#define SUNLIGHT_ERROR_FATAL 317 // Error SUNLIGHT - failed initialization
#define SUNLIGHT_ERROR_I2C 318 // Error SUNLIGHT - invalid address access
#define SUNLIGHT_ERROR_ALGORITHM 319 // Error SUNLIGHT - corrupted parameters detected
#define SUNLIGHT_ERROR_CAL 320 // Error SUNLIGHT - calibration failed
#define SUNLIGHT_ERROR_SELF_DIAG 321 // Error SUNLIGHT - internal failure
#define SUNLIGHT_ERROR_MEMORY 323 // Error SUNLIGHT - memory operation error
#define SUNLIGHT_ERROR_LOW_INTERNAL_VOLTAGE 324 // Error SUNLIGHT - low internal regulated voltage
#define SUNLIGHT_ERROR_MEASUREMENT_TIMEOUT 325 // Error SUNLIGHT - unable to complete measurement in time
#define SUNLIGHT_ERROR_ABNORMAL_SIGNAL_LEVEL 326 // Error SUNLIGHT - invalid measurement sample detected
#define SUNLIGHT_ERROR_SENSOR_GENERAL 327 // Error SUNLIGHT - generic sensor error
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
#define COZIR_LP3_ERROR_SENSOR_GENERAL 627 // Error CozIR-LP3 - generic sensor error

// CUBIC CM1107N
#define CM1107N_ERROR_CRC 713 // Error CM1107N - Invalid CRC
#define CM1107N_ERROR_OUT_OF_RANGE 716 // Error CM1107N - Measurement out of range
#define CM1107N_ERROR_FATAL 717 // Error CM1107N - Sensor error
#define CM1107N_ERROR_CAL 720 // Error CM1107N - Sensor not calibrated
#define CM1107N_ERROR_LIGHT_AGING 730 // Error CM1107N - Sensor light aging
#define CM1107N_ERROR_DRIFT 731 // Error CM1107N - Sensor drifting
#define CM1107N_ERROR_PREHEATING 732 // Error CM1107N - Sensor preheating

// EEPROM errors
#define EEPROM_ERROR_PAGE_OVERFLOW 833 // Error EEPROM - Page write overflow

// RTC errors
#define ERROR_RTC_INVALID_DATETIME 934 // Error RTC - Date time value is invalid

// MS5607 errors
#define MS5607_ERROR_INVALID_CRC 1013 // Error MS5607 - CRC is invalid
#define MS5607_ERROR_PROM_ZERO 1035 // Error MS5607 - read 0s from PROM
#define MS5607_ERROR_SHORT_BUFFER 1036 // Error MS5607 - short buffer
#define MS5607_ERROR_VALUE_LOW 1037 // Error MS5607 - value is too low
#define MS5607_ERROR_VALUE_HIGH 1038 // Error MS5607 - value is too high

// HYT271 errors
#define HYT271_ERROR_SHORT_BFFER 1136 // Error HYT271 - short buffer
#define HYT271_ERROR_GENERAL 1137 // Error HYT271 - generic sensor error

// Service comm error codes
#define P_ERR_RANGE 3 // Measured value range error
#define P_CO2_COM 10 // CO2 module - communication error
#define P_CO2_VAL 11 // CO2 module - measurement error
#define P_TRH_COM 15 // HYT271 - communication error
#define P_TRH_VAL 16 // HYT271 - measurement error
#define P_MS_PROM 20 // MS5607 - unable to read PROM
#define P_MS_VAL 21 // MS5607 - measurement error
#define P_UNDEFINED 52 // Error float conversion
#define P_NA 53 // Value NA - not available
#define P_ORD_RANGE 55 // Range error casting float to int
#define P_INV_REQ 129 // Invalid addressing space
#define P_ERR_DEVICE 130 // Device not responding
#define P_ERR_INVDL 131 // Invalid data length
#define P_ERR_NOREADSPACE 135 // Unable to read from specified space
#define P_ERR_NOWRITESPACE 136 // Unable to write to specified space
#define P_ERR_UNCMD 137 // Unknown command
#define P_ERR_REJECTED 138 // Command cannot be executed
#define P_ERR_ACC_SERVICE 189 // Cannot enter service mode



#endif