#define SUCCESS 0

// Main Errors
#define ERROR_STDIO_INIT 1 // Error initializing STDIO
#define ERROR_TIMER_SENSORS_INIT 2 // Error initializing sensor timer
#define ERROR_SENSOR_NOT_INITIALIZED 3 // Error sensor is not initialized
#define ERROR_SENSOR_INIT_FAILED 4 // Error sensor initialization failed
#define ERROR_NO_MEAS 5 // Error No measurement has been performed yet
#define ERROR_UNKNOWN_SENSOR 6 // Error Sensor identification failed - unknown sensor

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

// EE895 Errors
#define EE895_ERROR_NREG_REG -101 // Error ee895 - specified number of registers is invalid
#define EE895_ERROR_READ_RESP -102 // Error ee895 - read response is not valid
#define EE895_ERROR_INVALID_CRC -103 // Error ee895 - response CRC is not valid
#define EE895_ERROR_WRITE_RESP -104 // Error ee895 - write response is not valid
#define EE895_ERROR_DATA_READY_TIMEOUT -105 // Error ee895 - Data was not ready for more than 1.25 s
#define EE895_ERROR_RANGE -106 // Error ee895 - read data is out of range

// CDM7162 Errors
#define CDM7162_ERROR_WRITE_RESP -204 // Error CDM7162 - write response is not valid
#define CDM7162_ERROR_DATA_READY_TIMEOUT -205 // Error cdm7162 - data was not ready for more than 1 s
#define CDM7162_ERROR_RANGE -206 // Error cdm7162 - value is out of range

// SUNRISE Errors
#define SUNRISE_ERROR_WRITE_RESP -304 // Error SUNRISE - write response is not valid
#define SUNRISE_ERROR_DATA_READY_TIMEOUT -305 // Error SUNRISE - data not ready
#define SUNRISE_ERROR_FATAL -307 // Error SUNRISE - failed initialization
#define SUNRISE_ERROR_I2C -308 // Error SUNRISE - invalid address access
#define SUNRISE_ERROR_ALGORITHM -309 // Error SUNRISE - corrupted parameters detected
#define SUNRISE_ERROR_CAL -310 // Error SUNRISE - calibration failed
#define SUNRISE_ERROR_SELF_DIAG -311 // Error SUNRISE - internal failure
#define SUNRISE_ERROR_OUT_OF_RANGE -312 // Error SUNRISE - CO2, temperature or set pressure out of range
#define SUNRISE_ERROR_MEMORY -313 // Error SUNRISE - memory operation error
#define SUNRISE_ERROR_LOW_INTERNAL_VOLTAGE -314 // Error SUNRISE - low internal regulated voltage
#define SUNRISE_ERROR_MEASUREMENT_TIMEOUT -315 // Error SUNRISE - unable to complete measurement in time
#define SUNRISE_ERROR_ABNORMAL_SIGNAL_LEVEL -316 // Error SUNRISE - invalid measurement sample detected
#define SUNRISE_ERROR_SENSOR_GENERAL -317 // Error SUNRISE - general sensor error
#define SUNRISE_ERROR_WRONG_STATE -318 // Error SUNRISE - invalid buffer for reading/writing current state in single meas mode
#define SUNRISE_ERROR_WRONG_MODE -319 // Error SUNRISE - sensor is in a wrong mode

// SUNLIGHT Errors
#define SUNLIGHT_ERROR_WRITE_RESP -304 // Error SUNLIGHT - write response is not valid
#define SUNLIGHT_ERROR_DATA_READY_TIMEOUT -305 // Error SUNLIGHT - data not ready
#define SUNLIGHT_ERROR_FATAL -307 // Error SUNLIGHT - failed initialization
#define SUNLIGHT_ERROR_I2C -308 // Error SUNLIGHT - invalid address access
#define SUNLIGHT_ERROR_ALGORITHM -309 // Error SUNLIGHT - corrupted parameters detected
#define SUNLIGHT_ERROR_CAL -310 // Error SUNLIGHT - calibration failed
#define SUNLIGHT_ERROR_SELF_DIAG -311 // Error SUNLIGHT - internal failure
#define SUNLIGHT_ERROR_OUT_OF_RANGE -312 // Error SUNLIGHT - CO2, temperature or set pressure out of range
#define SUNLIGHT_ERROR_MEMORY -313 // Error SUNLIGHT - memory operation error
#define SUNLIGHT_ERROR_LOW_INTERNAL_VOLTAGE -314 // Error SUNLIGHT - low internal regulated voltage
#define SUNLIGHT_ERROR_MEASUREMENT_TIMEOUT -315 // Error SUNLIGHT - unable to complete measurement in time
#define SUNLIGHT_ERROR_ABNORMAL_SIGNAL_LEVEL -316 // Error SUNLIGHT - invalid measurement sample detected
#define SUNLIGHT_ERROR_SENSOR_GENERAL -317 // Error SUNLIGHT - general sensor error
#define SUNLIGHT_ERROR_WRONG_STATE -318 // Error SUNLIGHT - invalid buffer for reading/writing current state in single meas mode
#define SUNLIGHT_ERROR_NO_SINGLE_MEAS_MODE -319 // Error SUNLIGHT - sensor is not in a single measurement mode
