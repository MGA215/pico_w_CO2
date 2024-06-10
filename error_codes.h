#define SUCCESS 0
#define ERROR_STDIO_INIT 1 // Error initializing STDIO
#define ERROR_TIMER_SENSORS_INIT 2 // Error initializing sensor timer
#define ERROR_SENSOR_NOT_INITIALIZED 3 // Error sensor is not initialized

// PICO ERRORS

#define EE895_ERROR_NREG_REG -101 // Error ee895 - specified number of registers is invalid
#define EE895_ERROR_READ_RESP -102 // Error ee895 - read response is not valid
#define EE895_ERROR_INVALID_CRC -103 // Error ee895 - response CRC is not valid
#define EE895_ERROR_WRITE_RESP -104 // Error ee895 - write response is not valid
#define EE895_ERROR_DATA_READY_TIMEOUT -105 // Error ee895 - Data was not ready for more than 1.25 s
#define EE895_ERROR_RANGE -106 // Error ee895 - read data is out of range

#define CDM7162_ERROR_WRITE_RESP -204 // Error CDM7162 - write response is not valid
#define CDM7162_ERROR_DATA_READY_TIMEOUT -205 // Error cdm7162 - data was not ready for more than 1 s
#define CDM7162_ERROR_RANGE -206 // Error cdm7162 - value is out of range

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
#define SUNRISE_ERROR_NO_SINGLE_MEAS_MODE -319 // Error SUNRISE - sensor is not in a single measurement mode
