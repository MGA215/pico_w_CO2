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
