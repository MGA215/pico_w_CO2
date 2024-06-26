#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "../error_codes.h"
#include "../common/functions.h"
#include "string.h"
#include <math.h>
#include "../common/constants.h"

typedef enum scd41_meas_state
{
    SCD41_MEAS_FINISHED = 0,
    SCD41_MEAS_START = 1,
    SCD41_READ_MODE = 2,
    SCD41_WRITE_MEAS_CMD = 3,
    SCD41_READ_STATUS = 4,
    SCD41_READ_VALUE = 5
} scd41_meas_state_e;

typedef struct scd41_config
{
    bool pressure_comp;
    uint16_t pressure;
    bool enable_autocal;
    float temperature_offset;
    bool enable_altitude_comp;
    uint16_t altitude;
    bool enable_single_meas;
    bool power_global_control; // Global power control
} scd41_config_t;



typedef struct scd41
{
    float co2;
    float temperature;
    float humidity;
    int state;
    scd41_meas_state_e meas_state;
    absolute_time_t wake_time; // Time of next action
    scd41_config_t* config;
} scd41_t;


/**
 * @brief Reads data from the SCD41 sensor
 * 
 * @param command Command to execute (get specific data)
 * @param buf Buffer to save read data
 * @param len Length of the buffer
 * @return int32_t Return code
 */
int32_t scd41_read(uint16_t command, uint16_t* buf, uint32_t len);

/**
 * @brief Writes a value to the SCD41 sensor
 * 
 * @param command Command to write specific data
 * @param value Data to write
 * @return int32_t Return code
 */
int32_t scd41_write_value(uint16_t command, uint16_t value);

/**
 * @brief Executes a command on the SCD41 sensor
 * 
 * @param command Command to send
 * @return int32_t Return code
 */
int32_t scd41_write_command(uint16_t command);

/**
 * @brief Retrieves measured values from the SCD41 sensor
 * 
 * @param scd41 sensor structure; return code saved to the state variable
 */
void scd41_get_value(scd41_t* scd41);

/**
 * @brief Turns the sensor power [on]
 * 
 * @param scd41 Sensor structure
 * @param on Whether the power should be turned on
 */
void scd41_power(scd41_t* scd41, bool on);

/**
 * @brief Initializes the sensor and writes configuration to it
 * 
 * @param scd41 Sensor structure
 * @param config Configuration to write
 * @return int32_t Return code
 */
int32_t scd41_init(scd41_t* scd41, scd41_config_t* config);

/**
 * @brief Reads configuration from the SCD41 sensor
 * 
 * @param config Config structure the read configuration will be saved to
 * @return int32_t Return code
 */
int32_t scd41_read_config(scd41_config_t* config, bool single_meas_mode);

/**
 * @brief Intializes the sensor structure and sets variables to defaults
 * 
 * @param scd41 Sensor structure
 */
void scd41_init_struct(scd41_t* scd41);

/**
 * @brief Soft resets the sensor
 * 
 */
void scd41_reset(void);