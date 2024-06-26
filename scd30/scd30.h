#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "../error_codes.h"
#include "../common/functions.h"
#include "string.h"
#include <math.h>
#include "../common/constants.h"

typedef enum scd30_meas_state
{
    SCD30_MEAS_FINISHED = 0,
    SCD30_MEAS_START = 1,
    SCD30_READ_STATUS = 2,
    SCD30_READ_VALUE = 3
} scd30_meas_state_e;

typedef struct scd30_config
{
    bool pressure_comp;
    uint16_t pressure;
    bool enable_autocal;
    uint16_t autocal_value;
    uint16_t meas_period;
    float temperature_offset;
    bool enable_altitude_comp;
    uint16_t altitude;
    bool power_global_control; // Global power control
} scd30_config_t;

typedef struct scd30
{
    float co2;
    float temperature;
    float humidity;
    int state;
    scd30_meas_state_e meas_state;
    absolute_time_t wake_time; // Time of next action
    scd30_config_t* config;
} scd30_t;


/**
 * @brief Reads data from the SCD30 sensor
 * 
 * @param command Command to execute (get specific data)
 * @param buf Buffer to save read data
 * @param len Length of the buffer
 * @return int32_t Return code
 */
int32_t scd30_read(uint16_t command, uint16_t* buf, uint32_t len);

/**
 * @brief Writes a value to the SCD30 sensor
 * 
 * @param command Command to write specific data
 * @param value Data to write
 * @return int32_t Return code
 */
int32_t scd30_write_value(uint16_t command, uint16_t value);

/**
 * @brief Executes a command on the SCD30 sensor
 * 
 * @param command Command to send
 * @return int32_t Return code
 */
int32_t scd30_write_command(uint16_t command);

/**
 * @brief Retrieves measured values from the SCD30 sensor
 * 
 * @param scd30 Sensor structure; return code saved to the state variable
 */
void scd30_get_value(scd30_t* scd30);

/**
 * @brief Turns the sensor power [on]
 * 
 * @param scd30 Sensor structure
 * @param on Whether the power should be turned on
 */
void scd30_power(scd30_t* scd30, bool on);

/**
 * @brief Initializes the sensor and writes configuration to it
 * 
 * @param scd30 Sensor structure
 * @param config Configuration to write
 * @return int32_t Return code
 */
int32_t scd30_init(scd30_t* scd30, scd30_config_t* config);

/**
 * @brief Reads configuration from the SCD30 sensor
 * 
 * @param config Config structure the read configuration will be saved to
 * @return int32_t Return code
 */
int32_t scd30_read_config(scd30_config_t* config);

/**
 * @brief Intializes the sensor structure and sets variables to defaults
 * 
 * @param scd30 Sensor structure
 */
void scd30_init_struct(scd30_t* scd30);