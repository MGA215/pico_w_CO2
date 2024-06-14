#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "string.h"
#include "error_codes.h"
#include "math.h"
#include "common/functions.h"

/**
 * @brief FSM for the EE895 sensor
 * 
 */
typedef enum ee895_read_value_state
{
    EE895_MEAS_FINISHED = 0, // Measurement is finished / not started
    EE895_MEAS_START = 1, // Measurement has started
    EE895_READ_TRIGGER_RDY = 2, // Waiting for trigger ready signal (for single measurement)
    EE895_READ_STATUS = 3, // Reading sensor status
    EE895_READ_VALUE = 4 // Reading measured data
} ee895_meas_state_e;

/**
 * @brief Configuration of the EE895 sensor
 * 
 */
typedef struct ee895_config
{
    bool single_meas_mode; // Enable single masurement mode
    int16_t offset; // Set measured value offset
    int16_t filter_coeff; // Set filter coefficient (default 4)
    int16_t meas_period; // Set measurement period (default 15 s)
    bool power_global_control; // Global power control
} ee895_config_t;

/**
 * @brief EE895 sensor main structure
 * 
 */
typedef struct ee895
{
    float co2; // Measured CO2 concentration
    float temperature; // Measured temperature
    float pressure; // Measured pressure
    int state; // Sensor state
    ee895_meas_state_e meas_state; // Measurement state
    absolute_time_t wake_time; // Time of next action
    ee895_config_t* config; // EE895 configuration
} ee895_t;

/**
 * @brief Reads number of registers from the EE895
 * 
 * @param addr Address of the register to be read from
 * @param nreg Number of registers to read
 * @param buf Output buffer of values
 * @return int32_t Return code
 */
int32_t ee895_read(uint16_t addr, uint16_t nreg, uint8_t* buf);

/**
 * @brief Writes a value to the EE895
 * 
 * @param addr Address of the register to be written to
 * @param value Value to be written
 * @return int32_t Return code
 */
int32_t ee895_write(uint16_t addr, uint16_t value);

/**
 * @brief Gets CO2, temperature and pressure from the sensor
 * 
 * @param ee895 Sensor structure
 */
void ee895_get_value(ee895_t* ee895);

/**
 * @brief Reads number of registers from the EE895 with timing
 * 
 * @param addr Address of the register to be read from
 * @param nreg Number of registers to read
 * @param buf Output buffer of values
 * @return int32_t Return code
 */
int32_t ee895_read_reg(uint16_t addr, uint16_t nreg, uint8_t* buf);

/**
 * @brief Writes a value to the EE895 with timing
 * 
 * @param addr Address of the register to be written to
 * @param value Value to be written
 * @return int32_t Return code
 */
int32_t ee895_write_reg(uint16_t addr, uint16_t value);

/**
 * @brief Initializes the EE895 sensor
 * 
 * @param ee895 output EE895 sensor structure
 * @param config Configuration of the EE895 sensor to be written
 * @return int32_t Return code
 */
int32_t ee895_init(ee895_t* ee895, ee895_config_t* config);

/**
 * @brief Initilaizes the EE895 sensor structure
 * 
 * @param ee895 Sensor structure
 */
void ee895_init_struct(ee895_t* ee895);

/**
 * @brief Reads EE895 sensor configuration
 * 
 * @param config EE895 config structure the read configuration will be saved to
 * @return int32_t Return code
 */
int32_t ee895_read_config(ee895_config_t* config);
