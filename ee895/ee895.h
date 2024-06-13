#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "string.h"
#include "error_codes.h"
#include "math.h"
#include "common/functions.h"

typedef enum ee895_read_value_state
{
    EE895_MEAS_FINISHED = 0,
    EE895_MEAS_START = 1,
    EE895_READ_TRIGGER_RDY = 2,
    EE895_READ_STATUS = 3,
    EE895_READ_VALUE = 4
} ee895_meas_state_e;

typedef struct ee895_config
{
    bool single_meas_mode;
    int16_t offset;
    int16_t filter_coeff;
    int16_t meas_period;
} ee895_config_t;

typedef struct ee895
{
    float co2;
    float temperature;
    float pressure;
    int state;
    ee895_meas_state_e meas_state;
    absolute_time_t wake_time;
    ee895_config_t* config;
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
