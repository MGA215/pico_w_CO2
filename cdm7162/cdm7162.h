#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "error_codes.h"
#include "common/functions.h"
#include "math.h"

/**
 * @brief FSM for the CDM7162 sensor
 * 
 */
typedef enum cdm7162_read_value_state
{
    CDM7162_MEAS_FINISHED = 0, // Measurement is finished / not started
    CDM7162_MEAS_START = 1, // Measurement has started
    CDM7162_READ_STATUS = 2, // Reading sensor status
    CDM7162_READ_VALUE = 3 // Reading measured data
} cdm7162_meas_state_e;

/**
 * @brief Configuration of the CDM7162 sensor
 * 
 */
typedef struct cdm7162_config
{
    bool enable_PWM_pin; // Output 1 kHz PWM with duty proportional to CO2 concentration
    bool PWM_range_high; // CO2 = PWM high (us) * 5 if true, else * 2
    bool pressure_corr; // Enable pressure correction
    bool long_term_adj_1; // Enable long term adjustment 1
    bool long_term_adj_2; // Enable long term adjustment 2
    bool power_global_control; // Global power control
} cdm7162_config_t;

/**
 * @brief CDM7162 sensor main structure
 * 
 */
typedef struct cdm7162
{
    uint16_t co2; // Measured CO2 concentration
    int state; // Sensor state
    cdm7162_meas_state_e meas_state; // Measurement state
    absolute_time_t wake_time; // Time of next action
    cdm7162_config_t* config; // CDM7162 configuration
} cdm7162_t;


/**
 * @brief Reads data from cdm7162 sensor
 * 
 * @param addr register address to read from
 * @param buf output buffer
 * @param num_bytes number of bytes to read
 * @return int32_t error code
 */
int32_t cdm7162_read(uint8_t addr, uint8_t* buf, uint8_t num_bytes);

/**
 * @brief Writes data to cdm7162 sensor
 * 
 * @param addr Register address to write to
 * @param value Value to write
 * @return int32_t Return code
 */
int32_t cdm7162_write(uint8_t addr, uint8_t value);

/**
 * @brief Resets the sensor
 * 
 * @return int32_t Return code
 */
int32_t cdm7162_reset(void);

/**
 * @brief Initializes the cdm7162 sensor
 * 
 * @param cdm7162 output CDM7162 sensor structure
 * @param config Configuration of the CDM7162 sensor to be written
 * @return int32_t Return code
 */
int32_t cdm7162_init(cdm7162_t* cdm7162, cdm7162_config_t* config);

/**
 * @brief Sets atmospheric pressure for pressure correction
 * 
 * @param pressure Pressure in hPa
 * @return int32_t Return code
 */
int32_t cdm7162_set_atm_pressure(uint16_t pressure);

/**
 * @brief Sets default atmospheric pressure 1013 hPa
 * 
 * @return int32_t Return code
 */
int32_t cdm7162_set_default_atm_pressure(void);

/**
 * @brief Reads measured values from the sensor
 * 
 * @param cdm7162 CDM7162 sensor structure
 */
void cdm7162_get_value(cdm7162_t* cdm7162);

/**
 * @brief Initilaizes the CDM7162 sensor structure
 * 
 * @param cdm7162 Sensor structure
 */
void cdm7162_init_struct(cdm7162_t* cdm7162);

/**
 * @brief Reads CDM7162 sensor config
 * 
 * @param config CDM7162 config structure the read configuration will be saved to
 * @return int32_t Return code
 */
int32_t cdm7162_read_config(cdm7162_config_t* config);

