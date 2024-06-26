#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "../error_codes.h"
#include "../common/functions.h"
#include "string.h"
#include <math.h>
#include "../common/constants.h"

typedef enum cozir_lp3_meas_state
{
    COZIR_LP3_MEAS_FINISHED = 0,
    COZIR_LP3_MEAS_START = 1,
    COZIR_LP3_READ_VALUE = 2
} cozir_lp3_meas_state_e;

typedef enum cozir_lp3_cal_mode
{
    COZIR_LP3_CAL_ABC = 0,
    COZIR_LP3_CAL_NITROGEN = 1,
    COZIR_LP3_CAL_KNOWN_ZERO = 2,
} cozir_lp3_cal_mode_e;

typedef struct cozir_lp3_config
{
    bool pressure_comp;
    uint16_t pressure;
    bool temp_humidity_meas_en;
    bool PWM_en;
    bool enable_cal;
    uint16_t ABC_init_period;
    uint16_t ABC_period;
    uint16_t ABC_target_co2;
    uint16_t cal_target_fresh_air;
    cozir_lp3_cal_mode_e cal_mode;
    bool alarm_en;
    uint16_t alarm_treshold_co2;
    bool power_global_control;
} cozir_lp3_config_t;

typedef struct cozir_lp3
{
    float co2;
    float temperature;
    float humidity;
    int state;
    cozir_lp3_meas_state_e meas_state;
    absolute_time_t wake_time;
    cozir_lp3_config_t* config;
} cozir_lp3_t;

/**
 * @brief Writes data to the CozIR-LP3 sensor
 * 
 * @param addr Register address the data should be written to
 * @param data Data to write - MSB first
 * @param len Length of the data (in bytes)
 * @return int32_t Return code
 */
int32_t cozir_lp3_write(uint8_t addr, uint8_t* data, uint8_t len);

/**
 * @brief Reads data from the CozIR-LP3 sensor
 * 
 * @param addr Register address to read from
 * @param buf Read data - MSB first
 * @param len Length of the data to read (in bytes)
 * @return int32_t Return code
 */
int32_t cozir_lp3_read(uint8_t addr, uint8_t* buf, uint8_t len);

/**
 * @brief Retrieves measured values from the CozIR-LP3 sensor
 * 
 * @param cozir_lp3 Sensor structure; return code saved to the state variable
 */
void cozir_lp3_get_value(cozir_lp3_t* cozir_lp3);

/**
 * @brief Turns the sensor power [on]
 * 
 * @param cozir_lp3 Sensor structure
 * @param on Whether the power should be turned on
 */
void cozir_lp3_power(cozir_lp3_t* cozir_lp3, bool on);

/**
 * @brief Initializes the sensor structure and sets variables to defaults
 * 
 * @param cozir_lp3 Sensor structure
 */
void cozir_lp3_init_struct(cozir_lp3_t* cozir_lp3);

/**
 * @brief Reads configuration from the CozIR-LP3 sensor
 * 
 * @param config Configuration structure the read configuration will be saved to
 * @return int32_t Return code
 */
int32_t cozir_lp3_read_config(cozir_lp3_config_t* config);

/**
 * @brief Initializes the CozIR-LP3 sensor and writes configuration to it
 * 
 * @param cozir_lp3 Sensor structure
 * @param config Configuration to write
 * @return int32_t Return code
 */
int32_t cozir_lp3_init(cozir_lp3_t* cozir_lp3, cozir_lp3_config_t* config);