#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "string.h"
#include "../error_codes.h"
#include "../common/functions.h"
#include "math.h"
#include "../common/constants.h"

/**
 * @brief FSM for the SUNLIGHT sensor
 * 
 */
typedef enum sunlight_read_value_state
{
    SUNLIGHT_MEAS_FINISHED = 0, // Measurement is finished / not started
    SUNLIGHT_MEAS_START = 1, // Measurement has started
    SUNLIGHT_READ_MODE = 2, // Reading operation mode
    SUNLIGHT_WRITE_MEAS_CMD = 3, // Writing measure command (for single measurement)
    SUNLIGHT_READ_VALUE = 4, // Reading measured data
    SUNLIGHT_READ_STATUS = 5 // Reading & saving status (for single measurement)
} sunlight_meas_state_e;

/**
 * @brief Configuration of the SUNLIGHT sensor
 * 
 */
typedef struct sunlight_config
{
    bool single_meas_mode; // Enable single measurement mode
    uint16_t meas_period; // Set measurement period
    uint16_t meas_samples; // Set number of samples to measure (default 4)
    uint16_t abc_period; // Set ABC calibration period (set 0 to disable)
    uint16_t abc_target_value; // Set ABC calibration target value (default 400 ppm)
    bool enable_nRDY; // Output measurement done state on nRDY pin
    bool enable_ABC; // Enable ABC calibration
    bool enable_static_IIR; // Enable static IIR filtering (possibly smoothing)
    bool enable_dynamic_IIR; // Enable dynamic IIR filtering (reacting to spikes, works if static IIR is on)
    bool enable_pressure_comp; // Enable pressure compenzation
    bool invert_nRDY; // Invert nRDY pin logic
    bool power_global_control; // Global power control
} sunlight_config_t;

/**
 * @brief SUNLIGHT sensor main structure
 * 
 */
typedef struct sunlight
{
    float temperature; // Measured temperature
    int16_t co2; // Measured CO2 concentration
    int state; // Sensor state
    sunlight_meas_state_e meas_state; // Measurement state
    absolute_time_t wake_time; // Time of next action
    uint8_t state_reg[24]; // State registers (for single measurement mode)
    sunlight_config_t* config; // SUNLIGHT configuration
} sunlight_t;


/**
 * @brief Writes data to the SUNLIGHT sensor to specified address
 * 
 * @param addr Register address
 * @param buf Data to be sent
 * @param len Length of the data
 * @return int Return code
 */
int sunlight_write(uint8_t addr, uint8_t* buf, uint16_t len);

/**
 * @brief Reads data from the SUNLIGHT sensor
 * 
 * @param addr Register address to be read from
 * @param buf Data buffer
 * @param num_bytes Number of bytes to read
 * @return int Return code
 */
int sunlight_read(uint8_t addr, uint8_t* buf, uint16_t num_bytes);

/**
 * @brief Resets the sensor (soft reset)
 * 
 * @return int Return code
 */
int sunlight_reset(void);

/**
 * @brief Initializes SUNLIGHT sensor
 * 
 * @param sunlight Output SUNLIGHT sensor structure
 * @param config Configuration of the SUNLIGHT sensor to be written
 * @return int Return code
 */
int sunlight_init(sunlight_t* sunlight, sunlight_config_t* config);

/**
 * @brief Reads measured values from the sensor
 * 
 * @param sunlight SUNLIGHT sensor structure
 */
void sunlight_get_value(sunlight_t* sunlight);

/**
 * @brief Reads SUNLIGHT sensor configuration
 * 
 * @param config SUNLIGHT config structure the read configuration will be saved to
 * @return int Return code
 */
int sunlight_read_config(sunlight_config_t* config);

/**
 * @brief Initializes the SUNLIGHT sensor structure
 * 
 * @param sunlight sensor structure
 */
void sunlight_init_struct(sunlight_t* sunlight);

/**
 * @brief Switches sensor power [on] if not controlled globally
 * 
 * @param sunlight Sensor structure
 * @param on if the power should be switched on (true) or off (false)
 */
void sunlight_power(sunlight_t* sunlight, bool on);
