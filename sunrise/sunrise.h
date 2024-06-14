#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "string.h"
#include "../error_codes.h"
#include "../common/functions.h"
#include "math.h"

/**
 * @brief FSM for the SUNRISE sensor
 * 
 */
typedef enum sunrise_read_value_state
{
    SUNRISE_MEAS_FINISHED = 0, // Measurement is finished / not started
    SUNRISE_MEAS_START = 1, // Measurement has started
    SUNRISE_READ_MODE = 2, // Reading operation mode
    SUNRISE_WRITE_MEAS_CMD = 3, // Writing measure command (for single measurement)
    SUNRISE_READ_VALUE = 4, // Reading measured data
    SUNRISE_READ_STATUS = 5 // Reading & saving status (for single measurement)
} sunrise_meas_state_e;

/**
 * @brief Configuration of the SUNRISE sensor
 * 
 */
typedef struct sunrise_config
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
} sunrise_config_t;

/**
 * @brief SUNRISE sensor main structure
 * 
 */
typedef struct sunrise
{
    float temperature; // Measured temperature
    int16_t co2; // Measured CO2 concentration
    int state; // Sensor state
    sunrise_meas_state_e meas_state; // Measurement state
    absolute_time_t wake_time; // Time of next action
    uint8_t state_reg[24]; // State registers (for single measurement mode)
    sunrise_config_t* config; // SUNRISE configuration
} sunrise_t;


/**
 * @brief Writes data to the SUNRISE sensor to specified address
 * 
 * @param addr Register address
 * @param buf Data to be sent
 * @param len Length of the data
 * @return int Return code
 */
int sunrise_write(uint8_t addr, uint8_t* buf, uint16_t len);

/**
 * @brief Reads data from the SUNRISE sensor
 * 
 * @param addr Register address to be read from
 * @param buf Data buffer
 * @param num_bytes Number of bytes to read
 * @return int Return code
 */
int sunrise_read(uint8_t addr, uint8_t* buf, uint16_t num_bytes);

/**
 * @brief Resets the sensor (soft reset)
 * 
 * @return int Return code
 */
int sunrise_reset(void);

/**
 * @brief Initializes SUNRISE sensor
 * 
 * @param sunrise Output SUNRISE sensor structure
 * @param config Configuration of the SUNRISE sensor to be written
 * @return int Return code
 */
int sunrise_init(sunrise_t* sunrise, sunrise_config_t* config);

/**
 * @brief Reads measured values from the sensor
 * 
 * @param sunrise SUNRISE sensor structure
 */
void sunrise_get_value(sunrise_t* sunrise);

/**
 * @brief Reads SUNRISE sensor configuration
 * 
 * @param config SUNRISE config structure the read configuration will be saved to
 * @return int Return code
 */
int sunrise_read_config(sunrise_config_t* config);

/**
 * @brief Initializes the SUNRISE sensor structure
 * 
 * @param sunrise sensor structure
 */
void sunrise_init_struct(sunrise_t* sunrise);



