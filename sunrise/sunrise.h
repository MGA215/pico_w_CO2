#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "string.h"
#include "../error_codes.h"
#include "../common/functions.h"
#include "math.h"

typedef enum sunrise_read_value_state
{
    SUNRISE_MEAS_FINISHED = 0,
    SUNRISE_MEAS_START = 1,
    SUNRISE_READ_MODE = 2,
    SUNRISE_WRITE_MEAS_CMD = 3,
    SUNRISE_READ_VALUE = 4,
    SUNRISE_READ_STATUS = 5
} sunrise_meas_state_e;

typedef struct sunrise_config
{
    bool single_meas_mode;
    uint16_t meas_period;
    uint16_t meas_samples;
    uint16_t abc_period;
    uint16_t abc_target_value;
    bool enable_nRDY;
    bool enable_ABC;
    bool enable_static_IIR;
    bool enable_dynamic_IIR;
    bool enable_pressure_comp;
    bool invert_nRDY;
} sunrise_config_t;

typedef struct sunrise
{
    float temperature;
    int16_t co2;
    int state;
    sunrise_meas_state_e meas_state;
    absolute_time_t wake_time;
    uint8_t state_reg[24];
    sunrise_config_t* config;
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



