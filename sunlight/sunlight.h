#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "string.h"
#include "../error_codes.h"
#include "../common/functions.h"
#include "math.h"

typedef enum sunlight_read_value_state
{
    SUNLIGHT_MEAS_FINISHED = 0,
    SUNLIGHT_MEAS_START = 1,
    SUNLIGHT_READ_MODE = 2,
    SUNLIGHT_WRITE_MEAS_CMD = 3,
    SUNLIGHT_READ_VALUE = 4,
    SUNLIGHT_READ_STATUS = 5
} sunlight_meas_state_e;

typedef struct sunlight_config
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
} sunlight_config_t;

typedef struct sunlight
{
    float temperature;
    int16_t co2;
    int state;
    sunlight_meas_state_e meas_state;
    absolute_time_t wake_time;
    uint8_t state_reg[24];
    sunlight_config_t* config;
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


