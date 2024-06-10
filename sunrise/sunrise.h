#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "string.h"
#include "../error_codes.h"
#include "../common/functions.h"

typedef struct sunrise
{
    float temperature;
    int co2;
    int state;
} sunrise_t;


/**
 * @brief Writes data to the sunrise sensor to specified address
 * 
 * @param addr Register address
 * @param buf Data to be sent
 * @param len Length of the data
 * @return int Return code
 */
int sunrise_write(uint8_t addr, uint8_t* buf, uint16_t len);

/**
 * @brief Reads data from the sunrise sensor
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
 * @brief Initializes sunrise sensor
 * 
 * @param single_meas_mode if single measurement mode should be enabled
 * @param meas_period period of continuous measurement
 * @param meas_samples number of samples per measurement
 * @param abc_period period of abc calibration - set to 0 to disable abc calibration
 * @param abc_target_value target value of the abc calibration
 * @param nRDY_en enable nRDY pin (default enabled)
 * @param abc_en enable ABC calibration (default enabled)
 * @param static_iir_en enable static IIR filter (default enabled)
 * @param dyn_iir_en enable dynamic IIR filter (default enabled)
 * @param pressure_comp enable pressure compensation (default disabled)
 * @param invert_nRDY invert nRDY pin - high during measurement (default disabled)
 * @return int Return code
 */
int sunrise_init(bool single_meas_mode, uint16_t meas_period, uint16_t meas_samples, uint16_t abc_period, uint16_t abc_target_value, 
    bool nRDY_en, bool abc_en, bool static_iir_en, bool dyn_iir_en, bool pressure_comp, bool invert_nRDY);

/**
 * @brief Reads values from the sensor
 * 
 * @param co2 CO2 concentration
 * @param temperature Temperature of the chip
 * @return int Return code
 */
int sunrise_get_value(int* co2, float* temperature);

/**
 * @brief Performs single measurement (cannot use b/c EN pin is not accessible)
 * 
 * @param co2 CO2 concentration
 * @param temperature Temperature of the chip
 * @param state_reg Previous state of the sensor, outputs new state
 * @param state_reg_len_bytes Length of the state buffer - must be 24 bytes
 * @param no_state if no previous state available
 * @return int Return code
 */
int sunrise_single_meas(int* co2, float* temperature, uint8_t* state_reg, uint8_t state_reg_len_bytes, bool no_state);







