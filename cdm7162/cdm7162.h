#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "error_codes.h"
#include "common/functions.h"

typedef struct cdm7162
{
    uint16_t co2;
    int state;
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
 * @param pressure_corr if pressure correction should be enabled
 * @return int32_t Return code
 */
int32_t cdm7162_init(bool pressure_corr);

/**
 * @brief Sets device operation mode to power down
 * 
 * @return int32_t Return code
 */
int32_t cdm7162_deinit(void);

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
 * @brief Retrieves measured co2 value in ppm
 * 
 * @param co2 CO2 concentration in ppm
 * @return int32_t Return code
 */
int32_t cdm7162_get_value(uint16_t* co2);
