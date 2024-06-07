#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "string.h"
#include "error_codes.h"
#include "math.h"

typedef struct ee895
{
    float co2;
    float temperature;
    float pressure;
    int state;
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
 * @param co2 CO2 concentration in ppm
 * @param temperature Temperature in °C
 * @param pressure Pressure in hPa (mbar)
 */
int32_t ee895_get_value(float* co2, float* temperature, float* pressure);

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