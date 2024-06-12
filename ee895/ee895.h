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
    EE895_READ_STATUS = 2,
    EE895_READ_VALUE = 3
} ee895_meas_state_e;

typedef struct ee895
{
    float co2;
    float temperature;
    float pressure;
    int state;
    ee895_meas_state_e meas_state;
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

/*
 * @brief Gets CO2, temperature and pressure from the sensor
 * 
 * @param co2 CO2 concentration in ppm
 * @param temperature Temperature in °C
 * @param pressure Pressure in hPa (mbar)
 * @return int32_t Return code
 */
void ee895_get_value(absolute_time_t* wake_time, bool* enable_sensor_irq, ee895_t* ee895);

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
 * @return int32_t Return code
 */
int32_t ee895_init(void);