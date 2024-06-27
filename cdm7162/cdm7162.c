#include "cdm7162.h"
#include <stdio.h>

#define CDM7162_ADDR 0x68

#define REG_RST 0x00
#define REG_OP_MODE 0x01
#define REG_STATUS 0x02
#define REG_CO2_L 0x03
#define REG_CO2_H 0x04
#define REG_ATM_PRESSURE 0x09
#define REG_ALTITUDE 0x0A
#define REG_ALARM_CO2_H 0x0C
#define REG_ALARM_CO2_L 0x0D
#define REG_CAL 0x0E
#define REG_FUNC 0x0F
#define REG_ERROR 0x10
#define REG_CO2_CAL 0x12
#define REG_LTA_TARGET 0x13
#define REG_LTA_PERIOD 0x14

#define CO2_MIN_RANGE 0
#define CO2_MAX_RANGE 10000

#define msg(x) printf("[%llu] [CDM7162] "x"\n", to_us_since_boot(get_absolute_time()) / 1000)

/**
 * @brief Sets device operation mode to power down
 * 
 * @return int32_t Return code
 */
int32_t cdm7162_deinit(void); // DO NOT USE

/**
 * @brief Writes configuration to the CDM7162 sensor
 * 
 * @param config Configuration to be written
 * @return int32_t Return code
 */
int32_t cdm7162_write_config(sensor_config_t* config);

int32_t cdm7162_read(uint8_t addr, uint8_t* buf, uint8_t num_bytes)
{
    int32_t ret;
    if ((ret = i2c_write_timeout_us(I2C_SENSOR, CDM7162_ADDR, &addr, 1, true, I2C_TIMEOUT_US)) < 0) return ret; // Write address
    
    busy_wait_ms(2);
    if ((ret = i2c_read_timeout_us(I2C_SENSOR, CDM7162_ADDR, buf, num_bytes, false, I2C_TIMEOUT_US)) < 0) return ret; // Read number of bytes from the address
    return SUCCESS;
}

int32_t cdm7162_write(uint8_t addr, uint8_t value)
{
    int32_t ret;
    uint8_t buf[2] = {addr, value};
    if ((ret = i2c_write_timeout_us(I2C_SENSOR, CDM7162_ADDR, buf, 2, true, I2C_TIMEOUT_US)) < 0) return ret; // Write value
    busy_wait_ms(2);
    if ((ret = i2c_read_timeout_us(I2C_SENSOR, CDM7162_ADDR, &buf[0], 1, false, I2C_TIMEOUT_US)) < 0) return ret; // Confirm saved value
    if (buf[0] != value) return CDM7162_ERROR_WRITE_RESP; // Incorrect value returned
    return SUCCESS;
}

int32_t cdm7162_reset(void)
{
    int32_t ret;
    if ((ret = cdm7162_write(REG_RST, 0x01)) != 0) return ret; // Write reset
    busy_wait_ms(100); // might need to be adjusted
    return SUCCESS;
}

int32_t cdm7162_write_config(sensor_config_t* config)
{
    int32_t ret;
    uint8_t buf;
    if ((ret = cdm7162_read(REG_OP_MODE, &buf, 1)) != 0) // Read operation mode
    {
        return ret; 
    }
    busy_wait_ms(10);
    if (buf != 0x06) // If not in measurement mode
    {
        if ((ret = cdm7162_write(REG_OP_MODE, 0x06)) != 0) // Set measurement mode
        return ret; 
        busy_wait_ms(100);
    }

    if ((ret = cdm7162_read(REG_FUNC, &buf, 1)) != 0) // Read set functions
        return ret; 
    busy_wait_ms(10);

    if (((buf & 0b100) >> 2) == (bool)(config->enable_pressure_comp) &&
        ((buf & 0b001) >> 0) == (bool)(config->enable_PWM_pin) &&
        ((buf & 0b001) >> 3) == (bool)(config->PWM_range_high) &&
        ((buf & 0b001) >> 4) == (bool)(config->long_term_adj_2) &&
        ((buf & 0b001) >> 5) == (bool)(config->long_term_adj_1)) 
        {
            return SUCCESS; // All functions correctly set
        }

    uint8_t func_settings = 0;
    if (config->enable_PWM_pin) func_settings |= (0b1 << 0);
    if (config->enable_pressure_comp) func_settings |= (0b1 << 2);
    if (config->PWM_range_high) func_settings |= (0b1 << 3);
    if (config->long_term_adj_2) func_settings |= (0b1 << 4);
    if (config->long_term_adj_1) func_settings |= (0b1 << 5);
    if ((ret = cdm7162_write(REG_FUNC, func_settings)) != 0) // Set functions
    {
        return ret; 
    }
    return SUCCESS;
}

int32_t cdm7162_init(sensor_t* cdm7162, sensor_config_t* config)
{
    int32_t ret;
    uint8_t buf;
    cdm7162->config = config; // Save config
    cdm7162_power(cdm7162, true); // Power on

    ret = cdm7162_write_config(config); // Write configuration to the sensor
    
    sleep_ms(100);
    cdm7162_power(cdm7162, false); // Power off
    return ret;
}

int32_t cdm7162_read_config(sensor_config_t* config)
{
    int32_t ret;
    uint8_t buf;
    if ((ret = cdm7162_read(REG_FUNC, &buf, 1)) != 0) return ret; // Read functions register
    config->enable_PWM_pin = buf & (0b1 << 0); // Save data from the function register
    config->enable_pressure_comp = buf & (0b1 << 2);
    config->PWM_range_high = buf & (0b1 << 3);
    config->long_term_adj_2 = buf & (0b1 << 4);
    config->long_term_adj_1 = buf & (0b1 << 5);
    return SUCCESS;
}

int32_t cdm7162_deinit(void)
{
    int32_t ret;
    uint8_t op_mode;
    if ((ret = cdm7162_read(REG_OP_MODE, &op_mode, 1)) != 0) return ret; // Read if sensor turned off

    if (op_mode == 0) return SUCCESS;

    if ((ret = cdm7162_write(REG_OP_MODE, 0x00)) != 0) return ret; // Turn sensor off
    busy_wait_ms(100);
    return SUCCESS;
}

int32_t cdm7162_set_atm_pressure(uint16_t pressure)
{
    int32_t ret;
    if (pressure < 800 || pressure > 1055) return CDM7162_ERROR_RANGE; // Pressure out of range
    uint8_t hpa = pressure - 800; // Convert to value for the sensor

    uint8_t hpa_reg;
    if ((ret = cdm7162_read(REG_ATM_PRESSURE, &hpa_reg, 1)) != 0) return ret; // Read set pressure
    if (hpa == hpa_reg) return SUCCESS;

    ret = cdm7162_write(REG_ATM_PRESSURE, hpa); // Write pressure value
    busy_wait_ms(100);
    return ret;
}

int32_t cdm7162_set_default_atm_pressure(void)
{
    return cdm7162_set_atm_pressure(1013); // Set pressure to 1013 hPa
}

void cdm7162_get_value(sensor_t* cdm7162)
{
    int32_t ret;
    uint8_t buf[2];
    switch (cdm7162->meas_state)
    {
        case MEAS_FINISHED: // Measurement finished
        {
            #ifdef DEBUG
            msg("Meas finished");
            #endif
            cdm7162_power(cdm7162, false); // Power off
            cdm7162->wake_time = make_timeout_time_ms(INT32_MAX); // Disable sensor timer
            return;
        }
        case MEAS_STARTED: // Measurement started
        {
            #ifdef DEBUG
            msg("Meas started");
            #endif
            cdm7162_power(cdm7162, true); // Power on
            cdm7162->wake_time = make_timeout_time_ms(750); // can be modified
            cdm7162->meas_state = MEAS_READ_STATUS; // Next step - read status
            cdm7162->timeout_iterator = 0; // Initialize read status timeout iterator
            return;
        }
        case MEAS_READ_STATUS: // Reading status
        {
            #ifdef DEBUG
            msg("Read status");
            #endif
            ret = cdm7162_read(REG_STATUS, &buf[0], 1); // Read status
            if (ret != 0) // On invalid read
            {
                cdm7162->co2 = INT16_MAX; // Set CO2 to unknown
                cdm7162->state = ret; // Output return state
                cdm7162->meas_state = MEAS_FINISHED; // Measurement finished
                return;
            }    
            if ((buf[0] & (0b1 << 7)) == 0) // Data is ready to be read
            {
                cdm7162->meas_state = MEAS_READ_VALUE; // Next step - read data
                return;
            }
            if (cdm7162->timeout_iterator++ > 20) // If in timeout
            {
                cdm7162->co2 = INT16_MAX; // Set CO2 to unknown
                cdm7162->meas_state = MEAS_FINISHED; // Measurement finished
                cdm7162->state = CDM7162_ERROR_DATA_READY_TIMEOUT; // Output TIMEOUT state
                return;
            }
            cdm7162->wake_time = make_timeout_time_ms(25); // Wait 25 ms until next status check
            return;
        }
        case MEAS_READ_VALUE: // Reading measured value
        {
            #ifdef DEBUG
            msg("Read value");
            #endif
            ret = cdm7162_read(REG_CO2_L, buf, 2); // Read measured CO2
            if (ret != 0) // On invalid read
            {
                cdm7162->co2 = INT16_MAX; // Set CO2 to unknown
                cdm7162->meas_state = MEAS_FINISHED; // Measurement finished
                cdm7162->state = ret; // Output return state
                return;
            }
            uint16_t val = *((uint16_t*)&buf[0]); // Convert read CO2 to uint16_t
            if (val < CO2_MIN_RANGE || val > CO2_MAX_RANGE) // If value out of range
            {
                cdm7162->co2 = INT16_MAX; // Set CO2 to unknown
                cdm7162->meas_state = MEAS_FINISHED; // Measurement finished
                cdm7162->state = CDM7162_ERROR_RANGE; // Output RANGE ERROR state
                return;
            }
            cdm7162->co2 = val; // Save measured CO2
            cdm7162->state = SUCCESS; // Output SUCCESS state
            cdm7162->meas_state = MEAS_FINISHED; // Measurement finished
            return;
        }
    }
}

void cdm7162_power(sensor_t* cdm7162, bool on)
{
    if (!cdm7162->config->power_global_control) // If power not controlled globally
    {
        // Read power vector
        // Check if bit turned [on]
        // Write power vector
    }
}
