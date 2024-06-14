#include "cdm7162.h"

#define CDM7162_ADDR 0x68
#define CDM7162_I2C i2c0

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

/**
 * @brief Sets device operation mode to power down
 * 
 * @return int32_t Return code
 */
int32_t cdm7162_deinit(void); // DO NOT USE

int32_t cdm7162_read(uint8_t addr, uint8_t* buf, uint8_t num_bytes)
{
    int32_t ret;
    if ((ret = i2c_write_timeout_per_char_us(CDM7162_I2C, CDM7162_ADDR, &addr, 1, true, 1000)) < 0) return ret; // Write address
    
    busy_wait_ms(2);
    if ((ret = i2c_read_timeout_per_char_us(CDM7162_I2C, CDM7162_ADDR, buf, num_bytes, false, 1000)) < 0) return ret; // Read number of bytes from the address
    return SUCCESS;
}

int32_t cdm7162_write(uint8_t addr, uint8_t value)
{
    int32_t ret;
    uint8_t buf[2] = {addr, value};
    if ((ret = i2c_write_timeout_per_char_us(CDM7162_I2C, CDM7162_ADDR, buf, 2, true, 1000)) < 0) return ret; // Write value
    busy_wait_ms(2);
    if ((ret = i2c_read_timeout_per_char_us(CDM7162_I2C, CDM7162_ADDR, &buf[0], 1, false, 1000)) < 0) return ret; // Confirm saved value
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

int32_t cdm7162_init(cdm7162_t* cdm7162, cdm7162_config_t* config)
{
    int32_t ret;
    uint8_t buf;
    cdm7162_init_struct(cdm7162); // Initialize CDM7162 structure
    cdm7162->config = config; // Save config
    cdm7162_power(cdm7162, true); // Power on

    if ((ret = cdm7162_read(REG_OP_MODE, &buf, 1)) != 0) return ret; // Read operation mode
    busy_wait_ms(10);
    if (buf != 0x06) // If not in measurement mode
    {
        if ((ret = cdm7162_write(REG_OP_MODE, 0x06)) != 0) return ret; // Set measurement mode
        busy_wait_ms(100);
    }

    if ((ret = cdm7162_read(REG_FUNC, &buf, 1)) != 0) return ret; // Read set functions
    busy_wait_ms(10);

    if (((buf & 0b100) >> 2) == (bool)(config->pressure_corr) &&
        ((buf & 0b001) >> 0) == (bool)(config->enable_PWM_pin) &&
        ((buf & 0b001) >> 3) == (bool)(config->PWM_range_high) &&
        ((buf & 0b001) >> 4) == (bool)(config->long_term_adj_2) &&
        ((buf & 0b001) >> 5) == (bool)(config->long_term_adj_1)) return SUCCESS; // All functions correctly set

    uint8_t func_settings = 0;
    if (config->enable_PWM_pin) func_settings |= (0b1 << 0);
    if (config->pressure_corr) func_settings |= (0b1 << 2);
    if (config->PWM_range_high) func_settings |= (0b1 << 3);
    if (config->long_term_adj_2) func_settings |= (0b1 << 4);
    if (config->long_term_adj_1) func_settings |= (0b1 << 5);
    if ((ret = cdm7162_write(REG_FUNC, func_settings)) != 0) return ret; // Set functions
    busy_wait_ms(100);
    cdm7162_power(cdm7162, false); // Power off
    return SUCCESS;
}

int32_t cdm7162_read_config(cdm7162_config_t* config)
{
    int32_t ret;
    uint8_t buf;
    if ((ret = cdm7162_read(REG_FUNC, &buf, 1)) != 0) return ret; // Read functions register
    config->enable_PWM_pin = buf & (0b1 << 0); // Save data from the function register
    config->pressure_corr = buf & (0b1 << 2);
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

void cdm7162_get_value(cdm7162_t* cdm7162)
{
    int32_t ret;
    static int32_t i;
    uint8_t buf[2];
    switch (cdm7162->meas_state)
    {
        case CDM7162_MEAS_FINISHED: // Measurement finished
        {
            cdm7162_power(cdm7162, false); // Power off
            cdm7162->wake_time = make_timeout_time_ms(INT32_MAX); // Disable sensor timer
            return;
        }
        case CDM7162_MEAS_START: // Measurement started
        {
            cdm7162_power(cdm7162, true); // Power on
            cdm7162->wake_time = make_timeout_time_ms(750); // can be modified
            cdm7162->meas_state = CDM7162_READ_STATUS; // Next step - read status
            i = 0; // Initialize read status timeout iterator
            return;
        }
        case CDM7162_READ_STATUS: // Reading status
        {
            ret = cdm7162_read(REG_STATUS, &buf[0], 1); // Read status
            if (ret != 0) // On invalid read
            {
                cdm7162->co2 = INT16_MAX; // Set CO2 to unknown
                cdm7162->state = ret; // Output return state
                cdm7162->meas_state = CDM7162_MEAS_FINISHED; // Measurement finished
                return;
            }    
            if ((buf[0] & (0b1 << 7)) == 0) // Data is ready to be read
            {
                cdm7162->meas_state = CDM7162_READ_VALUE; // Next step - read data
                return;
            }
            if (i++ > 20) // If in timeout
            {
                cdm7162->co2 = INT16_MAX; // Set CO2 to unknown
                cdm7162->meas_state = CDM7162_MEAS_FINISHED; // Measurement finished
                cdm7162->state = CDM7162_ERROR_DATA_READY_TIMEOUT; // Output TIMEOUT state
                return;
            }
            cdm7162->wake_time = make_timeout_time_ms(25); // Wait 25 ms until next status check
            return;
        }
        case CDM7162_READ_VALUE: // Reading measured value
        {
            ret = cdm7162_read(REG_CO2_L, buf, 2); // Read measured CO2
            if (ret != 0) // On invalid read
            {
                cdm7162->co2 = INT16_MAX; // Set CO2 to unknown
                cdm7162->meas_state = CDM7162_MEAS_FINISHED; // Measurement finished
                cdm7162->state = ret; // Output return state
                return;
            }
            uint16_t val = *((uint16_t*)&buf[0]); // Convert read CO2 to uint16_t
            if (val < CO2_MIN_RANGE || val > CO2_MAX_RANGE) // If value out of range
            {
                cdm7162->co2 = INT16_MAX; // Set CO2 to unknown
                cdm7162->meas_state = CDM7162_MEAS_FINISHED; // Measurement finished
                cdm7162->state = CDM7162_ERROR_RANGE; // Output RANGE ERROR state
                return;
            }
            cdm7162->co2 = val; // Save measured CO2
            cdm7162->state = SUCCESS; // Output SUCCESS state
            cdm7162->meas_state = CDM7162_MEAS_FINISHED; // Measurement finished
            return;
        }
    }
}

void cdm7162_init_struct(cdm7162_t* cdm7162)
{
    cdm7162->co2 = 0; // Init CO2 concentration
    cdm7162->meas_state = (cdm7162_meas_state_e)0; // Set measurement state to measurement not started
    cdm7162->state = ERROR_SENSOR_NOT_INITIALIZED; // Set sensor state to not initialized
    cdm7162->wake_time = make_timeout_time_ms(INT32_MAX); // Disable timer
}

void cdm7162_power(cdm7162_t* cdm7162, bool on)
{
    if (!cdm7162->config->power_global_control)
    {
        // Read power vector
        // Check if bit turned [on]
        // Write power vector
    }
}
