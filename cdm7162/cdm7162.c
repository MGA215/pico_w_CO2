/**
 * @file cdm7162.c
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Implements communication with Figaro CDM7162 sensor
 * @version 0.1
 * @date 2024-06-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "cdm7162.h"
#include <stdio.h>

#define CDM7162_ADDR        0x68

#define REG_RST             0x00
#define REG_OP_MODE         0x01
#define REG_STATUS          0x02
#define REG_CO2_L           0x03
#define REG_CO2_H           0x04
#define REG_ATM_PRESSURE    0x09
#define REG_ALTITUDE        0x0A
#define REG_ALARM_CO2_H     0x0C
#define REG_ALARM_CO2_L     0x0D
#define REG_CAL             0x0E
#define REG_FUNC            0x0F
#define REG_ERROR           0x10
#define REG_CO2_CAL         0x12
#define REG_LTA_TARGET      0x13
#define REG_LTA_PERIOD      0x14

#define CO2_MIN_RANGE       0
#define CO2_MAX_RANGE       10000

#define msg(severity, x) printf("[%12llu] ["severity"] [CDM7162] "x"\n", to_us_since_boot(get_absolute_time()) / 1000)


/**
 * @brief Reads data from cdm7162 sensor
 * 
 * @param addr register address to read from
 * @param buf output buffer
 * @param num_bytes number of bytes to read
 * @return int32_t error code
 */
static int32_t cdm_read(uint8_t addr, uint8_t* buf, uint8_t num_bytes);

/**
 * @brief Writes data to cdm7162 sensor
 * 
 * @param addr Register address to write to
 * @param value Value to write
 * @return int32_t Return code
 */
static int32_t cdm_write(uint8_t addr, uint8_t value);

/**
 * @brief Writes configuration to the CDM7162 sensor
 * 
 * @param config Configuration to be written
 * @return int32_t Return code
 */
static int32_t cdm_write_config(sensor_config_t* config);

/**
 * @brief Switches sensor power [on] if not controlled globally
 * 
 * @param cdm7162 sensor structure
 * @param on if the power should be switched on (true) or off (false)
 */
static inline void cdm_power(sensor_t* cdm7162, bool on);

/**
 * @brief Sets device operation mode to power down
 * 
 * @return int32_t Return code
 */
static int32_t cdm_deinit(void); // DO NOT USE - Writes to EEPROM


static int32_t cdm_read(uint8_t addr, uint8_t* buf, uint8_t num_bytes)
{
    int32_t ret;
    if ((ret = i2c_write_timeout_us(I2C_SENSOR, CDM7162_ADDR, &addr, 1, true, I2C_TIMEOUT_US)) < 0) return ret; // Write address
    
    busy_wait_ms(2);
    if ((ret = i2c_read_timeout_us(I2C_SENSOR, CDM7162_ADDR, buf, num_bytes, false, I2C_TIMEOUT_US)) < 0) return ret; // Read number of bytes from the address
    return SUCCESS;
}

static int32_t cdm_write(uint8_t addr, uint8_t value)
{
    int32_t ret;
    uint8_t buf[2] = {addr, value};
    if ((ret = i2c_write_timeout_us(I2C_SENSOR, CDM7162_ADDR, buf, 2, true, I2C_TIMEOUT_US)) < 0) return ret; // Write value
    busy_wait_ms(2);
    if ((ret = i2c_read_timeout_us(I2C_SENSOR, CDM7162_ADDR, &buf[0], 1, false, I2C_TIMEOUT_US)) < 0) return ret; // Confirm saved value
    if (buf[0] != value) return CDM7162_ERROR_WRITE_RESP; // Incorrect value returned
    return SUCCESS;
}

void cdm7162_get_value(sensor_t* cdm7162)
{
    int32_t ret;
    uint8_t buf[2];
    switch (cdm7162->meas_state)
    {
        case MEAS_FINISHED: // Measurement finished
        {
            #if DEBUG_INFO
            msg("info", "Meas finished");
            #endif
            cdm_power(cdm7162, false); // Power off
            cdm7162->wake_time = at_the_end_of_time; // Disable sensor timer
            return;
        }
        case MEAS_STARTED: // Measurement started
        {
            #if DEBUG_INFO
            msg("info", "Meas started");
            #endif
            cdm_power(cdm7162, true); // Power on
            cdm7162->wake_time = make_timeout_time_ms(750); // can be modified
            cdm7162->meas_state = MEAS_READ_STATUS; // Next step - read status
            cdm7162->timeout_iterator = 0; // Initialize read status timeout iterator
            return;
        }
        case MEAS_READ_STATUS: // Reading status
        {
            #if DEBUG_INFO
            msg("info", "Read status");
            #endif
            ret = cdm_read(REG_STATUS, &buf[0], 1); // Read status
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
            #if DEBUG_INFO
            msg("info", "Read value");
            #endif
            ret = cdm_read(REG_CO2_L, buf, 2); // Read measured CO2
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

int32_t cdm7162_init(sensor_t* cdm7162, sensor_config_t* config)
{
    int32_t ret;
    uint8_t buf;
    cdm7162->config = config; // Save config
    cdm_power(cdm7162, true); // Power on

    ret = cdm_write_config(config); // Write configuration to the sensor
    
    sleep_ms(100);
    cdm_power(cdm7162, false); // Power off
    return ret;
}

int32_t cdm7162_read_config(sensor_config_t* config)
{
    int32_t ret;
    uint8_t buf[4] = {0xFF};
    if ((ret = cdm_read(REG_FUNC, buf, 1)) != 0) return ret; // Read functions register
    printf("enable pwm pin: %i\n", config->enable_PWM_pin);
    printf("read enable pwm pin: %i\n", (bool)(buf[0] & (0b1 << 0)));
    config->enable_PWM_pin = (bool)(buf[0] & (0b1 << 0)); // Save data from the function register
    printf("set pwm pin: %i\n", config->enable_PWM_pin);
    config->enable_pressure_comp = buf[0] & (0b1 << 2);
    config->PWM_range_high = buf[0] & (0b1 << 3);
    config->long_term_adj_2 = buf[0] & (0b1 << 4);
    config->long_term_adj_1 = buf[0] & (0b1 << 5);

    if ((ret = cdm_read(REG_ATM_PRESSURE, buf, 1)) != 0) return ret; // Read saved pressure
    config->pressure = (uint16_t)buf[0] + 800u;

    config->enable_altitude_comp = config->enable_pressure_comp;
    if ((ret = cdm_read(REG_ALTITUDE, buf, 1)) != 0) return ret; // Read saved altitude
    config->altitude = (uint16_t)buf[0] * 10u;

    if ((ret = cdm_read(REG_ALARM_CO2_H, buf, 1)) != 0) return ret; // Read saved alarm high treshold
    config->alarm_treshold_co2_high = (uint16_t)buf[0] * 10u;

    if ((ret = cdm_read(REG_ALARM_CO2_L, buf, 1)) != 0) return ret; // Read saved alarm low treshold
    config->alarm_treshold_co2_low = (uint16_t)buf[0] * 10u;

    if ((ret = cdm_read(REG_LTA_TARGET, buf, 1)) != 0) return ret; // Read saved LTA target value
    config->target_LTA = (uint16_t)buf[0] + 300u;

    if ((ret = cdm_read(REG_LTA_PERIOD, buf, 1)) != 0) return ret; // Read saved LTA period
    config->period_LTA = (uint16_t)(buf[0] & 0b00111111);
    if (buf[0] & (0b1 << 6)) config->period_LTA *= 7; // Week bit H
    else if (buf[0] & (0b1 << 7)) config->period_LTA *= 30; // Month bit H
    return SUCCESS;
}

static int32_t cdm_write_config(sensor_config_t* config)
{
    int32_t ret;
    uint8_t buf;
    if ((ret = cdm_read(REG_OP_MODE, &buf, 1)) != 0) // Read operation mode
    {
        return ret; 
    }
    busy_wait_ms(10);
    if (buf != 0x06) // If not in measurement mode
    {
        if ((ret = cdm_write(REG_OP_MODE, 0x06)) != 0) // Set measurement mode
            return ret; 
        busy_wait_ms(100);
    }
    sensor_config_t read_config;
    if ((ret = cdm7162_read_config(&read_config)) != 0) return ret; // Read configuration
    printf("outside: %i\n", read_config.enable_PWM_pin);
    busy_wait_ms(10);

    if (!(read_config.enable_PWM_pin == config->enable_PWM_pin && // Check for different flag
        ((read_config.enable_pressure_comp && (config->enable_pressure_comp || config->enable_altitude_comp)) ||
        (!read_config.enable_pressure_comp && !config->enable_pressure_comp && !config->enable_altitude_comp)) &&
        read_config.PWM_range_high == config->PWM_range_high &&
        read_config.long_term_adj_1 == config->long_term_adj_1 &&
        read_config.long_term_adj_2 == config->long_term_adj_2))
    {
        uint8_t func_settings = 0;
        if (config->enable_PWM_pin) func_settings |= (0b1 << 0);
        if (config->enable_pressure_comp) func_settings |= (0b1 << 2);
        if (config->PWM_range_high) func_settings |= (0b1 << 3);
        if (config->long_term_adj_2) func_settings |= (0b1 << 4);
        if (config->long_term_adj_1) func_settings |= (0b1 << 5);
        if ((ret = cdm_write(REG_FUNC, func_settings)) != 0) // Set functions
        {
            return ret; 
        }
    }

    if (read_config.pressure != config->pressure) // Check pressure
    {
        if ((ret = cdm_write(REG_ATM_PRESSURE, (uint8_t)(config->pressure - 800))) != 0) return ret; // Write pressure
    }
    if (read_config.altitude != config->altitude) // Check altitude
    {
        if ((ret = cdm_write(REG_ALTITUDE, (uint8_t)(config->altitude / 10))) != 0) return ret; // Write altitude
    }

    if (read_config.alarm_treshold_co2_high != config->alarm_treshold_co2_high) // Check high alarm
    {
        if ((ret = cdm_write(REG_ALARM_CO2_H, (uint8_t)(config->alarm_treshold_co2_high / 10))) != 0) return ret; // Write high alarm
    }
    if (read_config.alarm_treshold_co2_low != config->alarm_treshold_co2_low) // Check low alarm
    {
        if ((ret = cdm_write(REG_ALARM_CO2_L, (uint8_t)(config->alarm_treshold_co2_low / 10))) != 0) return ret; // Write low alarm
    }

    if (read_config.target_LTA != config->target_LTA) // Check target LTA
    {
        if ((ret = cdm_write(REG_LTA_TARGET, (uint8_t)(config->target_LTA - 300))) != 0) return ret; // Write target LTA concentration
    }
    
    if (read_config.period_LTA != config->period_LTA) // Check period LTA
    {
        buf = 0;
        uint8_t val;
        if (config->period_LTA % 30 == 0)
        {
            buf &= (0b1 << 7); // Set months bit
            val = config->period_LTA / 30;
        }
        else if (config->period_LTA % 7 == 0)
        {
            buf &= (0b1 << 6);
            val = config->period_LTA / 7;
        }
        if (val < 64)
        {
            buf += val;
            if ((ret = cdm_write(REG_LTA_PERIOD, buf)) != 0) return ret; // Write period LTA
        }
    }
    return SUCCESS;
}

static inline void cdm_power(sensor_t* cdm7162, bool on)
{
    if (!cdm7162->config->power_global_control) // If power not controlled globally
    {
        // Read power vector
        // Check if bit turned [on]
        // Write power vector
    }
}

int32_t cdm7162_reset(void)
{
    int32_t ret;
    if ((ret = cdm_write(REG_RST, 0x01)) != 0) return ret; // Write reset
    busy_wait_ms(100); // might need to be adjusted
    return SUCCESS;
}

static int32_t cdm_deinit(void)
{
    int32_t ret;
    uint8_t op_mode;
    if ((ret = cdm_read(REG_OP_MODE, &op_mode, 1)) != 0) return ret; // Read if sensor turned off

    if (op_mode == 0) return SUCCESS;

    if ((ret = cdm_write(REG_OP_MODE, 0x00)) != 0) return ret; // Turn sensor off
    busy_wait_ms(100);
    return SUCCESS;
}


