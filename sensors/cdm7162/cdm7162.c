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
#include "math.h"
#include "hardware/i2c.h"
#include "string.h"

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
    if ((ret = i2c_write_timeout_us(I2C_SENSOR, CDM7162_ADDR, &addr, 1, true, I2C_TIMEOUT_US * 3)) < 0) return ret; // Write address
    
    busy_wait_ms(2);
    if ((ret = i2c_read_timeout_us(I2C_SENSOR, CDM7162_ADDR, buf, num_bytes, false, I2C_TIMEOUT_US * 3)) < 0) return ret; // Read number of bytes from the address
    return SUCCESS;
}

static int32_t cdm_write(uint8_t addr, uint8_t value)
{
    int32_t ret;
    uint8_t buf[2] = {addr, value};
    if ((ret = i2c_write_timeout_us(I2C_SENSOR, CDM7162_ADDR, buf, 2, true, I2C_TIMEOUT_US * 3)) < 0) return ret; // Write value
    busy_wait_ms(2);
    if ((ret = i2c_read_timeout_us(I2C_SENSOR, CDM7162_ADDR, &buf[0], 1, false, I2C_TIMEOUT_US * 3)) < 0) return ret; // Confirm saved value
    if (buf[0] != value) return CDM7162_ERROR_WRITE_RESP; // Incorrect value returned
    sleep_ms(100);
    return SUCCESS;
}

void cdm7162_get_value(sensor_t* cdm7162)
{
    int32_t ret;
    uint8_t buf[2];
    if (cdm7162->config.sensor_type != CDM7162) // Check for correct sensor type
    {
        cdm7162->meas_state = MEAS_FINISHED;
        cdm7162->state = ERROR_UNKNOWN_SENSOR;
        cdm7162->co2 = NAN;
        return;
    } 
    switch (cdm7162->meas_state)
    {
        case MEAS_FINISHED: // Measurement finished
        {
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_CDM7162, "Meas finished");
            cdm_power(cdm7162, false); // Power off
            cdm7162->wake_time = at_the_end_of_time; // Disable sensor timer
            return;
        }
        case MEAS_STARTED: // Measurement started
        {
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_CDM7162, "Meas started");
            cdm_power(cdm7162, true); // Power on
            cdm7162->wake_time = make_timeout_time_ms(750); // can be modified
            cdm7162->meas_state = MEAS_READ_STATUS; // Next step - read status
            cdm7162->state = SUCCESS;
            cdm7162->timeout_iterator = 0; // Initialize read status timeout iterator
            return;
        }
        case MEAS_READ_STATUS: // Reading status
        {
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_CDM7162, "Read status");
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
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_CDM7162, "Read value");
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
        default:
        {
            cdm7162->meas_state = MEAS_FINISHED;
            return;
        }
    }
}

int32_t cdm7162_init(sensor_t* cdm7162, sensor_config_t* config)
{
    int32_t ret;
    uint8_t buf;
    if (config->sensor_type != CDM7162) return ERROR_UNKNOWN_SENSOR; // Check for correct sensor type
    memcpy(&cdm7162->config, config, sizeof(sensor_config_t)); // Save config
    //cdm7162->config = &config; // Save config
    cdm_power(cdm7162, true); // Power on

    ret = cdm_write_config(config); // Write configuration to the sensor
    sleep_ms(100);
    cdm_power(cdm7162, false); // Power off
    if (!ret)
    {
        if (cdm7162->meas_state == MEAS_STARTED) cdm7162->wake_time = make_timeout_time_ms(3000);
    }
    else cdm7162->meas_state = MEAS_FINISHED;
    return ret;
}

int32_t cdm7162_read_config(sensor_config_t* config)
{
    int32_t ret;
    uint8_t buf[4] = {0xFF};
    config->sensor_type = CDM7162;

    do {
        if ((ret = cdm_read(REG_STATUS, buf, 1)) != 0) return ret; // Check for busy
        sleep_ms(250);
    } while (buf[0] & (0b1 << 7));

    if ((ret = cdm_read(REG_FUNC, buf, 1)) != 0) return ret; // Read functions register
    config->enable_PWM_pin = (bool)(buf[0] & (0b1 << 0)); // Save data from the function register
    config->enable_pressure_comp = buf[0] & (0b1 << 2);
    config->PWM_range_high = buf[0] & (0b1 << 3);
    config->enable_alternate_abc = buf[0] & (0b1 << 4);
    config->enable_abc = buf[0] & (0b1 << 5);

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
    config->abc_target_value = (uint16_t)buf[0] + 300u;

    if ((ret = cdm_read(REG_LTA_PERIOD, buf, 1)) != 0) return ret; // Read saved LTA period
    config->abc_period = (uint16_t)(buf[0] & 0b00111111);
    if (buf[0] & (0b1 << 6)) config->abc_period *= 7; // Week bit H
    else if (buf[0] & (0b1 << 7)) config->abc_period *= 30; // Month bit H
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
        print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_CDM7162, "Config - Writing OP mode");
        if ((ret = cdm_write(REG_OP_MODE, 0x06)) != 0) // Set measurement mode
            return ret; 
        busy_wait_ms(100);
    }
    sensor_config_t read_config;
    bool power_down = false;
    if ((ret = cdm7162_read_config(&read_config)) != 0) return ret; // Read configuration
    
    do {
        if ((ret = cdm_read(REG_STATUS, &buf, 1)) != 0) return ret; // Check for busy
        sleep_ms(250);
    } while (buf & (0b1 << 7));

    if (!(read_config.enable_PWM_pin == config->enable_PWM_pin && // Check for different flag
        ((read_config.enable_pressure_comp && (config->enable_pressure_comp || config->enable_altitude_comp)) ||
        (!read_config.enable_pressure_comp && !config->enable_pressure_comp && !config->enable_altitude_comp)) &&
        read_config.PWM_range_high == config->PWM_range_high &&
        read_config.enable_abc == config->enable_abc &&
        read_config.enable_alternate_abc == config->enable_alternate_abc))
    {
        print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_CDM7162, "Config - Writing function flags");
        if (!power_down)
        {
            if ((ret = cdm_write(REG_OP_MODE, 0x00)) != 0) return ret;
            power_down = true;
            busy_wait_ms(100);
        }
        uint8_t func_settings = 0;
        if (config->enable_PWM_pin) func_settings |= (0b1 << 0);
        if (config->enable_pressure_comp) func_settings |= (0b1 << 2);
        if (config->PWM_range_high) func_settings |= (0b1 << 3);
        if (config->enable_alternate_abc) func_settings |= (0b1 << 4);
        if (config->enable_abc) func_settings |= (0b1 << 5);
        if ((ret = cdm_write(REG_FUNC, func_settings)) != 0) // Set functions
        {
            return ret; 
        }
    }

    if (read_config.pressure != config->pressure) // Check pressure
    {
        print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_CDM7162, "Config - Writing pressure");
        if (!power_down)
        {
            if ((ret = cdm_write(REG_OP_MODE, 0x00)) != 0) return ret;
            power_down = true;
            busy_wait_ms(100);
        }
        if ((ret = cdm_write(REG_ATM_PRESSURE, (uint8_t)(config->pressure - 800))) != 0) return ret; // Write pressure
    }
    if (read_config.altitude != config->altitude) // Check altitude
    {
        print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_CDM7162, "Config - Writing altitude");
        if (!power_down)
        {
            if ((ret = cdm_write(REG_OP_MODE, 0x00)) != 0) return ret;
            power_down = true;
            busy_wait_ms(100);
        }
        if ((ret = cdm_write(REG_ALTITUDE, (uint8_t)(config->altitude / 10))) != 0) return ret; // Write altitude
    }

    if (read_config.alarm_treshold_co2_high != config->alarm_treshold_co2_high) // Check high alarm
    {
        print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_CDM7162, "Config - Writing alarm high treshold");
        if (!power_down)
        {
            if ((ret = cdm_write(REG_OP_MODE, 0x00)) != 0) return ret;
            power_down = true;
            busy_wait_ms(100);
        }
        if ((ret = cdm_write(REG_ALARM_CO2_H, (uint8_t)(config->alarm_treshold_co2_high / 10))) != 0) return ret; // Write high alarm
    }
    if (read_config.alarm_treshold_co2_low != config->alarm_treshold_co2_low) // Check low alarm
    {
        print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_CDM7162, "Config - Writing alarm low treshold");
        if (!power_down)
        {
            if ((ret = cdm_write(REG_OP_MODE, 0x00)) != 0) return ret;
            power_down = true;
            busy_wait_ms(100);
        }
        if ((ret = cdm_write(REG_ALARM_CO2_L, (uint8_t)(config->alarm_treshold_co2_low / 10))) != 0) return ret; // Write low alarm
    }

    if (read_config.abc_target_value != config->abc_target_value) // Check target LTA
    {
        print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_CDM7162, "Config - Writing LTA target value");
        if (!power_down)
        {
            if ((ret = cdm_write(REG_OP_MODE, 0x00)) != 0) return ret;
            power_down = true;
            busy_wait_ms(100);
        }
        if ((ret = cdm_write(REG_LTA_TARGET, (uint8_t)(config->abc_target_value - 300))) != 0) return ret; // Write target LTA concentration
    }
    
    if (read_config.abc_period != config->abc_period) // Check period LTA
    {
        buf = 0;
        uint8_t val;
        if (config->abc_period % 30 == 0)
        {
            buf &= (0b1 << 7); // Set months bit
            val = config->abc_period / 30;
        }
        else if (config->abc_period % 7 == 0)
        {
            buf &= (0b1 << 6);
            val = config->abc_period / 7;
        }
        if (val < 64)
        {
            print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_CDM7162, "Config - Writing LTA period");
            if (!power_down)
            {
                if ((ret = cdm_write(REG_OP_MODE, 0x00)) != 0) return ret;
                power_down = true;
                busy_wait_ms(100);
            }
            buf += val;
            if ((ret = cdm_write(REG_LTA_PERIOD, buf)) != 0) return ret; // Write period LTA
        }
    }
    
    if (power_down) // If config changed turn power back on
    {
        if ((ret = cdm_write(REG_OP_MODE, 0x06)) != 0) // Set measurement mode
            return ret; 
        busy_wait_ms(100);
    }
    return SUCCESS;
}

static inline void cdm_power(sensor_t* cdm7162, bool on)
{
    if (!cdm7162->config.power_global_control) // If power not controlled globally
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


