/**
 * @file cm1107n.c
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief 
 * @version 0.1
 * @date 2024-07-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "cm1107n.h"
#include "math.h"
#include "hardware/i2c.h"
#include "string.h"

#define CM1107N_ADDR    0x31

#define CMD_READ_DATA   0x01 // Response length 3 bytes
#define CMD_ABC         0x10 
#define CMD_FORCE_CAL   0x03
#define CMD_SW_VERSION  0x1E // Response length 10 bytes
#define CMD_SER_NUM     0x1F // Response length 10 bytes

/**
 * @brief returns error code according to the error register value
 * 
 * @param error_reg error register
 * @return int returned error code 
 */
static inline int cm_get_error(uint8_t error_reg);

/**
 * @brief Reads data from the CM1107N sensor
 * 
 * @param addr Command (register to read from)
 * @param buf Read data (MSB first)
 * @param len Length of the data to read (in bytes)
 * @return int32_t Return code
 */
static int32_t cm_read(uint8_t addr, uint8_t* buf, uint8_t len);

/**
 * @brief Writes data to the CM1107N sensor
 * 
 * @param addr Command (register to write to)
 * @param data Data to write (MSB first)
 * @param len Length of the data (in bytes)
 * @return int32_t Return code
 */
static int32_t cm_write(uint8_t addr, uint8_t* data, uint8_t len);

/**
 * @brief Sends a command to the sensor
 * 
 * @param addr Command to send
 * @return int32_t Return code
 */
static int32_t cm_write_command(uint8_t addr);

/**
 * @brief Writes configuration to the sensor
 * 
 * @param config Configuration to write
 * @return int32_t Return code
 */
static int32_t cm_write_config(sensor_config_t* config);

/**
 * @brief Turns the sensor power [on]
 * 
 * @param cm1107n Sensor structure
 * @param on Whether the power should be turned on
 */
static inline void cm_power(sensor_t* cm1107n, bool on);


static inline int cm_get_error(uint8_t error_reg)
{
    if (error_reg & (0b1 << 0)) return CM1107N_ERROR_PREHEATING;
    if (error_reg & (0b1 << 1)) return CM1107N_ERROR_FATAL;
    if (error_reg & (0b1 << 2)) return CM1107N_ERROR_OUT_OF_RANGE;
    if (error_reg & (0b1 << 3)) return CM1107N_ERROR_OUT_OF_RANGE;
    if (error_reg & (0b1 << 4)) return CM1107N_ERROR_CAL;
    if (error_reg & (0b1 << 5)) return CM1107N_ERROR_LIGHT_AGING;
    if (error_reg & (0b1 << 6)) return CM1107N_ERROR_DRIFT;
    return SUCCESS;
}

static int32_t cm_read(uint8_t addr, uint8_t* buf, uint8_t len)
{
    int32_t ret;
    uint8_t command_buffer[len + 2]; // Data length + command + checksum
    memset(command_buffer, 0x00, len + 2); // Clear command buffer
    command_buffer[0] = addr; // Set address

    if ((ret = i2c_write_timeout_us(I2C_SENSOR, CM1107N_ADDR, command_buffer, 1, true, I2C_TIMEOUT_US)) < 0) return ret; // Send command

    sleep_ms(1);
    if ((ret = i2c_read_timeout_us(I2C_SENSOR, CM1107N_ADDR, command_buffer, len + 2, false, I2C_TIMEOUT_US)) < 0) return ret; // Read response

    uint16_t cs_neg = 0;
    for (int i = 0; i < len + 1; i++) // Create CS negation
    {
        cs_neg += command_buffer[i];
    }
    if (((cs_neg + command_buffer[len + 1]) & 0xFF) != 0x00) return CM1107N_ERROR_CRC; // Check checksum

    memcpy(buf, &command_buffer[1], len); // Copy read data
    return SUCCESS;
}

static int32_t cm_write(uint8_t addr, uint8_t *data, uint8_t len)
{
    int32_t ret;
    uint8_t command_buffer[len + 2]; // Data length + command + checksum
    memset(command_buffer, 0x00, len + 2); // Clear command buffer
    command_buffer[0] = addr; // Set address
    for (int i = 0; i < len; i++) // Add data
    {
        command_buffer[i + 1] = data[i];
    }

    if ((ret = i2c_write_timeout_us(I2C_SENSOR, CM1107N_ADDR, command_buffer, len + 1, true, I2C_TIMEOUT_US)) < 0) return ret; // Send data

    sleep_ms(1);
    if ((ret = i2c_read_timeout_us(I2C_SENSOR, CM1107N_ADDR, command_buffer, len + 2, false, I2C_TIMEOUT_US)) < 0) return ret; // Read response

    uint16_t cs_neg = 0;
    for (int i = 0; i < len + 1; i++) // Create CS negation
    {
        cs_neg += command_buffer[i];
    }
    if (((cs_neg + command_buffer[len + 1]) & 0xFF) != 0x00) return CM1107N_ERROR_CRC; // Check checksum
    return SUCCESS;
}

static int32_t cm_write_command(uint8_t addr)
{
    int32_t ret;
    if ((ret = i2c_write_timeout_us(I2C_SENSOR, CM1107N_ADDR, &addr, 1, false, I2C_TIMEOUT_US)) < 0) return ret;
    return SUCCESS;
}

void cm1107n_get_value(sensor_t* cm1107n)
{
    int32_t ret;
    uint8_t tempBuffer[3];
    if (cm1107n->config->sensor_type != CM1107N) // Check for correct sensor type
    {
        cm1107n->meas_state = MEAS_FINISHED;
        cm1107n->state = ERROR_UNKNOWN_SENSOR;
        cm1107n->co2 = NAN;
        return;
    } 
    switch(cm1107n->meas_state)
    {
        case MEAS_FINISHED:
        {
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_CM1107N, "Meas finished");
            cm_power(cm1107n, false); // Power off
            cm1107n->wake_time = at_the_end_of_time; // Disable timer
            return;
        }
        case MEAS_STARTED:
        {
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_CM1107N, "Meas started");
            cm_power(cm1107n, true); // Power on
            cm1107n->wake_time = make_timeout_time_ms(1000); // Timeout in 1 s - power stabilization
            cm1107n->state = SUCCESS;
            cm1107n->meas_state = MEAS_TRIGGER_SINGLE_MEAS; // Next FSM state - trigger measurement
            cm1107n->timeout_iterator = 0;
            return;
        }
        case MEAS_TRIGGER_SINGLE_MEAS:
        {
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_CM1107N, "Trigger measurement");
            ret = cm_write_command(CMD_READ_DATA); // Sent measurement trigger
            if (ret != 0) // On invalid write
            {
                cm1107n->co2 = NAN; // Set read value to NAN
                cm1107n->meas_state = MEAS_FINISHED; // Set FSM state to measurement finished
                cm1107n->state = ret; // Set state to ret value
                return;
            }
            cm1107n->wake_time = make_timeout_time_ms(2000); // Check after 2 seconds if measuement finished (should be 1 second)
            cm1107n->meas_state = MEAS_READ_VALUE; // Next FSM state - read value
            return;
        }
        case MEAS_READ_VALUE:
        {
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_CM1107N, "Read value");
            ret = cm_read(CMD_READ_DATA, tempBuffer, 3); // Read measurement
            if (ret != 0) // On invalid read
            {
                cm1107n->co2 = NAN; // Set read value to NAN
                cm1107n->meas_state = MEAS_FINISHED; // Set FSM state to measurement finished
                cm1107n->state = ret; // Set state to ret value
                return;
            }
            if (tempBuffer[2] != 0x00) // Check for sensor errors
            {
                ret = cm_get_error(tempBuffer[2]); // Get error
                if (ret == CM1107N_ERROR_PREHEATING)
                {
                    if (++(cm1107n->timeout_iterator) > 4) // 4 second preheating timeout
                    {
                        cm1107n->meas_state = MEAS_FINISHED; // Set FSM state to measurement finished
                        cm1107n->state = CM1107N_ERROR_PREHEATING; // Set state to preheating error
                        cm1107n->co2 = NAN; // Set read value to NAN
                        return;
                    }
                    print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_CM1107N, "Sensor preheating");
                    cm1107n->meas_state = MEAS_TRIGGER_SINGLE_MEAS; // Next state trigger another measurement
                    cm1107n->wake_time = make_timeout_time_ms(1);
                    return;
                }
                if (ret == CM1107N_ERROR_OUT_OF_RANGE)
                {
                    cm1107n->meas_state = MEAS_FINISHED; // Set FSM state to measurement finished
                    cm1107n->state = ERROR_NO_MEAS; // Invalid measurement has been performed
                    cm1107n->co2 = NAN; // Set read value to NAN
                    return;
                }
                cm1107n->co2 = NAN; // Set read value to NAN
                cm1107n->meas_state = MEAS_FINISHED; // Set FSM state to measurement finished
                cm1107n->state = ret; // Set state to read error
                return;
            }
            cm1107n->co2 = (float)ntoh16(*((uint16_t*)&tempBuffer[0])); // Convert read CO2 value
            cm1107n->meas_state = MEAS_FINISHED; // Set next state to measurement finished
            cm1107n->state = SUCCESS; // Set state to SUCCESS
            return;
        }
        default:
        {
            cm1107n->meas_state = MEAS_FINISHED;
            return;
        }
    }
}

int32_t cm1107n_init(sensor_t* cm1107n, sensor_config_t* config)
{
    int32_t ret;
    if (config->sensor_type != CM1107N) return ERROR_UNKNOWN_SENSOR; // Check for correct sensor type
    cm1107n->config = config; // Assign configuration
    cm_power(cm1107n, true); // Power on

    ret = cm_write_config(config); // Write configuration to sensor
    cm_power(cm1107n, false); // Power off
    if (!ret)
    {
        if (cm1107n->meas_state == MEAS_STARTED) cm1107n->wake_time = make_timeout_time_ms(3000);
    }
    else cm1107n->meas_state = MEAS_FINISHED;
    return ret;
}

int32_t cm1107n_read_config(sensor_config_t* config)
{
    int32_t ret;
    uint8_t buf[6];
    config->sensor_type = CM1107N;
    if ((ret = cm_read(CMD_ABC, buf, 6)) != 0) return ret; // Read ABC calibration data
    config->enable_abc = buf[1] != 2; // Check ABC on
    config->abc_period = buf[2]; // ABC period
    config->abc_target_value = ntoh16(*((uint16_t*)&buf[3])); // ABC target CO2
    return SUCCESS;
}

int32_t cm_write_config(sensor_config_t* config)
{
    int32_t ret;
    uint8_t buf[6];
    sensor_config_t read_config;
    if ((ret = cm1107n_read_config(&read_config)) != 0) return ret; // Read current sensor configuration

    if (read_config.enable_abc != config->enable_abc ||
        // config->enable_abc && (
        read_config.abc_period != config->abc_period ||
        read_config.abc_target_value != config->abc_target_value)/*)*/ // If configuration incorrect
    {
        print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_CM1107N, "Config - Writing ABC calibration data");
        buf[0] = 100; // Wrong code accelerate value ??
        buf[1] = config->enable_abc ? 0 : 2; // ABC enabled
        buf[2] = config->abc_period; // ABC period
        *((uint16_t*)&buf[3]) = ntoh16(config->abc_target_value); // ABC target CO2
        buf[5] = 100; // Reserved, default value

        if ((ret = cm_write(CMD_ABC, buf, 6)) != 0) return ret; // Write ABC data
    }

    return SUCCESS;
}

void cm_power(sensor_t* cm1107n, bool on)
{
    if (!cm1107n->config->power_global_control) // If power not controlled globally
    {
        // Read power vector
        // Check if bit turned [on]
        // Write power vector
    }
}




