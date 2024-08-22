/**
 * @file scd30.c
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Implements communication with Sensirion SCD30 sensor
 * @version 0.1
 * @date 2024-06-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "scd30.h"
#include "hardware/i2c.h"
#include "math.h"
#include "string.h"
#include "common/debug.h"
#include "error_codes.h"
#include "common/functions.h"
#include "../power/power.h"

#define SCD30_ADDR              0x61

#define CMD_START_CONT_MEAS     0x0010
#define CMD_STOP_CONT_MEAS      0x0104
#define CMD_MEAS_INTERVAL       0x4600
#define CMD_DATA_READY          0x0202
#define CMD_READ_MEAS           0x0300
#define CMD_AUTO_CAL            0x5306
#define CMD_FORCE_RECAL         0x5204
#define CMD_T_OFFSET            0x5403
#define CMD_ALTITUDE_COMP       0x5102
#define CMD_FW_VERSION          0xD100
#define CMD_SOFT_RESET          0xD304

/**
 * @brief Computes CRC for specified buffer
 * 
 * @param buf buffer the CRC is computed from
 * @param len Length of the buffer
 * @return uint8_t CRC value
 */
static inline uint8_t s30_crc(uint8_t* buf, uint32_t len);

/**
 * @brief Writes configuration to the sensor
 * 
 * @param config Configuration to write
 * @return int32_t Return code
 */
static int32_t s30_write_config(sensor_config_t* config);

/**
 * @brief Switches sensor power [on] if not controlled globally
 * 
 * @param scd30 Sensor structure
 * @param on if the power should be switched on (true) or off (false)
 */
static inline void s30_power(sensor_t* scd30, bool on);


static inline uint8_t s30_crc(uint8_t* buf, uint32_t len)
{
    uint8_t crc = 0xFF;
    uint32_t i;

    for (i = 0; i < len; i++)
    {
        crc ^= buf[i];
        for (int i = 8; i != 0; i--)
        {
            if ((crc & 0x80) != 0) 
            {
                crc <<= 1;
                crc ^= 0x31;
            } 
            else 
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

int32_t scd30_read(uint16_t command, uint16_t* buf, uint32_t len)
{
    int32_t ret;
    uint8_t read_data[len * 3];
    memset(read_data, 0x00, (len * 3)); // Clear read buffer
    if ((ret = scd30_write_command(command))) return ret; // Send command
    busy_wait_ms(3);
    if ((ret = i2c_read_timeout_us(I2C_SENSOR, SCD30_ADDR, read_data, (len * 3), false, I2C_TIMEOUT_US)) < 0) return ret; // Read resonse
    for (int i = 0; i < len; i++) // Check each word CRC
    {
        if (s30_crc(&read_data[3 * i], 3) != 0) return SCD30_ERROR_CRC; // Check data CRC
        uint16_t val = ((read_data[3 * i + 1]) << 8) | (read_data[3 * i]);
        buf[i] = ntoh16(val); // Save data to the output buffer
    }
    return SUCCESS;
}

int32_t scd30_write_value(uint16_t command, uint16_t value)
{
    int32_t ret;
    uint8_t commandBuffer[5];
    memset(commandBuffer, 0x00, 5); // Clearing command buffer

    // *( (uint16_t*)&commandBuffer[0]) = ntoh16(command);
    // *( (uint16_t*)&commandBuffer[2]) = ntoh16(value);

    commandBuffer[0] = (command & 0xFF00) >> 8; // Adding command
    commandBuffer[1] = (command & 0x00FF) >> 0;

    commandBuffer[2] = (value & 0xFF00) >> 8; // Adding value
    commandBuffer[3] = (value & 0x00FF) >> 0;

    commandBuffer[4] = s30_crc(&commandBuffer[2], 2); // Adding CRC

    if ((ret = i2c_write_timeout_us(I2C_SENSOR, SCD30_ADDR, commandBuffer, 5, false, I2C_TIMEOUT_US)) < 0) return ret; // Write command to the sensor
    return SUCCESS;
}

int32_t scd30_write_command(uint16_t command)
{
    int32_t ret;
    uint8_t commandBuffer[2];
    commandBuffer[0] = (command & 0xFF00) >> 8; // Adding command
    commandBuffer[1] = (command & 0x00FF) >> 0;

    if ((ret = i2c_write_timeout_us(I2C_SENSOR, SCD30_ADDR, commandBuffer, 2, false, I2C_TIMEOUT_US)) < 0) return ret; // Send command
    return SUCCESS;
}

void scd30_get_value(sensor_t* scd30)
{
    uint16_t tempBuffer = 0;
    int32_t ret;
    if (scd30->config.sensor_type != SCD30) // Check for correct sensor type
    {
        scd30->meas_state = MEAS_FINISHED;
        scd30->state = ERROR_UNKNOWN_SENSOR;
        scd30->co2 = NAN;
        scd30->humidity = NAN;
        scd30->temperature = NAN;
        return;
    } 
    switch(scd30->meas_state)
    {
        case MEAS_FINISHED: // Measurement finished
        {
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_SCD30, "Meas finished");
            s30_power(scd30, false); // Power off
            scd30->wake_time = at_the_end_of_time; // Disable timer
            return;
        }
        case MEAS_STARTED: // Measurement started
        {
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_SCD30, "Meas start");
            s30_power(scd30, true); // Power off
            scd30->wake_time = make_timeout_time_ms(1500); // Time for power stabilization
            scd30->meas_state = MEAS_READ_STATUS; // Next step - read status
            scd30->timeout_iterator = 0; // Initialize read status timeout iterator
            scd30->state = SUCCESS;
            return;
        }
        case MEAS_READ_STATUS: // Reading status
        {
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_SCD30, "Read state");
            ret = scd30_read(CMD_DATA_READY, &tempBuffer, 1); // Reading status register
            if (ret != 0) // On invalid read
            {
                scd30->co2 = NAN; // Set values to NaN
                scd30->temperature = NAN;
                scd30->humidity = NAN;
                scd30->meas_state = MEAS_FINISHED; // Finished measurement
                scd30->state = ret; // Set sensor state to return value
                return;
            }
            if (tempBuffer == 1) // On data ready
            {
                scd30->meas_state = MEAS_READ_VALUE; // Next step - read values
                return;
            }
            if (scd30->timeout_iterator++ > 20) // On timeout
            {
                scd30->co2 = NAN; // Set values to NaN
                scd30->temperature = NAN;
                scd30->humidity = NAN;
                scd30->state = SCD30_ERROR_DATA_READY_TIMEOUT; // Set sensor state
                scd30->meas_state = MEAS_FINISHED; // Finished measurement
                return;
            }
            scd30->wake_time = make_timeout_time_ms(50); // Check status after 50 ms
            return;
        }
        case MEAS_READ_VALUE: // Reading values
        {
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_SCD30, "Read value");
            uint16_t buf[6];
            ret = scd30_read(CMD_READ_MEAS, buf, 6); // Read measured data
            if (ret != 0)
            {
                scd30->co2 = NAN; // Set values to NaN
                scd30->temperature = NAN;
                scd30->humidity = NAN;
                scd30->state = SCD30_ERROR_DATA_READY_TIMEOUT; // Set sensor state
                scd30->meas_state = MEAS_FINISHED; // Finished measurement
                return;
            }
            // uint32_t val = ntoh16(*( (uint32_t*)&buf[0])); // Convert co2 to uint32_t bytes
            // scd30->co2 = byte2float(val); // Convert to float
            uint32_t val = 0;
            val |= ntoh16(buf[0]) << 0;
            val |= ntoh16(buf[1]) << 16;
            scd30->co2 = byte2float((val));
            // val = ntoh16(*( (uint32_t*)&buf[2])); // Convert temperature to uint32_t bytes
            // scd30->temperature = byte2float(val); // Convert to float
            val = 0;
            val |= ntoh16(buf[2]) << 0;
            val |= ntoh16(buf[3]) << 16;
            scd30->temperature = byte2float((val));
            // val = ntoh16(*( (uint32_t*)&buf[4])); // Convert humidity to uint32_t bytes
            // scd30->humidity = byte2float(val); // Convert to float
            val = 0;
            val |= ntoh16(buf[4]) << 0;
            val |= ntoh16(buf[5]) << 16;
            scd30->humidity = byte2float((val));

            scd30->meas_state = MEAS_FINISHED; // Finished measurement
            scd30->state = SUCCESS; // Set state
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_SCD30, "Measured CO2 value: %f", scd30->co2);
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_SCD30, "Measured temperature value: %f", scd30->temperature);
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_SCD30, "Measured RH value: %f", scd30->humidity);
            return;
        }
        default:
        {
            scd30->meas_state = MEAS_FINISHED;
            return;
        }
    }
}

int32_t scd30_init(sensor_t* scd30, sensor_config_t* config)
{
    int32_t ret;
    if (config->sensor_type != SCD30) return ERROR_UNKNOWN_SENSOR; // Check for correct sensor type
    memcpy(&scd30->config, config, sizeof(sensor_config_t));
    s30_power(scd30, true); // Power on

    ret = s30_write_config(config); // Write configuration to sensor
    s30_power(scd30, false); // Power off
    if (!ret)
    {
        if (scd30->meas_state == MEAS_STARTED) scd30->wake_time = make_timeout_time_ms(3000);
    }
    else scd30->meas_state = MEAS_FINISHED;
    return ret;
}

int32_t scd30_read_config(sensor_config_t* config)
{
    int32_t ret;
    uint16_t val;
    config->sensor_type = SCD30;
    if ((ret = scd30_read(CMD_START_CONT_MEAS, &val, 1)) != 0) return ret; // Read pressure
    config->pressure = val;
    config->enable_pressure_comp = (val != 0);

    if ((ret = scd30_read(CMD_MEAS_INTERVAL, &val, 1)) != 0) return ret; // Read measurement interval
    config->meas_period = val;

    if ((ret = scd30_read(CMD_AUTO_CAL, &val, 1)) != 0) return ret; // Read auto calibration
    config->abc_target_value = val;
    config->enable_abc = (val != 0);

    if ((ret = scd30_read(CMD_T_OFFSET, &val, 1)) != 0) return ret; // Read temperature offset
    config->temperature_offset = val / 100.f;

    if ((ret = scd30_read(CMD_ALTITUDE_COMP, &val, 1)) != 0) return ret; // Read altitude compensation
    config->altitude = val;
    config->enable_altitude_comp = (val != 0);

    return SUCCESS;
}

static int32_t s30_write_config(sensor_config_t* config)
{
    int32_t ret;
    uint16_t val;
    sensor_config_t read_config;
    if ((ret = scd30_read_config(&read_config)) != 0) return ret; // Read config

    

    if (read_config.meas_period != config->meas_period) // Check mesaurement period
    {
        print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SCD30, "Config - Writing measurement period");
        if ((ret = scd30_write_value(CMD_MEAS_INTERVAL, config->meas_period)) != 0) return ret; // Set measurement interval
    }

    if (config->enable_abc != read_config.enable_abc) // Check abc enabled
    {
        print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SCD30, "Config - Writing ABC enable");
        if ((ret = scd30_write_value(CMD_AUTO_CAL, config->enable_abc)) != 0) return ret;
    }

    if ((read_config.temperature_offset - config->temperature_offset) > 0.01f) // Check temperature offset
    {
        print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SCD30, "Config - Writing temperature offset");
        if ((ret = scd30_write_value(CMD_T_OFFSET, (uint16_t)(config->temperature_offset * 100))) != 0) return ret; // Set temperature offset
    }

    if (config->enable_altitude_comp != read_config.enable_altitude_comp ||
        (config->enable_altitude_comp && (config->altitude != read_config.altitude))) // Check altitude enable and value
    {
        print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SCD30, "Config - Writing altitude");
        if (config->enable_altitude_comp) // Altitude compensation enabled
        {
            if ((ret = scd30_write_value(CMD_ALTITUDE_COMP, config->altitude)) != 0) return ret; // Write altitude
        }
        else // Altitude compensation disabled
        {
            if ((ret = scd30_write_value(CMD_ALTITUDE_COMP, 0)) != 0) return ret; // Disable altitude compensation
        }
    }
    if (config->enable_pressure_comp != read_config.enable_pressure_comp ||
        (config->enable_pressure_comp && (config->pressure != read_config.pressure)) || !config->single_meas_mode) // Check pressure enable and value
    {
        print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SCD30, "Config - Writing pressure");
        if (config->enable_pressure_comp) // Pressure compensation enabled
        {
            if ((ret = scd30_write_value(CMD_START_CONT_MEAS, config->pressure)) != 0) return ret; // Write pressure
        }
        else // Pressure compensation disabled
        {
            if ((ret = scd30_write_value(CMD_START_CONT_MEAS, 0)) != 0) return ret; // Disable pressure compensation
        }
    }

    return SUCCESS;
}

static inline void s30_power(sensor_t* scd30, bool on)
{
    if (!scd30->config.power_global_control && !scd30->config.power_continuous) // If power not controlled globally
    {
        power_en_set_index(scd30->index, on);
    }
}










