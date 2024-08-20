/**
 * @file scd41.c
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Implements communication with Sensirion SCD41 sensor
 * @version 0.1
 * @date 2024-06-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "scd41.h"
#include "math.h"
#include "hardware/i2c.h"
#include "string.h"
#include "error_codes.h"
#include "common/debug.h"
#include "common/functions.h"
#include "../power/power.h"

#define CMD_START_PER_MEAS                  0x21B1
#define CMD_READ_MEAS                       0xEC05
#define CMD_STOP_PER_MEAS                   0x3F86

#define CMD_SET_T_OFFSET                    0x241D
#define CMD_GET_T_OFFSET                    0x2318
#define CMD_SET_ALTITUDE                    0x2427
#define CMD_GET_ALTITUDE                    0x2322
#define CMD_SET_PRESSURE                    0xE000
#define CMD_GET_PRESSURE                    0xE000

#define CMD_FORCE_CAL                       0x362F
#define CMD_SET_AUTO_SELF_CAL_EN            0x2416
#define CMD_GET_AUTO_SELF_CAL_EN            0x2313

#define CMD_START_LOW_POWER_PER_MEAS        0x21AC
#define CMD_GET_DATA_RDY_STATUS             0xE4B8

#define CMD_PERSIST_SETTINGS                0x3615
#define CMD_GET_SER_NUM                     0x3682
#define CMD_SELF_TEST                       0x3639
#define CMD_FACTORY_RESET                   0x3632
#define CMD_RESET                           0x3646

#define CMD_MEASURE_SINGLE_ALL              0x219D
#define CMD_MEAS_SINGLE_RH_T                0x2196
#define CMD_POWER_DOWN                      0x36E0
#define CMD_WAKE_UP                         0x36F6
#define CMD_SET_AUTO_SELF_CAL_INIT_PER      0x2445
#define CMD_GET_AUTO_SELF_CAL_INIT_PER      0x2340
#define CMD_SET_AUTO_SELF_CAL_STANDARD_PER  0x244E
#define CMD_GET_AUTO_SELF_CAL_STANDARD_PER  0x234B

#define SCD41_ADDR 0x62

/**
 * @brief Computes CRC for specified buffer
 * 
 * @param buf buffer the CRC is computed from
 * @param len Length of the buffer
 * @return uint8_t CRC value
 */
static inline uint8_t s41_crc(uint8_t* buf, uint32_t len);

/**
 * @brief Writes configuration to the sensor
 * 
 * @param config Configuration to write
 * @return int32_t Return code
 */
static int32_t s41_write_config(sensor_config_t* config);

/**
 * @brief Turns the sensor power [on]
 * 
 * @param scd41 Sensor structure
 * @param on Whether the power should be turned on
 */
static inline void s41_power(sensor_t* scd41, bool on);

/**
 * @brief Performs factory reset on the sensor
 * 
 */
static void s41_factory_reset(void);


static inline uint8_t s41_crc(uint8_t* buf, uint32_t len)
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

int32_t scd41_read(uint16_t command, uint16_t* buf, uint32_t len)
{
    int32_t ret;
    uint8_t read_data[len * 3];
    memset(read_data, 0x00, len * 3); // Clear buffer
    uint8_t commandBuf[2];
    // *( (uint16_t*)&commandBuf[0]) = ntoh16(command);
    commandBuf[0] = (command & 0xFF00) >> 8; // Prepare ommand buffer
    commandBuf[1] = (command & 0x00FF) >> 0;
    if ((ret = i2c_write_timeout_us(I2C_SENSOR, SCD41_ADDR, commandBuf, 2, true, I2C_TIMEOUT_US)) < 0) return ret; // Send command blocking
    busy_wait_ms(1); // Wait between write and read
    if ((ret = i2c_read_timeout_us(I2C_SENSOR, SCD41_ADDR, read_data, (len * 3), false, I2C_TIMEOUT_US)) < 0) return ret; // Read response
    for (int i = 0; i < len; i++) // Check response crcs
    {
        if (s41_crc(&read_data[3 * i], 3) != 0) return SCD41_ERROR_CRC; // Check word CRC
        uint16_t val = ((read_data[3 * i + 1]) << 8) | (read_data[3 * i]);
        buf[i] = ntoh16(val); // Save response word
    }
    return SUCCESS;
}

int32_t scd41_write_value(uint16_t command, uint16_t value)
{
    int32_t ret;
    uint8_t commandBuffer[5];
    memset(commandBuffer, 0x00, 5); // Clearing command buffer

    // *( (uint16_t*)&commandBuffer[0]) = ntoh16(command); 
    commandBuffer[0] = (command & 0xFF00) >> 8; // Adding command
    commandBuffer[1] = (command & 0x00FF) >> 0;
    // *( (uint16_t*)&commandBuffer[2]) = ntoh16(value); 
    commandBuffer[2] = (value & 0xFF00) >> 8; // Adding value
    commandBuffer[3] = (value & 0x00FF) >> 0;
    commandBuffer[4] = s41_crc(&commandBuffer[2], 2); // Adding CRC

    if ((ret = i2c_write_timeout_us(I2C_SENSOR, SCD41_ADDR, commandBuffer, 5, false, I2C_TIMEOUT_US)) < 0) return ret; // Send command
    sleep_ms(100);
    return SUCCESS;
}

int32_t scd41_write_command(uint16_t command)
{
    int32_t ret;
    uint8_t commandBuffer[2]; // Command buffer
    // *( (uint16_t*)&commandBuffer[0]) = ntoh16(command); 
    commandBuffer[0] = (command & 0xFF00) >> 8; // Prepare command buffer
    commandBuffer[1] = (command & 0x00FF) >> 0;

    if ((ret = i2c_write_timeout_us(I2C_SENSOR, SCD41_ADDR, commandBuffer, 2, false, I2C_TIMEOUT_US)) < 0) return ret; // Send command
    return SUCCESS;
}

void scd41_get_value(sensor_t* scd41)
{
    uint16_t tempBuffer = 0;
    int32_t ret;
    if (scd41->config.sensor_type != SCD41) // Check for correct sensor type
    {
        scd41->meas_state = MEAS_FINISHED;
        scd41->state = ERROR_UNKNOWN_SENSOR;
        scd41->co2 = NAN;
        scd41->humidity = NAN;
        scd41->temperature = NAN;
        return;
    }
    switch(scd41->meas_state)
    {
        case MEAS_FINISHED: // Measurement finished
        {
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_SCD41, "Meas finished");
            s41_power(scd41, false); // Power off
            scd41->wake_time = at_the_end_of_time; // Disable timer
            return;
        }
        case MEAS_STARTED: // Measurement started
        {
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_SCD41, "Meas started");
            s41_power(scd41, true); // Power off
            scd41->wake_time = make_timeout_time_ms(30); // Time for power stabilization
            scd41->meas_state = MEAS_READ_MODE; // Next step - read status
            scd41->state = SUCCESS;
            scd41->timeout_iterator = 0; // Initialize read status timeout iterator
            scd41->measurement_iterator = 0; // Initialize read single measurement iterator
            return;
        }
        case MEAS_READ_MODE:
        {
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_SCD41, "Read mode");
            uint16_t val = 0;
            if ((ret = scd41_read(CMD_GET_PRESSURE, &val, 1)) != 0) return; // Read pressure
            if (scd41->config.enable_pressure_comp) // Pressure compensation enabled
            {
                ret = scd41_write_value(CMD_SET_PRESSURE, scd41->config.pressure); // Write pressure
            }
            else // Pressure compensation disabled
            {
                ret = scd41_write_value(CMD_SET_PRESSURE, 1013); // Write pressure
            }
            if (ret != 0) // On invalid write
            {
                scd41->co2 = NAN; // Set values to NaN
                scd41->temperature = NAN;
                scd41->humidity = NAN;
                scd41->meas_state = MEAS_FINISHED; // Finished measurement
                scd41->state = ret; // Set sensor state to return value
                return;
            }
        
            if (scd41->config.single_meas_mode) // If in single meas mode
            {
                scd41->meas_state = MEAS_TRIGGER_SINGLE_MEAS; // Set next state to send single meas command
                scd41->measurement_iterator = 0;
            }
            else
            {
                scd41->meas_state = MEAS_READ_STATUS; // Set next state to read measurement status
                scd41->wake_time = make_timeout_time_ms(4000); // Wait 5 seconds
            }
            return;
        }
        case MEAS_TRIGGER_SINGLE_MEAS:
        {
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_SCD41, "Write measure command");
            ret = scd41_write_command(CMD_MEASURE_SINGLE_ALL); // Send start measurement command
            if (ret != 0) // On invalid write
            {
                scd41->co2 = NAN; // Set values to NaN
                scd41->temperature = NAN;
                scd41->humidity = NAN;
                scd41->meas_state = MEAS_FINISHED; // Finished measurement
                scd41->state = ret; // Set sensor state to return value
                return;
            }
            scd41->meas_state = MEAS_READ_STATUS; // Set next state to read measurement status
            scd41->wake_time = make_timeout_time_ms(4000); // Wait 5 seconds
            return;
        }
        case MEAS_READ_STATUS: // Reading status
        {
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_SCD41, "Read status");
            ret = scd41_read(CMD_GET_DATA_RDY_STATUS, &tempBuffer, 1); // Reading status register
            if (ret != 0) // On invalid read
            {
                scd41->co2 = NAN; // Set values to NaN
                scd41->temperature = NAN;
                scd41->humidity = NAN;
                scd41->meas_state = MEAS_FINISHED; // Finished measurement
                scd41->state = ret; // Set sensor state to return value
                return;
            }
            if ((tempBuffer & 0x7FF) == 0x06) // On data ready - might cause troubles later (should be equal to 0, might be config not saved problem??)
            {
                if (scd41->measurement_iterator++ == 0 && scd41->config.single_meas_mode)
                {
                    scd41->meas_state = MEAS_TRIGGER_SINGLE_MEAS;
                    return;
                }
                scd41->meas_state = MEAS_READ_VALUE; // Next step - read values
                return;
            }
            if (scd41->timeout_iterator++ > 50) // On timeout
            {
                scd41->co2 = NAN; // Set values to NaN
                scd41->temperature = NAN;
                scd41->humidity = NAN;
                scd41->state = SCD41_ERROR_DATA_READY_TIMEOUT; // Set sensor state
                scd41->meas_state = MEAS_FINISHED; // Finished measurement
                print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_SCD41, "Read status failed, abort...");
                return;
            }
            scd41->wake_time = make_timeout_time_ms(100); // Check status after 100 ms
            return;
        }
        case MEAS_READ_VALUE: // Reading values
        {
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_SCD41, "Read value");
            uint16_t buf[6];
            ret = scd41_read(CMD_READ_MEAS, buf, 6); // Read measured data
            if (ret != 0)
            {
                scd41->co2 = NAN; // Set values to NaN
                scd41->temperature = NAN;
                scd41->humidity = NAN;
                scd41->state = SCD41_ERROR_DATA_READY_TIMEOUT; // Set sensor state
                scd41->meas_state = MEAS_FINISHED; // Finished measurement
                return;
            }
            scd41->co2 = (float)buf[0]; // Convert co2 to float
            scd41->temperature = -45 + 175 * (float)buf[1] / UINT16_MAX; // Convert temperature to float
            scd41->humidity = 100 * (float)buf[2] / UINT16_MAX; // Convert humidity to float

            scd41->meas_state = MEAS_FINISHED; // Finished measurement
            scd41->state = SUCCESS; // Set state
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_SCD41, "Measured CO2 value: %f", scd41->co2);
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_SCD41, "Measured temperature value: %f", scd41->temperature);
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_SCD41, "Measured RH value: %f", scd41->humidity);
            return;
        }
        default:
        {
            scd41->meas_state = MEAS_FINISHED;
            return;
        }
    }
}

int32_t scd41_init(sensor_t* scd41, sensor_config_t* config)
{
    int32_t ret;
    if (config->sensor_type != SCD41) return ERROR_UNKNOWN_SENSOR; // Check for correct sensor type
    memcpy(&scd41->config, config, sizeof(sensor_config_t));
    s41_power(scd41, true); // Turn sensor power on

    ret = s41_write_config(config); // Write config to the sensor
    s41_power(scd41, false); // Power off
    if (!ret)
    {
        if (scd41->meas_state == MEAS_STARTED) scd41->wake_time = make_timeout_time_ms(5000);
    }
    else scd41->meas_state = MEAS_FINISHED;
    sleep_ms(100);
    return ret;
}

int32_t scd41_read_config(sensor_config_t* config, bool single_meas_mode)
{
    int32_t ret;
    uint16_t val;
    config->sensor_type = SCD41;
    config->single_meas_mode = single_meas_mode;

    if ((ret = scd41_write_command(CMD_STOP_PER_MEAS)) != 0) return ret; // Stop measurement
    sleep_ms(500);

    if ((ret = scd41_read(CMD_GET_PRESSURE, &val, 1)) != 0) return ret; // Read pressure
    config->pressure = val;
    config->enable_pressure_comp = (val != 1013 && val != 0);

    if ((ret = scd41_read(CMD_GET_AUTO_SELF_CAL_EN, &val, 1)) != 0) return ret; // Read auto cal state
    config->enable_abc = (bool)val;

    if ((ret = scd41_read(CMD_GET_AUTO_SELF_CAL_INIT_PER, &val, 1)) != 0) return ret; // Read auto cal init period
    config->abc_init_period = val;

    if ((ret = scd41_read(CMD_GET_AUTO_SELF_CAL_STANDARD_PER, &val, 1)) != 0) return ret; // Read auto cal standard period
    config->abc_period = val;

    if ((ret = scd41_read(CMD_GET_T_OFFSET, &val, 1)) != 0) return ret; // Read temperature offset
    config->temperature_offset = val * (175.0f / UINT16_MAX);

    if ((ret = scd41_read(CMD_GET_ALTITUDE, &val, 1)) != 0) return ret; // Read altitude
    config->altitude = val;
    config->enable_altitude_comp = (val != 0);

    if (!single_meas_mode) // If in periodic measurement mode
    {
        if ((ret = scd41_write_command(CMD_START_PER_MEAS)) != 0) return ret; // Start periodic measurement
    }

    return SUCCESS;
}

static int32_t s41_write_config(sensor_config_t* config)
{
    int32_t ret;
    uint16_t val;
    bool changed = false;
    sensor_config_t read_config;

    // if ((ret = scd41_write_command(CMD_WAKE_UP)) != 0) return ret; // Trying to wake sensor up
    // sleep_ms(30);
    if ((ret = scd41_read_config(&read_config, config->single_meas_mode)) != 0) return ret; // Read config

    if ((ret = scd41_write_command(CMD_STOP_PER_MEAS)) != 0) return ret; // Stop periodic measurement
    sleep_ms(500);

    if (config->enable_abc != read_config.enable_abc)
    {
        print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SCD41, "Config - Writing ABC enable");
        if ((ret = scd41_write_value(CMD_SET_AUTO_SELF_CAL_EN, config->enable_abc)) != 0) return ret;
        changed = true;
    }

    if (fabs(config->temperature_offset - read_config.temperature_offset) > 0.01f) // Set temperature offset
    {
        print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SCD41, "Config - Writing temperature offset");
        if ((ret = scd41_write_value(CMD_SET_T_OFFSET, (uint16_t)(config->temperature_offset * (UINT16_MAX / 175)))) != 0) return ret; // Write temperature offset
        changed = true;
    }

    if (config->enable_altitude_comp != read_config.enable_altitude_comp ||
        config->enable_altitude_comp && (config->altitude != read_config.altitude)) // Check altitude
    {
        print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SCD41, "Config - Writing altitude");
        if (config->enable_altitude_comp) // If altitude compensation should be enabled
        {
            if ((ret = scd41_write_value(CMD_SET_ALTITUDE, config->altitude)) != 0) return ret;
        }
        else // Altitude compensation disabled
        {
            if ((ret = scd41_write_value(CMD_SET_ALTITUDE, 0)) != 0) return ret;
        }
        changed = true;
    }

    if (config->abc_init_period != read_config.abc_init_period) // Check ABC initial period
    {
        print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SCD41, "Config - Writing ABC initial period");
        if ((ret = scd41_write_value(CMD_SET_AUTO_SELF_CAL_INIT_PER, config->abc_init_period)) != 0) return ret; // Set ABC initial period
    }

    if (config->abc_period != read_config.abc_period) // Check ABC standard period
    {
        print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SCD41, "Config - Writing ABC period");
        if ((ret = scd41_write_value(CMD_SET_AUTO_SELF_CAL_STANDARD_PER, config->abc_period)) != 0) return ret; // Set ABC standard period
    }

    if (changed) // If value was changed save settings to EEPROM
    {
        print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SCD41, "Config - Saving config to EEPROM");
        if ((ret = scd41_write_command(CMD_PERSIST_SETTINGS)) != 0) return ret; // Save settings to EEPROM
        sleep_ms(800);
    }

    if (!(config->single_meas_mode)) // If in periodic measurement mode
    {
        if ((ret = scd41_write_command(CMD_START_PER_MEAS)) != 0) return ret; // Start periodic measurement
    }

    return SUCCESS;
}

static inline void s41_power(sensor_t* scd41, bool on)
{
    if (!scd41->config.power_global_control && !scd41->config.power_continuous) // If power not controlled globally
    {
        power_en_set_index(scd41->index, on);
    }
}

void scd41_reset(void)
{
    scd41_write_command(CMD_RESET); // Soft reset the sensor
    sleep_ms(30);
}

static void s41_factory_reset(void)
{
    scd41_write_command(CMD_FACTORY_RESET); // Factory reset the sensor
    sleep_ms(1200);
}





