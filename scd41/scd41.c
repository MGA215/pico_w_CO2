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
#include <stdio.h>

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

#define msg(severity, x) printf("[%12llu] ["severity"] [SCD41] "x"\n", to_us_since_boot(get_absolute_time()) / 1000)

/**
 * @brief Computes CRC for specified buffer
 * 
 * @param buf buffer the CRC is computed from
 * @param len Length of the buffer
 * @return uint8_t CRC value
 */
static inline uint8_t s41_crc(uint8_t* buf, uint32_t len);

/**
 * @brief Reads data from the SCD41 sensor
 * 
 * @param command Command to execute (get specific data)
 * @param buf Buffer to save read data
 * @param len Length of the buffer
 * @return int32_t Return code
 */
static int32_t s41_read(uint16_t command, uint16_t* buf, uint32_t len);

/**
 * @brief Writes a value to the SCD41 sensor
 * 
 * @param command Command to write specific data
 * @param value Data to write
 * @return int32_t Return code
 */
static int32_t s41_write_value(uint16_t command, uint16_t value);

/**
 * @brief Executes a command on the SCD41 sensor
 * 
 * @param command Command to send
 * @return int32_t Return code
 */
static int32_t s41_write_command(uint16_t command);

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

static int32_t s41_read(uint16_t command, uint16_t* buf, uint32_t len)
{
    int32_t ret;
    uint8_t read_data[len * 3];
    memset(read_data, 0x00, len * 3); // Clear buffer
    uint8_t commandBuf[2];
    *((uint16_t*)&commandBuf[0]) = ntoh16(command); // Prepare ommand buffer
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

static int32_t s41_write_command(uint16_t command)
{
    int32_t ret;
    uint8_t commandBuffer[2]; // Command buffer
    *((uint16_t*)&commandBuffer[0]) = ntoh16(command); // Prepare command buffer

    if ((ret = i2c_write_timeout_us(I2C_SENSOR, SCD41_ADDR, commandBuffer, 2, false, I2C_TIMEOUT_US)) < 0) return ret; // Send command
    return SUCCESS;
}

static int32_t s41_write_value(uint16_t command, uint16_t value)
{
    int32_t ret;
    uint8_t commandBuffer[5];
    memset(commandBuffer, 0x00, 5); // Clearing command buffer

    *((uint16_t*)&commandBuffer[0]) = ntoh16(command); // Adding command
    *((uint16_t*)&commandBuffer[2]) = ntoh16(value); // Adding value
    commandBuffer[4] = s41_crc(&commandBuffer[2], 2); // Adding CRC

    if ((ret = i2c_write_timeout_us(I2C_SENSOR, SCD41_ADDR, commandBuffer, 5, false, I2C_TIMEOUT_US)) < 0) return ret; // Send command
    sleep_ms(100);
    return SUCCESS;
}

void scd41_get_value(sensor_t* scd41)
{
    uint16_t tempBuffer = 0;
    int32_t ret;
    switch(scd41->meas_state)
    {
        case MEAS_FINISHED: // Measurement finished
        {
            #ifdef DEBUG
            msg("info", "Meas finished");
            #endif
            s41_power(scd41, false); // Power off
            scd41->wake_time = at_the_end_of_time; // Disable timer
            return;
        }
        case MEAS_STARTED: // Measurement started
        {
            #ifdef DEBUG
            msg("info", "Meas started");
            #endif
            s41_power(scd41, true); // Power off
            scd41->wake_time = make_timeout_time_ms(30); // Time for power stabilization
            scd41->meas_state = MEAS_READ_MODE; // Next step - read status
            scd41->timeout_iterator = 0; // Initialize read status timeout iterator
            scd41->measurement_iterator = 0; // Initialize read single measurement iterator
            return;
        }
        case MEAS_READ_MODE:
        {
            #ifdef DEBUG
            msg("info", "Read mode");
            #endif
            uint16_t val = 0;
            if ((ret = s41_read(CMD_GET_PRESSURE, &val, 1)) != 0) return; // Read pressure
            if (scd41->config->enable_pressure_comp) // Pressure compensation enabled
            {
                ret = s41_write_value(CMD_SET_PRESSURE, scd41->config->pressure); // Write pressure
            }
            else // Pressure compensation disabled
            {
                ret = s41_write_value(CMD_SET_PRESSURE, 1013); // Write pressure
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
        
            if (scd41->config->single_meas_mode) // If in single meas mode
            {
                scd41->meas_state = MEAS_TRIGGER_SINGLE_MEAS; // Set next state to send single meas command
                scd41->measurement_iterator = 0;
            }
            else
            {
                scd41->meas_state = MEAS_READ_STATUS; // Set next state to read measurement status
                scd41->wake_time = make_timeout_time_ms(5000); // Wait 5 seconds
            }
            return;
        }
        case MEAS_TRIGGER_SINGLE_MEAS:
        {
            #ifdef DEBUG
            msg("info", "Write measure command");
            #endif
            ret = s41_write_command(CMD_MEASURE_SINGLE_ALL); // Send start measurement command
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
            scd41->wake_time = make_timeout_time_ms(5000); // Wait 5 seconds
            return;
        }
        case MEAS_READ_STATUS: // Reading status
        {
            #ifdef DEBUG
                msg("info", "Read status");
                uint16_t val = 0;
                if ((ret = s41_read(CMD_GET_PRESSURE, &val, 1)) != 0) return; // Read pressure
                printf("[%u] [SCD41] Pressure: %u\n", to_ms_since_boot(get_absolute_time()), val);
            #endif
            ret = s41_read(CMD_GET_DATA_RDY_STATUS, &tempBuffer, 1); // Reading status register
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
                if (scd41->measurement_iterator++ == 0 && scd41->config->single_meas_mode)
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
                printf("Read status failed, abort...\n");
                return;
            }
            scd41->wake_time = make_timeout_time_ms(100); // Check status after 100 ms
            return;
        }
        case MEAS_READ_VALUE: // Reading values
        {
            #ifdef DEBUG
                msg("info", "Read value");
                uint16_t val = 0;
                if ((ret = s41_read(CMD_GET_PRESSURE, &val, 1)) != 0) return; // Read pressure
                printf("[%u] [SCD41] Pressure: %u\n", to_ms_since_boot(get_absolute_time()), val);
            #endif
            uint16_t buf[6];
            ret = s41_read(CMD_READ_MEAS, buf, 6); // Read measured data
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
            return;
        }
    }
}

int32_t scd41_init(sensor_t* scd41, sensor_config_t* config)
{
    int32_t ret;
    scd41->config = config;
    s41_power(scd41, true); // Turn sensor power on

    ret = s41_write_config(config); // Write config to the sensor
    s41_power(scd41, false); // Power off
    return ret;
}

int32_t scd41_read_config(sensor_config_t* config, bool single_meas_mode)
{
    int32_t ret;
    uint16_t val;

    if ((ret = s41_write_command(CMD_STOP_PER_MEAS)) != 0) return ret; // Stop measurement

    if ((ret = s41_read(CMD_GET_PRESSURE, &val, 1)) != 0) return ret; // Read pressure
    config->pressure = val;
    config->enable_pressure_comp = (val != 0);

    if ((ret = s41_read(CMD_GET_AUTO_SELF_CAL_EN, &val, 1)) != 0) return ret; // Read auto cal state
    config->enable_abc = (bool)val;

    if ((ret = s41_read(CMD_GET_T_OFFSET, &val, 1)) != 0) return ret; // Read temperature offset
    config->temperature_offset = val * (175.0f / UINT16_MAX);

    if ((ret = s41_read(CMD_GET_ALTITUDE, &val, 1)) != 0) return ret; // Read altitude
    config->altitude = val;
    config->enable_altitude_comp = (val != 0);

    if (!single_meas_mode) // If in periodic measurement mode
    {
        if ((ret = s41_write_command(CMD_START_PER_MEAS)) != 0) return ret; // Start periodic measurement
    }

    return SUCCESS;
}

static int32_t s41_write_config(sensor_config_t* config)
{
    int32_t ret;
    uint16_t val;
    bool changed = false;

    printf("Init...\n");
    if ((ret = s41_write_command(CMD_WAKE_UP)) != 0) return ret; // Trying to wake sensor up
    sleep_ms(30);
    if ((ret = s41_write_command(CMD_STOP_PER_MEAS)) != 0) return ret; // Stop periodic measurement
    sleep_ms(500);

    if ((ret = s41_read(CMD_GET_AUTO_SELF_CAL_EN, &val, 1)) != 0) return ret; // Read auto calibration enabled
    printf("Read autocal: %i\nSet autocal:  %i\n", val, config->enable_abc);
    if (config->enable_abc != (val != 0))
    {
        if (config->enable_abc) // If autocal should be enabled
        {
            if ((ret = s41_write_value(CMD_SET_AUTO_SELF_CAL_EN, 1)) != 0) return ret;
        }
        else // Autocal disabled
        {
            if ((ret = s41_write_value(CMD_SET_AUTO_SELF_CAL_EN, 0)) != 0) return ret;
        }
        changed = true;
    }

    if ((ret = s41_read(CMD_GET_T_OFFSET, &val, 1)) != 0) return ret; // Read temperature offset
    printf("Read t_off: %i\nSet t_off:  %u\n", val, (uint16_t)(config->temperature_offset * (UINT16_MAX / 175)));
    if ((uint16_t)(config->temperature_offset * (UINT16_MAX / 175)) != val) // Set temperature offset
    {
        if ((ret = s41_write_value(CMD_SET_T_OFFSET, (uint16_t)(config->temperature_offset * (UINT16_MAX / 175)))) != 0) return ret; // Write temperature offset
        changed = true;
    }

    if ((ret = s41_read(CMD_GET_ALTITUDE, &val, 1)) != 0) return ret; // Read altitude value
    printf("Read altitude: %i\nSet altitude:  %i\n", val, config->altitude);
    if (config->enable_altitude_comp != (val != 0))
    {
        if (config->enable_altitude_comp) // If altitude compensation should be enabled
        {
            if ((ret = s41_write_value(CMD_SET_ALTITUDE, config->altitude)) != 0) return ret;
        }
        else // Altitude compensation disabled
        {
            if ((ret = s41_write_value(CMD_SET_ALTITUDE, 0)) != 0) return ret;
        }
        changed = true;
    }

    if (changed) // If value was changed save settings to EEPROM
    {
        printf("Writing to EEPROM...\n");
        if ((ret = s41_write_command(CMD_PERSIST_SETTINGS)) != 0) return ret; // Save settings to EEPROM
        sleep_ms(800);
    }

    if (!(config->single_meas_mode)) // If in periodic measurement mode
    {
        if ((ret = s41_write_command(CMD_START_PER_MEAS)) != 0) return ret; // Start periodic measurement
    }

    return SUCCESS;
}

static inline void s41_power(sensor_t* scd41, bool on)
{
    if (!scd41->config->power_global_control) // If power not controlled globally
    {
        // Read power vector
        // Check if bit turned [on]
        // Write power vector
    }
}

void scd41_reset(void)
{
    s41_write_command(CMD_RESET); // Soft reset the sensor
    sleep_ms(30);
}

static void s41_factory_reset(void)
{
    s41_write_command(CMD_FACTORY_RESET); // Factory reset the sensor
    sleep_ms(1200);
}





