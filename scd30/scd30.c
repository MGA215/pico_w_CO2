#include "scd30.h"
#include <stdio.h>

#define CMD_START_CONT_MEAS 0x0010
#define CMD_STOP_CONT_MEAS 0x0104
#define CMD_MEAS_INTERVAL 0x4600
#define CMD_DATA_READY 0x0202
#define CMD_READ_MEAS 0x0300
#define CMD_AUTO_CAL 0x5306
#define CMD_FORCE_RECAL 0x5204
#define CMD_T_OFFSET 0x5403
#define CMD_ALTITUDE_COMP 0x5102
#define CMD_FW_VERSION 0xD100
#define CMD_SOFT_RESET 0xD304

#define SCD30_I2C i2c0
#define SCD30_ADDR 0x61



/**
 * @brief Computes CRC for specified buffer
 * 
 * @param buf buffer the CRC is computed from
 * @param len Length of the buffer
 * @return uint8_t CRC value
 */
uint8_t scd30_crc(uint8_t* buf, uint32_t len);

int32_t scd30_write_config(scd30_config_t* config);



uint8_t scd30_crc(uint8_t* buf, uint32_t len)
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
    memset(read_data, 0x00, (len * 3) + 2);
    if ((ret = scd30_write_command(command))) return ret;
    busy_wait_ms(3);
    if ((ret = i2c_read_timeout_per_char_us(SCD30_I2C, SCD30_ADDR, read_data, (len * 3 + 2), false, 1000)) < 0) return ret;
    for (int i = 0; i < len; i++)
    {
        if (scd30_crc(&read_data[3 * i], 3) != 0) return SCD30_ERROR_CRC; // Check data CRC
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

    *((uint16_t*)&commandBuffer[0]) = ntoh16(command); // Adding command
    *((uint16_t*)&commandBuffer[2]) = ntoh16(value); // Adding value
    commandBuffer[4] = scd30_crc(&commandBuffer[2], 2); // Adding CRC

    if ((ret = i2c_write_timeout_per_char_us(SCD30_I2C, SCD30_ADDR, commandBuffer, 5, false, 1000)) < 0) return ret; // Write command to the sensor
    return SUCCESS;
}

int32_t scd30_write_command(uint16_t command)
{
    int32_t ret;
    uint8_t commandBuffer[2];
    *((uint16_t*)&commandBuffer[0]) = ntoh16(command); // Prepare command

    if ((ret = i2c_write_timeout_per_char_us(SCD30_I2C, SCD30_ADDR, commandBuffer, 2, false, 1000)) < 0) return ret; // Send command
    return SUCCESS;
}

void scd30_get_value(scd30_t* scd30)
{
    uint16_t tempBuffer = 0;
    int32_t ret;
    static int32_t i = 0;
    switch(scd30->meas_state)
    {
        case SCD30_MEAS_FINISHED: // Measurement finished
        {
            scd30_power(scd30, false); // Power off
            scd30->wake_time = make_timeout_time_ms(INT32_MAX); // Disable timer
            return;
        }
        case SCD30_MEAS_START: // Measurement started
        {
            scd30_power(scd30, true); // Power off
            scd30->wake_time = make_timeout_time_ms(1500); // Time for power stabilization
            scd30->meas_state = SCD30_READ_STATUS; // Next step - read status
            i = 0; // Initialize read status timeout iterator
            return;
        }
        case SCD30_READ_STATUS: // Reading status
        {
            ret = scd30_read(CMD_DATA_READY, &tempBuffer, 1); // Reading status register
            if (ret != 0) // On invalid read
            {
                scd30->co2 = NAN; // Set values to NaN
                scd30->temperature = NAN;
                scd30->humidity = NAN;
                scd30->meas_state = SCD30_MEAS_FINISHED; // Finished measurement
                scd30->state = ret; // Set sensor state to return value
                return;
            }
            if (tempBuffer == 1) // On data ready
            {
                scd30->meas_state = SCD30_READ_VALUE; // Next step - read values
                return;
            }
            if (i++ > 20) // On timeout
            {
                scd30->co2 = NAN; // Set values to NaN
                scd30->temperature = NAN;
                scd30->humidity = NAN;
                scd30->state = SCD30_ERROR_DATA_READY_TIMEOUT; // Set sensor state
                scd30->meas_state = SCD30_MEAS_FINISHED; // Finished measurement
                return;
            }
            scd30->wake_time = make_timeout_time_ms(50); // Check status after 50 ms
            return;
        }
        case SCD30_READ_VALUE: // Reading values
        {
            uint16_t buf[6];
            ret = scd30_read(CMD_READ_MEAS, buf, 6); // Read measured data
            if (ret != 0)
            {
                scd30->co2 = NAN; // Set values to NaN
                scd30->temperature = NAN;
                scd30->humidity = NAN;
                scd30->state = SCD30_ERROR_DATA_READY_TIMEOUT; // Set sensor state
                scd30->meas_state = SCD30_MEAS_FINISHED; // Finished measurement
                return;
            }
            uint32_t val = ntoh16(*((uint32_t*)&buf[0])); // Convert co2 to uint32_t bytes
            scd30->co2 = byte2float(val); // Convert to float
            val = ntoh16(*((uint32_t*)&buf[2])); // Convert temperature to uint32_t bytes
            scd30->temperature = byte2float(val); // Convert to float
            val = ntoh16(*((uint32_t*)&buf[4])); // Convert humidity to uint32_t bytes
            scd30->humidity = byte2float(val); // Convert to float

            scd30->meas_state = SCD30_MEAS_FINISHED; // Finished measurement
            scd30->state = SUCCESS; // Set state
            return;
        }
    }
}

void scd30_power(scd30_t* scd30, bool on)
{
    if (!scd30->config->power_global_control) // If power not controlled globally
    {
        // Read power vector
        // Check if bit turned [on]
        // Write power vector
    }
}

int32_t scd30_init(scd30_t* scd30, scd30_config_t* config)
{
    int32_t ret;
    scd30->config = config; // Save configuration
    scd30_power(scd30, true); // Power on

    scd30_init_struct(scd30); // Init defaults in struct

    ret = scd30_write_config(config); // Write configuration to sensor
    scd30_power(scd30, false); // Power off
    return ret;
}

int32_t scd30_write_config(scd30_config_t* config)
{
    int32_t ret;
    uint16_t val;
    if ((ret = scd30_read(CMD_START_CONT_MEAS, &val, 1)) != 0) return ret;
    if (config->pressure_comp != (val != 0))
    {
        if (config->pressure_comp) // If pressure compensation enabled
        {
            if ((ret = scd30_write_value(CMD_START_CONT_MEAS, config->pressure)) != 0) return ret;
        }
        else // pressure compensation disabled
        {
            if ((ret = scd30_write_value(CMD_START_CONT_MEAS, 0)) != 0) return ret;
        }
    }

    if ((ret = scd30_read(CMD_MEAS_INTERVAL, &val, 1)) != 0) return ret;
    if (val != config->meas_period)
    {
        if ((ret = scd30_write_value(CMD_MEAS_INTERVAL, config->meas_period)) != 0) return ret;
    }

    if ((ret = scd30_read(CMD_AUTO_CAL, &val, 1)) != 0) return ret;
    if (config->enable_autocal != (val != 0))
    {
        if (config->enable_autocal)
        {
            if ((ret = scd30_write_value(CMD_AUTO_CAL, config->autocal_value)) != 0) return ret;
        }
        else
        {
            if ((ret = scd30_write_value(CMD_AUTO_CAL, 0)) != 0) return ret;
        }
    }

    if ((ret = scd30_read(CMD_T_OFFSET, &val, 1)) != 0) return ret;
    if ((uint16_t)(config->temperature_offset * 100) != val)
    {
        if ((ret = scd30_write_value(CMD_T_OFFSET, (uint16_t)(config->temperature_offset * 100))) != 0) return ret;
    }

    if ((ret = scd30_read(CMD_ALTITUDE_COMP, &val, 1)) != 0) return ret;
    if (config->enable_altitude_comp != (val != 0))
    {
        if (config->enable_altitude_comp)
        {
            if ((ret = scd30_write_value(CMD_ALTITUDE_COMP, config->altitude)) != 0) return ret;
        }
        else
        {
            if ((ret = scd30_write_value(CMD_ALTITUDE_COMP, 0)) != 0) return ret;
        }
    }

    return SUCCESS;
}

int32_t scd30_read_config(scd30_config_t* config)
{
    int32_t ret;
    uint16_t val;
    if ((ret = scd30_read(CMD_START_CONT_MEAS, &val, 1)) != 0) return ret;
    config->pressure = val;
    config->pressure_comp = (val != 0);

    if ((ret = scd30_read(CMD_MEAS_INTERVAL, &val, 1)) != 0) return ret;
    config->meas_period = val;

    if ((ret = scd30_read(CMD_AUTO_CAL, &val, 1)) != 0) return ret;
    config->autocal_value = val;
    config->enable_autocal = (val != 0);

    if ((ret = scd30_read(CMD_T_OFFSET, &val, 1)) != 0) return ret;
    config->temperature_offset = val / 100.f;

    if ((ret = scd30_read(CMD_ALTITUDE_COMP, &val, 1)) != 0) return ret;
    config->altitude = val;
    config->enable_altitude_comp = (val != 0);

    return SUCCESS;
}

void scd30_init_struct(scd30_t* scd30)
{
    scd30->co2 = .0f; // Init CO2
    scd30->humidity = .0f; // Init humidity
    scd30->temperature = .0f; // Init temperature
    scd30->state = ERROR_SENSOR_NOT_INITIALIZED; // Sensor not initialized state
    scd30->meas_state = (scd30_meas_state_e)0; // Measurement not started
    scd30->wake_time = make_timeout_time_ms(INT32_MAX); // Disable timeout
}











