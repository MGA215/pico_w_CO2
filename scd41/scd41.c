#include "scd41.h"

#define CMD_START_PER_MEAS 0x21B1
#define CMD_READ_MEAS 0xEC05
#define CMD_STOP_PER_MEAS 0x3F86

#define CMD_SET_T_OFFSET 0x241D
#define CMD_GET_T_OFFSET 0x2318
#define CMD_SET_ALTITUDE 0x2427
#define CMD_GET_ALTITUDE 0x2322
#define CMD_SET_PRESSURE 0xE000
#define CMD_GET_PRESSURE 0xE000

#define CMD_FORCE_CAL 0x362F
#define CMD_SET_AUTO_SELF_CAL_EN 0x2416
#define CMD_GET_AUTO_SELF_CAL_EN 0x2313

#define CMD_START_LOW_POWER_PER_MEAS 0x21AC
#define CMD_GET_DATA_RDY_STATUS 0xE4B8

#define CMD_PERSIST_SETTINGS 0x3615
#define CMD_GET_SER_NUM 0x3682
#define CMD_SELF_TEST 0x3639
#define CMD_FACTORY_RESET 0x3632
#define CMD_RESET 0x3646

#define CMD_MEASURE_SINGLE_ALL 0x219D
#define CMD_MEAS_SINGLE_RH_T 0x2196
#define CMD_POWER_DOWN 0x36E0
#define CMD_WAKE_UP 0x36F6
#define CMD_SET_AUTO_SELF_CAL_INIT_PER 0x2445
#define CMD_GET_AUTO_SELF_CAL_INIT_PER 0x2340
#define CMD_SET_AUTO_SELF_CAL_STANDARD_PER 0x244E
#define CMD_GET_AUTO_SELF_CAL_STANDARD_PER 0x234B

#define SCD41_I2C i2c0
#define SCD41_ADDR 0x62

/**
 * @brief Computes CRC for specified buffer
 * 
 * @param buf buffer the CRC is computed from
 * @param len Length of the buffer
 * @return uint8_t CRC value
 */
uint8_t scd41_crc(uint8_t* buf, uint32_t len);

int32_t scd41_write_config(scd41_config_t* config);


uint8_t scd41_crc(uint8_t* buf, uint32_t len)
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
    memset(read_data, 0x00, (len * 3) + 2);
    uint8_t commandBuf[2];
    *((uint16_t*)&commandBuf[0]) = ntoh16(command);
    if ((ret = i2c_write_timeout_per_char_us(SCD41_I2C, SCD41_ADDR, commandBuf, 2, true, 1000)) < 0) return ret;
    busy_wait_ms(1);
    if ((ret = i2c_read_timeout_per_char_us(SCD41_I2C, SCD41_ADDR, read_data, (len * 3 + 2), false, 1000)) < 0) return ret;
    for (int i = 0; i < len; i++)
    {
        if (scd41_crc(&read_data[3 * i], 3) != 0) return SCD30_ERROR_CRC;
        uint16_t val = ((read_data[3 * i + 1]) << 8) | (read_data[3 * i]);
        buf[i] = ntoh16(val);
    }
    return SUCCESS;
}

int32_t scd41_write_command(uint16_t command)
{
    int32_t ret;
    uint8_t commandBuffer[2];
    *((uint16_t*)&commandBuffer[0]) = ntoh16(command);

    if ((ret = i2c_write_timeout_per_char_us(SCD41_I2C, SCD41_ADDR, commandBuffer, 2, false, 1000)) < 0) return ret;
    return SUCCESS;
}

int32_t scd41_write_value(uint16_t command, uint16_t value)
{
    int32_t ret;
    uint8_t commandBuffer[5];
    memset(commandBuffer, 0x00, 5); // Clearing command buffer

    *((uint16_t*)&commandBuffer[0]) = ntoh16(command); // Adding command
    *((uint16_t*)&commandBuffer[2]) = ntoh16(value); // Adding value
    commandBuffer[4] = scd41_crc(&commandBuffer[2], 2); // Adding CRC

    if ((ret = i2c_write_timeout_per_char_us(SCD41_I2C, SCD41_ADDR, commandBuffer, 5, false, 1000)) < 0) return ret;
    sleep_ms(100);
    return SUCCESS;
}

void scd41_get_value(scd41_t* scd41)
{
    uint16_t tempBuffer = 0;
    int32_t ret;
    static int32_t i = 0;
    switch(scd41->meas_state)
    {
        case SCD41_MEAS_FINISHED: // Measurement finished
        {
            scd41_power(scd41, false); // Power off
            scd41->wake_time = make_timeout_time_ms(INT32_MAX); // Disable timer
            return;
        }
        case SCD41_MEAS_START: // Measurement started
        {
            scd41_power(scd41, true); // Power off
            scd41->wake_time = make_timeout_time_ms(30); // Time for power stabilization
            scd41->meas_state = SCD41_READ_MODE; // Next step - read status
            i = 0; // Initialize read status timeout iterator
            return;
        }
        case SCD41_READ_MODE:
        {
            if (scd41->config->enable_single_meas)
            {
                scd41->meas_state = SCD41_WRITE_MEAS_CMD; // Set next state to send single meas command
            }
            else
            {
                scd41->meas_state = SCD41_READ_STATUS; // Set next state to read measurement status
                scd41->wake_time = make_timeout_time_ms(5000); // Wait 5 seconds
            }
            return;
        }
        case SCD41_WRITE_MEAS_CMD:
        {
            ret = scd41_write_command(CMD_MEASURE_SINGLE_ALL); // Send start measurement command
            if (ret != 0) // On invalid write
            {
                scd41->co2 = NAN; // Set values to NaN
                scd41->temperature = NAN;
                scd41->humidity = NAN;
                scd41->meas_state = SCD41_MEAS_FINISHED; // Finished measurement
                scd41->state = ret; // Set sensor state to return value
                return;
            }
            scd41->meas_state = SCD41_READ_STATUS; // Set next state to read measurement status
            scd41->wake_time = make_timeout_time_ms(5000); // Wait 5 seconds
            return;
        }
        case SCD41_READ_STATUS: // Reading status
        {
            ret = scd41_read(CMD_GET_DATA_RDY_STATUS, &tempBuffer, 1); // Reading status register
            if (ret != 0) // On invalid read
            {
                scd41->co2 = NAN; // Set values to NaN
                scd41->temperature = NAN;
                scd41->humidity = NAN;
                scd41->meas_state = SCD41_MEAS_FINISHED; // Finished measurement
                scd41->state = ret; // Set sensor state to return value
                return;
            }
            if (tempBuffer & ~(0x7FF) != 0) // On data ready
            {
                scd41->meas_state = SCD41_READ_VALUE; // Next step - read values
                return;
            }
            if (i++ > 50) // On timeout
            {
                scd41->co2 = NAN; // Set values to NaN
                scd41->temperature = NAN;
                scd41->humidity = NAN;
                scd41->state = SCD30_ERROR_DATA_READY_TIMEOUT; // Set sensor state
                scd41->meas_state = SCD41_MEAS_FINISHED; // Finished measurement
                return;
            }
            scd41->wake_time = make_timeout_time_ms(100); // Check status after 100 ms
            return;
        }
        case SCD41_READ_VALUE: // Reading values
        {
            uint16_t buf[6];
            ret = scd30_read(CMD_READ_MEAS, buf, 6); // Read measured data
            if (ret != 0)
            {
                scd41->co2 = NAN; // Set values to NaN
                scd41->temperature = NAN;
                scd41->humidity = NAN;
                scd41->state = SCD30_ERROR_DATA_READY_TIMEOUT; // Set sensor state
                scd41->meas_state = SCD41_MEAS_FINISHED; // Finished measurement
                return;
            }
            scd41->co2 = (float)buf[0]; // Convert co2 to float
            scd41->temperature = -45 + 175 * (float)buf[1] / UINT16_MAX; // Convert temperature to float
            scd41->humidity = 100 * (float)buf[2] / UINT16_MAX; // Convert humidity to float

            scd41->meas_state = SCD41_MEAS_FINISHED; // Finished measurement
            scd41->state = SUCCESS; // Set state
            return;
        }
    }
}

void scd41_power(scd41_t* scd41, bool on)
{
    if (!scd41->config->power_global_control) // If power not controlled globally
    {
        // Read power vector
        // Check if bit turned [on]
        // Write power vector
    }
}

int32_t scd41_init(scd41_t* scd41, scd41_config_t* config)
{
    int32_t ret;
    scd41->config = config;
    scd41_power(scd41, true);

    scd41_init_struct(scd41);

    ret = scd30_write_config(config);
    scd41_power(scd41, false);
    return ret;
}

int32_t scd30_write_config(scd41_config_t* config)
{
    int32_t ret;
    uint16_t val;

    if ((ret = scd41_write_command(CMD_STOP_PER_MEAS)) != 0) return ret;
    sleep_ms(500);
    if ((ret = scd41_read(CMD_GET_PRESSURE, &val, 1)) != 0) return ret;
    if (config->pressure_comp != (val != 1013))
    {
        if (config->pressure_comp)
        {
            if ((ret = scd41_write_value(CMD_SET_PRESSURE, config->pressure)) != 0) return ret;
        }
        else
        {
            if ((ret = scd41_write_value(CMD_SET_PRESSURE, 1013)) != 0) return ret;
        }
    }

    if ((ret = scd41_read(CMD_GET_AUTO_SELF_CAL_EN, &val, 1)) != 0) return ret;
    if (config->enable_autocal != (val != 0))
    {
        if (config->enable_autocal)
        {
            if ((ret = scd41_write_value(CMD_SET_AUTO_SELF_CAL_EN, 1)) != 0) return ret;
        }
        else
        {
            if ((ret = scd41_write_value(CMD_SET_AUTO_SELF_CAL_EN, 0)) != 0) return ret;
        }
    }

    if ((ret = scd41_read(CMD_GET_T_OFFSET, &val, 1)) != 0) return ret;
    if ((uint16_t)(config->temperature_offset * (UINT16_MAX / 175)) != val)
    {
        if ((ret = scd41_write_value(CMD_SET_T_OFFSET, (uint16_t)(config->temperature_offset * (UINT16_MAX / 175)))) != 0) return ret;
    }

    if ((ret = scd41_read(CMD_GET_ALTITUDE, &val, 1)) != 0) return ret;
    if (config->enable_altitude_comp != (val != 0))
    {
        if (config->enable_altitude_comp)
        {
            if ((ret = scd41_write_value(CMD_SET_ALTITUDE, config->altitude)) != 0) return ret;
        }
        else
        {
            if ((ret = scd41_write_value(CMD_SET_ALTITUDE, 0)) != 0) return ret;
        }
    }

    //if ((ret = scd41_write_command(CMD_PERSIST_SETTINGS)) != 0) return ret; // Save settings to EEPROM

    if (!config->enable_single_meas)
    {
        if ((ret = scd41_write_command(CMD_START_PER_MEAS)) != 0) return ret;
    }

    return SUCCESS;
}

int32_t scd30_read_config(scd41_config_t* config)
{
    int32_t ret;
    uint16_t val;
    if ((ret = scd30_read(CMD_GET_PRESSURE, &val, 1)) != 0) return ret;
    config->pressure = val;
    config->pressure_comp = (val != 0);

    if ((ret = scd30_read(CMD_GET_AUTO_SELF_CAL_EN, &val, 1)) != 0) return ret;
    config->enable_autocal = (bool)val;

    if ((ret = scd30_read(CMD_GET_T_OFFSET, &val, 1)) != 0) return ret;
    config->temperature_offset = val * (175.0f / UINT16_MAX);

    if ((ret = scd30_read(CMD_GET_ALTITUDE, &val, 1)) != 0) return ret;
    config->altitude = val;
    config->enable_altitude_comp = (val != 0);

    return SUCCESS;
}

void scd41_init_struct(scd41_t* scd41)
{
    scd41->co2 = .0f; // Init CO2
    scd41->humidity = .0f; // Init humidity
    scd41->temperature = .0f; // Init temperature
    scd41->state = ERROR_SENSOR_NOT_INITIALIZED; // Sensor not initialized state
    scd41->meas_state = (scd41_meas_state_e)0; // Measurement not started
    scd41->wake_time = make_timeout_time_ms(INT32_MAX); // Disable timeout
}

void scd41_reset(void)
{
    scd41_write_command(CMD_RESET);
    sleep_ms(30);
}

void scd41_factory_reset(void)
{
    scd41_write_command(CMD_FACTORY_RESET);
    sleep_ms(1200);
}





