/**
 * @file generic_co2.c
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Implements communication with a generic HC CO2 probe
 * @version 0.1
 * @date 2024-06-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "generic_co2.h"
#include "math.h"
#include "hardware/i2c.h"
#include "string.h"
#include <stdio.h>
#include "common/debug.h"
#include "error_codes.h"
#include "common/functions.h"
#include "../power/power.h"

#include "uart/uart.h"
#include "hardware/watchdog.h"

#define GENERIC_CO2_SENSOR_ADDR              0x1A
#define GENERIC_CO2_SENSOR_MAX_REG_READ      8

#define MODBUS_FN_CODE_READ_MULTI 0x03
#define MODBUS_FN_CODE_WRITE_MULTI 0x10


// Register addresses - float
#define REG_T_C_FLOAT           (0x03EA)
#define REG_CO2_AVG_FLOAT       (0x0424)
#define REG_CO2_RAW_NPC_FLOAT   (0x042A)
#define REG_P_MBAR_FLOAT        (0x04B0)

#define REG_STATUS              (0x01F9)
#define REG_STATUS_DETAIL       (0x0258)

sensor_functions_t generic_co2_functions = {
    .sensor_get_value = generic_co2_get_value,
    .sensor_init = generic_co2_init,
    .sensor_read_config = NULL
};

/**
 * @brief Computes Modbus CRC for specified buffer
 * 
 * @param buf Buffer the CRC is computed from
 * @param len Length of the buffer
 * @return uint16_t CRC value
 */
static inline uint16_t generic_co2_modbus_crc(uint8_t* buf, uint32_t len);

/**
 * @brief Switches sensor power to [on] state if not controlled globally
 * 
 * @param sensor Sensor structure
 * @param on if the power should be switched on (true) or off (false)
 */
static inline void generic_co2_power(sensor_t* sensor, bool on);


static inline uint16_t generic_co2_modbus_crc(uint8_t* buf, uint32_t len)
{
    uint16_t crc = 0xFFFF;
    uint32_t i;

    for (i = 0; i < len; i++) 
    {
        crc ^= (uint16_t)buf[i];

        for (int i = 8; i != 0; i--) 
        {
            if ((crc & 0x0001) != 0) 
            {
                crc >>= 1;
                crc ^= 0xA001;
            } 
            else 
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

int32_t generic_co2_read(uint16_t addr, uint16_t nreg, uint16_t* buf)
{
    int32_t ret;
    uint8_t commandBuffer[(GENERIC_CO2_SENSOR_MAX_REG_READ * 2) + 8];
    if (nreg < 1 || nreg > 8) return EE895_ERROR_NREG_REG; // Number of registers to read not in [1, 8]

    commandBuffer[0] = GENERIC_CO2_SENSOR_ADDR; // Slave address
    commandBuffer[1] = MODBUS_FN_CODE_READ_MULTI; // Read multiple holding registers
    // *( (uint16_t*)&commandBuffer[2]) = ntoh16(addr);
    commandBuffer[2] = (addr & 0xFF00) >> 8; // Convert reg address to big endian
    commandBuffer[3] = (addr & 0x00FF) >> 0;
    // *( (uint16_t*)&commandBuffer[4]) = ntoh16(nreg);
    commandBuffer[4] = (nreg & 0xFF00) >> 8; // Convert number of registers to big endian
    commandBuffer[5] = (nreg & 0x00FF) >> 0;
    // *( (uint16_t*)&commandBuffer[6]) = ee_modbus_crc(commandBuffer, 6);
    uint16_t crc = generic_co2_modbus_crc(commandBuffer, 6); // CRC computation
    commandBuffer[6] = (crc & 0x00FF) >> 0;
    commandBuffer[7] = (crc & 0xFF00) >> 8;

    if ((ret = i2c_write_timeout_us(I2C_SENSOR, GENERIC_CO2_SENSOR_ADDR, &commandBuffer[1], 7, true, I2C_TIMEOUT_US)) < 0) return ret; // Write to slave
    // busy_wait_ms(2);

    if ((ret = i2c_read_timeout_us(I2C_SENSOR, GENERIC_CO2_SENSOR_ADDR, &commandBuffer[1], nreg * 2 + 4, false, I2C_TIMEOUT_US)) < 0) return ret; // Read from slave
    if (commandBuffer[1] != 0x03 || commandBuffer[2] != 2 * nreg) return EE895_ERROR_READ_RESP; // Check valid command & number of registers

    if (generic_co2_modbus_crc(commandBuffer, nreg * 2 + 5) != 0) return EE895_ERROR_INVALID_CRC; // Check CRC
    for (uint8_t i = 0; i < nreg; i++)
    {
        buf[i] = (commandBuffer[4 + 2 * i] * 256) + commandBuffer[3 + 2 * i];
    }
    return 0;
}

int32_t generic_co2_write(uint16_t addr, uint16_t nreg, uint16_t* buffer)
{
    int32_t ret;
    uint8_t commandBuffer[2 * GENERIC_CO2_SENSOR_MAX_REG_READ + 9];

    commandBuffer[0] = GENERIC_CO2_SENSOR_ADDR; // Slave address
    commandBuffer[1] = MODBUS_FN_CODE_WRITE_MULTI; // Write multiple holding registers

    commandBuffer[2] = (addr & 0xFF00) >> 8; // Convert reg address to big endian
    commandBuffer[3] = (addr & 0x00FF) >> 0;
    commandBuffer[4] = (nreg & 0xFF00) >> 8; // Convert number of registers to big endian
    commandBuffer[5] = (nreg & 0x00FF) >> 0;
    commandBuffer[6] = nreg * 2;
    for (uint8_t i = 0; i < nreg; i++)
    {
        commandBuffer[7 + 2 * i] = buffer[i] % 256;
        commandBuffer[8 + 2 * i] = buffer[i] / 256;
    }
    uint16_t crc = generic_co2_modbus_crc(commandBuffer, 2 * nreg + 7);
    commandBuffer[2 * nreg + 7] = (crc & 0x00FF) >> 0;
    commandBuffer[2 * nreg + 8] = (crc & 0xFF00) >> 8;

    if ((ret = i2c_write_timeout_us(I2C_SENSOR, GENERIC_CO2_SENSOR_ADDR, &commandBuffer[1], 2 * nreg + 8, true, I2C_TIMEOUT_US)) < 0) return ret; // Write to slave
    // busy_wait_ms(3);

    memset(&commandBuffer[1], 0x00, 2 * nreg + 9);
    if ((ret = i2c_read_timeout_us(I2C_SENSOR, GENERIC_CO2_SENSOR_ADDR, &commandBuffer[1], 7, false, I2C_TIMEOUT_US)) < 0) return ret; // Read from slave

    if (commandBuffer[1] != MODBUS_FN_CODE_WRITE_MULTI) return EE895_ERROR_WRITE_RESP; // Check valid command & value
    if (generic_co2_modbus_crc(commandBuffer, 8) != 0) return EE895_ERROR_INVALID_CRC; // Check CRC

    return 0;
}

int32_t generic_co2_write_float(uint16_t addr, float value)
{
    uint32_t float_hex = float2byte(value);
    uint16_t buffer[2];
    buffer[0] = (float_hex & 0xFFFF0000) >> 16;
    buffer[1] = (float_hex & 0x0000FFFF) >> 0;

    return generic_co2_write(addr, 2, buffer);
}

int32_t generic_co2_read_float(uint16_t addr, float* value)
{
    uint16_t fbuffer[2];
    int32_t ret = generic_co2_read(addr, 2, fbuffer);
    uint32_t float_hex = ((uint32_t)fbuffer[0] << 16) | fbuffer[1];
    *value = byte2float(float_hex);
    return ret;
}

static inline void generic_co2_power(sensor_t* sensor, bool on)
{
    if (!sensor->config.power_global_control && !sensor->config.power_continuous) // If power not controlled globally
    {
        power_en_set_index(sensor->index, on);
    }
}

void generic_co2_init(sensor_t* sensor)
{
    sensor->internal_error_state = STATE_OK;
    return;
}

void generic_co2_get_value(sensor_t* sensor)
{
    uint16_t tempBuffer[2] = {0};
    int32_t ret;
    if (sensor->sensor_type != GENERIC_CO2) // Check for correct sensor type
    {
        sensor->meas_state = MEAS_FINISHED;
        sensor->error_state = ERROR_SENSOR_UNKNOWN_SENSOR;
        sensor->co2 = NAN;
        sensor->pressure = NAN;
        sensor->temperature = NAN;
        return;
    } 
    switch(sensor->meas_state)
    {
        case MEAS_FINISHED: // Measurement finished
        {
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_GENERIC_CO2, "Meas finished");
            sensor->wake_time = at_the_end_of_time; // Disable timer
            return;
        }
        case MEAS_STARTED: // Measurement started
        {
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_GENERIC_CO2, "Meas started");
            sensor->internal_error_state = PICO_OK;
            if (!sensor->config.power_continuous) sensor->wake_time = make_timeout_time_us(1000 * (uint64_t)sensor->config.sensor_power_up_time); // Time for power stabilization
            sensor->meas_state = MEAS_READ_STATUS; // Next step - read status
            sensor->timeout_iterator = 0; // Initialize read status timeout iterator
            return;
        }
        case MEAS_READ_STATUS: // Reading status
        {
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_GENERIC_CO2, "Read status");
            ret = generic_co2_read(REG_STATUS, 2, tempBuffer); // Reading status register
            if (ret != 0) // On invalid read
            {
                sensor->co2 = NAN; // Set values to NaN
                sensor->temperature = NAN;
                sensor->pressure = NAN;
                sensor->meas_state = MEAS_FINISHED; // Finished measurement
                sensor->internal_error_state = ret; // Set sensor state to return value
                return;
            }
            uint16_t status = tempBuffer[0] | tempBuffer[1];
            if (!status) // On data ready
            {
                sensor->meas_state = MEAS_READ_VALUE; // Next step - read values
                return;
            }
            if (sensor->timeout_iterator++ > 20) // On timeout
            {
                sensor->co2 = NAN; // Set values to NaN
                sensor->temperature = NAN;
                sensor->pressure = NAN;
                sensor->internal_error_state = EE895_ERROR_DATA_READY_TIMEOUT; // Set sensor state
                sensor->meas_state = MEAS_FINISHED; // Finished measurement
                return;
            }
            sensor->wake_time = make_timeout_time_us(25000); // Check status after 25 ms
            return;
        }
        case MEAS_READ_VALUE: // Reading values
        {
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_GENERIC_CO2, "Read value");
            float val;
            ret = generic_co2_read_float(REG_T_C_FLOAT, &val); // Read temperature
            if (ret != 0) // On invalid read
            {
                sensor->temperature = NAN; // Set values to NaN
                sensor->pressure = NAN;
                sensor->co2 = NAN;
                sensor->meas_state = MEAS_FINISHED; // Measurement finished
                sensor->internal_error_state = ret; // Set sensor state to return value
                return;
            }
            sensor->temperature = val; // Assign value

            ret = generic_co2_read_float(REG_CO2_AVG_FLOAT, &val); // Read co2
            if (ret != 0) // On invalid read
            {
                sensor->co2 = NAN; // Set values to NaN
                sensor->pressure = NAN;
                sensor->meas_state = MEAS_FINISHED; // Measurement finished
                sensor->internal_error_state = ret; // Set sensor state to return value
                return;
            }
            sensor->co2 = val; // Assign value

            ret = generic_co2_read_float(REG_P_MBAR_FLOAT, &val); // Read pressure
            if (ret != 0) // On invalid read
            {
                sensor->pressure = NAN; // Set value to NaN
                sensor->meas_state = MEAS_FINISHED; // Measurement finished
                sensor->internal_error_state = ret; // Set sensor state to return value
                return;
            }
            sensor->pressure = val; // Assign value
            sensor->meas_state = MEAS_FINISHED; // Finished measurement
            sensor->internal_error_state = SUCCESS; // Set state
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_GENERIC_CO2, "Measured CO2 value: %f", sensor->co2);
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_GENERIC_CO2, "Measured temperature value: %f", sensor->temperature);
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_GENERIC_CO2, "Measured pressure value: %f", sensor->pressure);
            return;
        }
        default:
        {
            sensor->meas_state = MEAS_FINISHED;
            return;
        }
    }
}