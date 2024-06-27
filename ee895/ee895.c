#include "ee895.h"
#include <stdio.h>

#define EE895_ADDR 0x5F
#define EE895_MAX_REG_READ 8

// CO2 range
#define CO2_MIN_RANGE           0.0
#define CO2_MAX_RANGE           10000.0

// temperature range
#define T_MIN_RANGE            -40.0
#define T_MAX_RANGE             60.0

// Register addresses - float
#define REG_T_C_FLOAT           (0x03EA)
#define REG_T_F_FLOAT           (0x03EC)
#define REG_T_K_FLOAT           (0x03F0)
#define REG_CO2_AVG_FLOAT       (0x0424)
#define REG_CO2_RAW_FLOAT       (0x0426)
#define REG_CO2_AVG_NPC_FLOAT   (0x0428)
#define REG_CO2_RAW_NPC_FLOAT   (0x042A)
#define REG_P_MBAR_FLOAT        (0x04B0)
#define REG_P_PSI_FLOAT         (0x04B2)
// Register addresses - int16
#define REG_T_C_INT16           (0x0FA1)
#define REG_T_F_INT16           (0x0FA2)
#define REG_T_K_INT16           (0x0FA4)
#define REG_CO2_AVG_INT16       (0x0FBE)
#define REG_CO2_RAW_INT16       (0x0FBF)
#define REG_CO2_AVG_NPC_INT16   (0x0FC0)
#define REG_CO2_RAW_NPC_INT16   (0x0FC1)
#define REG_P_MBAR_INT16        (0x1004)
#define REG_P_PSI_INT16         (0x1005)
// User register addresses
#define REG_USER1               (0x16A8)
#define REG_USER2               (0x16A9)
// Sensor information
#define REG_SER_NR              (0x0000)
#define REG_FW_VER              (0x0008)
#define REG_FW_NAME             (0x0009)
// Sensor general settings
#define REG_MEAS_MODE           (0x01F8)
#define REG_STATUS              (0x01F9)
#define REG_MEAS_TRIGGER        (0x01FA)
#define REG_STATUS_DETAIL       (0x0258)
// Sensor CO2 settings
#define REG_MEAS_INTERVAL       (0x1450)
#define REG_MEAS_FILTER         (0x1451)
#define REG_MEAS_OFFSET         (0x1452)

#define msg(x) printf("[%u] [EE895] %s\n", to_ms_since_boot(get_absolute_time()), x)

/**
 * @brief Computes Modbus CRC for specified buffer
 * 
 * @param buf Buffer the CRC is computed from
 * @param len Length of the buffer
 * @return uint16_t CRC value
 */
uint16_t ee895_modbus_crc(uint8_t* buf, uint32_t len);

/**
 * @brief Writes configuration to the sensor
 * 
 * @param config Configuration to write
 * @return int32_t Return code
 */
int32_t ee_write_config(sensor_config_t* config);

uint16_t ee895_modbus_crc(uint8_t* buf, uint32_t len)
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

int32_t ee895_read(uint16_t addr, uint16_t nreg, uint8_t* buf)
{
    int32_t ret;
    uint8_t commandBuffer[(EE895_MAX_REG_READ * 2) + 8];
    if (nreg < 1 || nreg > 8) return EE895_ERROR_NREG_REG; // Number of registers to read not in [1, 8]

    commandBuffer[0] = EE895_ADDR; // Slave address
    commandBuffer[1] = 0x03; // Read multiple holding registers
    *((uint16_t*)&commandBuffer[2]) = ntoh16(addr); // Convert reg address to big endian
    *((uint16_t*)&commandBuffer[4]) = ntoh16(nreg); // Convert number of registers to big endian
    *((uint16_t*)&commandBuffer[6]) = ee895_modbus_crc(commandBuffer, 6); // CRC computation

    if ((ret = i2c_write_timeout_us(I2C_SENSOR, EE895_ADDR, &commandBuffer[1], 7, true, I2C_TIMEOUT_US)) < 0) return ret; // Write to slave
    busy_wait_ms(2);

    if ((ret = i2c_read_timeout_us(I2C_SENSOR, EE895_ADDR, &commandBuffer[1], nreg * 2 + 4, false, I2C_TIMEOUT_US)) < 0) return ret; // Read from slave
    if (commandBuffer[1] != 0x03 || commandBuffer[2] != 2 * nreg) return EE895_ERROR_READ_RESP; // Check valid command & number of registers

    if (ee895_modbus_crc(commandBuffer, nreg * 2 + 5) != 0) return EE895_ERROR_INVALID_CRC; // Check CRC
    memcpy(buf, &commandBuffer[3], nreg * 2); // Copy to output buffer
    return 0;
}

int32_t ee895_write(uint16_t addr, uint16_t value)
{
    int32_t ret;
    uint8_t commandBuffer[2 * EE895_MAX_REG_READ + 8];

    commandBuffer[0] = EE895_ADDR; // Slave address
    commandBuffer[1] = 0x06; // Write multiple holding registers
    *((uint16_t*)&commandBuffer[2]) = ntoh16(addr); // Convert reg address to big endian
    *((uint16_t*)&commandBuffer[4]) = ntoh16(value); // Convert number of registers to big endian
    *((uint16_t*)&commandBuffer[6]) = ee895_modbus_crc(commandBuffer, 6); // CRC computation

    if ((ret = i2c_write_timeout_us(I2C_SENSOR, EE895_ADDR, &commandBuffer[1], 7, false, I2C_TIMEOUT_US)) < 0) return ret; // Write to slave
    busy_wait_ms(3);

    memset(&commandBuffer[1], 0x00, 7);
    if ((ret = i2c_read_timeout_us(I2C_SENSOR, EE895_ADDR, &commandBuffer[1], 7, false, I2C_TIMEOUT_US)) < 0) return ret; // Read from slave

    if (commandBuffer[1] != 0x06 || (ntoh16(*((uint16_t*)&commandBuffer[4]))) != value) return EE895_ERROR_WRITE_RESP; // Check valid command & value
    if (ee895_modbus_crc(commandBuffer, 8) != 0) return EE895_ERROR_INVALID_CRC; // Check CRC

    return 0;
}

void ee895_get_value(sensor_t* ee895)
{
    uint8_t tempBuffer[4] = {0};
    int32_t ret;
    static int32_t i = 0;
    switch(ee895->meas_state)
    {
        case MEAS_FINISHED: // Measurement finished
        {
            #ifdef DEBUG
            msg("Meas finished");
            #endif
            ee895_power(ee895, false); // Power off
            ee895->wake_time = make_timeout_time_ms(INT32_MAX); // Disable timer
            return;
        }
        case MEAS_STARTED: // Measurement started
        {
            #ifdef DEBUG
            msg("Meas started");
            #endif
            ee895_power(ee895, true); // Power off
            ee895->wake_time = make_timeout_time_ms(750); // Time for power stabilization
            if (ee895->config->single_meas_mode) 
            {
                ee895->meas_state = MEAS_TRIGGER_SINGLE_MEAS; // If single measurement mode - wait for trigger ready
            }
            else ee895->meas_state = MEAS_READ_STATUS; // Next step - read status
            i = 0; // Initialize read status timeout iterator
            return;
        }
        case MEAS_TRIGGER_SINGLE_MEAS:
        {
            #ifdef DEBUG
            msg("Read trigger ready");
            #endif
            ret = ee895_read(REG_STATUS, 1, tempBuffer); // Read status register
            if (ret != 0) // On invalid read
            {
                ee895->co2 = NAN; // Set values to NaN
                ee895->temperature = NAN;
                ee895->pressure = NAN;
                ee895->meas_state = MEAS_FINISHED; // Finished measurement
                ee895->state = ret; // Set sensor state to return value
                return;
            }
            if (tempBuffer[1] & 0x02) // On trigger ready
            {
                ret = ee895_write(REG_MEAS_TRIGGER, 1); // Send trigger
                printf("Sending trigger...\n");
                if (ret != 0) // On invalid write
                {
                    ee895->co2 = NAN; // Set values to NaN
                    ee895->temperature = NAN;
                    ee895->pressure = NAN;
                    ee895->meas_state = MEAS_FINISHED; // Finished measurement
                    ee895->state = ret; // Set sensor state to return value
                    return;
                }
                ee895->wake_time = make_timeout_time_ms(300); // Wait for the measurement
                ee895->meas_state = MEAS_READ_STATUS; // Next step - read status
                i = 0; // Reset iterator
                return;
            }
            if (i++ > 20) // If timeout
            {
                ee895->co2 = NAN; // Set values to NaN
                ee895->temperature = NAN;
                ee895->pressure = NAN;
                ee895->state = EE895_ERROR_DATA_READY_TIMEOUT; // Set sensor state
                ee895->meas_state = MEAS_FINISHED; // Finished measurement
                return;
            }
            ee895->wake_time = make_timeout_time_ms(25); // Check next in 25 ms
            return;
        }
        case MEAS_READ_STATUS: // Reading status
        {
            #ifdef DEBUG
            msg("Read status");
            #endif
            ret = ee895_read(REG_STATUS, 1, tempBuffer); // Reading status register
            if (ret != 0) // On invalid read
            {
                ee895->co2 = NAN; // Set values to NaN
                ee895->temperature = NAN;
                ee895->pressure = NAN;
                ee895->meas_state = MEAS_FINISHED; // Finished measurement
                ee895->state = ret; // Set sensor state to return value
                return;
            }
            if (tempBuffer[1] & 0x01) // On data ready
            {
                ee895->meas_state = MEAS_READ_VALUE; // Next step - read values
                return;
            }
            if (i++ > 20) // On timeout
            {
                ee895->co2 = NAN; // Set values to NaN
                ee895->temperature = NAN;
                ee895->pressure = NAN;
                ee895->state = EE895_ERROR_DATA_READY_TIMEOUT; // Set sensor state
                ee895->meas_state = MEAS_FINISHED; // Finished measurement
                return;
            }
            ee895->wake_time = make_timeout_time_ms(25); // Check status after 25 ms
            return;
        }
        case MEAS_READ_VALUE: // Reading values
        {
            #ifdef DEBUG
            msg("Read value");
            #endif
            ret = ee895_read(REG_T_C_FLOAT, 2, tempBuffer); // Read temperature
            if (ret != 0) // On invalid read
            {
                ee895->temperature = NAN; // Set values to NaN
                ee895->pressure = NAN;
                ee895->co2 = NAN;
                ee895->meas_state = MEAS_FINISHED; // Measurement finished
                ee895->state = ret; // Set sensor state to return value
                return;
            }
            *((uint32_t*)&tempBuffer[0]) = ntoh32(*((uint32_t*)&tempBuffer[0])); // Fix read value to uint32
            float val = byte2float(*((uint32_t*)&tempBuffer[0])); // Convert read value to float
            if (val < T_MIN_RANGE || val > T_MAX_RANGE) // Check float range
            {
                ee895->temperature = NAN; // Set values to NaN
                ee895->pressure = NAN;
                ee895->co2 = NAN;
                ee895->meas_state = MEAS_FINISHED; // Finished measurement
                ee895->state = EE895_ERROR_RANGE; // Set state
                return;
            }
            ee895->temperature = val; // Assign value

            ret = ee895_read(REG_CO2_RAW_FLOAT, 2, tempBuffer); // Read co2
            if (ret != 0) // On invalid read
            {
                ee895->co2 = NAN; // Set values to NaN
                ee895->pressure = NAN;
                ee895->meas_state = MEAS_FINISHED; // Measurement finished
                ee895->state = ret; // Set sensor state to return value
                return;
            }
            *((uint32_t*)&tempBuffer[0]) = ntoh32(*((uint32_t*)&tempBuffer[0])); // Fix read value to uint32_t
            val = byte2float(*((uint32_t*)&tempBuffer[0])); // Convert read value to float
            if (val < CO2_MIN_RANGE || val > CO2_MAX_RANGE) // Check float range
            {
                ee895->co2 = NAN; // Set values to NaN
                ee895->pressure = NAN;
                ee895->meas_state = MEAS_FINISHED; // Finished measurement
                ee895->state = EE895_ERROR_RANGE; // Set state
                return;
            }
            ee895->co2 = val; // Assign value

            ret = ee895_read(REG_P_MBAR_FLOAT, 2, tempBuffer); // Read pressure
            if (ret != 0) // On invalid read
            {
                ee895->pressure = NAN; // Set value to NaN
                ee895->meas_state = MEAS_FINISHED; // Measurement finished
                ee895->state = ret; // Set sensor state to return value
                return;
            }
            *((uint32_t*)&tempBuffer[0]) = ntoh32(*((uint32_t*)&tempBuffer[0])); // Fix read value to uint32_t
            val = byte2float(*((uint32_t*)&tempBuffer[0])); // Convert read value to float
            ee895->pressure = val; // Assign value
            ee895->meas_state = MEAS_FINISHED; // Finished measurement
            ee895->state = SUCCESS; // Set state
            return;
        }
    }
}

int32_t ee895_read_reg(uint16_t addr, uint16_t nreg, uint8_t* buf)
{
    int32_t ret;
    busy_wait_ms(250);

    // for specific registers add delay
    if (((addr >= REG_T_C_FLOAT) || ((addr + nreg) >= REG_T_C_FLOAT)) && (addr <= REG_P_PSI_FLOAT)) {
        busy_wait_ms(750);
    }
    if (((addr >= REG_T_C_INT16) || ((addr + nreg) >= REG_T_C_INT16)) && (addr <= REG_P_PSI_INT16)) {
        busy_wait_ms(750);
    }

    ret = ee895_read(addr, nreg, buf); // Read data from sensor
    return ret;
}

int32_t ee895_write_reg(uint16_t addr, uint16_t value)
{
    int32_t ret;
    busy_wait_ms(250);
    ret = ee895_write(addr, value); // Write data to address
    busy_wait_ms(10);
    return ret;
}

void ee895_power(sensor_t* ee895, bool on)
{
    if (!ee895->config->power_global_control) // If power not controlled globally
    {
        // Read power vector
        // Check if bit turned [on]
        // Write power vector
    }
}

int32_t ee895_init(sensor_t* ee895, sensor_config_t* config)
{
    int32_t ret;

    ee895->config = config; // Save configuration
    ee895_power(ee895, true); // Power on

    uint8_t fw_read_name[16];
    if ((ret = ee895_read_reg(REG_FW_NAME, 8, fw_read_name)) != 0) // Read sensor name
    {
        ee895_power(ee895, false); // Power off
        return ret;
    }
    if (strcmp(fw_read_name, "EE895") != 0) // Check sensor name
    {
        ee895_power(ee895, false); // Power off
        return ERROR_UNKNOWN_SENSOR;
    }

    if ((ret = ee_write_config(config)) != 0) // Write configuration to sensor
    {
        ee895_power(ee895, false); // Power off
        return ret;
    }

    ee895_power(ee895, false); // Power off

    return SUCCESS;
}

int32_t ee_write_config(sensor_config_t* config)
{
    int32_t ret;
    uint8_t buf[6] = {0};

    bool single_meas_mode = config->single_meas_mode ? 1 : 0; // just to be sure - true in C doesnt have to be 1

    if ((ret = ee895_read_reg(REG_MEAS_INTERVAL, 3, buf)) != 0) return ret; // Read config
    int16_t meas_period = (int16_t)ntoh16(*((uint16_t*)&buf[0]));
    int16_t filter_coeff = (int16_t)ntoh16(*((uint16_t*)&buf[2]));
    int16_t offset = (int16_t)ntoh16(*((uint16_t*)&buf[4]));

    if ((ret = ee895_read_reg(REG_MEAS_MODE, 1, buf)) != 0) return ret; // Read measurement mode

    if (meas_period != config->meas_period * 10) { // If measurement period already set
        if ((ret = ee895_write_reg(REG_MEAS_INTERVAL, (uint16_t)config->meas_period * 10)) != 0) return ret;
    }
    if (filter_coeff != config->filter_coeff) { // If filter coeff already set
        if ((ret = ee895_write_reg(REG_MEAS_FILTER, (uint16_t)config->filter_coeff)) != 0) return ret;
    }
    if (offset != config->co2_offset) // If offset already set
    {
        if ((ret = ee895_write_reg(REG_MEAS_OFFSET, (uint16_t)config->co2_offset)) != 0) return ret;
    }
    if (ntoh16(*((uint16_t*)&buf[0])) != single_meas_mode) // If measurement mode already set
    {
        if ((ret = ee895_write_reg(REG_MEAS_MODE, (uint16_t)single_meas_mode)) != 0) return ret;
    }
    return SUCCESS;
}

int32_t ee895_read_config(sensor_config_t* config)
{
    int32_t ret;
    uint8_t buf[6] = {0};

    if ((ret = ee895_read_reg(REG_MEAS_INTERVAL, 3, buf)) != 0) return ret; // Read config
    config->meas_period = (int16_t)ntoh16(*((uint16_t*)&buf[0])); // Save measurement interval
    config->filter_coeff = (int16_t)ntoh16(*((uint16_t*)&buf[2])); // Save filter coefficient
    config->co2_offset = (int16_t)ntoh16(*((uint16_t*)&buf[4])); // Save offset

    if ((ret = ee895_read_reg(REG_MEAS_MODE, 1, buf)) != 0) return ret; // Read measurement mode
    config->single_meas_mode = (bool)ntoh16(*((uint16_t*)&buf[0])); // Save measurement mode
    return SUCCESS;
}

