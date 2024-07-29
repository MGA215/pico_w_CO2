/**
 * @file ee895.c
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Implements communication with E+E EE895 sensor
 * @version 0.1
 * @date 2024-06-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "ee895.h"
#include "math.h"
#include "hardware/i2c.h"
#include "string.h"

#define EE895_ADDR              0x5F
#define EE895_MAX_REG_READ      8

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

/**
 * @brief Computes Modbus CRC for specified buffer
 * 
 * @param buf Buffer the CRC is computed from
 * @param len Length of the buffer
 * @return uint16_t CRC value
 */
static inline uint16_t ee_modbus_crc(uint8_t* buf, uint32_t len);

/**
 * @brief Reads number of registers from the EE895
 * 
 * @param addr Address of the register to be read from
 * @param nreg Number of registers to read
 * @param buf Output buffer of values
 * @return int32_t Return code
 */
static int32_t ee_read(uint16_t addr, uint16_t nreg, uint8_t* buf);

/**
 * @brief Writes a value to the EE895
 * 
 * @param addr Address of the register to be written to
 * @param value Value to be written
 * @return int32_t Return code
 */
static int32_t ee_write(uint16_t addr, uint16_t value);

/**
 * @brief Writes configuration to the sensor
 * 
 * @param config Configuration to write
 * @return int32_t Return code
 */
static int32_t ee_write_config(sensor_config_t* config);

/**
 * @brief Switches sensor power to [on] state if not controlled globally
 * 
 * @param ee895 Sensor structure
 * @param on if the power should be switched on (true) or off (false)
 */
static inline void ee_power(sensor_t* ee895, bool on);


static inline uint16_t ee_modbus_crc(uint8_t* buf, uint32_t len)
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

static int32_t ee_read(uint16_t addr, uint16_t nreg, uint8_t* buf)
{
    int32_t ret;
    uint8_t commandBuffer[(EE895_MAX_REG_READ * 2) + 8];
    if (nreg < 1 || nreg > 8) return EE895_ERROR_NREG_REG; // Number of registers to read not in [1, 8]

    commandBuffer[0] = EE895_ADDR; // Slave address
    commandBuffer[1] = 0x03; // Read multiple holding registers
    // *( (uint16_t*)&commandBuffer[2]) = ntoh16(addr);
    commandBuffer[2] = (addr & 0xFF00) >> 8; // Convert reg address to big endian
    commandBuffer[3] = (addr & 0x00FF) >> 0;
    // *( (uint16_t*)&commandBuffer[4]) = ntoh16(nreg);
    commandBuffer[4] = (nreg & 0xFF00) >> 8; // Convert number of registers to big endian
    commandBuffer[5] = (nreg & 0x00FF) >> 0;
    // *( (uint16_t*)&commandBuffer[6]) = ee_modbus_crc(commandBuffer, 6);
    uint16_t crc = ee_modbus_crc(commandBuffer, 6); // CRC computation
    commandBuffer[6] = (crc & 0x00FF) >> 0;
    commandBuffer[7] = (crc & 0xFF00) >> 8;

    if ((ret = i2c_write_timeout_us(I2C_SENSOR, EE895_ADDR, &commandBuffer[1], 7, true, I2C_TIMEOUT_US)) < 0) return ret; // Write to slave
    busy_wait_ms(2);

    if ((ret = i2c_read_timeout_us(I2C_SENSOR, EE895_ADDR, &commandBuffer[1], nreg * 2 + 4, false, I2C_TIMEOUT_US)) < 0) return ret; // Read from slave
    if (commandBuffer[1] != 0x03 || commandBuffer[2] != 2 * nreg) return EE895_ERROR_READ_RESP; // Check valid command & number of registers

    if (ee_modbus_crc(commandBuffer, nreg * 2 + 5) != 0) return EE895_ERROR_INVALID_CRC; // Check CRC
    memcpy(buf, &commandBuffer[3], nreg * 2); // Copy to output buffer
    return 0;
}

static int32_t ee_write(uint16_t addr, uint16_t value)
{
    int32_t ret;
    uint8_t commandBuffer[2 * EE895_MAX_REG_READ + 8];

    commandBuffer[0] = EE895_ADDR; // Slave address
    commandBuffer[1] = 0x06; // Write multiple holding registers
    // *( (uint16_t*)&commandBuffer[2]) = ntoh16(addr); // Convert reg address to big endian
    // *( (uint16_t*)&commandBuffer[4]) = ntoh16(value); // Convert number of registers to big endian
    // *( (uint16_t*)&commandBuffer[6]) = ee_modbus_crc(commandBuffer, 6); // CRC computation

    commandBuffer[2] = (addr & 0xFF00) >> 8; // Convert reg address to big endian
    commandBuffer[3] = (addr & 0x00FF) >> 0;
    commandBuffer[4] = (value & 0xFF00) >> 8; // Convert number of registers to big endian
    commandBuffer[5] = (value & 0x00FF) >> 0;
    uint16_t crc = ee_modbus_crc(commandBuffer, 6); // CRC computation
    commandBuffer[6] = (crc & 0x00FF) >> 0;
    commandBuffer[7] = (crc & 0xFF00) >> 8;

    if ((ret = i2c_write_timeout_us(I2C_SENSOR, EE895_ADDR, &commandBuffer[1], 7, false, I2C_TIMEOUT_US)) < 0) return ret; // Write to slave
    busy_wait_ms(3);

    memset(&commandBuffer[1], 0x00, 7);
    if ((ret = i2c_read_timeout_us(I2C_SENSOR, EE895_ADDR, &commandBuffer[1], 7, false, I2C_TIMEOUT_US)) < 0) return ret; // Read from slave
    uint16_t val = 0;
    memcpy(&val, &commandBuffer[4], 2);
    if (commandBuffer[1] != 0x06 || (ntoh16(val)) != value) return EE895_ERROR_WRITE_RESP; // Check valid command & value
    if (ee_modbus_crc(commandBuffer, 8) != 0) return EE895_ERROR_INVALID_CRC; // Check CRC

    return 0;
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

    ret = ee_read(addr, nreg, buf); // Read data from sensor
    return ret;
}

int32_t ee895_write_reg(uint16_t addr, uint16_t value)
{
    int32_t ret;
    busy_wait_ms(250);
    ret = ee_write(addr, value); // Write data to address
    busy_wait_ms(10);
    return ret;
}

void ee895_get_value(sensor_t* ee895)
{
    uint8_t tempBuffer[4] = {0};
    int32_t ret;
    if (ee895->config.sensor_type != EE895) // Check for correct sensor type
    {
        ee895->meas_state = MEAS_FINISHED;
        ee895->state = ERROR_UNKNOWN_SENSOR;
        ee895->co2 = NAN;
        ee895->pressure = NAN;
        ee895->temperature = NAN;
        return;
    } 
    switch(ee895->meas_state)
    {
        case MEAS_FINISHED: // Measurement finished
        {
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_EE895, "Meas finished");
            ee_power(ee895, false); // Power off
            ee895->wake_time = at_the_end_of_time; // Disable timer
            return;
        }
        case MEAS_STARTED: // Measurement started
        {
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_EE895, "Meas started");
            ee_power(ee895, true); // Power on
            ee895->wake_time = make_timeout_time_ms(750); // Time for power stabilization
            if (ee895->config.single_meas_mode) 
            {
                ee895->meas_state = MEAS_TRIGGER_SINGLE_MEAS; // If single measurement mode - wait for trigger ready
            }
            else ee895->meas_state = MEAS_READ_STATUS; // Next step - read status
            ee895->timeout_iterator = 0; // Initialize read status timeout iterator
            ee895->state = SUCCESS;
            return;
        }
        case MEAS_TRIGGER_SINGLE_MEAS:
        {
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_EE895, "Read status for trigger");
            ret = ee_read(REG_STATUS, 1, tempBuffer); // Read status register
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
                ret = ee_write(REG_MEAS_TRIGGER, 1); // Send trigger
                print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_EE895, "Sending trigger");
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
                ee895->timeout_iterator = 0; // Reset iterator
                return;
            }
            if (ee895->timeout_iterator++ > 20) // If timeout
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
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_EE895, "Read status");
            ret = ee_read(REG_STATUS, 1, tempBuffer); // Reading status register
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
            if (ee895->timeout_iterator++ > 20) // On timeout
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
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_EE895, "Read value");
            ret = ee_read(REG_T_C_FLOAT, 2, tempBuffer); // Read temperature
            if (ret != 0) // On invalid read
            {
                ee895->temperature = NAN; // Set values to NaN
                ee895->pressure = NAN;
                ee895->co2 = NAN;
                ee895->meas_state = MEAS_FINISHED; // Measurement finished
                ee895->state = ret; // Set sensor state to return value
                return;
            }
            // *( (uint32_t*)&tempBuffer[0]) = ntoh32(*( (uint32_t*)&tempBuffer[0])); // Fix read value to uint32
            uint32_t tempval = 0; // Fix read value to uint32
            tempval |= (tempBuffer[0]) << 0;
            tempval |= (tempBuffer[1]) << 8;
            tempval |= (tempBuffer[2]) << 16;
            tempval |= (tempBuffer[3]) << 24;
            float val = byte2float(ntoh32(tempval)); // Convert read value to float
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

            ret = ee_read(REG_CO2_AVG_FLOAT, 2, tempBuffer); // Read co2
            if (ret != 0) // On invalid read
            {
                ee895->co2 = NAN; // Set values to NaN
                ee895->pressure = NAN;
                ee895->meas_state = MEAS_FINISHED; // Measurement finished
                ee895->state = ret; // Set sensor state to return value
                return;
            }
            // *( (uint32_t*)&tempBuffer[0]) = ntoh32(*( (uint32_t*)&tempBuffer[0])); // Fix read value to uint32_t
            tempval = 0; // Fix read value to uint32_t
            tempval |= (tempBuffer[0]) << 0;
            tempval |= (tempBuffer[1]) << 8;
            tempval |= (tempBuffer[2]) << 16;
            tempval |= (tempBuffer[3]) << 24;
            val = byte2float(ntoh32(tempval)); // Convert read value to float
            if (val < CO2_MIN_RANGE || val > CO2_MAX_RANGE) // Check float range
            {
                ee895->co2 = NAN; // Set values to NaN
                ee895->pressure = NAN;
                ee895->meas_state = MEAS_FINISHED; // Finished measurement
                ee895->state = EE895_ERROR_RANGE; // Set state
                return;
            }
            ee895->co2 = val; // Assign value

            ret = ee_read(REG_P_MBAR_FLOAT, 2, tempBuffer); // Read pressure
            if (ret != 0) // On invalid read
            {
                ee895->pressure = NAN; // Set value to NaN
                ee895->meas_state = MEAS_FINISHED; // Measurement finished
                ee895->state = ret; // Set sensor state to return value
                return;
            }
            tempval = 0; // Fix read value to uint32_t
            tempval |= (tempBuffer[0]) << 0;
            tempval |= (tempBuffer[1]) << 8;
            tempval |= (tempBuffer[2]) << 16;
            tempval |= (tempBuffer[3]) << 24;
            val = byte2float(ntoh32(tempval)); // Convert read value to float
            ee895->pressure = val; // Assign value
            ee895->meas_state = MEAS_FINISHED; // Finished measurement
            ee895->state = SUCCESS; // Set state
            return;
        }
        default:
        {
            ee895->meas_state = MEAS_FINISHED;
            return;
        }
    }
}

int32_t ee895_init(sensor_t* ee895, sensor_config_t* config)
{
    int32_t ret;
    if (config->sensor_type != EE895) return ERROR_UNKNOWN_SENSOR; // Check for correct sensor type
    memcpy(&ee895->config, config, sizeof(sensor_config_t));
    ee_power(ee895, true); // Power on

    uint8_t fw_read_name[16];
    if ((ret = ee895_read_reg(REG_FW_NAME, 8, fw_read_name)) != 0) // Read sensor name
    {
        ee_power(ee895, false); // Power off
        return ret;
    }
    if (strcmp(fw_read_name, "EE895") != 0) // Check sensor name
    {
        ee_power(ee895, false); // Power off
        return ERROR_UNKNOWN_SENSOR;
    }

    ret = ee_write_config(config); // Write configuration to sensor

    ee_power(ee895, false); // Power off
    if (!ret)
    {
        if (ee895->meas_state == MEAS_STARTED) ee895->wake_time = make_timeout_time_ms(1000);
    }
    else ee895->meas_state = MEAS_FINISHED;

    return ret;
}

int32_t ee895_read_config(sensor_config_t* config)
{
    int32_t ret;
    uint8_t buf[6] = {0};
    config->sensor_type = EE895;

    if ((ret = ee895_read_reg(REG_MEAS_INTERVAL, 3, buf)) != 0) return ret; // Read config
    memcpy(&config->meas_period, &buf[0], 2);
    config->meas_period = ntoh16(config->meas_period) / 10; // Save measurement interval
    // config->meas_period = (uint16_t)ntoh16(*( (uint16_t*)&buf[0])) / 10;

    memcpy(&config->filter_coeff, &buf[2], 2);
    config->filter_coeff = ntoh16(config->filter_coeff); // Save measurement interval
    //config->filter_coeff = (uint16_t)ntoh16(*( (uint16_t*)&buf[2])); // Save filter coefficient

    memcpy(&config->co2_offset, &buf[4], 2);
    config->co2_offset = ntoh16(config->co2_offset); // Save measurement interval
    // config->co2_offset = (int16_t)ntoh16(*( (uint16_t*)&buf[4])); // Save offset

    if ((ret = ee895_read_reg(REG_MEAS_MODE, 1, buf)) != 0) return ret; // Read measurement mode
    uint16_t val = 0;
    val |= buf[0] << 8;
    val |= buf[1] << 0;
    config->single_meas_mode = (bool)ntoh16(val); // Save measurement mode
    return SUCCESS;
}

static int32_t ee_write_config(sensor_config_t* config)
{
    int32_t ret;
    uint8_t buf[6] = {0};

    sensor_config_t read_config;
    if ((ret = ee895_read_config(&read_config)) != 0) return ret; // Read config

    if (read_config.meas_period != config->meas_period) { // If measurement period changed
        print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_EE895, "Config - Writing measurement period");
        if ((ret = ee895_write_reg(REG_MEAS_INTERVAL, config->meas_period * 10)) != 0) return ret;
    }
    if (read_config.filter_coeff != config->filter_coeff) { // If filter coeff changed
        print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_EE895, "Config - Writing filter coefficient");
        if ((ret = ee895_write_reg(REG_MEAS_FILTER, config->filter_coeff)) != 0) return ret;
    }
    if (read_config.co2_offset != config->co2_offset) // If offset changed
    {
        print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_EE895, "Config - Writing CO2 offset");
        if ((ret = ee895_write_reg(REG_MEAS_OFFSET, (uint16_t)config->co2_offset)) != 0) return ret;
    }
    if (read_config.single_meas_mode != config->single_meas_mode) // If measurement mode changed
    {
        print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_EE895, "Config - Writing measurement mode");
        if ((ret = ee895_write_reg(REG_MEAS_MODE, config->single_meas_mode)) != 0) return ret;
    }
    return SUCCESS;
}

static inline void ee_power(sensor_t* ee895, bool on)
{
    if (!ee895->config.power_global_control) // If power not controlled globally
    {
        // Read power vector
        // Check if bit turned [on]
        // Write power vector
    }
}