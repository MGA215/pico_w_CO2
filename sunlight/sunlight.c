#include "sunlight.h"
#include <stdio.h>

#define SUNLIGHT_ADDR 0x68

#define SUNLIGHT_EN INT32_MAX

#define REG_ERR_H 0x00
#define REG_ERR_L 0x01
#define REG_CO2_PRESS_IIR_COMP_H 0x06
#define REG_CO2_PRESS_IIR_COMP_L 0x07
#define REG_TEMP_H 0x08
#define REG_TEMP_L 0x09
#define REG_MEAS_COUNT 0x0D
#define REG_MEAS_CYCLE_COUNT_H 0x0E
#define REG_MEAS_CYCLE_COUNT_L 0x0F
#define REG_CO2_PRESS_COMP_H 0x10
#define REG_CO2_PRESS_COMP_L 0x11
#define REG_CO2_IIR_COMP_H 0x12
#define REG_CO2_IIR_COMP_L 0x13
#define REG_CO2_UNFILTERED_H 0x14
#define REG_CO2_UNFILTERED_L 0x15

#define REG_FW_TYPE 0x2F
#define REG_FW_REV_H 0x38
#define REG_FW_REV_L 0x39
#define REG_ID_MM 0x3A
#define REG_ID_ML 0x3B
#define REG_ID_LM 0x3C
#define REG_ID_LL 0x3D

#define REG_PRODUCT_CODE 0x70 // 16 byte null-terminated string register

#define REG_CAL_STATUS 0x81
#define REG_CAL_CMD_H 0x82
#define REG_CAL_CMD_L 0x83
#define REG_CAL_TARGET 0x84
#define REG_MEAS_CONC_OVERRIDE_H 0x86
#define REG_MEAS_CONC_OVERRIDE_L 0x87
#define REG_ABC_TIME_H 0x88
#define REG_ABC_TIME_L 0x89
#define REG_ABC_PAR0_H 0x8A
#define REG_ABC_PAR0_L 0x8B
#define REG_ABC_PAR1_H 0x8C
#define REG_ABC_PAR1_L 0x8D
#define REG_ABC_PAR2_H 0x8E
#define REG_ABC_PAR2_L 0x8F
#define REG_ABC_PAR3_H 0x90
#define REG_ABC_PAR3_L 0x91

#define REG_START_SINGLE_MEAS 0x93
#define REG_MEAS_MODE 0x95
#define REG_MEAS_PERIOD_H 0x96
#define REG_MEAS_PERIOD_L 0x97
#define REG_NUM_SAMPLES_H 0x98
#define REG_NUM_SAMPLES_L 0x99
#define REG_ABC_PERIOD_H 0x9A
#define REG_ABC_PERIOD_L 0x9B
#define REG_CLEAR_ERROR 0x9D
#define REG_ABC_TARGET_H 0x9E
#define REG_ABC_TARGET_L 0x9F

#define REG_STATIC_IIR_PARAM 0xA1
#define REG_SOFT_RESET 0xA3 // Write 0xFF to reset
#define REG_METER_CONTROL 0xA5
#define REG_I2C_ADDR 0xA7

#define REG_CAL_STATUS_MIR 0xC1 // Intended for single-write i2c command for single measurement mode
#define REG_START_SINGLE_MEAS_MIR 0xC3
#define REG_ABC_TIME_MIR_H 0xC4
#define REG_ABC_TIME_MIR_L 0xC5
#define REG_ABC_PAR0_MIR_H 0xC6
#define REG_ABC_PAR0_MIR_L 0xC7
#define REG_ABC_PAR1_MIR_H 0xC8
#define REG_ABC_PAR1_MIR_L 0xC9
#define REG_ABC_PAR2_MIR_H 0xCA
#define REG_ABC_PAR2_MIR_L 0xCB
#define REG_ABC_PAR3_MIR_H 0xCC
#define REG_ABC_PAR3_MIR_L 0xCD
#define REG_FILTER_PAR0_H 0xCE
#define REG_FILTER_PAR0_L 0xCF
#define REG_FILTER_PAR1_H 0xD0
#define REG_FILTER_PAR1_L 0xD1
#define REG_FILTER_PAR2_H 0xD2
#define REG_FILTER_PAR2_L 0xD3
#define REG_FILTER_PAR3_H 0xD4
#define REG_FILTER_PAR3_L 0xD5
#define REG_FILTER_PAR4_H 0xD6
#define REG_FILTER_PAR4_L 0xD7
#define REG_FILTER_PAR5_H 0xD8
#define REG_FILTER_PAR5_L 0xD9
#define REG_FILTER_PAR6_H 0xDA
#define REG_FILTER_PAR6_L 0xDB

#define REG_AIR_PRESSURE_H 0xDC
#define REG_AIR_PRESSURE_L 0xDD
#define REG_ABC_PRESSURE_H 0xDE
#define REG_ABC_PRESSURE_L 0xDF

#define msg(x) printf("[%u] [SUNLIGHT] %s\n", to_ms_since_boot(get_absolute_time()), x)

/**
 * @brief returns error code according to the error register value
 * 
 * @param error_reg error register
 * @return int returned error code
 */
int sl_get_error(uint16_t error_reg);

/**
 * @brief Wakes up the sensor
 * 
 */
void sl_wake_up(void);

/**
 * @brief Writes configuration to the sensor
 * 
 * @param config Configuration to be written
 * @return int Return code
 */
int sl_write_config(sunlight_config_t* config);

/**
 * @brief Writes config for the meter control register
 * 
 * @param config Config for the SUNLIGHT sensor
 * @return int Return code
 */
int sl_write_meter_control(sunlight_config_t* config);

int sunlight_write(uint8_t addr, uint8_t* buf, uint16_t len)
{
    int32_t ret;
    uint8_t command_buffer[len + 1];
    command_buffer[0] = addr;
    memcpy(&command_buffer[1], buf, len);
    sl_wake_up(); // Wake sensor up
    if ((ret = i2c_write_timeout_us(I2C_SENSOR, SUNLIGHT_ADDR, command_buffer, len + 1, false, I2C_TIMEOUT_US)) < 0) return ret; // Write data to sensor
    busy_wait_ms(12);
    // uint8_t resp[len];
    // if ((ret = i2c_read_timeout_us(I2C_SENSOR, SUNLIGHT_ADDR, resp, len, false, 1000)) < 0) return ret;
    // if (memcmp(buf, resp, len)) return SUNLIGHT_ERROR_WRITE_RESP;
    return SUCCESS;
}

int sunlight_read(uint8_t addr, uint8_t* buf, uint16_t num_bytes)
{
    int32_t ret;
    sl_wake_up(); // Wake sensor up
    if ((ret = i2c_write_timeout_us(I2C_SENSOR, SUNLIGHT_ADDR, &addr, 1, true, I2C_TIMEOUT_US)) < 0) return ret; // Write address to read from
    
    busy_wait_ms(2);
    if ((ret = i2c_read_timeout_us(I2C_SENSOR, SUNLIGHT_ADDR, buf, num_bytes, false, I2C_TIMEOUT_US)) < 0) return ret; // Read data from sensor
    busy_wait_ms(1);
    return SUCCESS;
}

int sunlight_reset(void)
{
    int32_t ret;
    uint8_t data = 0xFF;
    if ((ret = sunlight_write(REG_SOFT_RESET, &data, 1)) != 0) return ret; // Send reset command to sensor
    return SUCCESS;
}

int sunlight_init(sunlight_t* sunlight, sunlight_config_t* config)
{
    int32_t ret;

    sunlight_init_struct(sunlight); // Initialize sensor structure
    
    sunlight->config = config; // Save sensor config

    sunlight_power(sunlight, true); // Power on

    // uint8_t buf[16];
    // if ((ret = sunlight_read(REG_PRODUCT_CODE, buf, 16)) != 0) return ret; // Product code returns buffer full of 0x88
    // if (strcmp(buf, "006-0-0007") != 0) return ERROR_UNKNOWN_SENSOR;
    // memset(buf, 0x00, 16);

    if ((ret = sl_write_config(config)) != 0) // Write configuration
    {
        sunlight_power(sunlight, false);
        return ret; 
    }

    sunlight_reset(); // Reset sensor
    
    sunlight_power(sunlight, false); // Power off
    return SUCCESS;
}

int sl_write_config(sunlight_config_t* config)
{
    int32_t ret;
    uint8_t buf[11] = {0};
    uint8_t command_buf[11] = {0};
    command_buf[0] = (uint8_t)config->single_meas_mode; // Prepare command buffer
    *((uint16_t*)&command_buf[1]) = ntoh16(config->meas_period);
    *((uint16_t*)&command_buf[3]) = ntoh16(config->meas_samples);
    *((uint16_t*)&command_buf[5]) = ntoh16(config->abc_period);

    *((uint16_t*)&command_buf[9]) = ntoh16(config->abc_target_value);

    if ((ret = sl_write_meter_control(config)) != 0) return ret; // Write meter control register

    if ((ret = sunlight_read(REG_MEAS_MODE, buf, 11)) != 0) return ret; // Read measurement registers
    if (memcmp(buf, command_buf, 11) == 0) return SUCCESS; // If registers already set return SUCCESS
    if ((ret = sunlight_write(REG_MEAS_MODE, command_buf, 11)) != 0) return ret; // Write measurement registers
    return SUCCESS;
}

int sunlight_read_config(sunlight_config_t* config)
{
    int32_t ret;
    uint8_t buf[11] = {0};
    uint8_t data = 0;
    if ((ret = sunlight_read(REG_MEAS_MODE, buf, 11)) != 0) return ret; // Read measurement registers
    config->single_meas_mode = (bool)buf[0]; // Save measurement settings data
    config->meas_period = ntoh16(*((uint16_t*)&buf[1]));
    config->meas_samples = ntoh16(*((uint16_t*)&buf[3]));
    config->abc_period = ntoh16(*((uint16_t*)&buf[5]));
    config->abc_target_value = ntoh16(*((uint16_t*)&buf[9]));

    if ((ret = sunlight_read(REG_METER_CONTROL, &data, 1)) != 0) return ret; // Read meter control register
    config->enable_nRDY = data & 0b1 << 0; // Save meter control data
    config->enable_ABC = data & 0b1 << 1;
    config->enable_static_IIR = data & 0b1 << 2;
    config->enable_dynamic_IIR = data & 0b1 << 3;
    config->enable_pressure_comp = data & 0b1 << 4;
    config->invert_nRDY = data & 0b1 << 5;

    return SUCCESS;
}

int sl_write_meter_control(sunlight_config_t* config)
{
    uint8_t data = 0; // Prepare command
    data |= config->enable_nRDY;
    data |= config->enable_ABC << 1;
    data |= config->enable_static_IIR << 2;
    data |= config->enable_dynamic_IIR << 3;
    data |= config->enable_pressure_comp << 4;
    data |= config->invert_nRDY << 5;

    uint8_t buf;
    int32_t ret;
    if ((ret = sunlight_read(REG_METER_CONTROL, &buf, 1)) != 0) return ret; // Read meter control register
    if (data == buf) return SUCCESS; // If already set return SUCCESS

    if ((ret = sunlight_write(REG_METER_CONTROL, &data, 1)) != 0) return ret; // Write measurement control register
    return SUCCESS;
}

int sl_get_error(uint16_t error_reg)
{
    if (error_reg == 0) return SUCCESS;
    else if (error_reg & 0x0001) return SUNLIGHT_ERROR_FATAL;
    else if (error_reg & 0x0002) return SUNLIGHT_ERROR_I2C;
    else if (error_reg & 0x0004) return SUNLIGHT_ERROR_ALGORITHM;
    else if (error_reg & 0x0008) return SUNLIGHT_ERROR_CAL;
    else if (error_reg & 0x0010) return SUNLIGHT_ERROR_SELF_DIAG;
    else if (error_reg & 0x0020) return SUNLIGHT_ERROR_OUT_OF_RANGE;
    else if (error_reg & 0x0040) return SUNLIGHT_ERROR_MEMORY;
    else if (error_reg & 0x0080) return SUNLIGHT_ERROR_DATA_READY_TIMEOUT;
    else if (error_reg & 0x0100) return SUNLIGHT_ERROR_LOW_INTERNAL_VOLTAGE;
    else if (error_reg & 0x0200) return SUNLIGHT_ERROR_MEASUREMENT_TIMEOUT;
    else if (error_reg & 0x0400) return SUNLIGHT_ERROR_ABNORMAL_SIGNAL_LEVEL;
    else return SUNLIGHT_ERROR_SENSOR_GENERAL;
}

void sunlight_get_value(sunlight_t* sunlight)
{
    int32_t ret;
    static int32_t i;

    switch (sunlight->meas_state)
    {
        case SUNLIGHT_MEAS_FINISHED: // Measurement finished
        {
            #ifdef DEBUG
            msg("Meas finished");
            #endif
            sunlight_power(sunlight, false); // Power off
            sunlight->wake_time = make_timeout_time_ms(INT32_MAX); // Disable timer
            return;
        }
        case SUNLIGHT_MEAS_START: // Measurement start
        {
            #ifdef DEBUG
            msg("Meas start");
            #endif
            sunlight_power(sunlight, true); // Power on
            sunlight->wake_time = make_timeout_time_ms(100); // Timer 100 ms - power stabilization
            sunlight->meas_state = SUNLIGHT_READ_MODE; // Next step - read mode
            i = 0; // Initialize iterator value
            return;
        }
        case SUNLIGHT_READ_MODE: // Reading mode
        {
            #ifdef DEBUG
            msg("Read mode");
            #endif
            uint8_t data;
            ret = sunlight_read(REG_MEAS_MODE, &data, 1); // Reading measurement mode
            if (ret != 0) // On invalid read
            {
                sunlight->meas_state = SUNLIGHT_MEAS_FINISHED; // Measurement finished
                sunlight->co2 = INT16_MAX; // Set CO2 to unknown
                sunlight->temperature = NAN; // Set temperature to unknown
                sunlight->state = ret; // Output return state
                return;
            }
            if (!((sunlight->config->single_meas_mode && data == 0x01) || (!sunlight->config->single_meas_mode && data == 0x00))) // If wrong mode set
            {
                sunlight->meas_state = SUNLIGHT_MEAS_FINISHED; // Measurement finished
                sunlight->co2 = INT16_MAX; // Set CO2 to unknown
                sunlight->temperature = NAN; // Set temperature to unknown
                sunlight->state = SUNLIGHT_ERROR_WRONG_MODE; // Output Wrong mode error state
                return;
            }
            if (sunlight->config->single_meas_mode) // If in single measurement mode
            {
                sunlight->meas_state = SUNLIGHT_WRITE_MEAS_CMD; // Next step - write measurement command
                sunlight->wake_time = make_timeout_time_ms(10); // Timer 10 ms
            }
            else
            {
                sunlight->meas_state = SUNLIGHT_READ_VALUE; // Next step - read measurement data
                sunlight->wake_time = make_timeout_time_ms(300); // Timer 300 ms
            }
            return;
        }
        case SUNLIGHT_WRITE_MEAS_CMD: // Writing measurement command
        {
            #ifdef DEBUG
            msg("Write measure command");
            #endif
            uint8_t buf[24] = {0}; 
            if (memcmp(sunlight->state_reg, buf, 24)) // If last state was zeros (not set)
            {
                uint8_t data = 0x01; // Prepare data
                ret = sunlight_write(REG_START_SINGLE_MEAS_MIR, &data, 1); // Set measurement trigger
            }
            else // Last state exists
            {
                uint8_t data[25];
                data[0] = 0x01; // Prepare data
                memcpy(&data[1], sunlight->state_reg, 24);
                ret = sunlight_write(REG_START_SINGLE_MEAS_MIR, data, 25); // Set measurement trigger
            }
            if (ret != 0) // On invalid write
            {
                sunlight->meas_state = SUNLIGHT_MEAS_FINISHED; // Measurement finished
                sunlight->co2 = INT16_MAX; // Set CO2 to unknown
                sunlight->temperature = NAN; // Set temperature to unknown
                sunlight->state = ret; // Output return state
                return;
            }
            sunlight->wake_time = make_timeout_time_ms(300); // Set timer 300 ms
            sunlight->meas_state = SUNLIGHT_READ_VALUE; // Next step - read measurement data
            return;
        }
        case SUNLIGHT_READ_VALUE: // Reading measurement data
        {
            #ifdef DEBUG
            msg("Read value");
            #endif
            uint8_t buf[10] = {0};
            ret = sunlight_read(REG_ERR_H, buf, 10); // Read data
            if (ret != 0) // On invalid read
            {
                sunlight->meas_state = SUNLIGHT_MEAS_FINISHED; // Measurement finished
                sunlight->co2 = INT16_MAX; // Set CO2 to unknown
                sunlight->temperature = NAN; // Set temperature to unknown
                sunlight->state = ret; // Output return state
                return;
            }
            if (i++ > (4 * sunlight->config->meas_samples)) // If data not present for 4* the measurement time needed
            {
                sunlight->meas_state = SUNLIGHT_MEAS_FINISHED; // Measurement finished
                sunlight->co2 = INT16_MAX; // Set CO2 to unknown
                sunlight->temperature = NAN; // Set temperature to unknown
                sunlight->state = SUNLIGHT_ERROR_DATA_READY_TIMEOUT; // Output timeout error state
                return;
            }
            ret = sl_get_error(ntoh16(*((uint16_t*)&buf[0]))); // Check returned error
            if (ret != 0 && ret != SUNLIGHT_ERROR_DATA_READY_TIMEOUT) // If error & not timeout error
            {
                sunlight->meas_state = SUNLIGHT_MEAS_FINISHED; // Measurement finished
                sunlight->co2 = INT16_MAX; // Set CO2 to unknown
                sunlight->temperature = NAN; // Set temperature to unknown
                sunlight->state = ret; // Output return state
                return;
            }
            else if (ret == SUNLIGHT_ERROR_DATA_READY_TIMEOUT) // On data not ready
            {
                sunlight->wake_time = make_timeout_time_ms(300); // Try again after 300 ms
                return;
            }
            sunlight->co2 = (int16_t)(ntoh16(*((uint16_t*)&buf[6]))); // Set CO2 value
            sunlight->temperature = ntoh16(*((uint16_t*)&buf[8])) / 100.0f; // Set temperature value
            if (sunlight->config->single_meas_mode) // If single measurement mode
            {
                sunlight->wake_time = make_timeout_time_ms(10); // Timer 10 ms
                sunlight->meas_state = SUNLIGHT_READ_STATUS; // Next step - read status
            }
            else // Not in single measurement mode
            {
                sunlight->meas_state = SUNLIGHT_MEAS_FINISHED; // Measurement finished
            }
            sunlight->state = SUCCESS; // Output SUCCESS state
            return;
        }
        case SUNLIGHT_READ_STATUS: // Reading status registers
        {
            #ifdef DEBUG
            msg("Read status");
            #endif
            ret = sunlight_read(REG_ABC_TIME_MIR_H, sunlight->state_reg, 24); // Read status registers
            if (ret != 0) // On invalid read
            {
                memset(sunlight->state_reg, 0x00, 24); // Clear last state registers
                sunlight->state = ret; // Output return state
            }
            sunlight->meas_state = SUNLIGHT_MEAS_FINISHED; // Measurement finished
            return;
        }
    }
}

void sl_wake_up(void)
{
    int32_t ret = i2c_write_timeout_us(I2C_SENSOR, SUNLIGHT_ADDR, NULL, 0, false, 100); // Send empty command
}

void sunlight_init_struct(sunlight_t* sunlight)
{
    sunlight->co2 = 0; // Init CO2
    sunlight->temperature = 0; // Init temperature
    sunlight->state = ERROR_SENSOR_NOT_INITIALIZED; // Set sensor state to not initialized
    sunlight->meas_state = (sunlight_meas_state_e)0; // Set measurement state to not started
    sunlight->wake_time = make_timeout_time_ms(INT32_MAX); // Disable timer
    memset(sunlight->state_reg, 0x00, 24); // Clear saved state register
}

void sunlight_power(sunlight_t* sunlight, bool on)
{
    if (!sunlight->config->power_global_control) // If power not controlled globally
    {
        // Read power vector
        // Check if bit turned [on]
        // Write power vector
    }
}

