#include "sunrise.h"
#include "stdio.h"

#define SUNRISE_ADDR 0x68
#define SUNRISE_I2C i2c0

#define SUNRISE_EN INT32_MAX

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

/**
 * @brief returns error code according to the error register value
 * 
 * @param error_reg error register
 * @return int returned error code
 */
int sr_get_error(uint16_t error_reg);

/**
 * @brief Wakes up the sensor
 * 
 */
void sr_wake_up(void);

/**
 * @brief Writes configuration to the sensor
 * 
 * @param config COnfiguration to be written
 * @return int Return code
 */
int sr_write_config(sunrise_config_t* config);

/**
 * @brief Writes config for the meter control register
 * 
 * @param config Config for the SUNRISE sensor
 * @return int Return code
 */
int sr_write_meter_control(sunrise_config_t* config);

int sunrise_write(uint8_t addr, uint8_t* buf, uint16_t len)
{
    int32_t ret;
    uint8_t command_buffer[len + 1];
    command_buffer[0] = addr;
    memcpy(&command_buffer[1], buf, len);
    sr_wake_up();
    if ((ret = i2c_write_timeout_per_char_us(SUNRISE_I2C, SUNRISE_ADDR, command_buffer, len + 1, false, 1000)) < 0) return ret;
    busy_wait_ms(12);
    // uint8_t resp[len];
    // if ((ret = i2c_read_timeout_per_char_us(SUNRISE_I2C, SUNRISE_ADDR, resp, len, false, 1000)) < 0) return ret;
    // if (memcmp(buf, resp, len)) return SUNRISE_ERROR_WRITE_RESP;
    return SUCCESS;
}

int sunrise_read(uint8_t addr, uint8_t* buf, uint16_t num_bytes)
{
    int32_t ret;
    sr_wake_up();
    if ((ret = i2c_write_timeout_per_char_us(SUNRISE_I2C, SUNRISE_ADDR, &addr, 1, true, 1000)) < 0) return ret;
    
    busy_wait_us(1000);
    if ((ret = i2c_read_timeout_per_char_us(SUNRISE_I2C, SUNRISE_ADDR, buf, num_bytes, false, 1000)) < 0) return ret;
    busy_wait_ms(1);
    return SUCCESS;
}

int sunrise_reset(void)
{
    int32_t ret;
    uint8_t data = 0xFF;
    if ((ret = sunrise_write(REG_SOFT_RESET, &data, 1)) != 0) return ret;
    return SUCCESS;
}

int sunrise_init(sunrise_t* sunrise, sunrise_config_t* config)
{
    int32_t ret;
    
    sunrise_init_struct(sunrise);

    uint8_t buf[16];
    if ((ret = sunrise_read(REG_PRODUCT_CODE, buf, 16)) != 0) return ret; // Check sensor product code
    if (strcmp(buf, "006-0-0008") != 0) return ERROR_UNKNOWN_SENSOR;

    if ((ret = sr_write_config(config)) != 0) return ret;

    sunrise->config = config;

    sunrise_reset();
    return SUCCESS;
}

int sr_write_config(sunrise_config_t* config)
{
    int32_t ret;
    uint8_t buf[11] = {0};
    uint8_t command_buf[11] = {0};
    command_buf[0] = (uint8_t)config->single_meas_mode;
    *((uint16_t*)&command_buf[1]) = ntoh16(config->meas_period);
    *((uint16_t*)&command_buf[3]) = ntoh16(config->meas_samples);
    *((uint16_t*)&command_buf[5]) = ntoh16(config->abc_period);

    *((uint16_t*)&command_buf[9]) = ntoh16(config->abc_target_value);

    if ((ret = sr_write_meter_control(config)) != 0) return ret;

    if ((ret = sunrise_read(REG_MEAS_MODE, buf, 11)) != 0) return ret;
    if (memcmp(buf, command_buf, 11) == 0) return SUCCESS;
    if ((ret = sunrise_write(REG_MEAS_MODE, command_buf, 11)) != 0) return ret;
    return SUCCESS;
}

int sunrise_read_config(sunrise_config_t* config)
{
    int32_t ret;
    uint8_t buf[11] = {0};
    uint8_t data = 0;
    if ((ret = sunrise_read(REG_MEAS_MODE, buf, 11)) != 0) return ret;
    config->single_meas_mode = (bool)buf[0];
    config->meas_period = ntoh16(*((uint16_t*)&buf[1]));
    config->meas_samples = ntoh16(*((uint16_t*)&buf[3]));
    config->abc_period = ntoh16(*((uint16_t*)&buf[5]));
    config->abc_target_value = ntoh16(*((uint16_t*)&buf[9]));

    if ((ret = sunrise_read(REG_METER_CONTROL, &data, 1)) != 0) return ret;
    config->enable_nRDY = data & 0b1 << 0;
    config->enable_ABC = data & 0b1 << 1;
    config->enable_static_IIR = data & 0b1 << 2;
    config->enable_dynamic_IIR = data & 0b1 << 3;
    config->enable_pressure_comp = data & 0b1 << 4;
    config->invert_nRDY = data & 0b1 << 5;

    return SUCCESS;
}

int sr_write_meter_control(sunrise_config_t* config)
{
    uint8_t data = 0;
    data |= config->enable_nRDY;
    data |= config->enable_ABC << 1;
    data |= config->enable_static_IIR << 2;
    data |= config->enable_dynamic_IIR << 3;
    data |= config->enable_pressure_comp << 4;
    data |= config->invert_nRDY << 5;

    uint8_t buf;
    int32_t ret;
    if ((ret = sunrise_read(REG_METER_CONTROL, &buf, 1)) != 0) return ret;
    if (data == buf) return SUCCESS;

    if ((ret = sunrise_write(REG_METER_CONTROL, &data, 1)) != 0) return ret;
    return SUCCESS;
}

int sr_get_error(uint16_t error_reg)
{
    if (error_reg == 0) return SUCCESS;
    else if (error_reg & 0x0001) return SUNRISE_ERROR_FATAL;
    else if (error_reg & 0x0002) return SUNRISE_ERROR_I2C;
    else if (error_reg & 0x0004) return SUNRISE_ERROR_ALGORITHM;
    else if (error_reg & 0x0008) return SUNRISE_ERROR_CAL;
    else if (error_reg & 0x0010) return SUNRISE_ERROR_SELF_DIAG;
    else if (error_reg & 0x0020) return SUNRISE_ERROR_OUT_OF_RANGE;
    else if (error_reg & 0x0040) return SUNRISE_ERROR_MEMORY;
    else if (error_reg & 0x0080) return SUNRISE_ERROR_DATA_READY_TIMEOUT;
    else if (error_reg & 0x0100) return SUNRISE_ERROR_LOW_INTERNAL_VOLTAGE;
    else if (error_reg & 0x0200) return SUNRISE_ERROR_MEASUREMENT_TIMEOUT;
    else if (error_reg & 0x0400) return SUNRISE_ERROR_ABNORMAL_SIGNAL_LEVEL;
    else return SUNRISE_ERROR_SENSOR_GENERAL;
}

void sunrise_get_value(sunrise_t* sunrise)
{
    int32_t ret;
    static int32_t i;

    switch (sunrise->meas_state)
    {
        case SUNRISE_MEAS_FINISHED:
        {
            if (!sunrise->config->power_global_control)
            {
                // Read power vector
                // Check if bit turned off
                // Write power vector
            }
            sunrise->wake_time = make_timeout_time_ms(INT32_MAX);
            return;
        }
        case SUNRISE_MEAS_START:
        {
            if (!sunrise->config->power_global_control)
            {
                // Read power vector - possibly not needed
                // Check if bit turned on
                // Write power vector
            }
            sunrise->wake_time = make_timeout_time_ms(100);
            sunrise->meas_state = SUNRISE_READ_MODE;
            i = 0;
            return;
        }
        case SUNRISE_READ_MODE:
        {
            uint8_t data;
            ret = sunrise_read(REG_MEAS_MODE, &data, 1);
            if (ret != 0)
            {
                sunrise->meas_state = SUNRISE_MEAS_FINISHED;
                sunrise->co2 = INT16_MAX;
                sunrise->temperature = NAN;
                sunrise->state = ret;
                return;
            }
            if (!((sunrise->config->single_meas_mode && data == 0x01) || (!sunrise->config->single_meas_mode && data == 0x00)))
            {
                sunrise->meas_state = SUNRISE_MEAS_FINISHED;
                sunrise->co2 = INT16_MAX;
                sunrise->temperature = NAN;
                sunrise->state = SUNRISE_ERROR_WRONG_MODE;
                return;
            }
            if (sunrise->config->single_meas_mode)
            {
                sunrise->meas_state = SUNRISE_WRITE_MEAS_CMD;
                sunrise->wake_time = make_timeout_time_ms(10);
            }
            else
            {
                sunrise->meas_state = SUNRISE_READ_VALUE;
                sunrise->wake_time = make_timeout_time_ms(300);
            }
            return;
        }
        case SUNRISE_WRITE_MEAS_CMD:
        {
            uint8_t buf[24] = {0};
            if (memcmp(sunrise->state_reg, buf, 24))
            {
                uint8_t data = 0x01;
                ret = sunrise_write(REG_START_SINGLE_MEAS_MIR, &data, 1);
            }
            else
            {
                uint8_t data[25];
                data[0] = 0x01;
                memcpy(&data[1], sunrise->state_reg, 24);
                ret = sunrise_write(REG_START_SINGLE_MEAS_MIR, data, 25);
            }
            if (ret != 0)
            {
                sunrise->meas_state = SUNRISE_MEAS_FINISHED;
                sunrise->co2 = INT16_MAX;
                sunrise->temperature = NAN;
                sunrise->state = ret;
                return;
            }
            sunrise->wake_time = make_timeout_time_ms(300);
            sunrise->meas_state = SUNRISE_READ_VALUE;
            return;
        }
        case SUNRISE_READ_VALUE:
        {
            uint8_t buf[10] = {0};
            ret = sunrise_read(REG_ERR_H, buf, 10);
            if (ret != 0)
            {
                sunrise->meas_state = SUNRISE_MEAS_FINISHED;
                sunrise->co2 = INT16_MAX;
                sunrise->temperature = NAN;
                sunrise->state = ret;
                return;
            }
            if (i++ > (sunrise->config->meas_samples))
            {
                sunrise->meas_state = SUNRISE_MEAS_FINISHED;
                sunrise->co2 = INT16_MAX;
                sunrise->temperature = NAN;
                sunrise->state = SUNRISE_ERROR_DATA_READY_TIMEOUT;
                return;
            }
            ret = sr_get_error(ntoh16(*((uint16_t*)&buf[0])));
            if (ret != 0 && ret != SUNRISE_ERROR_DATA_READY_TIMEOUT)
            {
                sunrise->meas_state = SUNRISE_MEAS_FINISHED;
                sunrise->co2 = INT16_MAX;
                sunrise->temperature = NAN;
                sunrise->state = ret;
                return;
            }
            else if (ret == SUNRISE_ERROR_DATA_READY_TIMEOUT)
            {
                sunrise->wake_time = make_timeout_time_ms(300);
                return;
            }
            sunrise->co2 = (int16_t)(ntoh16(*((uint16_t*)&buf[6])));
            sunrise->temperature = ntoh16(*((uint16_t*)&buf[8])) / 100.0f;
            if (sunrise->config->single_meas_mode)
            {
                sunrise->wake_time = make_timeout_time_ms(10);
                sunrise->meas_state = SUNRISE_READ_STATUS;
            }
            else 
            {
                sunrise->meas_state = SUNRISE_MEAS_FINISHED;
            }
            sunrise->state = SUCCESS;
            return;
        }
        case SUNRISE_READ_STATUS:
        {
            ret = sunrise_read(REG_ABC_TIME_MIR_H, sunrise->state_reg, 24);
            if (ret != 0)
            {
                memset(sunrise->state_reg, 0x00, 24);
                sunrise->state = ret;
            }
            sunrise->meas_state = SUNRISE_MEAS_FINISHED;
            return;
        }
    }
}

void sr_wake_up(void)
{
    int32_t ret = i2c_write_timeout_per_char_us(SUNRISE_I2C, SUNRISE_ADDR, NULL, 0, false, 100);
}

void sunrise_init_struct(sunrise_t* sunrise)
{
    sunrise->co2 = 0;
    sunrise->temperature = 0;
    sunrise->state = ERROR_SENSOR_NOT_INITIALIZED;
    sunrise->meas_state = (sunrise_meas_state_e)0;
    sunrise->wake_time = make_timeout_time_ms(INT32_MAX);
    memset(sunrise->state_reg, 0x00, 24);
}
