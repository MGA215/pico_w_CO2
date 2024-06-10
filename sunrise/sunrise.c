#include "sunrise.h"

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
int get_error(uint16_t error_reg);

/**
 * @brief Wakes up the sensor
 * 
 */
void wake_up(void);

/**
 * @brief Initializes meter control register
 * 
 * @param nRDY_en enable nRDY pin (default enabled)
 * @param abc_en enable ABC calibration (default enabled)
 * @param static_iir_en enable static IIR filter (default enabled)
 * @param dyn_iir_en enable dynamic IIR filter (default enabled)
 * @param pressure_comp enable pressure compensation (default disabled)
 * @param invert_nRDY invert nRDY pin - high during measurement (default disabled)
 * @return int Return code
 */
int init_meter_control(bool nRDY_en, bool abc_en, bool static_iir_en, bool dyn_iir_en, bool pressure_comp, bool invert_nRDY);

int sunrise_write(uint8_t addr, uint8_t* buf, uint16_t len)
{
    int32_t ret;
    uint8_t command_buffer[len + 1];
    command_buffer[0] = addr;
    memcpy(&command_buffer[1], buf, len);
    wake_up();
    if ((ret = i2c_write_timeout_per_char_us(SUNRISE_I2C, SUNRISE_ADDR, command_buffer, len + 1, true, 1000)) < 0) return ret;
    busy_wait_ms(2);
    uint8_t resp[len];
    if ((ret = i2c_read_timeout_per_char_us(SUNRISE_I2C, SUNRISE_ADDR, resp, len, false, 1000)) < 0) return ret;
    if (memcmp(buf, resp, len)) return SUNRISE_ERROR_WRITE_RESP;
    return SUCCESS;
}

int sunrise_read(uint8_t addr, uint8_t* buf, uint16_t num_bytes)
{
    int32_t ret;
    wake_up();
    if ((ret = i2c_write_timeout_per_char_us(SUNRISE_I2C, SUNRISE_ADDR, &addr, 1, true, 1000)) < 0) return ret;
    
    busy_wait_ms(2);
    if ((ret = i2c_read_timeout_per_char_us(SUNRISE_I2C, SUNRISE_ADDR, buf, num_bytes, false, 1000)) < 0) return ret;
    return SUCCESS;
}

int sunrise_reset(void)
{
    int32_t ret;
    uint8_t data = 0xFF;
    if ((ret = sunrise_write(REG_SOFT_RESET, &data, 1)) != 0) return ret;
    return SUCCESS;
}

int sunrise_init(bool single_meas_mode, uint16_t meas_period, uint16_t meas_samples, uint16_t abc_period, uint16_t abc_target_value, 
    bool nRDY_en, bool abc_en, bool static_iir_en, bool dyn_iir_en, bool pressure_comp, bool invert_nRDY)
{
    uint8_t command_buffer[11];
    command_buffer[0] = (uint8_t)single_meas_mode;
    *((uint16_t*)&command_buffer[1]) = meas_period;
    *((uint16_t*)&command_buffer[3]) = meas_samples;
    *((uint16_t*)&command_buffer[5]) = abc_period;
    command_buffer[7] = 0x00;
    command_buffer[8] = 0x00;
    *((uint16_t*)&command_buffer[9]) = abc_target_value;

    uint8_t buf[11];

    int32_t ret;
    if ((ret = sunrise_read(REG_MEAS_MODE, buf, 11)) != 0) return ret;
    if (memcmp(buf, command_buffer, 11) == 0) return SUCCESS;
    if ((ret = sunrise_write(REG_MEAS_MODE, command_buffer, 11)) != 0) return ret;

    if ((ret = init_meter_control(nRDY_en, abc_en, static_iir_en, dyn_iir_en, pressure_comp, invert_nRDY)) != 0) return ret;
    return SUCCESS;
}

int init_meter_control(bool nRDY_en, bool abc_en, bool static_iir_en, bool dyn_iir_en, bool pressure_comp, bool invert_nRDY)
{
    uint8_t data = {0};
    data |= nRDY_en;
    data |= abc_en << 1;
    data |= static_iir_en << 2;
    data |= dyn_iir_en << 3;
    data |= pressure_comp << 4;
    data |= invert_nRDY << 5;

    uint8_t buf;
    int32_t ret;
    if ((ret = sunrise_read(REG_METER_CONTROL, &buf, 1)) != 0) return ret;
    if (data == buf) return SUCCESS;

    if ((ret = sunrise_write(REG_METER_CONTROL, &data, 1)) != 0) return ret;
    return SUCCESS;
}

int get_error(uint16_t error_reg)
{
    if (error_reg & 0x0001) return SUNRISE_ERROR_FATAL;
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

int sunrise_read_value(int* co2, float* temperature)
{
    uint8_t buf[10];
    int32_t ret, i;

    i = 0;
    while (true)
    {
        busy_wait_ms(300);
        if ((ret = sunrise_read(REG_ERR_H, buf, 10)) != 0) return ret;
        if (i > 64) return SUNRISE_ERROR_DATA_READY_TIMEOUT;
        ret = get_error(*((uint16_t*)&buf[0]));
        if (ret == 0) break;
        else if (ret == SUNRISE_ERROR_DATA_READY_TIMEOUT) i++;
        else return ret;
    }

    if ((buf[0] & 0x07) || buf[1])
    {
        return get_error(*((uint16_t*)&buf[0]));
    }
    *co2 = (int)(*((uint16_t*)&buf[6]));
    *temperature = *((uint16_t*)&buf[8]) / 100.0f;

}

int sunrise_single_meas(int* co2, float* temperature, uint8_t* state_reg, uint8_t state_reg_len_bytes, bool no_state)
{
    if (SUNRISE_EN != INT32_MAX) gpio_put(SUNRISE_EN, 1);
    int32_t ret;
    uint8_t buf[1 + state_reg_len_bytes];
    if (state_reg_len_bytes != 24) return SUNRISE_ERROR_WRONG_STATE;
    uint8_t data;
    if ((ret = sunrise_read(REG_MEAS_MODE, &data, 1)) != 0) return ret;
    if (data != 0x01) return SUNRISE_ERROR_NO_SINGLE_MEAS_MODE;
    if (no_state)
    {
        uint8_t data = 0x01;
        sunrise_write(REG_START_SINGLE_MEAS_MIR, &data, 1);
    }
    else
    {
        uint8_t buf[state_reg_len_bytes + 1];
        buf[0] = 0x01;
        memcpy(&buf[1], state_reg, state_reg_len_bytes);
        sunrise_write(REG_START_SINGLE_MEAS_MIR, buf, state_reg_len_bytes + 1);
    }
    busy_wait_ms(2);
    if ((ret = sunrise_read_value(co2, temperature)) != 0)
    {
        if (SUNRISE_EN != INT32_MAX) gpio_put(SUNRISE_EN, 0);
        return ret;
    }
    if ((ret = sunrise_read(REG_ABC_TIME_MIR_H, state_reg, 24))) 
    if (SUNRISE_EN != INT32_MAX) gpio_put(SUNRISE_EN, 0);
    return SUCCESS;
}

void wake_up(void)
{
    i2c_write_timeout_per_char_us(SUNRISE_I2C, SUNRISE_ADDR, NULL, 0, false, 100);
}


