#include "cdm7162.h"

#define CDM7162_ADDR 0x68
#define CDM7162_I2C i2c0

#define REG_RST 0x00
#define REG_OP_MODE 0x01
#define REG_STATUS 0x02
#define REG_CO2_L 0x03
#define REG_CO2_H 0x04
#define REG_ATM_PRESSURE 0x09
#define REG_ALTITUDE 0x0A
#define REG_ALARM_CO2_H 0x0C
#define REG_ALARM_CO2_L 0x0D
#define REG_CAL 0x0E
#define REG_FUNC 0x0F
#define REG_ERROR 0x10
#define REG_CO2_CAL 0x12
#define REG_LTA_TARGET 0x13
#define REG_LTA_PERIOD 0x14

#define CO2_MIN_RANGE 0
#define CO2_MAX_RANGE 10000

int32_t cdm7162_read(uint8_t addr, uint8_t* buf, uint8_t num_bytes)
{
    int32_t ret;
    if ((ret = i2c_write_timeout_per_char_us(CDM7162_I2C, CDM7162_ADDR, &addr, 1, true, 1000)) < 0) return ret;
    
    busy_wait_ms(2);
    if ((ret = i2c_read_timeout_per_char_us(CDM7162_I2C, CDM7162_ADDR, buf, num_bytes, false, 1000)) < 0) return ret;
    return SUCCESS;
}

int32_t cdm7162_write(uint8_t addr, uint8_t value)
{
    int32_t ret;
    uint8_t buf[2] = {addr, value};
    if ((ret = i2c_write_timeout_per_char_us(CDM7162_I2C, CDM7162_ADDR, buf, 2, true, 1000)) < 0) return ret;
    busy_wait_ms(2);
    if ((ret = i2c_read_timeout_per_char_us(CDM7162_I2C, CDM7162_ADDR, &buf[0], 1, false, 1000)) < 0) return ret;
    if (buf[0] != value) return CDM7162_ERROR_WRITE_RESP;
    return SUCCESS;
}

int32_t cdm7162_reset(void)
{
    int32_t ret;
    if ((ret = cdm7162_write(REG_RST, 0x01)) != 0) return ret;
    busy_wait_ms(100); // might need to be adjusted
    return SUCCESS;
}

int32_t cdm7162_init(bool pressure_corr)
{
    int32_t ret;
    uint8_t buf;
    if ((ret = cdm7162_read(REG_OP_MODE, &buf, 1)) != 0) return ret;
    busy_wait_ms(100);
    if (buf != 0x06)
    {
        if ((ret = cdm7162_write(REG_OP_MODE, 0x06)) != 0) return ret;
        busy_wait_ms(100);
    }

    if ((ret = cdm7162_read(REG_FUNC, &buf, 1)) != 0) return ret;
    busy_wait_ms(100);
    if (((buf & 0b100) >> 2) == pressure_corr) return SUCCESS;

    uint8_t func_settings = 0;
    if (pressure_corr) func_settings |= (0b1 << 2);
    if ((ret = cdm7162_write(REG_FUNC, func_settings)) != 0) return ret;
    busy_wait_ms(100);
    return SUCCESS;
}

int32_t cdm7162_deinit(void)
{
    int32_t ret;
    uint8_t op_mode;
    if ((ret = cdm7162_read(REG_OP_MODE, &op_mode, 1)) != 0) return ret;

    if (op_mode == 0) return SUCCESS;

    if ((ret = cdm7162_write(REG_OP_MODE, 0x00)) != 0) return ret;
    busy_wait_ms(100);
    return SUCCESS;
}

int32_t cdm7162_set_atm_pressure(uint16_t pressure)
{
    int32_t ret;
    if (pressure < 800 || pressure > 1055) return CDM7162_ERROR_RANGE;
    uint8_t hpa = pressure - 800;

    uint8_t hpa_reg;
    if ((ret = cdm7162_read(REG_ATM_PRESSURE, &hpa_reg, 1)) != 0) return ret;
    if (hpa == hpa_reg) return SUCCESS;

    ret = cdm7162_write(REG_ATM_PRESSURE, hpa);
    busy_wait_ms(100);
    return ret;
}

int32_t cdm7162_set_default_atm_pressure(void)
{
    return cdm7162_set_atm_pressure(1013);
}

void cdm7162_get_value(cdm7162_t* cdm7162)
{
    int32_t i, ret;
    uint8_t buf[2];
    switch (cdm7162->meas_state)
    {
        case CDM7162_MEAS_FINISHED: // Measurement finished
        {
            // Power off
            cdm7162->wake_time = make_timeout_time_ms(INT32_MAX); // Disable sensor timer
            return;
        }
        case CDM7162_MEAS_START: // Measurement started
        {
            // Power on
            cdm7162->wake_time = make_timeout_time_ms(750); // can be modified
            cdm7162->meas_state = CDM7162_READ_STATUS; // Next step - read status
            i = 0; // Initialize read status timeout iterator
            return;
        }
        case CDM7162_READ_STATUS:
        {
            ret = cdm7162_read(REG_STATUS, &buf[0], 1);
            if (ret != 0)
            {
                cdm7162->co2 = INT16_MAX;
                cdm7162->state = ret;
                cdm7162->meas_state = CDM7162_MEAS_FINISHED;
                return;
            }    
            if ((buf[0] & (0b1 << 7)) == 0)
            {
                cdm7162->meas_state = CDM7162_READ_VALUE;
                return;
            }
            if (i++ > 20)
            {
                cdm7162->co2 = INT16_MAX;
                cdm7162->meas_state = CDM7162_MEAS_FINISHED;
                cdm7162->state = CDM7162_ERROR_DATA_READY_TIMEOUT;
                return;
            }
            cdm7162->wake_time = make_timeout_time_ms(25);
            return;
        }
        case CDM7162_READ_VALUE:
        {
            ret = cdm7162_read(REG_CO2_L, buf, 2);
            if (ret != 0)
            {
                cdm7162->co2 = INT16_MAX;
                cdm7162->meas_state = CDM7162_MEAS_FINISHED;
                cdm7162->state = ret;
                return;
            }
            uint16_t val = *((uint16_t*)&buf[0]);
            if (val < CO2_MIN_RANGE || val > CO2_MAX_RANGE)
            {
                cdm7162->co2 = INT16_MAX;
                cdm7162->meas_state = CDM7162_MEAS_FINISHED;
                cdm7162->state = CDM7162_ERROR_RANGE;
                return;
            }
            cdm7162->co2 = val;
            cdm7162->state = SUCCESS;
            cdm7162->meas_state = CDM7162_MEAS_FINISHED;
            return;
        }
    }



    // int32_t ret, i;
    // uint8_t buf[2];
    // i = 0;
    // while (true)
    // {
    //     if ((ret = cdm7162_read(REG_STATUS, &buf[0], 1)) != 0)
    //     {
    //         *co2 = -1;
    //         return ret;
    //     }    
    //     if ((buf[0] & (0b1 << 7)) == 0) break;
    //     if (i > 20)
    //     {
    //         *co2 = -1;
    //         return CDM7162_ERROR_DATA_READY_TIMEOUT;
    //     }
    //     busy_wait_ms(25);
    //     i++;
    // }
    // busy_wait_ms(100);
    // ret = cdm7162_read(REG_CO2_L, buf, 2);
    // if (ret != 0)
    // {
    //     *co2 = -1;
    //     return ret;
    // }
    // uint16_t val = *((uint16_t*)&buf[0]);
    // if (val < CO2_MIN_RANGE || val > CO2_MAX_RANGE)
    // {
    //     *co2 = -1;
    //     return CDM7162_ERROR_RANGE;
    // }
    // *co2 = val;
    // return SUCCESS;
}