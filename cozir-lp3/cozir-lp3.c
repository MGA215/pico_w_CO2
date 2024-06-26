#include "cozir-lp3.h"
#include <stdio.h>

#define COZIR_LP3_ADDR 0x41

#define REG_MEAS_CONTROL 0x00
#define REG_CO2 0x02
#define REG_DIGITAL_FILTER_SETTING 0x04
#define REG_SENSOR_CONTROL_SETTING 0x05
#define REG_ABC_INIT_PERIOD 0x06
#define REG_ABC_PERIOD 0x08
#define REG_ABC_TARGET 0x0C
#define REG_CAL_TARGET_FRESH_AIR 0x12
#define REG_CAL_KNOWN_CO2 0x14
#define REG_TEMPERATURE 0x20
#define REG_RH 0x22
#define REG_SER_NUMBER 0x26
#define REG_ALARM_LEVEL 0x2C
#define REG_PWM_CONTROL 0x2E
#define REG_CO2_UNFILTERED 0x34
#define REG_ABC_CONTROL 0x4E
#define REG_SELF_TEST 0x50
#define REG_TOTAL_HOURS 0x5A
#define REG_ALTITUDE_PRESSURE 0x76
#define REG_TH_CONTROL 0x8C

#define msg(x) printf("[%u] [COZIR-LP3] %s\n", to_ms_since_boot(get_absolute_time()), x)


int32_t cozir_lp3_write(uint8_t addr, uint8_t* data, uint8_t len)
{
    if (len < 1 || len > 4) return COZIR_LP3_ERROR_NREG_REG; // Check number of written bytes is 1 .. 4
    int32_t ret;
    uint8_t command_buffer[len + 1]; // data length + command
    memset(command_buffer, 0x00, len + 1); // Clear command buffer
    command_buffer[0] = addr; // Add address to buffer
    for (int i = 0; i < len + 1; i++)
    {
        command_buffer[i + 1] = data[i]; // Add data to buffer
    }

    if ((ret = i2c_write_timeout_us(I2C_SENSOR, COZIR_LP3_ADDR, command_buffer, len + 1, false, I2C_TIMEOUT_US)) < 0) return ret; // Send command
    sleep_ms(1);
    return SUCCESS;
}

int32_t cozir_lp3_read(uint8_t addr, uint8_t* buf, uint8_t len)
{
    if (len < 1 || len > 4) return COZIR_LP3_ERROR_NREG_REG;
    int32_t ret;
    
    if ((ret = i2c_write_timeout_us(I2C_SENSOR, COZIR_LP3_ADDR, &addr, 1, true, I2C_TIMEOUT_US)) < 0) return ret;

    if ((ret = i2c_read_timeout_us(I2C_SENSOR, COZIR_LP3_ADDR, buf, len, false, I2C_TIMEOUT_US)) < 0) return ret;

    return SUCCESS;
}

void cozir_lp3_get_value(cozir_lp3_t* cozir_lp3)
{
    
}