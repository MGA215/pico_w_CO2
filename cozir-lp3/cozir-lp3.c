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

#define msg(severity, x) printf("[%12llu] ["severity"] [COZIR-LP3] "x"\n", to_us_since_boot(get_absolute_time()) / 1000)


/**
 * @brief Writes configuration to the sensor
 * 
 * @param config Configuration to write
 * @return int32_t Return code
 */
int32_t cozir_lp3_write_config(sensor_config_t* config);

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
    if (len < 1 || len > 4) return COZIR_LP3_ERROR_NREG_REG; // Check number of read bytes is 1 .. 4
    int32_t ret;
    
    if ((ret = i2c_write_timeout_us(I2C_SENSOR, COZIR_LP3_ADDR, &addr, 1, true, I2C_TIMEOUT_US)) < 0) return ret; // Send command

    if ((ret = i2c_read_timeout_us(I2C_SENSOR, COZIR_LP3_ADDR, buf, len, false, I2C_TIMEOUT_US)) < 0) return ret; // Read data

    return SUCCESS;
}

void cozir_lp3_get_value(sensor_t* cozir_lp3)
{
    int32_t ret;
    uint8_t tempBuffer[5];
    switch(cozir_lp3->meas_state)
    {
        case MEAS_FINISHED:
        {
            #ifdef DEBUG
            msg("info", "Meas finished");
            #endif
            cozir_lp3_power(cozir_lp3, false); // Power off
            cozir_lp3->wake_time = at_the_end_of_time; // Disable timer
            return;
        }
        case MEAS_STARTED:
        {
            #ifdef DEBUG
            msg("info", "Meas started");
            #endif
            cozir_lp3_power(cozir_lp3, true);
            cozir_lp3->wake_time = make_timeout_time_ms(2000);
            cozir_lp3->meas_state = MEAS_READ_VALUE;
            return;
        }
        case MEAS_READ_VALUE:
        {
            #ifdef DEBUG
            msg("info", "Read value");
            #endif
            ret = cozir_lp3_read(REG_CO2, tempBuffer, 2);
            if (ret != 0)
            {
                cozir_lp3->co2 = NAN;
                cozir_lp3->humidity = NAN;
                cozir_lp3->temperature = NAN;
                cozir_lp3->meas_state = MEAS_FINISHED;
                cozir_lp3->state = ret;
                return;
            }
            cozir_lp3->co2 = (float)ntoh16(*((uint16_t*)&tempBuffer[0]));

            cozir_lp3->meas_state = MEAS_FINISHED;
            cozir_lp3->state = SUCCESS;
            return;
        }
    }
}

int32_t cozir_lp3_init(sensor_t* cozir_lp3, sensor_config_t* config)
{
    int32_t ret;
    cozir_lp3->config = config;
    cozir_lp3_power(cozir_lp3, true);

    ret = cozir_lp3_write_config(config);
    cozir_lp3_power(cozir_lp3, false);
    return ret;
}

int32_t cozir_lp3_write_config(sensor_config_t* config)
{
    int32_t ret;
    uint8_t buf[2];
    sensor_config_t read_config;
    cozir_lp3_read_config(&read_config);
    
    if (read_config.enable_pressure_comp != config->enable_pressure_comp || read_config.pressure != config->pressure)
    {
        if (config->enable_pressure_comp && read_config.pressure != config->pressure)
        {
            *((uint16_t*)&buf[0]) = ntoh16(config->pressure);
            if ((ret = cozir_lp3_write(REG_ALTITUDE_PRESSURE, buf, 2)) != 0) return ret;
        }
        else if (!config->enable_pressure_comp && read_config.pressure != 1013)
        {
            *((uint16_t*)&buf[0]) = ntoh16(1013);
            if ((ret = cozir_lp3_write(REG_ALTITUDE_PRESSURE, buf, 2)) != 0) return ret;
        }
    }

    if (read_config.enable_PWM_pin != config->enable_PWM_pin)
    {
        buf[0] = config->enable_PWM_pin ? 0b1 << 7 : 0;
        if ((ret = cozir_lp3_write(REG_PWM_CONTROL, &buf[0], 1)) != 0) return ret;
    }

    if (read_config.enable_abc != config->enable_abc)
    {
        buf[0] = config->enable_abc ? 0b10 : 0b00;
        if ((ret = cozir_lp3_write(REG_ABC_CONTROL, &buf[0], 1)) != 0) return ret;
    }
    if (config->enable_abc)
    {
        if (read_config.abc_init_period != config->abc_init_period)
        {
            *((uint16_t*)&buf[0]) = ntoh16(config->abc_init_period);
            if ((ret = cozir_lp3_write(REG_ABC_INIT_PERIOD, buf, 2)) != 0) return ret;
        }
        if (read_config.abc_period != config->abc_period)
        {
            *((uint16_t*)&buf[0]) = ntoh16(config->abc_period);
            if ((ret = cozir_lp3_write(REG_ABC_PERIOD, buf, 2)) != 0) return ret;
        }
        if (read_config.abc_target_value != config->abc_target_value)
        {
            *((uint16_t*)&buf[0]) = ntoh16(config->abc_target_value);
            if ((ret = cozir_lp3_write(REG_ABC_TARGET, buf, 2)) != 0) return ret;
        }
    }
    if (read_config.alarm_treshold_co2_high != config->alarm_treshold_co2_high)
    {
        if (config->alarm_en)
        {
            *((uint16_t*)&buf[0]) = ntoh16(config->alarm_treshold_co2_high);
            if ((ret = cozir_lp3_write(REG_ALARM_LEVEL, buf, 2)) != 0) return ret;
        }
        else
        {
            *((uint16_t*)&buf[0]) = ntoh16(0);
            if ((ret = cozir_lp3_write(REG_ALARM_LEVEL, buf, 2)) != 0) return ret;
        }
    }
    return SUCCESS;
}

int32_t cozir_lp3_read_config(sensor_config_t* config)
{
    int32_t ret;
    uint8_t buf[2];
    if ((ret = cozir_lp3_read(REG_ALTITUDE_PRESSURE, buf, 2)) != 0) return ret;
    config->pressure = ntoh16(*((uint16_t*)(&buf[0])));
    config->enable_pressure_comp = config->pressure != 1013;

    if ((ret = cozir_lp3_read(REG_PWM_CONTROL, &buf[0], 1)) != 0) return ret;
    config->enable_PWM_pin = ((buf[0] >> 7) & 0x1);

    if ((ret = cozir_lp3_read(REG_ABC_CONTROL, &buf[0], 1)) != 0) return ret;
    config->enable_abc = (buf[0] >> 1) & 0x1;

    if ((ret = cozir_lp3_read(REG_ABC_INIT_PERIOD, buf, 2)) != 0) return ret;
    config->abc_init_period = ntoh16(*((uint16_t*)(&buf[0])));

    if ((ret = cozir_lp3_read(REG_ABC_PERIOD, buf, 2)) != 0) return ret;
    config->abc_period = ntoh16(*((uint16_t*)(&buf[0])));

    if ((ret = cozir_lp3_read(REG_ABC_TARGET, buf, 2)) != 0) return ret;
    config->abc_target_value = ntoh16(*((uint16_t*)(&buf[0])));

    if ((ret = cozir_lp3_read(REG_ALARM_LEVEL, buf, 2)) != 0) return ret;
    config->alarm_treshold_co2_high = ntoh16(*((uint16_t*)(&buf[0])));
    config->alarm_en = config->alarm_treshold_co2_high != 0;
    
    return SUCCESS;
}

void cozir_lp3_power(sensor_t* cozir_lp3, bool on)
{
    if (!cozir_lp3->config->power_global_control) // If power not controlled globally
    {
        // Read power vector
        // Check if bit turned [on]
        // Write power vector
    }
}