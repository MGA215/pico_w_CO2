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


/**
 * @brief Writes configuration to the sensor
 * 
 * @param config Configuration to write
 * @return int32_t Return code
 */
int32_t cozir_lp3_write_config(cozir_lp3_config_t* config);

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

void cozir_lp3_get_value(cozir_lp3_t* cozir_lp3)
{
    int32_t ret;
    uint8_t tempBuffer[5];
    switch(cozir_lp3->meas_state)
    {
        case COZIR_LP3_MEAS_FINISHED:
        {
            #ifdef DEBUG
            msg("Meas finished");
            #endif
            cozir_lp3_power(cozir_lp3, false); // Power off
            cozir_lp3->wake_time = make_timeout_time_ms(UINT32_MAX); // Disable timer
            return;
        }
        case COZIR_LP3_MEAS_START:
        {
            #ifdef DEBUG
            msg("Meas started");
            #endif
            cozir_lp3_power(cozir_lp3, true);
            cozir_lp3->wake_time = make_timeout_time_ms(2000);
            cozir_lp3->meas_state = COZIR_LP3_READ_VALUE;
            return;
        }
        case COZIR_LP3_READ_VALUE:
        {
            #ifdef DEBUG
            msg("Read value");
            #endif
            ret = cozir_lp3_read(REG_CO2, tempBuffer, 2);
            if (ret != 0)
            {
                cozir_lp3->co2 = NAN;
                cozir_lp3->humidity = NAN;
                cozir_lp3->temperature = NAN;
                cozir_lp3->meas_state = COZIR_LP3_MEAS_FINISHED;
                cozir_lp3->state = ret;
                return;
            }
            cozir_lp3->co2 = (float)ntoh16(*((uint16_t*)&tempBuffer[0]));

            // ret = cozir_lp3_read(REG_RH, tempBuffer, 2);
            // if (ret != 0)
            // {
            //     cozir_lp3->humidity = NAN;
            //     cozir_lp3->temperature = NAN;
            //     cozir_lp3->meas_state = COZIR_LP3_MEAS_FINISHED;
            //     cozir_lp3->state = ret;
            //     return;
            // }
            // cozir_lp3->humidity = (float)ntoh16(*((uint16_t*)&tempBuffer[0]));

            // ret = cozir_lp3_read(REG_TEMPERATURE, tempBuffer, 2);
            // if (ret != 0)
            // {
            //     cozir_lp3->temperature = NAN;
            //     cozir_lp3->meas_state = COZIR_LP3_MEAS_FINISHED;
            //     cozir_lp3->state = ret;
            //     return;
            // }
            // cozir_lp3->temperature = (float)ntoh16(*((uint16_t*)&tempBuffer[0]));
            
            cozir_lp3->meas_state = COZIR_LP3_MEAS_FINISHED;
            cozir_lp3->state = SUCCESS;
            return;
        }
    }
}

int32_t cozir_lp3_init(cozir_lp3_t* cozir_lp3, cozir_lp3_config_t* config)
{
    int32_t ret;
    cozir_lp3->config = config;
    cozir_lp3_power(cozir_lp3, true);

    cozir_lp3_init_struct(cozir_lp3);

    ret = cozir_lp3_write_config(config);
    cozir_lp3_power(cozir_lp3, false);
    return ret;
}

void cozir_lp3_init_struct(cozir_lp3_t* cozir_lp3)
{
    cozir_lp3->co2 = .0f; // Init CO2
    cozir_lp3->humidity = .0f; // Init humidity
    cozir_lp3->temperature = .0f; // Init temperature
    cozir_lp3->state = ERROR_SENSOR_NOT_INITIALIZED; // Sensor not initialized state
    cozir_lp3->meas_state = (cozir_lp3_meas_state_e)0; // Measurement not started
    cozir_lp3->wake_time = make_timeout_time_ms(UINT32_MAX); // Disable timer
}

int32_t cozir_lp3_write_config(cozir_lp3_config_t* config)
{
    int32_t ret;
    uint8_t buf[2];
    cozir_lp3_config_t read_config;
    cozir_lp3_read_config(&read_config);
    
    if (read_config.pressure_comp != config->pressure_comp || read_config.pressure != config->pressure)
    {
        if (config->pressure_comp && read_config.pressure != config->pressure)
        {
            *((uint16_t*)&buf[0]) = ntoh16(config->pressure);
            if ((ret = cozir_lp3_write(REG_ALTITUDE_PRESSURE, buf, 2)) != 0) return ret;
        }
        else if (!config->pressure_comp && read_config.pressure != 1013)
        {
            *((uint16_t*)&buf[0]) = ntoh16(1013);
            if ((ret = cozir_lp3_write(REG_ALTITUDE_PRESSURE, buf, 2)) != 0) return ret;
        }
    }
    
    // buf[0] = config->temp_humidity_meas_en ? 1 : 0;
    // if ((ret = cozir_lp3_write(REG_TH_CONTROL, &buf[0], 1)) != 0) return ret;

    if (read_config.PWM_en != config->PWM_en)
    {
        buf[0] = config->PWM_en ? 0b1 << 7 : 0;
        if ((ret = cozir_lp3_write(REG_PWM_CONTROL, &buf[0], 1)) != 0) return ret;
    }

    if (read_config.enable_cal != config->enable_cal)
    {
        buf[0] = config->enable_cal ? 0b10 : 0b00;
        if ((ret = cozir_lp3_write(REG_ABC_CONTROL, &buf[0], 1)) != 0) return ret;
    }
    if (config->enable_cal)
    {
        if (read_config.ABC_init_period != config->ABC_init_period)
        {
            *((uint16_t*)&buf[0]) = ntoh16(config->ABC_init_period);
            if ((ret = cozir_lp3_write(REG_ABC_INIT_PERIOD, buf, 2)) != 0) return ret;
        }
        if (read_config.ABC_period != config->ABC_period)
        {
            *((uint16_t*)&buf[0]) = ntoh16(config->ABC_period);
            if ((ret = cozir_lp3_write(REG_ABC_PERIOD, buf, 2)) != 0) return ret;
        }
        if (read_config.ABC_target_co2 != config->ABC_target_co2)
        {
            *((uint16_t*)&buf[0]) = ntoh16(config->ABC_target_co2);
            if ((ret = cozir_lp3_write(REG_ABC_TARGET, buf, 2)) != 0) return ret;
        }
        if (read_config.cal_target_fresh_air != config->cal_target_fresh_air)
        {
            *((uint16_t*)&buf[0]) = ntoh16(config->cal_target_fresh_air);
            if ((ret = cozir_lp3_write(REG_CAL_TARGET_FRESH_AIR, buf, 2)) != 0) return ret;
        }
        buf[0] = 0b1 << config->cal_mode;
        if ((ret = cozir_lp3_write(REG_SENSOR_CONTROL_SETTING, &buf[0], 1)) != 0) return ret;
    }
    if (read_config.alarm_treshold_co2 != config->alarm_treshold_co2)
    {
        if (config->alarm_en)
        {
            *((uint16_t*)&buf[0]) = ntoh16(config->alarm_treshold_co2);
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

int32_t cozir_lp3_read_config(cozir_lp3_config_t* config)
{
    int32_t ret;
    uint8_t buf[2];
    if ((ret = cozir_lp3_read(REG_ALTITUDE_PRESSURE, buf, 2)) != 0) return ret;
    config->pressure = ntoh16(*((uint16_t*)(&buf[0])));
    config->pressure_comp = config->pressure != 1013;

    if ((ret = cozir_lp3_read(REG_PWM_CONTROL, &buf[0], 1)) != 0) return ret;
    config->PWM_en = ((buf[0] >> 7) & 0x1);

    if ((ret = cozir_lp3_read(REG_ABC_CONTROL, &buf[0], 1)) != 0) return ret;
    config->enable_cal = (buf[0] >> 1) & 0x1;

    if ((ret = cozir_lp3_read(REG_ABC_INIT_PERIOD, buf, 2)) != 0) return ret;
    config->ABC_init_period = ntoh16(*((uint16_t*)(&buf[0])));

    if ((ret = cozir_lp3_read(REG_ABC_PERIOD, buf, 2)) != 0) return ret;
    config->ABC_period = ntoh16(*((uint16_t*)(&buf[0])));

    if ((ret = cozir_lp3_read(REG_ABC_TARGET, buf, 2)) != 0) return ret;
    config->ABC_target_co2 = ntoh16(*((uint16_t*)(&buf[0])));

    if ((ret = cozir_lp3_read(REG_CAL_TARGET_FRESH_AIR, buf, 2)) != 0) return ret;
    config->cal_target_fresh_air = ntoh16(*((uint16_t*)(&buf[0])));

    if ((ret = cozir_lp3_read(REG_ALARM_LEVEL, buf, 2)) != 0) return ret;
    config->alarm_treshold_co2 = ntoh16(*((uint16_t*)(&buf[0])));
    config->alarm_en = config->alarm_treshold_co2 != 0;
    
    return SUCCESS;
}

void cozir_lp3_power(cozir_lp3_t* cozir_lp3, bool on)
{
    if (!cozir_lp3->config->power_global_control) // If power not controlled globally
    {
        // Read power vector
        // Check if bit turned [on]
        // Write power vector
    }
}