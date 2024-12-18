/**
 * @file cozir-lp3.c
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Implements communication with GSS CozIR-LP3 sensor
 * @version 0.1
 * @date 2024-06-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "cozir-lp3.h"
#include "math.h"
#include "hardware/i2c.h"
#include "string.h"
#include "common/debug.h"
#include "error_codes.h"
#include "common/functions.h"
#include "../power/power.h"
#include "common/shared.h"

#define COZIR_LP3_ADDR              0x41

#define REG_MEAS_CONTROL            0x00
#define REG_CO2                     0x02
#define REG_DIGITAL_FILTER_SETTING  0x04
#define REG_SENSOR_CONTROL_SETTING  0x05
#define REG_ABC_INIT_PERIOD         0x06
#define REG_ABC_PERIOD              0x08
#define REG_ABC_TARGET              0x0C
#define REG_CAL_TARGET_FRESH_AIR    0x12
#define REG_CAL_KNOWN_CO2           0x14
#define REG_TEMPERATURE             0x20
#define REG_RH                      0x22
#define REG_SER_NUMBER              0x26
#define REG_ALARM_LEVEL             0x2C
#define REG_PWM_CONTROL             0x2E
#define REG_CO2_UNFILTERED          0x34
#define REG_ABC_CONTROL             0x4E
#define REG_SELF_TEST               0x50
#define REG_TOTAL_HOURS             0x5A
#define REG_ALTITUDE_PRESSURE       0x76
#define REG_TH_CONTROL              0x8C


/**
 * @brief Writes configuration to the sensor
 * 
 * @param config Configuration to write
 * @return int32_t Return code
 */
static int32_t lp3_write_config(sensor_config_t* config);

/**
 * @brief Turns the sensor power [on]
 * 
 * @param cozir_lp3 Sensor structure
 * @param on Whether the power should be turned on
 */
static inline void lp3_power(sensor_t* cozir_lp3, bool on);


int32_t cozir_lp3_read(uint8_t addr, uint8_t* buf, uint8_t len)
{
    if (len < 1 || len > 4) return COZIR_LP3_ERROR_NREG_REG; // Check number of read bytes is 1 .. 4
    int32_t ret;
    
    if ((ret = i2c_write_timeout_us(I2C_SENSOR, COZIR_LP3_ADDR, &addr, 1, true, I2C_TIMEOUT_US)) < 0) return ret; // Send command

    if ((ret = i2c_read_timeout_us(I2C_SENSOR, COZIR_LP3_ADDR, buf, len, false, I2C_TIMEOUT_US)) < 0) return ret; // Read data

    return SUCCESS;
}

int32_t cozir_lp3_write(uint8_t addr, uint8_t* data, uint8_t len)
{
    if (len < 1 || len > 4) return COZIR_LP3_ERROR_NREG_REG; // Check number of written bytes is 1 .. 4
    int32_t ret;
    uint8_t command_buffer[len + 2]; // data length + command
    memset(command_buffer, 0x00, len + 2); // Clear command buffer
    command_buffer[0] = addr; // Add address to buffer
    for (int i = 0; i < len + 1; i++)
    {
        command_buffer[i + 1] = data[i]; // Add data to buffer
    }

    if ((ret = i2c_write_timeout_us(I2C_SENSOR, COZIR_LP3_ADDR, command_buffer, len + 1, false, I2C_TIMEOUT_US)) < 0) return ret; // Send command
    sleep_ms(1);
    return SUCCESS;
}

void cozir_lp3_get_value(sensor_t* cozir_lp3)
{
    int32_t ret;
    uint8_t tempBuffer[5];
    if (cozir_lp3->config.sensor_type != COZIR_LP3) // Check for correct sensor type
    {
        cozir_lp3->meas_state = MEAS_FINISHED;
        cozir_lp3->state = ERROR_UNKNOWN_SENSOR;
        cozir_lp3->co2 = NAN;
        cozir_lp3->humidity = NAN;
        cozir_lp3->temperature = NAN;
        return;
    } 
    switch(cozir_lp3->meas_state)
    {
        case MEAS_FINISHED:
        {
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_COZIR_LP3, "Meas finished");
            lp3_power(cozir_lp3, false); // Power off
            cozir_lp3->wake_time = at_the_end_of_time; // Disable timer
            if (cozir_lp3->state) cozir_lp3->state = ERROR_NO_MEAS;
            cozir_lp3->timeout_iterator = 0; // Initialize iterator
            return;
        }
        case MEAS_STARTED:
        {
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_COZIR_LP3, "Meas started");
            lp3_power(cozir_lp3, true);
            if (!cozir_lp3->config.power_continuous) cozir_lp3->wake_time = make_timeout_time_ms(cozir_lp3->config.sensor_power_up_time); // Time for power stabilization
            cozir_lp3->meas_state = MEAS_READ_VALUE; // Ignore read status - is implemented within read value
            return;
        }
        case MEAS_READ_VALUE:
        {
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_COZIR_LP3, "Read value");
            ret = cozir_lp3_read(REG_CO2, tempBuffer, 3);
            if (ret != 0)
            {
                cozir_lp3->co2 = NAN;
                cozir_lp3->humidity = NAN;
                cozir_lp3->temperature = NAN;
                cozir_lp3->meas_state = MEAS_FINISHED;
                cozir_lp3->state = ret;
                return;
            }
            if (tempBuffer[2] != 85)
            {
                cozir_lp3->state = COZIR_LP3_ERROR_SENSOR_GENERAL;
                cozir_lp3->co2 = NAN;
                cozir_lp3->humidity = NAN;
                cozir_lp3->temperature = NAN;
                cozir_lp3->meas_state = MEAS_FINISHED;
                return;
            }

            // uint8_t buf[2]; // Testing CozIR filtering
            // {
            //     ret = cozir_lp3_read(REG_CO2_UNFILTERED, buf, 2);
            //     if (ret != 0)
            //     {
            //         cozir_lp3->co2 = NAN;
            //         cozir_lp3->humidity = NAN;
            //         cozir_lp3->temperature = NAN;
            //         cozir_lp3->meas_state = MEAS_FINISHED;
            //         cozir_lp3->state = ret;
            //         return;
            //     }
            // }

            if (cozir_lp3->timeout_iterator > 20)
            {
                cozir_lp3->co2 = NAN;
                cozir_lp3->humidity = NAN;
                cozir_lp3->temperature = NAN;
                cozir_lp3->state = COZIR_LP3_ERROR_DATA_READY_TIMEOUT;
                cozir_lp3->state = MEAS_FINISHED;
                return;
            }
            if (tempBuffer[0] == 0xFF && tempBuffer[1] == 0xFF)
            {
                cozir_lp3->timeout_iterator++;
                cozir_lp3->wake_time = make_timeout_time_ms(100);
                return;
            }

            // { // Testing CozIR filtering
            //     if (buf[0] == 0xFF && buf[1] == 0xFF)
            //     {
            //         cozir_lp3->timeout_iterator++;
            //         cozir_lp3->wake_time = make_timeout_time_ms(100);
            //         return;
            //     }
            // }

            uint16_t val = 0;
            memcpy(&val, &tempBuffer[0], 2);
            cozir_lp3->co2 = (float)ntoh16(val);

            // { // Testing CozIR filtering
            //     val = 0;
            //     memcpy(&val, &buf[0], 2);
            //     cozir_unfiltered = (float)ntoh16(val);
            // }

            cozir_lp3->meas_state = MEAS_FINISHED;
            cozir_lp3->state = SUCCESS;
            print_ser_output(SEVERITY_TRACE, SOURCE_SENSORS, SOURCE_COZIR_LP3, "Measured CO2 value: %f", cozir_lp3->co2);
            return;
        }
        default:
        {
            cozir_lp3->meas_state = MEAS_FINISHED;
            return;
        }
    }
}

int32_t cozir_lp3_init(sensor_t* cozir_lp3, sensor_config_t* config)
{
    int32_t ret;
    if (config->sensor_type != COZIR_LP3) return ERROR_UNKNOWN_SENSOR; // Check for correct sensor type
    memcpy(&cozir_lp3->config, config, sizeof(sensor_config_t));

    ret = lp3_write_config(config);
    if (!ret)
    {
        if (cozir_lp3->meas_state == MEAS_STARTED) cozir_lp3->wake_time = make_timeout_time_ms(3000);
    }
    else cozir_lp3->meas_state = MEAS_FINISHED;
    return ret;
}

int32_t cozir_lp3_read_config(sensor_config_t* config)
{
    int32_t ret;
    uint8_t buf[2];
    config->sensor_type = COZIR_LP3;
    if ((ret = cozir_lp3_read(REG_MEAS_CONTROL, &buf[0], 1)) != 0) return ret;
    if ((ret = cozir_lp3_read(REG_DIGITAL_FILTER_SETTING, &buf[0], 1)) != 0) return ret;
    config->filter_coeff = buf[0];

    if ((ret = cozir_lp3_read(REG_ALTITUDE_PRESSURE, buf, 2)) != 0) return ret;
    memcpy(&config->pressure, &buf[0], 2);
    config->pressure = ntoh16(config->pressure);
    config->enable_pressure_comp = config->pressure != 1013;

    if ((ret = cozir_lp3_read(REG_PWM_CONTROL, &buf[0], 1)) != 0) return ret;
    config->enable_PWM_pin = ((buf[0] >> 7) & 0x1);

    if ((ret = cozir_lp3_read(REG_ABC_CONTROL, &buf[0], 1)) != 0) return ret;
    config->enable_abc = (buf[0] >> 1) & 0x1;

    if ((ret = cozir_lp3_read(REG_ABC_INIT_PERIOD, buf, 2)) != 0) return ret;
    memcpy(&config->abc_init_period, &buf[0], 2);
    config->abc_init_period = (uint16_t)(((uint32_t)ntoh16(config->abc_init_period) * 50) / 3600);

    if ((ret = cozir_lp3_read(REG_ABC_PERIOD, buf, 2)) != 0) return ret;
    memcpy(&config->abc_period, &buf[0], 2);
    config->abc_period = (uint16_t)(((uint32_t)ntoh16(config->abc_period) * 50) / 3600);

    if ((ret = cozir_lp3_read(REG_ABC_TARGET, buf, 2)) != 0) return ret;
    memcpy(&config->abc_target_value, &buf[0], 2);
    config->abc_target_value = ntoh16(config->abc_target_value);

    if ((ret = cozir_lp3_read(REG_ALARM_LEVEL, buf, 2)) != 0) return ret;
    memcpy(&config->alarm_treshold_co2_high, &buf[0], 2);
    config->alarm_treshold_co2_high = ntoh16(config->alarm_treshold_co2_high);
    config->alarm_en = config->alarm_treshold_co2_high != 0;
    
    return SUCCESS;
}

static int32_t lp3_write_config(sensor_config_t* config)
{
    int32_t ret;
    uint8_t buf[2];
    sensor_config_t read_config;
    if ((ret = cozir_lp3_read_config(&read_config)) != 0) return ret;
    
    if (config->filter_coeff != read_config.filter_coeff)
    {
        print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_COZIR_LP3, "Config - Writing filter coefficient");
        buf[0] = (uint8_t)config->filter_coeff;
        if ((ret = cozir_lp3_write(REG_DIGITAL_FILTER_SETTING, &buf[0], 1)) != 0) return ret;
    }

    if (read_config.enable_pressure_comp != config->enable_pressure_comp || read_config.pressure != config->pressure)
    {
        print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_COZIR_LP3, "Config - Writing pressure");
        if (config->enable_pressure_comp && read_config.pressure != config->pressure)
        {
            buf[0] = (config->pressure & 0xFF00) >> 8;
            buf[1] = (config->pressure & 0x00FF) >> 0;
            if ((ret = cozir_lp3_write(REG_ALTITUDE_PRESSURE, buf, 2)) != 0) return ret;
        }
        else if (!config->enable_pressure_comp && read_config.pressure != 1013)
        {
            buf[0] = (1013 & 0xFF00) >> 8;
            buf[1] = (1013 & 0x00FF) >> 0;
            if ((ret = cozir_lp3_write(REG_ALTITUDE_PRESSURE, buf, 2)) != 0) return ret;
        }
    }

    if (read_config.enable_PWM_pin != config->enable_PWM_pin)
    {
        print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_COZIR_LP3, "Config - Writing PWM pin");
        buf[0] = config->enable_PWM_pin ? 0b1 << 7 : 0;
        if ((ret = cozir_lp3_write(REG_PWM_CONTROL, &buf[0], 1)) != 0) return ret;
    }

    if (read_config.enable_abc != config->enable_abc)
    {
        print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_COZIR_LP3, "Config - Writing ABC enable");
        buf[0] = config->enable_abc ? 0b10 : 0b00;
        if ((ret = cozir_lp3_write(REG_ABC_CONTROL, &buf[0], 1)) != 0) return ret;
    }

    if (config->abc_init_period == config->abc_period)
    {
        if (config->abc_period == (uint16_t)(((uint32_t)0xFFFF * 50) / 3600)) config->abc_period = (uint16_t)(((uint32_t)0xFFFF * 50) / 3600) - 1; // Converting to hours
        else config->abc_period += 1;
        print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_COZIR_LP3, "Config - ABC initial period and standard period have the same value, ABC standard period set to %04X", config->abc_period);
    }
    if (read_config.abc_init_period != config->abc_init_period)
    {
        print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_COZIR_LP3, "Config - Writing ABC initial period");
        uint16_t init_per = (uint16_t)((3600 * (uint32_t)config->abc_init_period) / 50);
        buf[0] = (init_per & 0xFF00) >> 8;
        buf[1] = (init_per & 0x00FF) >> 0;
        if ((ret = cozir_lp3_write(REG_ABC_INIT_PERIOD, buf, 2)) != 0) return ret;
    }
    if (read_config.abc_period != config->abc_period)
    {
        print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_COZIR_LP3, "Config - Writing ABC standard period");
        uint16_t per = (uint16_t)((3600 * (uint32_t)config->abc_period) / 50);
        buf[0] = (per & 0xFF00) >> 8;
        buf[1] = (per & 0x00FF) >> 0;
        if ((ret = cozir_lp3_write(REG_ABC_PERIOD, buf, 2)) != 0) return ret;
    }
    if (read_config.abc_target_value != config->abc_target_value)
    {
        print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_COZIR_LP3, "Config - Writing ABC target value");
        buf[0] = (config->abc_target_value & 0xFF00) >> 8;
        buf[1] = (config->abc_target_value & 0x00FF) >> 0;
        if ((ret = cozir_lp3_write(REG_ABC_TARGET, buf, 2)) != 0) return ret;
    }
    
    if (read_config.alarm_treshold_co2_high != config->alarm_treshold_co2_high)
    {
        print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_COZIR_LP3, "Config - Writing alarm treshold");
        if (config->alarm_en)
        {
            buf[0] = (config->alarm_treshold_co2_high & 0xFF00) >> 8;
            buf[1] = (config->alarm_treshold_co2_high & 0x00FF) >> 0;
            if ((ret = cozir_lp3_write(REG_ALARM_LEVEL, buf, 2)) != 0) return ret;
        }
        else
        {
            buf[0] = 0;
            buf[1] = 00;
            if ((ret = cozir_lp3_write(REG_ALARM_LEVEL, buf, 2)) != 0) return ret;
        }
    }
    return SUCCESS;
}

static inline void lp3_power(sensor_t* cozir_lp3, bool on)
{
    if (!cozir_lp3->config.power_global_control && !cozir_lp3->config.power_continuous) // If power not controlled globally
    {
        power_en_set_index(cozir_lp3->index, on);
    }
}