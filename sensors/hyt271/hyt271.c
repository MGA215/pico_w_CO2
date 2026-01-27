#include "hyt271.h"
#include "pico/stdlib.h"
#include "common/structs.h"
#include "hardware/i2c.h"
#include "common/debug.h"
#include "common/constants.h"
#include "string.h"
#include "math.h"
#include "error_codes.h"

#define HYT271_ADDR           0x28

sensor_functions_t hyt271_functions = {
    .sensor_get_value = hyt271_get_value,
    .sensor_init = NULL,
    .sensor_read_config = NULL
};




void hyt271_get_value(sensor_t* sensor)
{
    int32_t ret;
    uint8_t tmp_buffer[4];
    switch (sensor->meas_state)
    {
        case MEAS_STARTED:
        {
            sensor->internal_error_state = PICO_OK;
            tmp_buffer[0] = 0;
            if ((ret = i2c_write_timeout_us(I2C_DEVICE, HYT271_ADDR, tmp_buffer, 1, false, I2C_TIMEOUT_US)) < 0) // Start measurement
            {
                print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_HYT271, "Failed to write measurement start: %i", ret);
                sensor->meas_state = MEAS_FINISHED; // On error reset values
                sensor->internal_error_state = ret;
                sensor->humidity = NAN;
                sensor->temperature = NAN;
                memset(sensor->humidity_raw, 0x00, 2);
                memset(sensor->temperature_raw, 0x00, 2);
                return;
            }
            sensor->meas_state = MEAS_READ_VALUE; // Next state - read value
            sensor->internal_error_state = SUCCESS;
            sensor->wake_time = make_timeout_time_us(200000);
            return;
        }
        case MEAS_READ_VALUE:
        {
            sleep_us(1);
            if ((ret = i2c_read_timeout_us(I2C_DEVICE, HYT271_ADDR, tmp_buffer, 4, false, I2C_TIMEOUT_US)) < 0) // Read measurement
            {
                print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_HYT271, "Failed to read measurement data: %i", ret);
                sensor->meas_state = MEAS_FINISHED; // On error reset values
                sensor->internal_error_state = ret;
                sensor->humidity = NAN;
                sensor->temperature = NAN;
                memset(sensor->humidity_raw, 0x00, 2);
                memset(sensor->temperature_raw, 0x00, 2);
                return;
            }

            if ((tmp_buffer[0] & 0xC0) != 0) // Check status bits
            {
                sensor->meas_state = MEAS_FINISHED; // On error reset values
                sensor->internal_error_state = HYT271_ERROR_GENERAL;
                sensor->humidity = NAN;
                sensor->temperature = NAN;
                memset(sensor->humidity_raw, 0x00, 2);
                memset(sensor->temperature_raw, 0x00, 2);
                return;
            }

            memcpy(sensor->humidity_raw, tmp_buffer, 2); // Copy raw values
            uint16_t temp_val = (tmp_buffer[2] * 256 + tmp_buffer[3]) >> 2;
            sensor->temperature_raw[0] = (temp_val & 0xFF00) >> 8;
            sensor->temperature_raw[1] = temp_val & 0x00FF;

            // Computing final temperature and humidity values
            sensor->humidity = (float)(tmp_buffer[0] * 256 + tmp_buffer[1]) * (100.0 / 16383.0);
            sensor->temperature = ((float)(((tmp_buffer[2] * 256 + tmp_buffer[3]) >> 2)) * (165.0 / 16383.0)) - 40.0;

            sensor->internal_error_state = SUCCESS;
            sensor->meas_state = MEAS_FINISHED;
            return;
        }
        case MEAS_FINISHED:
        {
            sensor->wake_time = at_the_end_of_time;
            return;
        }
        default:
        {
            sensor->meas_state = MEAS_FINISHED;
            return;
        }
    }
}
