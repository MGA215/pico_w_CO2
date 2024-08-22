#include "hyt271.h"
#include "pico/stdlib.h"
#include "common/shared.h"
#include "hardware/i2c.h"
#include "common/debug.h"
#include "common/constants.h"
#include "string.h"
#include "math.h"
#include "error_codes.h"

#define HYT271_ADDR           0x28





void hyt271_get_value()
{
    int32_t ret;
    uint8_t tmp_buffer[4];
    switch (hyt271.meas_state)
    {
        case MEAS_STARTED:
        {
            tmp_buffer[0] = 0;
            if ((ret = i2c_write_timeout_us(I2C_DEVICE, HYT271_ADDR, tmp_buffer, 1, false, I2C_TIMEOUT_US)) < 0) // Start measurement
            {
                print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_HYT271, "Failed to write measurement start: %i", ret);
                hyt271.meas_state = MEAS_FINISHED; // On error reset values
                hyt271.state = ret;
                hyt271.humidity = NAN;
                hyt271.temperature = NAN;
                memset(hyt271.humidity_raw, 0x00, 2);
                memset(hyt271.temperature_raw, 0x00, 2);
                return;
            }
            hyt271.meas_state = MEAS_READ_VALUE; // Next state - read value
            hyt271.state = SUCCESS;
            hyt271.wake_time = make_timeout_time_ms(200);
            return;
        }
        case MEAS_READ_VALUE:
        {
            sleep_us(1);
            if ((ret = i2c_read_timeout_us(I2C_DEVICE, HYT271_ADDR, tmp_buffer, 4, false, I2C_TIMEOUT_US)) < 0) // Read measurement
            {
                print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_HYT271, "Failed to read measurement data: %i", ret);
                hyt271.meas_state = MEAS_FINISHED; // On error reset values
                hyt271.state = ret;
                hyt271.humidity = NAN;
                hyt271.temperature = NAN;
                memset(hyt271.humidity_raw, 0x00, 2);
                memset(hyt271.temperature_raw, 0x00, 2);
                return;
            }

            if ((tmp_buffer[0] & 0xC0) != 0) // Check status bits
            {
                hyt271.meas_state = MEAS_FINISHED; // On error reset values
                hyt271.state = HYT271_ERROR_GENERAL;
                hyt271.humidity = NAN;
                hyt271.temperature = NAN;
                memset(hyt271.humidity_raw, 0x00, 2);
                memset(hyt271.temperature_raw, 0x00, 2);
                return;
            }

            memcpy(hyt271.humidity_raw, tmp_buffer, 2); // Copy raw values
            uint16_t temp_val = (tmp_buffer[2] * 256 + tmp_buffer[3]) >> 2;
            hyt271.temperature_raw[0] = (temp_val & 0xFF00) >> 8;
            hyt271.temperature_raw[1] = temp_val & 0x00FF;

            // Computing final temperature and humidity values
            hyt271.humidity = (float)(tmp_buffer[0] * 256 + tmp_buffer[1]) * (100.0 / 16383.0);
            hyt271.temperature = ((float)(((tmp_buffer[2] * 256 + tmp_buffer[3]) >> 2)) * (165.0 / 16383.0)) - 40.0;

            hyt271.state = SUCCESS;
            hyt271.meas_state = MEAS_FINISHED;
            return;
        }
        case MEAS_FINISHED:
        {
            hyt271.wake_time = at_the_end_of_time;
            return;
        }
        default:
        {
            hyt271.meas_state = MEAS_FINISHED;
            return;
        }
    }
}
