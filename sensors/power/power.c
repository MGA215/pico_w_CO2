#include "power.h"
#include "shared.h"
#include "hardware/i2c.h"
#include "../../common/debug.h"
#include "error_codes.h"

int32_t power_en_set_vector(uint8_t affected_sensors_vector, bool on)
{
    uint8_t power_vector;
    int32_t ret;
    uint8_t power_vector_out;
    sleep_us(1);
    if ((ret = i2c_read_timeout_us(I2C_DEVICE, POWER_EN_ADDR, &power_vector, 1, false, I2C_TIMEOUT_US)) < 0)
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_POWER, "Failed to read power en vector: %i", ret);
        return ret;
    }
    if (!on) power_vector_out = power_vector | affected_sensors_vector; // Enable LOW
    else power_vector_out = power_vector & (~affected_sensors_vector);
    if ((ret = i2c_write_timeout_us(I2C_DEVICE, POWER_EN_ADDR, &power_vector_out, 1, false, I2C_TIMEOUT_US)) < 0)
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_POWER, "Failed to write power en vector %x: %i", power_vector_out, ret);
        return ret;
    }
    print_ser_output(SEVERITY_DEBUG, SOURCE_SENSORS, SOURCE_POWER, "Successfully set power en vector");
    return SUCCESS;
}

int32_t power_en_set_index(uint8_t sensor_index, bool on)
{
    uint8_t power_vector;
    uint8_t power_vector_out;
    int32_t ret;

    if ((ret = i2c_read_timeout_us(I2C_DEVICE, POWER_EN_ADDR, &power_vector, 1, false, I2C_TIMEOUT_US)) < 0)
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_POWER, "Failed to read power en vector: %i", ret);
        return ret;
    }
    if (!on) power_vector_out = power_vector | (0b1 << sensors[sensor_index].power_index); // Enable LOW
    else power_vector_out = power_vector & (~(0b1 << sensors[sensor_index].power_index));
    if ((ret = i2c_write_timeout_us(I2C_DEVICE, POWER_EN_ADDR, &power_vector_out, 1, false, I2C_TIMEOUT_US)) < 0)
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_POWER, "Failed to set power en value for sensor %i: %i", sensor_index, ret);
        return ret;
    }
    print_ser_output(SEVERITY_DEBUG, SOURCE_SENSORS, SOURCE_POWER, "Successfully set power en value for sensor %i", sensor_index);
    return SUCCESS;
}

int32_t power_5v_set_vector(uint8_t power_vector)
{
    int32_t ret;
    sleep_us(1);
    if ((ret = i2c_write_timeout_us(I2C_DEVICE, POWER_5V_ADDR, &power_vector, 1, false, I2C_TIMEOUT_US)) < 0)
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_POWER, "Failed to write power 5v vector %x: %i", power_vector, ret);
        return ret;
    }
    print_ser_output(SEVERITY_DEBUG, SOURCE_SENSORS, SOURCE_POWER, "Successfully set power 5v vector");
    return SUCCESS;
}

void power_reset_all(void)
{
    power_en_set_vector(0xFF, false);
    power_5v_set_vector(0);
}