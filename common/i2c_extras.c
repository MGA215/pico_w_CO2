#include "i2c_extras.h"
#include "constants.h"
#include "debug.h"
#include "hardware/i2c.h"

static uint32_t i2c_baud;

void reset_i2c(void)
{
    i2c_deinit(I2C_SENSOR); // Deinit I2C for sensors

    gpio_set_function(I2C_SENSOR_SCL, GPIO_FUNC_SIO); // Change SCL from I2C pin to SIO
    gpio_set_dir(I2C_SENSOR_SCL, GPIO_OUT); // Set to output direction
    gpio_set_function(I2C_SENSOR_SDA, GPIO_FUNC_SIO); // Change SDA from I2C pin to SIO
    gpio_set_dir(I2C_SENSOR_SDA, GPIO_OUT); // Set to input direction

    print_ser_output(SEVERITY_INFO, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Resetting I2C...");
    for (uint8_t i = 0; i < 3; i++)
    {
        print_ser_output(SEVERITY_DEBUG, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Sending I2C reset burst %d...", i);
        // start condition
        gpio_put(I2C_SENSOR_SDA, 0);
        sleep_us(50);
        gpio_put(I2C_SENSOR_SCL, 0);
        sleep_us(100);

        for (int j = 0; j < 9; j++)
        {
            gpio_put(I2C_SENSOR_SCL, 1); // Pull up
            sleep_us(100);
            gpio_put(I2C_SENSOR_SCL, 0); // Pull down
            sleep_us(100);
        }
        // stop condition
        gpio_put(I2C_SENSOR_SCL, 1);
        sleep_us(50);
        gpio_put(I2C_SENSOR_SDA, 1);
        sleep_us(100);
        // start condition
        gpio_put(I2C_SENSOR_SDA, 0);
        sleep_us(50);
        gpio_put(I2C_SENSOR_SCL, 0);
        sleep_us(100);
        // stop condition
        gpio_put(I2C_SENSOR_SCL, 1);
        sleep_us(50);
        gpio_put(I2C_SENSOR_SDA, 1);
        sleep_us(100);

        if (gpio_get(I2C_SENSOR_SDA) && gpio_get(I2C_SENSOR_SCL)) break; // SDA set high
    }
    print_ser_output(gpio_get(I2C_SENSOR_SDA) ? SEVERITY_INFO : SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_NO_SOURCE, 
        "I2C reset result: %s", gpio_get(I2C_SENSOR_SDA) ? "SUCCESS" : "FAILURE");
    gpio_set_function(I2C_SENSOR_SDA, GPIO_FUNC_I2C); // Reset SDA to I2C pin
    gpio_set_function(I2C_SENSOR_SCL, GPIO_FUNC_I2C); // Reset SCL to I2C pin
    gpio_pull_up(I2C_SENSOR_SDA); // Pull I2C pins up
    gpio_pull_up(I2C_SENSOR_SCL);
    i2c_init(I2C_SENSOR, I2C_FREQ); // Initialize I2C
    sleep_us(100);
}

void init_sensor_i2c(void)
{
    gpio_init(I2C_SENSOR_SDA); // Initialize data pin
    gpio_set_function(I2C_SENSOR_SDA, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SENSOR_SDA);

    gpio_init(I2C_SENSOR_SCL); // Initialize clock pin
    gpio_set_function(I2C_SENSOR_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SENSOR_SCL);

    i2c_baud = i2c_init(I2C_SENSOR, 100000); // Initialize I2C
    print_ser_output(SEVERITY_DEBUG, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Initialized sensor I2C bus");
    sleep_us(100);
}

void init_device_i2c(void)
{
    i2c_init(I2C_DEVICE, I2C_DEVICE_FERQ);
    gpio_init(I2C_DEVICE_SDA);
    gpio_init(I2C_DEVICE_SCL);
    gpio_set_function(I2C_DEVICE_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_DEVICE_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_DEVICE_SDA);
    gpio_pull_up(I2C_DEVICE_SCL);

    print_ser_output(SEVERITY_DEBUG, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Initialized device I2C bus");
    sleep_us(100);
}
