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
    sleep_us(100);
    print_ser_output(SEVERITY_INFO, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Resetting I2C...");
    int i = 0;
    for (i = 0; i < 10; i++)
    {
        print_ser_output(SEVERITY_DEBUG, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Sending I2C reset burst %d...", i);
        for (int j = 0; j < 32; j++)
        {
            gpio_put(I2C_SENSOR_SCL, 0); // Pull down
            sleep_us(10);
            gpio_put(I2C_SENSOR_SCL, 1); // Pull up
            sleep_us(10);
            gpio_put(I2C_SENSOR_SDA, 1); // Pull up
            sleep_us(10);
        }
        if (gpio_get(I2C_SENSOR_SDA) && gpio_get(I2C_SENSOR_SCL)) break; // SDA set high
        sleep_ms(1);
    }
    print_ser_output(gpio_get(I2C_SENSOR_SDA) ? SEVERITY_INFO : SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_NO_SOURCE, 
        "I2C reset result: %s", gpio_get(I2C_SENSOR_SDA) ? "SUCCESS" : "FAILURE");
    gpio_set_function(I2C_SENSOR_SDA, GPIO_FUNC_I2C); // Reset SDA to I2C pin
    gpio_set_function(I2C_SENSOR_SCL, GPIO_FUNC_I2C); // Reset SCL to I2C pin
    gpio_pull_up(I2C_SENSOR_SDA); // Pull I2C pins up
    gpio_pull_up(I2C_SENSOR_SCL);
    sleep_us(100);

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

    i2c_baud = i2c_init(I2C_SENSOR, I2C_FREQ); // Initialize I2C
    print_ser_output(SEVERITY_DEBUG, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Initialized sensor I2C bus");
    sleep_ms(10);
}
