
#include "main.h"

/**
 * Initialization error codes
 * 0: Success
 * -1: Failed STDIO initialization
*/

/**
 * Initialization function for the project
 * returns: 0 if successful init, otherwise value, see the error codes map
*/

// structure containing info about the RTC module
struct ds3231_rtc rtc;

// value representing the interval between loop calls in ms
uint32_t loop_interval = 1000;

int main()
{
    init();
    sleep_ms(1000);
    while (true) {
        loop();
        sleep_ms(loop_interval);
    }
}

int init(void)
{
    if (!stdio_init_all()) return -1; // Initializing STDIO

    ds3231_init(DS3231_I2C_PORT, DS3231_I2C_SDA_PIN, DS3231_I2C_SCL_PIN, &rtc); // Initializing I2C for communication with RTC module

    loop_interval = 1000; // Setting the loop timer
}

int loop(void)
{
    ds3231_datetime_t dt;
    uint8_t dt_str[30];

    ds3231_get_datetime(&dt, &rtc);
    ds3231_datetime2str(dt_str, sizeof(dt_str), &dt);
    printf("[%s] Hello World!\n", dt_str);
}

int ds3231_datetime2str(char *buf, uint8_t buf_size, const ds3231_datetime_t *dt)
{
    return snprintf(buf, buf_size, "%04u-%02u-%02u %02u:%02u:%02u", dt->year, dt->month, dt->day, dt->hour, dt->minutes, dt->seconds);
}
