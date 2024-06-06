#include "main.h"
#include <stdlib.h>
#include <string.h>
/**
 * Initialization error codes
 * 0: Success
 * -1: Failed STDIO initialization
 * -2: Failed timer for reading inputs initialization
 * -3: Failed display timer initialization
*/

/**
 * Initialization function for the project
 * returns: 0 if successful init, otherwise value, see the error codes map
*/

// structure containing info about the RTC module
struct ds3231_rtc rtc;

// structure containing sensor readings
sensors_t sensor_readings;

// GFX Pack previous button state
uint8_t buttons_prev_state = 0;

// value representing the interval between loop calls in ms
uint32_t loop_interval = 33;

// value representing the interval between button input reads in ms
uint32_t update_interval = 33;

// gfx_pack backlight brightness
uint8_t blight_brightness = 255;

// string holding the datetime value
uint8_t datetime_str[30]; 

// gfx_pack backlight on
bool blight_on = true;

/**
 * @brief Sets RTC's datetime, modify datetime inside
 * 
 */
void set_datetime(void);

int main()
{
    init(); // init function
    sleep_ms(1000);

    // repeating_timer_t timer_sensor;

    // // Initialize timer to read current time from RTC
    // if (!add_repeating_timer_ms(update_interval, update, NULL, &timer_sensor)) { // negative timeout means exact delay (rather than delay between callbacks)
    //     printf("Failed to add timer for sensor\n");
    //     return -2;
    // }

    while (true) {
        //update(); // update loop
        sleep_ms(loop_interval);
        loop(); // main loop
    }
    return 0;
}

int init(void)
{
    if (!stdio_init_all()) return -1; // Initializing STDIO

    ds3231_init(DS3231_I2C_PORT, DS3231_I2C_SDA_PIN, DS3231_I2C_SCL_PIN, &rtc); // Initializing I2C for communication with RTC module
    gfx_pack_init(); // initialize display
    ee895_init();


    init_sensors();

    loop_interval = 2000; // Setting the loop timer
}


int loop(void)
{
    int32_t ret = ee895_get_value(&sensor_readings.ee895.co2, &sensor_readings.ee895.temperature, &sensor_readings.ee895.pressure);
    if (ret != 0) printf("[ERROR] Failed reading from the sensor: %i\n", ret);
    else
    {
        printf("Read values: CO2: %.0f ppm; temperature: %.2f °C; pressure: %.1f hPa\n", sensor_readings.ee895.co2, sensor_readings.ee895.temperature, sensor_readings.ee895.pressure);
    }
    return 0;
}

void set_datetime(void)
{
    ds3231_datetime_t dt = {
        .year = 2024,
        .month = 6,
        .dotw = 2,
        .day = 4,
        .hour = 10,
        .minutes = 19,
        .seconds = 00
    };
    ds3231_set_datetime(&dt, &rtc); // refresh datetime
}

void get_datetime(uint8_t* datetime_str, uint8_t datetime_len)
{
    ds3231_datetime_t dt;

    ds3231_get_datetime(&dt, &rtc); // read datetime
    ds3231_datetime2str(datetime_str, datetime_len, &dt); // convert datetime to string
}

int ds3231_datetime2str(char *buf, uint8_t buf_size, const ds3231_datetime_t *dt)
{
    return snprintf(buf, buf_size, "%04u-%02u-%02u %02u:%02u:%02u", dt->year, dt->month, dt->day, dt->hour, dt->minutes, dt->seconds); // Conversion of the datetime struct to date time string
}

void update()
{
    update_RTC(); // Updates datetime
    read_inputs(); // Updates button inputs
    update_display(); // Updates display
    return;
}

void update_RTC()
{
    get_datetime(datetime_str, sizeof(datetime_str)); // Retrieves current datetime
}

void read_inputs()
{
    if (gfx_pack_read_button(GFX_PACK_BUTTON_A)) // Button A down
    {
        if ((buttons_prev_state & (0b1 << 0)) == 0) // Button A pressed - single action
        {
            blight_on = !blight_on; // Turn backlight off
            buttons_prev_state |= (0b1 << 0);
        }
        // Button A down - repeat action
    }
    else // Button A up
    {
        buttons_prev_state &= ~(0b1 << 0);
    }

    if (gfx_pack_read_button(GFX_PACK_BUTTON_B)) // Button B down
    {
        if ((buttons_prev_state & (0b1 << 1)) == 0) // Button B pressed - single action
        {
            buttons_prev_state |= (0b1 << 1);
        }
        // Button B down - repeat action
        if (blight_brightness > 5) blight_brightness -= 5; // Decrease backlight brightness
    }
    else // Button B up
    {
            buttons_prev_state &= ~(0b1 << 1);
    }

    if (gfx_pack_read_button(GFX_PACK_BUTTON_C)) // Button C down
    {
        if ((buttons_prev_state & (0b1 << 2)) == 0) // Button C pressed - single action
        {
            buttons_prev_state |= (0b1 << 2);
        }
        // Button C down - repeat action
        if (blight_brightness < 251) blight_brightness += 5; // Increase backlight brightness
    }
    else // Button C up
    {
        buttons_prev_state &= ~(0b1 << 2);
    }

    if (gfx_pack_read_button(GFX_PACK_BUTTON_D)) // Button D down
    {
        if ((buttons_prev_state & (0b1 << 3)) == 0) // Button D pressed - single action
        {
            buttons_prev_state |= (0b1 << 3);
        }
        // Button D down - repeat action
    }
    else // Button D up
    {
        buttons_prev_state &= ~(0b1 << 3);
    }

    if (gfx_pack_read_button(GFX_PACK_BUTTON_E)) // Button E down
    {
        if ((buttons_prev_state & (0b1 << 4)) == 0) // Button E pressed - single action
        {
            buttons_prev_state |= (0b1 << 4);
            gfx_pack_clear_display();
            //gfx_pack_update();
        }
        // Button E down - repeat action
    }
    else // Button E up
    {
        buttons_prev_state &= ~(0b1 << 4);
    }

    return;
}

void write_display(void)
{
    gfx_pack_clear_display();
    point_t position = {.x = 0, .y = 0}; // position of datetime string on display
    gfx_pack_write_text(&position, (char*)datetime_str); // write datetime string to display
    position.x = 0;
    position.y = 1;
    gfx_pack_write_text(&position, "E+E EE895");
    position.x = 0;
    position.y = 2;
    if (sensor_readings.ee895.state != 0)
    {
        char buf[4];
        memset(buf, 0x00, 4);
        gfx_pack_write_text(&position, snprintf(buf, 4, "E%i", sensor_readings.ee895.state));
    }
    else
    {
        char buf[16];
        memset(buf, 0x00, 16);
        gfx_pack_write_text(&position, snprintf(buf, 16, "CO2: %.0f ppm", sensor_readings.ee895.co2));
        position.x = 0;
        position.y = 3;
        memset(buf, 0x00, 16);
        gfx_pack_write_text(&position, snprintf(buf, 16, "T: %4.2f °C", sensor_readings.ee895.temperature));
        position.x = 0;
        position.y = 3;
        memset(buf, 0x00, 16);
        gfx_pack_write_text(&position, snprintf(buf, 16, "p: %4.1f hPa", sensor_readings.ee895.pressure));
    }
}

void update_display()
{
    gfx_pack_set_backlight(blight_brightness * blight_on); // set display backlight brightness
    gfx_pack_update(); // Update display
    return;
}

void init_sensors(void)
{
    sensor_readings.ee895.co2 = .0f;
    sensor_readings.ee895.pressure = .0f;
    sensor_readings.ee895.temperature = .0f;
}

void read_sensors(void)
{
    sensor_readings.ee895.state = ee895_get_value(&sensor_readings.ee895.co2, &sensor_readings.ee895.temperature, &sensor_readings.ee895.pressure);
}
