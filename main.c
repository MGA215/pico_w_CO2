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

uint8_t buttons_prev_state = 0;

// value representing the interval between loop calls in ms
uint32_t loop_interval = 1000;

// value representing the interval between button input reads in ms
uint32_t read_inputs_interval = 50;

// gfx_pack backlight brightness
uint8_t blight_brightness = 255;

int main()
{
    init(); // init function
    sleep_ms(1000);

    repeating_timer_t timer;

    // negative timeout means exact delay (rather than delay between callbacks)
    if (!add_repeating_timer_ms(-read_inputs_interval, read_inputs, NULL, &timer)) {
        printf("Failed to add timer for reading inputs\n");
        return 1;
    }

    while (!loop()) { // main loop
        sleep_ms(loop_interval);
    }
    
    return 0;
}

int init(void)
{
    if (!stdio_init_all()) return -1; // Initializing STDIO

    ds3231_init(DS3231_I2C_PORT, DS3231_I2C_SDA_PIN, DS3231_I2C_SCL_PIN, &rtc); // Initializing I2C for communication with RTC module

    gfx_pack_init(); // initialize display

    // ds3231_datetime_t dt = {
    //     .year = 2024,
    //     .month = 6,
    //     .dotw = 2,
    //     .day = 4,
    //     .hour = 10,
    //     .minutes = 19,
    //     .seconds = 00
    // };
    // ds3231_set_datetime(&dt, &rtc); // refresh datetime

    loop_interval = 1000; // Setting the loop timer
}

int loop(void)
{
    get_datetime(); // Retrieves current datetime & prints it to the console
   
    return 0;
}

void get_datetime(void)
{
    ds3231_datetime_t dt;
    uint8_t dt_str[30];

    ds3231_get_datetime(&dt, &rtc); // read datetime
    ds3231_datetime2str(dt_str, sizeof(dt_str), &dt); // convert datetime to string
    printf("[%s] Hello World!\n", dt_str); // print temporary message containing time and Hello World! string
}

int ds3231_datetime2str(char *buf, uint8_t buf_size, const ds3231_datetime_t *dt)
{
    return snprintf(buf, buf_size, "%04u-%02u-%02u %02u:%02u:%02u", dt->year, dt->month, dt->day, dt->hour, dt->minutes, dt->seconds); // Conversion of the datetime struct to date time string
}

bool read_inputs(repeating_timer_t *rt)
{
    if (gfx_pack_read_button(GFX_PACK_BUTTON_A)) 
    {
        if ((buttons_prev_state & (0b1 << 0)) == 0)
        {
            gfx_pack_set_backlight(0);
            buttons_prev_state |= (0b1 << 0);
        }
        else
        {
            buttons_prev_state &= ~(0b1 << 0);
        }
    }
    else
    {
        gfx_pack_set_backlight(blight_brightness);
    }

    if (gfx_pack_read_button(GFX_PACK_BUTTON_B)) 
    {
        if ((buttons_prev_state & (0b1 << 1)) == 0)
        {
            if (blight_brightness < 251) blight_brightness += 5;
            buttons_prev_state |= (0b1 << 1);
        }
        else 
        {
            buttons_prev_state &= ~(0b1 << 1);
        }
    }
    else
    {
        
    }

    if (gfx_pack_read_button(GFX_PACK_BUTTON_C)) 
    {
        if ((buttons_prev_state & (0b1 << 2)) == 0)
        {
            if (blight_brightness > 5) blight_brightness -= 5;
            buttons_prev_state |= (0b1 << 2);
        }
        else 
        {
            buttons_prev_state &= ~(0b1 << 2);
        }
    }
    else
    {

    }

    if (gfx_pack_read_button(GFX_PACK_BUTTON_D) && ((buttons_prev_state & (0b1 << 3)) == 0)) 
    {
        if ((buttons_prev_state & (0b1 << 3)) == 0)
        {
            buttons_prev_state |= (0b1 << 3);
        }
        else 
        {
            buttons_prev_state &= ~(0b1 << 3);
        }
    }
    else
    {

    }

    if (gfx_pack_read_button(GFX_PACK_BUTTON_E) && ((buttons_prev_state & (0b1 << 4)) == 0)) 
    {
        if ((buttons_prev_state & (0b1 << 4)) == 0)
        {
            buttons_prev_state |= (0b1 << 4);
        }
        else 
        {
            buttons_prev_state &= ~(0b1 << 4);
        }
    }
    else
    {
        
    }

    return true;
}