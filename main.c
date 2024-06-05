#include "main.h"

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

// GFX Pack previous button state
uint8_t buttons_prev_state = 0;

// value representing the interval between loop calls in ms
uint32_t loop_interval = 1000;

// value representing the interval between button input reads in ms
uint32_t read_inputs_interval = 50;

// gfx_pack backlight brightness
uint8_t blight_brightness = 255;

/**
 * @brief Sets RTC's datetime, modify datetime inside
 * 
 */
void set_datetime(void);


int main()
{
    init(); // init function
    sleep_ms(1000);

    repeating_timer_t timerInput;

    // Initialize timer to read inputs
    if (!add_repeating_timer_ms(-read_inputs_interval, read_inputs, NULL, &timerInput)) { // negative timeout means exact delay (rather than delay between callbacks)
        printf("Failed to add timer for reading inputs\n");
        return -2;
    }

    // repeating_timer_t timer_display;
    // uint8_t display_freq = 30;
    // if (!add_repeating_timer_ms(-1000 / (float)display_freq, update_display, NULL, &timer_display)) { // negative timeout means exact delay (rather than delay between callbacks)
    //     printf("Failed to add timer for display\n");
    //     return -3;
    // }

    while (true) { // main loop
        loop();
        sleep_ms(loop_interval);
    }
    
    return 0;
}

int init(void)
{
    if (!stdio_init_all()) return -1; // Initializing STDIO

    ds3231_init(DS3231_I2C_PORT, DS3231_I2C_SDA_PIN, DS3231_I2C_SCL_PIN, &rtc); // Initializing I2C for communication with RTC module
    gfx_pack_init(); // initialize display

    loop_interval = 1000; // Setting the loop timer
}

int loop(void)
{
    uint8_t datetime_str[30]; // string holding the datetime value
    get_datetime(datetime_str, sizeof(datetime_str)); // Retrieves current datetime & prints it to the console
    update_display();
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
    printf("[%s] Hello World!\n", datetime_str); // print temporary message containing time and Hello World! string
}

int ds3231_datetime2str(char *buf, uint8_t buf_size, const ds3231_datetime_t *dt)
{
    return snprintf(buf, buf_size, "%04u-%02u-%02u %02u:%02u:%02u", dt->year, dt->month, dt->day, dt->hour, dt->minutes, dt->seconds); // Conversion of the datetime struct to date time string
}

bool read_inputs(repeating_timer_t *rt)
{
    if (gfx_pack_read_button(GFX_PACK_BUTTON_A)) // Button A down
    {
        if ((buttons_prev_state & (0b1 << 0)) == 0) // Button A pressed - single action
        {
            gfx_pack_set_backlight(0); // Turn backlight off
            buttons_prev_state |= (0b1 << 0);
        }
        // Button A down - repeat action
    }
    else // Button A up
    {
        gfx_pack_set_backlight(blight_brightness); // Turn backlight on
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
        }
        // Button E down - repeat action
    }
    else // Button E up
    {
        buttons_prev_state &= ~(0b1 << 4);
    }

    return true;
}

bool update_display()
{
    printf("display\n");
    gfx_pack_update();
    return true;
}

