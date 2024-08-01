/**
 * @file main.c
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief PICO and peripherals control implementation
 * @version 0.1
 * @date 2024-06-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "main.h"
#include "common/debug.h"
#include "malloc.h"
#include "pico/printf.h"
#include "hardware/watchdog.h"

// string holding the datetime value
uint8_t datetime_str[30] = {0};

// GFX Pack previous button state
uint8_t buttons_prev_state = 0;

// gfx_pack backlight brightness
uint8_t blight_brightness = 255;

// gfx_pack backlight on
bool blight_on = true;

// Should the display buffer be updated with new data
bool update_display_buffer;

// Sensor I2C actual baudrate
uint32_t i2c_baud;

// INdex of currently displayed sensor
uint8_t display_sensor = 0;

// Time value to check if display & button update should be performed
absolute_time_t process_update_time;

// Number of total SOAP channels
uint8_t soap_channels = 16;

// SOAP message buffer
uint8_t soap_buffer1[MAX_SOAP_SIZE] = {0};
uint8_t soap_buffer2[MAX_SOAP_SIZE] = {0};

absolute_time_t memory_timer;


/**
 * @brief Sets RTC's datetime, modify datetime inside
 * 
 */
void set_datetime(void);

void getFreeHeap(void);

int main()
{
    int32_t ret;
    if ((ret = init()) != 0) // init function
    {
        print_ser_output(SEVERITY_FATAL, SOURCE_MAIN_INIT, SOURCE_NO_SOURCE, "Initialization failure: %i; Aborting...", ret);
        return ret;
    } 

    while (true) {
        tight_loop_contents();
        if ((ret = loop()) != 0) // main loop
        {
            print_ser_output(SEVERITY_FATAL, SOURCE_MAIN_LOOP, SOURCE_NO_SOURCE, "Loop failure: %i; Aborting...", ret);
            return ret;
        }
    }
    return SUCCESS;
}

void core1_main(void)
{
    wifi_main();
}

int init(void)
{
    int32_t ret;
    if (!stdio_init_all()) return ERROR_STDIO_INIT; // Initializing STDIO

    if (watchdog_enable_caused_reboot()) print_ser_output(SEVERITY_FATAL, SOURCE_MAIN_INIT, SOURCE_NO_SOURCE, "Reboot caused by watchdog");
    else print_ser_output(SEVERITY_INFO, SOURCE_MAIN_INIT, SOURCE_NO_SOURCE, "Booting clean");

    if (!mutex_is_initialized(&soap_data1.data_mutex)) // Initializes mutexes for soap message buffer
        mutex_init(&soap_data1.data_mutex);

    multicore_launch_core1(core1_main); // Launch second core

    ds3231_init(I2C_DEVICE, I2C_DEVICE_SDA, I2C_DEVICE_SCL, &rtc); // Initializing I2C for communication with RTC module

    gfx_pack_init(); // initialize display

    sensors_init_all(configuration_map, 8); // initialize sensors

    assign_soap_channels(); // Assign SOAP channels
    soap_init(sensors, channels1, channels1_len); // Initialize SOAP channels 1
    soap_init(sensors, channels2, channels2_len); // Initialize SOAP channels 2

    process_update_time = make_timeout_time_ms(display_interval); // Set display & input checking interval

    memory_timer = make_timeout_time_ms(1000);

    update_display_buffer = true; // Redraw display
    sleep_ms(1000); // Init wait
    watchdog_enable(3000, true); // 3 sec watchdog
    return SUCCESS;
}

int loop(void)
{
    if (!service_mode)
    {
        sensors_read_all(); // Read sensor values
        create_soap_messages(); // Create SOAP messages
    }
    if (config_data.command_rdy)
    {
        service_comm_eng_process_command();
    }
    if (time_reached(process_update_time)) update(); // Update display & buttons
    if (time_reached(memory_timer)) getFreeHeap(); // Check free heap
    watchdog_update();
    return SUCCESS;
}

void set_datetime(void)
{
    ds3231_datetime_t dt = {
        .year = 2024,
        .month = 6,
        .dotw = 5,
        .day = 14,
        .hour = 6,
        .minutes = 56,
        .seconds = 00
    };
    ds3231_set_datetime(&dt, &rtc); // refresh datetime
}

void get_datetime(uint8_t* datetime_str, uint8_t datetime_len)
{
    ds3231_get_datetime(&dt, &rtc); // read datetime
    datetime2str(datetime_str, datetime_len, &dt); // convert datetime to string
}

int datetime2str(char *buf, uint8_t buf_size, const ds3231_datetime_t *dt)
{
    return snprintf(buf, buf_size, "%02u.%02u.%04u %02u:%02u:%02u", dt->day, dt->month, dt->year, 
                    dt->hour, dt->minutes, dt->seconds); // Conversion of the datetime struct to date time string
}

void update()
{
    update_RTC(); // Updates datetime
    read_inputs(); // Updates button inputs
    if (update_display_buffer)
    {
        write_display(); // Writes data to be displayed to display frame buffer
        update_display(); // Updates display
        #if !DEBUG_TIME
        update_display_buffer = false; // Wait with next display update
        #endif
    }
    process_update_time = make_timeout_time_us(display_interval * 1000); // Wait for next update
    return;
}

void update_RTC()
{
    uint8_t loc_datetime_str[30] = {0};
    get_datetime(loc_datetime_str, sizeof(loc_datetime_str)); // Retrieves current datetime
    if (memcmp(loc_datetime_str, datetime_str, 30) != 0)  // If new datetime string
    {
        memcpy(datetime_str, loc_datetime_str, 30); // Update datetime string
        update_display_buffer = true; // Refresh display
    }
}

void read_inputs(void)
{
    if (gfx_pack_read_button(GFX_PACK_BUTTON_A)) // Button A down
    {
        if ((buttons_prev_state & (0b1 << 0)) == 0) // Button A pressed - single action
        {
            // blight_on = !blight_on; // Turn backlight off
            // gfx_pack_set_backlight(blight_brightness * blight_on); // set display backlight brightness
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
        gfx_pack_set_backlight(blight_brightness * blight_on); // set display backlight brightness
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
        gfx_pack_set_backlight(blight_brightness * blight_on); // set display backlight brightness
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
            if (--display_sensor >= CONNECTED_SENSORS) display_sensor = CONNECTED_SENSORS - 1;
            update_display_buffer = true;
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
            if (++display_sensor >= CONNECTED_SENSORS) display_sensor = 0;
            update_display_buffer = true;
        }
        // Button E down - repeat action
    }
    else // Button E up
    {
        buttons_prev_state &= ~(0b1 << 4);
    }

    return;
}

void get_sensor_name_string(sensor_t* sensor, uint8_t* buf, uint8_t len)
{
    if (len < 24) return;
    switch(sensor->sensor_type) // On sensor type generate string
    {
        case EE895:
        {
            snprintf(buf, len, "E+E EE895");
            break;
        }
        case CDM7162:
        {
            snprintf(buf, len, "Figaro CDM7162");
            break;
        }
        case SUNRISE:
        {
            snprintf(buf, len, "Senseair SUNRISE");
            break;
        }
        case SUNLIGHT:
        {
            snprintf(buf, len, "Senseair SUNLIGHT");
            break;
        }
        case SCD30:
        {
            snprintf(buf, len, "Sensirion SCD30");
            break;
        }
        case SCD41:
        {
            snprintf(buf, len, "Sensirion SCD41");
            break;
        }
        case COZIR_LP3:
        {
            snprintf(buf, len, "GSS CozIR-LP3");
            break;
        }
        case CM1107N:
        {
            snprintf(buf, len, "Cubic CM1107N");
            break;
        }
        default:
        {
            memset(buf, 0x00, len);
            break;
        }
    }
    return;
}

void write_display(void)
{
    gfx_pack_clear_display(); // Clear display
    point_t position = {.x = 0, .y = 0}; // position of datetime string on display
    gfx_pack_write_text(&position, (char*)datetime_str); // write datetime string to display
    uint8_t sensor_name[24];
    memset(sensor_name, 0x00, 24);

    get_sensor_name_string(&(sensors[display_sensor]), sensor_name, 24); // Get string name of the sensor
    
    if (strncmp(sensor_name, "\0", 1) == 0) // If no sensor found
    {
        position.x = 0;
        position.y = 1;
        snprintf(sensor_name, 18, "ERR_NO_SENSOR %i", display_sensor); // Write error no sensor to display
        gfx_pack_write_text(&position, sensor_name);
    }
    else write_display_sensor(sensor_name, sensors[display_sensor].state, 
            sensors[display_sensor].config.co2_en, sensors[display_sensor].co2,
            sensors[display_sensor].config.temp_en, sensors[display_sensor].temperature,
            sensors[display_sensor].config.pressure_en, sensors[display_sensor].pressure,
            sensors[display_sensor].config.RH_en, sensors[display_sensor].humidity); // Write sensor readings to the display
    #if DEBUG_TIME
    uint8_t hwtime[21];
    snprintf(hwtime, 21, "Time: %llu ms", to_us_since_boot(get_absolute_time()) / 1000); // Prepare time since boot string
    position.x = 21 - strlen(hwtime);
    position.y = 5;
    gfx_pack_write_text(&position, hwtime);
    #endif
}

void update_display(void)
{
    gfx_pack_update(); // Update display
    return;
}

void assign_soap_channels(void)
{
    for (int i = 0; i < soap_channels; i++) // Assign channels from channel map
    {
        if (i >= 32) break; // On channel overflow 32
        else if (i >= 16) channels2[i - 16] = channel_map2[i - 16]; // On channel overflow 16
        else channels1[i] = channel_map1[i]; // Assign channel
    }
    for (int i = soap_channels; i < 32; i++) // for channels from number of channels to 32
    {
        if (i < 16) channels1[i] = NULL; // Fill unused channels with NULL
        else channels2[i - 16] = NULL;
    }
    channels1_len = soap_channels > 16 ? 16 : soap_channels; // Number of channels in 1st buffer
    channels2_len = soap_channels < 16 ? 0 : (soap_channels > 32 ? 16 : (soap_channels % 16)); // Number of channels in 2nd buffer
}

void create_soap_messages(void)
{
    if (!sensors_measurement_ready || sensors_was_measurement_read) return; // Generate new message if new data available
    if (!soap_build(SOAP_TESTER_NAME_1, SOAP_TESTER_SN_1, datetime_str, channels1, channels1_len)) // Create SOAP message
        print_ser_output(SEVERITY_ERROR, SOURCE_SOAP, SOURCE_NO_SOURCE, "Failed to generate SOAP message");
    else print_ser_output(SEVERITY_INFO, SOURCE_SOAP, SOURCE_NO_SOURCE, "Generated SOAP message");
    sensors_was_measurement_read = true;
    return;
}

void write_display_sensor(uint8_t* sensor_name, int state, 
        bool co2, float co2_value, 
        bool temp, float temp_value, 
        bool pressure, float pressure_value,
        bool humidity, float humidity_value)
{
    uint8_t row = 1;
    point_t position;
    position.x = 0;
    position.y = row;
    gfx_pack_write_text(&position, sensor_name); // Write sensor name
    
    row++;
    position.x = 0;
    position.y = row;

    if (state != 0) // If sensor in invalid state
    {
        uint8_t buf[6];
        
        snprintf(buf, 6, "E%i", state);
        gfx_pack_write_text(&position, buf); // Write error code (sensor state)
        row++;
        position.x = 0;
        position.y = row;
    }
    else
    {
        uint8_t buf[16];
        memset(buf, 0x00, 16);

        if (co2)
        {
            snprintf(buf, 16, "CO2: %.0f ppm", co2_value);
            gfx_pack_write_text(&position, buf); // Write co2 concentration
            row++;
            position.x = 0;
            position.y = row;
            memset(buf, 0x00, 16);
        }
        if (temp)
        {
            snprintf(buf, 16, "T: %4.2f %cC", temp_value, 0xB0); // C char does not support ° symbol
            gfx_pack_write_text(&position, buf); // Write temperature
            row++;
            position.x = 0;
            position.y = row;
            memset(buf, 0x00, 16);

        }
        if (pressure)
        {
            snprintf(buf, 16, "p: %4.1f hPa", pressure_value);
            gfx_pack_write_text(&position, buf); // Write pressure
            row++;
            position.x = 0;
            position.y = row;
            memset(buf, 0x00, 16);
        }
        if (humidity)
        {
            snprintf(buf, 16, "RH: %4.1f %%", humidity_value);
            gfx_pack_write_text(&position, buf); // Write humidity
            row++;
            position.x = 0;
            position.y = row;
            memset(buf, 0x00, 16);
        }
    }
}

uint32_t getTotalHeap(void) {
   extern char __StackLimit, __bss_end__;
   
   return &__StackLimit  - &__bss_end__;
}

void getFreeHeap(void) {
    struct mallinfo m = mallinfo();
    print_ser_output(SEVERITY_WARN, SOURCE_MAIN_LOOP, SOURCE_RAM, "Available memory: %d bytes", getTotalHeap() - m.uordblks);
    memory_timer = make_timeout_time_ms(1000);
    return;
}