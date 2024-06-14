#include "main.h"

#define DS3231_I2C_PORT i2c1
#define DS3231_I2C_SDA_PIN 6
#define DS3231_I2C_SCL_PIN 7

#define I2C_SCL 5
#define I2C_SDA 4
#define I2C_DEFAULT i2c0
#define I2C_BAUDRATE 40000

#define CONNECTED_SENSORS 5

// structure containing info about the RTC module
struct ds3231_rtc rtc;

// string holding the datetime value
uint8_t datetime_str[30] = {0}; // MUTEX

// structure containing sensor readings & info
sensors_t sensor_readings; // MUTEX

// GFX Pack previous button state
uint8_t buttons_prev_state = 0;

// gfx_pack backlight brightness
uint8_t blight_brightness = 255;

// gfx_pack backlight on
bool blight_on = true;

// Should the display buffer be updated with new data
bool update_display_buffer;

// value representing the interval between display draws in ms
uint32_t display_interval = 33;

// value representing the interval between sensor readings in ms
uint32_t sensor_read_interval_ms = 15000;

// Sensor I2C actual baudrate
uint32_t i2c_baud;

// INdex of currently displayed sensor
uint8_t display_sensor = 0;

// Time value to check if display & button update should be performed
absolute_time_t process_update_time;

// Time value to check if measurement should start
absolute_time_t sensor_start_measurement_time;

// Vector of timers that have reached time; on reached time bit goes high
uint8_t sensor_timer_vector; 

// Vector of measurements, bit is high if measurement running
uint8_t sensor_measurement_vector; 


/**
 * @brief Sets RTC's datetime, modify datetime inside
 * 
 */
void set_datetime(void);

int main()
{
    int32_t ret;
    if ((ret = init()) != 0) // init function
    {
        printf("Init error: %i\n", ret);
        return ret;
    } 

    while (true) {
        sleep_ms(1);
        if ((ret = loop()) != 0) // main loop
        {
            printf("Loop error: %i\n", ret);
            return ret;
        } 
    }
    return SUCCESS;
}

int init(void)
{
    int32_t ret;
    if (!stdio_init_all()) return ERROR_STDIO_INIT; // Initializing STDIO
    ds3231_init(DS3231_I2C_PORT, DS3231_I2C_SDA_PIN, DS3231_I2C_SCL_PIN, &rtc); // Initializing I2C for communication with RTC module

    gfx_pack_init(); // initialize display
    init_sensor_i2c(); // Initialize I2C for sensor communication

    init_sensors(); // initialize sensors
        
    process_update_time = make_timeout_time_ms(display_interval); // Set display & input checking interval
    sensor_start_measurement_time = make_timeout_time_ms(sensor_read_interval_ms); // Set measurement interval

    sensor_timer_vector = 0; // No sensor individual timer is running
    sensor_measurement_vector = 0; // No sensor is measuring

    update_display_buffer = true; // Redraw display
    sleep_ms(1000); // Init wait
    return SUCCESS;
}

int loop(void)
{
    sensor_timer_vector_update(); // Update timer vector

    if (time_reached(sensor_start_measurement_time)) read_sensors_start(); // Start measurement
    if (time_reached(process_update_time)) update(); // Update display & buttons
    if (sensor_timer_vector) read_sensors(); // Read sensors if time of any sensor timer reached
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
    ds3231_datetime_t dt;

    ds3231_get_datetime(&dt, &rtc); // read datetime
    ds3231_datetime2str(datetime_str, datetime_len, &dt); // convert datetime to string
}

int ds3231_datetime2str(char *buf, uint8_t buf_size, const ds3231_datetime_t *dt)
{
    return snprintf(buf, buf_size, "%04u-%02u-%02u %02u:%02u:%02u", dt->year, dt->month, dt->day, 
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
        update_display_buffer = false;
    }
    process_update_time = make_timeout_time_ms(display_interval);
    return;
}

void update_RTC()
{
    uint8_t loc_datetime_str[30] = {0};
    get_datetime(loc_datetime_str, sizeof(loc_datetime_str)); // Retrieves current datetime
    if (memcmp(loc_datetime_str, datetime_str, 30) != 0)  // If new datetime string
    {
        memcpy(datetime_str, loc_datetime_str, 30); // Update datetime string
        update_display_buffer = true;
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

void write_display(void)
{
    gfx_pack_clear_display(); // Clear display
    point_t position = {.x = 0, .y = 0}; // position of datetime string on display
    gfx_pack_write_text(&position, (char*)datetime_str); // write datetime string to display
    switch (display_sensor)
    {
        case 0: 
        {
            write_display_sensor("E+E EE895", sensor_readings.ee895.state, 
                                    true, sensor_readings.ee895.co2, 
                                    true, sensor_readings.ee895.temperature, 
                                    true, sensor_readings.ee895.pressure, false, 0.0f); // Write ee895 readings to the display
            break; 
        }
        case 1:
        {
            write_display_sensor("FIGARO CDM7162", sensor_readings.cdm7162.state,
                                    true, (float)sensor_readings.cdm7162.co2,
                                    false, 0.0f, false, 0.0f, false, 0.0f); // Write CDM7162 readings to the display
            break;
        }
        case 2: 
        {
            write_display_sensor("Senseair SUNRISE", sensor_readings.sunrise.state,
                                    true, (float)sensor_readings.sunrise.co2,
                                    true, sensor_readings.sunrise.temperature,
                                    false, 0.0f, false, 0.0f); // Write SUNRISE readings to the display
            break;
        }
        case 3:
        {
            write_display_sensor("Senseair SUNLIGHT", sensor_readings.sunlight.state,
                                    true, (float)sensor_readings.sunlight.co2,
                                    true, sensor_readings.sunlight.temperature,
                                    false, 0.0f, false, 0.0f); // Write SUNLIGHT readings to the display
            break;
        }
        case 4:
        {
            write_display_sensor("Sensirion SCD30", sensor_readings.scd30.state,
                                    true, sensor_readings.scd30.co2,
                                    true, sensor_readings.scd30.temperature,
                                    false, 0.0f,
                                    true, sensor_readings.scd30.humidity); // Write SCD30 readings to the display
            break;
        }
        default: 
        {
            position.x = 0;
            position.y = 1;
            uint8_t buf[18];
            memset(buf, 0x00, 18);
            snprintf(buf, 18, "ERR_NO_SENSOR %i", display_sensor); // Write error no sensor to display
            gfx_pack_write_text(&position, buf);
            break;
        }
    }
}

void update_display(void)
{
    gfx_pack_update(); // Update display
    return;
}

void init_sensors(void)
{
    int32_t ret;

    ee895_init_struct(&(sensor_readings.ee895)); // Initialize sensor structures
    cdm7162_init_struct(&(sensor_readings.cdm7162));
    sunrise_init_struct(&(sensor_readings.sunrise));
    sunlight_init_struct(&(sensor_readings.sunlight));
    scd30_init_struct(&(sensor_readings.scd30));

    set_power_mode();

    // ret = ee895_init(&(sensor_readings.ee895), &sensor_ee895_config); // Initialize EE895 sensor
    // sensor_readings.ee895.state = (ret != 0) ? ERROR_SENSOR_INIT_FAILED : ERROR_NO_MEAS;

    // ret = cdm7162_init(&(sensor_readings.cdm7162), &sensor_cdm7162_config); // Initialize CDM7162 sensor
    // sensor_readings.cdm7162.state = ret != 0 ? ERROR_SENSOR_INIT_FAILED : ERROR_NO_MEAS;

    // ret = sunrise_init(&(sensor_readings.sunrise), &sensor_sunrise_config); // Initialize SUNRISE sensor
    // sensor_readings.sunrise.state = ret != 0 ? ERROR_SENSOR_INIT_FAILED : ERROR_NO_MEAS;

    // ret = sunlight_init(&(sensor_readings.sunlight), &sensor_sunlight_config); // Initialize SUNLIGHT sensor
    // sensor_readings.sunlight.state = ret != 0 ? ERROR_SENSOR_INIT_FAILED : ERROR_NO_MEAS;

    ret = scd30_init(&(sensor_readings.scd30), &sensor_scd30_config); // Initialize SCD30 sensor
    sensor_readings.scd30.state = ret != 0 ? ERROR_SENSOR_INIT_FAILED : ERROR_NO_MEAS;
}

void init_sensor_i2c(void)
{
    gpio_init(I2C_SDA); // Initialize data pin
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);

    gpio_init(I2C_SCL); // Initialize clock pin
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SCL);

    i2c_baud = i2c_init(I2C_DEFAULT, I2C_BAUDRATE); // Initialize I2C
}

void set_power_mode(void)
{
    sensor_ee895_config.power_global_control = global_power; // Set sensor power mode to global power control
    sensor_cdm7162_config.power_global_control = global_power;
    sensor_sunrise_config.power_global_control = global_power;
    sensor_sunlight_config.power_global_control = global_power;
    sensor_scd30_config.power_global_control = global_power;
}

void set_power(bool on)
{
    if (global_power)
    {
        // Set power [on]
    }
}

void read_sensors_start()
{
    if (!sensor_measurement_vector) // Measurement initialization if no sensor is measuring
    {
        sensor_readings.ee895.meas_state = EE895_MEAS_START; // Start measurement .. ToDo: add more sensors
        sensor_readings.cdm7162.meas_state = CDM7162_MEAS_START; // Start CDM7162 measurement
        sensor_readings.sunrise.meas_state = SUNRISE_MEAS_START; // Start SUNRISE measurement
        sensor_readings.sunlight.meas_state = SUNLIGHT_MEAS_START; // Start SUNLIGHT measurement
        sensor_readings.scd30.meas_state = SCD30_MEAS_START; // Start SCD30 measurement
        
        sensor_start_measurement_time = make_timeout_time_ms(sensor_read_interval_ms); // Set another mesurement start in sensor_read_interval_ms time
        sensor_timer_vector |= ~(0b0); // Set timer vector so all sensors will start measurement
        sensor_measurement_vector |= ~(0b0); // Set measurement vector to all sensors measuring

        set_power(true);
    }
    else 
    {
        sensor_start_measurement_time = make_timeout_time_ms(100); // if cannot process - check after 100 ms
    }
}

void sensor_timer_vector_update(void)
{
    if (time_reached(sensor_readings.ee895.wake_time)) // If EE895 timer reached
    {
        sensor_timer_vector |= (0b1 << 0);
    }
    if (time_reached(sensor_readings.cdm7162.wake_time)) // If CDM7162 timer reached
    {
        sensor_timer_vector |= (0b1 << 1);
    }
    if (time_reached(sensor_readings.sunrise.wake_time)) // If SUNRISE timer reached
    {
        sensor_timer_vector |= (0b1 << 2);
    }
    if (time_reached(sensor_readings.sunlight.wake_time)) // If SUNLIGHT timer reached
    {
        sensor_timer_vector |= (0b1 << 3);
    }
    if (time_reached(sensor_readings.scd30.wake_time)) // If SCD30 timer reached
    {
        sensor_timer_vector |= (0b1 << 4);
    }
}

void read_sensors()
{
    int32_t ret;

    // if (sensor_timer_vector & (0b1 << 0)) // If EE895 should react to a timer reached
    // {
    //     sensor_timer_vector &= ~(0b1 << 0); // clear ee895 timer reached bit
    //     if (sensor_readings.ee895.state != ERROR_SENSOR_INIT_FAILED) // If sensor initialized
    //     {
    //         ee895_get_value(&sensor_readings.ee895); // Read EE895 values
    //     }
    //     else // Try initializing the sensor
    //     {
    //         ret = ee895_init(&(sensor_readings.ee895), &sensor_ee895_config); // Initialize EE895 sensor
    //         sensor_readings.ee895.state = ret != 0 ? ERROR_SENSOR_INIT_FAILED : ERROR_NO_MEAS;
    //     }
    //     if (sensor_readings.ee895.meas_state == EE895_MEAS_FINISHED) sensor_measurement_vector &= ~(0b1 << 0); // If measurement completed clear sensor measurement bit
    // }

    // if (sensor_timer_vector & (0b1 << 1)) // If cdm7162 should react to a timer reached
    // {
    //     sensor_timer_vector &= ~(0b1 << 1); // clear cdm7162 timer reached bit
    //     if (sensor_readings.cdm7162.state != ERROR_SENSOR_INIT_FAILED) // If sensor initialized
    //     {
    //         cdm7162_get_value(&sensor_readings.cdm7162); // Read CDM7162 values
    //     }
    //     else // Try initializing the sensor
    //     {
    //         ret = cdm7162_init(&(sensor_readings.cdm7162), &sensor_cdm7162_config); // Initialize CDM7162 sensor
    //         sensor_readings.cdm7162.state = ret != 0 ? ERROR_SENSOR_INIT_FAILED : ERROR_NO_MEAS;
    //     }
    //     if (sensor_readings.cdm7162.meas_state == CDM7162_MEAS_FINISHED) sensor_measurement_vector &= ~(0b1 << 1); // If measurement completed clear sensor measurement bit
    // }

    // if (sensor_timer_vector & (0b1 << 2)) // If SUNRISE should react to a timer reached
    // {
    //     sensor_timer_vector &= ~(0b1 << 2); // clear SUNRISE timer reached bit
    //     if (sensor_readings.sunrise.state != ERROR_SENSOR_INIT_FAILED) // If sensor initialized
    //     {
    //         sunrise_get_value(&sensor_readings.sunrise); // Read SUNRISE values
    //     }
    //     else // Try initializing the sensor
    //     {
    //         ret = sunrise_init(&(sensor_readings.sunrise), &sensor_sunrise_config); // Initialize SUNRISE sensor
    //         sensor_readings.sunrise.state = ret != 0 ? ERROR_SENSOR_INIT_FAILED : ERROR_NO_MEAS;
    //     }
    //     if (sensor_readings.sunrise.meas_state == SUNRISE_MEAS_FINISHED) sensor_measurement_vector &= ~(0b1 << 2); // If measurement completed clear sensor measurement bit
    // }

    // if (sensor_timer_vector & (0b1 << 3)) // If SUNLIGHT should react to a timer reached
    // {
    //     sensor_timer_vector &= ~(0b1 << 3); // clear SUNLIGHT timer reached bit
    //     if (sensor_readings.sunlight.state != ERROR_SENSOR_INIT_FAILED) // If sensor initialized
    //     {
    //         sunlight_get_value(&sensor_readings.sunlight); // Read SUNLIGHT values
    //     }
    //     else // Try initializing the sensor
    //     {
    //         ret = sunlight_init(&(sensor_readings.sunlight), &sensor_sunlight_config); // Initialize SUNLIGHT sensor
    //         sensor_readings.sunlight.state = ret != 0 ? ERROR_SENSOR_INIT_FAILED : ERROR_NO_MEAS;
    //     }
    //     if (sensor_readings.sunlight.meas_state == SUNLIGHT_MEAS_FINISHED) sensor_measurement_vector &= ~(0b1 << 3); // If measurement completed clear sensor measurement bit
    // }

    if (sensor_timer_vector & (0b1 << 4)) // If SCD30 should react to a timer reached
    {
        sensor_timer_vector &= ~(0b1 << 4); // clear SCD30 timer reached bit
        if (sensor_readings.scd30.state != ERROR_SENSOR_INIT_FAILED) // If sensor initialized
        {
            scd30_get_value(&sensor_readings.scd30); // Read SCD30 values
        }
        else // Try initializing the sensor
        {
            ret = scd30_init(&(sensor_readings.scd30), &sensor_scd30_config); // Initialize SCD30 sensor
            sensor_readings.scd30.state = ret != 0 ? ERROR_SENSOR_INIT_FAILED : ERROR_NO_MEAS;
        }
        if (sensor_readings.scd30.meas_state == SCD30_MEAS_FINISHED) sensor_measurement_vector &= ~(0b1 << 4); // If measurement completed clear sensor measurement bit
    }

    if (true) // Must set corresponding bit for the currently active sensor
    {
        sensor_measurement_vector &= (0b10000 << 0); // All other measurements finished - temporary
        sensor_timer_vector &= (0b10000 << 0); // All other timers are not reached - temporary
    }

    if (!sensor_measurement_vector) // If all measurements finished - turn off power globally if possible
    {
        set_power(false);
    }

    update_display_buffer = true; // Update display
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
            snprintf(buf, 16, "T: %4.2f |C", temp_value); // C char does not support ° symbol
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