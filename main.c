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

#define msg(severity, source, message) printf("[%12llu] ["severity"] [MAIN-"source"] "message"\n", to_us_since_boot(get_absolute_time()) / 1000)
#define msgbuf(severity, source, message) printf("[%12llu] ["severity"] [MAIN-"source"] %s\n", to_us_since_boot(get_absolute_time()) / 1000, message)
#define msgbufstr(severity, source, message) printf("[%12llu] ["severity"] [MAIN-%s] "message"\n", to_us_since_boot(get_absolute_time()) / 1000, source)

// structure containing info about the RTC module
struct ds3231_rtc rtc;

// string holding the datetime value
uint8_t datetime_str[30] = {0}; // MUTEX

// array of sensors
sensor_t sensors[8];

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

// Time value to check if measurement should start
absolute_time_t sensor_start_measurement_time;

// Vector of timers that have reached time; on reached time bit goes high
uint8_t sensor_timer_vector; 

// Vector of measurements, bit is high if measurement running
uint8_t sensor_measurement_vector; 

// Vector of active sensors
uint8_t active_sensors = 0b00000001;


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
    datetime2str(datetime_str, datetime_len, &dt); // convert datetime to string
}

int datetime2str(char *buf, uint8_t buf_size, const ds3231_datetime_t *dt)
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
            sensors[display_sensor].config->co2_en, sensors[display_sensor].co2,
            sensors[display_sensor].config->temp_en, sensors[display_sensor].temperature,
            sensors[display_sensor].config->pressure_en, sensors[display_sensor].pressure,
            sensors[display_sensor].config->RH_en, sensors[display_sensor].humidity); // Write sensor readings to the display
}

void update_display(void)
{
    gfx_pack_update(); // Update display
    return;
}

void init_sensors(void)
{
    int32_t ret;

    for (uint8_t i = 0; i < 8; i++)
    {
        #if DEBUG_INFO
        uint8_t buf3[18];
        snprintf(buf3, 18, "Init structure %i", i);
        msgbuf("info", "SENSOR", buf3);
        #endif
        common_init_struct(&sensors[i], i); // Initialize sensor structures
        if (configuration_map[i] != NULL) sensors[i].sensor_type = configuration_map[i]->sensor_type; // Copy sensor type to sensor structure
        else sensors[i].sensor_type = UNKNOWN;
    }


    for (int i = 0; i < 8; i++)
    {
        if (active_sensors & (0b1 << i))
        {
            reset_i2c();
            switch (configuration_map[i]->sensor_type)
            {
                case EE895:
                {
                    #if DEBUG_INFO
                    uint8_t buf[24];
                    snprintf(buf, 24, "Init sensor %i...", i);
                    msgbuf("info", "EE895", buf);
                    #endif
                    ret = ee895_init(&(sensors[i]), configuration_map[i]); // Initialize EE895 sensor
                    break;
                }
                case CDM7162:
                {
                    #if DEBUG_INFO
                    uint8_t buf[24];
                    snprintf(buf, 24, "Init sensor %i...", i);
                    msgbuf("info", "CDM7162", buf);
                    #endif
                    ret = cdm7162_init(&(sensors[i]), configuration_map[i]); // Initialize CDM7162 sensor
                    break;
                }
                case SUNRISE:
                {
                    #if DEBUG_INFO
                    uint8_t buf[24];
                    snprintf(buf, 24, "Init sensor %i...", i);
                    msgbuf("info", "SUNRISE", buf);
                    #endif
                    ret = sunrise_init(&(sensors[i]), configuration_map[i]); // Initialize SUNRISE sensor
                    break;
                }
                case SUNLIGHT:
                {
                    #if DEBUG_INFO
                    uint8_t buf[24];
                    snprintf(buf, 24, "Init sensor %i...", i);
                    msgbuf("info", "SUNLIGHT", buf);
                    #endif
                    ret = sunlight_init(&(sensors[i]), configuration_map[i]); // Initialize SUNLIGHT sensor
                    break;
                }
                case SCD30:
                {
                    #if DEBUG_INFO
                    uint8_t buf[24];
                    snprintf(buf, 24, "Init sensor %i...", i);
                    msgbuf("info", "SCD30", buf);
                    #endif
                    ret = scd30_init(&(sensors[i]), configuration_map[i]); // Initialize SCD30 sensor
                    break;
                }
                case SCD41:
                {
                    #if DEBUG_INFO
                    uint8_t buf[24];
                    snprintf(buf, 24, "Init sensor %i...", i);
                    msgbuf("info", "SCD41", buf);
                    #endif
                    ret = scd41_init(&(sensors[i]), configuration_map[i]); // Initialize SCD41 sensor
                    break;
                }
                case COZIR_LP3:
                {
                    #if DEBUG_INFO
                    uint8_t buf[24];
                    snprintf(buf, 24, "Init sensor %i...", i);
                    msgbuf("info", "CozIR-LP3", buf);
                    #endif
                    ret = cozir_lp3_init(&(sensors[i]), configuration_map[i]); // Initialize CozIR-LP3 sensor
                    break;
                }
                
                default:
                {
                    #if DEBUG
                    msg("ERROR", "SENSOR", "Unknown sensor, init abort");
                    #endif
                    sensors[i].state = ERROR_UNKNOWN_SENSOR;
                    break;
                }
            }
            if (sensors[i].state == ERROR_UNKNOWN_SENSOR) continue;
            #if DEBUG_INFO
            if (!ret)
            {
                msg("info", "SENSOR", "Init success");
            }
            #endif
            #if DEBUG
            if (ret)
            {
                uint8_t buf3[36];
                snprintf(buf3, 36, "Init sensor %i failed: %i", i, ret);
                msgbuf("ERROR", "SENSOR", buf3);
            }
            #endif
            sensors[i].state = ret != 0 ? ERROR_SENSOR_INIT_FAILED : ERROR_NO_MEAS;
        }
    }

    set_power_mode(); // Set power control mode
}

void reset_i2c(void)
{
    i2c_deinit(I2C_SENSOR);
    sleep_ms(1);
    i2c_init(I2C_SENSOR, I2C_BAUDRATE);
}

void init_sensor_i2c(void)
{
    gpio_init(I2C_SDA); // Initialize data pin
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);

    gpio_init(I2C_SCL); // Initialize clock pin
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SCL);

    i2c_baud = i2c_init(I2C_SENSOR, I2C_BAUDRATE); // Initialize I2C
}

void set_power_mode(void)
{
    for (int i = 0; i < 8; i++)
    {
        if (sensors[i].config == NULL) continue;
        sensors[i].config->power_global_control = global_power; // Set sensor power mode to global power control
    }
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
        #if DEBUG_INFO
        msg("info", "SENSOR", "Measurement start");
        #endif
        for (int i = 0; i < 8; i++)
        {
            if (sensors[i].state == ERROR_SENSOR_NOT_INITIALIZED) continue; // If not initialized sensor (aka disabled) continue
            sensors[i].meas_state = MEAS_STARTED; // Start measurement
        }
        
        sensor_start_measurement_time = make_timeout_time_ms(sensor_read_interval_ms); // Set another mesurement start in sensor_read_interval_ms time
        sensor_timer_vector |= ~(0b0); // Set timer vector so all sensors will start measurement
        sensor_measurement_vector |= ~(0b0); // Set measurement vector to all sensors measuring

        set_power(true);
    }
    else 
    {
        sensor_start_measurement_time = make_timeout_time_ms(100); // if cannot start measurement - check after 100 ms
    }
}

void sensor_timer_vector_update(void)
{
    for (int i = 0; i < 8; i++)
    {
        if (time_reached(sensors[i].wake_time)) // If sensor timer reached
        {
            sensor_timer_vector |= (0b1 << i);
        }
    }
}

void read_sensors()
{
    int32_t ret = -99;
    bool i2c_reset = true;

    for (uint8_t i = 0; i < 8; i++)
    {
        if ((sensor_timer_vector & (0b1 << i)) && (active_sensors & (0b1 << i))) // If sensor should react to a timer reached
        {
            sensor_timer_vector &= ~(0b1 << i); // Clear timer reached bit
            if (sensors[i].state == SUCCESS || sensors[i].state == ERROR_NO_MEAS) // If sensor initialized
            {
                switch (configuration_map[i]->sensor_type)
                {
                    case EE895:
                    {
                        #if DEBUG_INFO
                        uint8_t buf[24];
                        snprintf(buf, 24, "Reading sensor %i...", i);
                        msgbuf("info", "EE895", buf);
                        #endif
                        ee895_get_value(&(sensors[i])); // Read EE895 values
                        break;
                    }
                    case CDM7162:
                    {
                        #if DEBUG_INFO
                        uint8_t buf[24];
                        snprintf(buf, 24, "Reading sensor %i...", i);
                        msgbuf("info", "CDM7162", buf);
                        #endif
                        cdm7162_get_value(&(sensors[i])); // Read CDM7162 values
                        break;
                    }
                    case SUNRISE:
                    {
                        #if DEBUG_INFO
                        uint8_t buf[24];
                        snprintf(buf, 24, "Reading sensor %i...", i);
                        msgbuf("info", "SUNRISE", buf);
                        #endif
                        sunrise_get_value(&(sensors[i])); // Read SUNRISE values
                        break;
                    }
                    case SUNLIGHT:
                    {
                        #if DEBUG_INFO
                        uint8_t buf[24];
                        snprintf(buf, 24, "Reading sensor %i...", i);
                        msgbuf("info", "SUNLIGHT", buf);
                        #endif
                        sunlight_get_value(&(sensors[i])); // Read SUNLIGHT values
                        break;
                    }
                    case SCD30:
                    {
                        #if DEBUG_INFO
                        uint8_t buf[24];
                        snprintf(buf, 24, "Reading sensor %i...", i);
                        msgbuf("info", "SCD30", buf);
                        #endif
                        scd30_get_value(&(sensors[i])); // Read SCD30 values
                        break;
                    }
                    case SCD41:
                    {
                        #if DEBUG_INFO
                        uint8_t buf[24];
                        snprintf(buf, 24, "Reading sensor %i...", i);
                        msgbuf("info", "SCD41", buf);
                        #endif
                        scd41_get_value(&(sensors[i])); // Read SCD41 values
                        break;
                    }
                    case COZIR_LP3:
                    {
                        #if DEBUG_INFO
                        uint8_t buf[24];
                        snprintf(buf, 24, "Reading sensor %i...", i);
                        msgbuf("info", "CozIR-LP3", buf);
                        #endif
                        cozir_lp3_get_value(&(sensors[i])); // Read CozIR-LP3 values
                        break;
                    }
                    
                    default:
                    {
                        #if DEBUG
                        msg("ERROR", "SENSOR", "Reading unknown sensor");
                        #endif
                        sensors[i].state = ERROR_UNKNOWN_SENSOR;
                        break;
                    }
                }
                #if DEBUG
                if (sensors[i].state && sensors[i].state != ERROR_NO_MEAS)
                {
                    uint8_t buf2[36];
                    snprintf(buf2, 36, "Reading sensor %i failed: %i", i, sensors[i].state);
                    msgbuf("ERROR", "SENSOR", buf2);
                }
                #endif
                #if DEBUG_INFO
                if (sensors[i].meas_state == MEAS_FINISHED && sensors[i].state == SUCCESS && !(sensor_measurement_vector & (0b1 << i)))
                {
                    uint8_t buf2[36];
                    snprintf(buf2, 36, "Successfully read sensor %i", i);
                    msgbuf("info", "SENSOR", buf2);
                }
                #endif
            }
            else if (sensors[i].state != ERROR_SENSOR_NOT_INITIALIZED)
            {
                reset_i2c();
                common_init_struct(&sensors[i], i);
                switch (configuration_map[i]->sensor_type)
                {
                    case EE895:
                    {
                        #if DEBUG_INFO
                        uint8_t buf[24];
                        snprintf(buf, 24, "Init sensor %i...", i);
                        msgbuf("info", "EE895", buf);
                        #endif
                        ret = ee895_init(&(sensors[i]), configuration_map[i]); // Initialize EE895 sensor
                        break;
                    }
                    case CDM7162:
                    {
                        #if DEBUG_INFO
                        uint8_t buf[24];
                        snprintf(buf, 24, "Init sensor %i...", i);
                        msgbuf("info", "CDM7162", buf);
                        #endif
                        ret = cdm7162_init(&(sensors[i]), configuration_map[i]); // Initialize CDM7162 sensor
                        break;
                    }
                    case SUNRISE:
                    {
                        #if DEBUG_INFO
                        uint8_t buf[24];
                        snprintf(buf, 24, "Init sensor %i...", i);
                        msgbuf("info", "SUNRISE", buf);
                        #endif
                        ret = sunrise_init(&(sensors[i]), configuration_map[i]); // Initialize SUNRISE sensor
                        break;
                    }
                    case SUNLIGHT:
                    {
                        #if DEBUG_INFO
                        uint8_t buf[24];
                        snprintf(buf, 24, "Init sensor %i...", i);
                        msgbuf("info", "SUNLIGHT", buf);
                        #endif
                        ret = sunlight_init(&(sensors[i]), configuration_map[i]); // Initialize SUNLIGHT sensor
                        break;
                    }
                    case SCD30:
                    {
                        #if DEBUG_INFO
                        uint8_t buf[24];
                        snprintf(buf, 24, "Init sensor %i...", i);
                        msgbuf("info", "SCD30", buf);
                        #endif
                        ret = scd30_init(&(sensors[i]), configuration_map[i]); // Initialize SCD30 sensor
                        break;
                    }
                    case SCD41:
                    {
                        #if DEBUG_INFO
                        uint8_t buf[24];
                        snprintf(buf, 24, "Init sensor %i...", i);
                        msgbuf("info", "SCD41", buf);
                        #endif
                        ret = scd41_init(&(sensors[i]), configuration_map[i]); // Initialize SCD41 sensor
                        break;
                    }
                    case COZIR_LP3:
                    {
                        #if DEBUG_INFO
                        uint8_t buf[24];
                        snprintf(buf, 24, "Init sensor %i...", i);
                        msgbuf("info", "CozIR-LP3", buf);
                        #endif
                        ret = cozir_lp3_init(&(sensors[i]), configuration_map[i]); // Initialize CozIR-LP3 sensor
                        break;
                    }
                    
                    default:
                    {
                        #if DEBUG
                        msg("ERROR", "SENSOR", "Unknown sensor, init abort");
                        #endif
                        sensors[i].state = ERROR_UNKNOWN_SENSOR;
                        break;
                    }
                }
                if (sensors[i].state == ERROR_UNKNOWN_SENSOR) continue;
                #if DEBUG_INFO
                if (!ret)
                {
                    msg("info", "SENSOR", "Init success");
                }
                #endif
                #if DEBUG
                if (ret)
                {
                    uint8_t buf3[36];
                    snprintf(buf3, 36, "Init sensor %i failed: %i", i, ret);
                    msgbuf("ERROR", "SENSOR", buf3);
                }
                #endif
                sensors[i].state = ret != 0 ? ERROR_SENSOR_INIT_FAILED : ERROR_NO_MEAS;
            }
            if (sensors[i].meas_state == MEAS_FINISHED) sensor_measurement_vector &= ~(0b1 << i); // If measurement completed clear sensor measurement bit
        }
    }

    sensor_measurement_vector &= active_sensors; // All other measurements finished - temporary
    sensor_timer_vector &= active_sensors; // All other timers are not reached - temporary

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