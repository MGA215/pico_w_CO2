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

// structure containing info about the RTC module
struct ds3231_rtc rtc;

// string holding the datetime value
uint8_t datetime_str[30] = {0};

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

// Time value to check if SOAP message should be created
absolute_time_t soap_create_message_time;

// Vector of timers that have reached time; on reached time bit goes high
uint8_t sensor_timer_vector; 

// Vector of measurements, bit is high if measurement running
uint8_t sensor_measurement_vector; 

// Vector of active sensors
uint8_t active_sensors = 0b01111111;

// Number of total SOAP channels
uint8_t soap_channels = 16;

// SOAP sensor channels
message_channel* channels1[16]; // First 16 channels
uint8_t channels1_len; // Number of channels in first buffer
message_channel* channels2[16]; // Second 16 channels
uint8_t channels2_len; // Number of channels in second buffer

// SOAP message buffer
uint8_t soap_buffer1[MAX_SOAP_SIZE] = {0}; // MUTEX
uint8_t soap_buffer2[MAX_SOAP_SIZE] = {0}; // MUTEX

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
        print_ser_output(SEVERITY_FATAL, "MAIN-STDIO", "Initialization failure: %i; Aborting...", ret);
        return ret;
    } 

    while (true) {
        sleep_ms(1);
        if ((ret = loop()) != 0) // main loop
        {
            print_ser_output(SEVERITY_FATAL, "MAIN-LOOP", "Loop failure: %i; Aborting...", ret);
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
    mux_init(); // Initialize multiplexer

    init_sensors(); // initialize sensors

    assign_soap_channels(); // Assign SOAP channels
    soap_init(sensors, channels1, channels1_len); // Initialize SOAP channels 1
    soap_init(sensors, channels2, channels2_len); // Initialize SOAP channels 2

    process_update_time = make_timeout_time_ms(display_interval); // Set display & input checking interval
    sensor_start_measurement_time = make_timeout_time_ms(sensor_read_interval_ms); // Set measurement interval
    soap_create_message_time = make_timeout_time_ms(soap_write_message_s * 1000 + soap_write_message_initial_delay_s * 1000); // Set SOAP write message initial interval

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
    if (time_reached(soap_create_message_time)) create_soap_messages(); // Create SOAP messages
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
        #if !DEBUG
        update_display_buffer = false;
        #endif
    }
    process_update_time = make_timeout_time_us(display_interval * 1000);
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
            sensors[display_sensor].config->co2_en, sensors[display_sensor].co2,
            sensors[display_sensor].config->temp_en, sensors[display_sensor].temperature,
            sensors[display_sensor].config->pressure_en, sensors[display_sensor].pressure,
            sensors[display_sensor].config->RH_en, sensors[display_sensor].humidity); // Write sensor readings to the display
    #if DEBUG
    uint8_t hwtime[21];
    snprintf(hwtime, 21, "Time: %llu ms", to_us_since_boot(get_absolute_time()) / 1000);
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
    for (int i = 0; i < soap_channels; i++)
    {
        if (i >= 32) break;
        else if (i >= 16) channels2[i - 16] = channel_map2[i - 16];
        else channels1[i] = channel_map1[i];
    }
    for (int i = soap_channels; i < 32; i++)
    {
        if (i < 16) channels1[i] = NULL;
        else channels2[i - 16] = NULL;
    }
    channels1_len = soap_channels > 16 ? 16 : soap_channels;
    channels2_len = soap_channels < 16 ? 0 : (soap_channels > 32 ? 16 : (soap_channels % 16));
}

void create_soap_messages(void)
{
    memset(soap_buffer1, 0x00, MAX_SOAP_SIZE); // Clear old message
    if (!soap_build("Tester_01", 24069001, datetime_str, channels1, channels1_len, soap_buffer1, MAX_SOAP_SIZE)) // Create SOAP message 1
    {
        memset(soap_buffer1, 0x00, MAX_SOAP_SIZE); // Delete message if write wasnt successful
    }
    print_ser_output(SEVERITY_INFO, "MAIN-SOAP", "Generated SOAP message 1");
    memset(soap_buffer2, 0x00, MAX_SOAP_SIZE); // Clear old message
    if (!soap_build("Tester_02", 24069002, datetime_str, channels2, channels2_len, soap_buffer2, MAX_SOAP_SIZE)) // Create SOAP message 2
    {
        memset(soap_buffer2, 0x00, MAX_SOAP_SIZE); // Delete message if write wasnt successful
    }
    print_ser_output(SEVERITY_INFO, "MAIN-SOAP", "Generated SOAP message 2");
    printf("%s\n", soap_buffer1);
    printf("%s\n", soap_buffer2);


    soap_create_message_time = make_timeout_time_ms(soap_write_message_s * 1000); // Create another message in time
}

void init_sensors(void)
{
    int32_t ret;

    for (uint8_t i = 0; i < 8; i++)
    {
        print_ser_output(SEVERITY_DEBUG, "MAIN-SENSOR", "Init structure %i", i);
        common_init_struct(&sensors[i], i); // Initialize sensor structures
        sensors[i].sensor_type = configuration_map[i] != NULL ? configuration_map[i]->sensor_type : UNKNOWN; // Copy sensor type to sensor structure
    }
    uint8_t sensor_indices[8] = {0};

    for (int i = 0; i < 8; i++)
    {
        if (active_sensors & (0b1 << i))
        {
            reset_i2c();
            if ((ret = mux_enable_sensor(i)) != 0) 
            {
                print_ser_output(SEVERITY_ERROR, "MAIN-MUX", "Failed to mux sensor %i: e%i", i, ret);
                gpio_put(MUX_RST, 0);
                sleep_us(1);
                gpio_put(MUX_RST, 1);
                sleep_us(10);
                continue;
            }
            switch (configuration_map[i]->sensor_type)
            {
                case EE895:
                {
                    print_ser_output(SEVERITY_DEBUG, "MAIN-EE895", "Init sensor %i...", i);
                    ret = ee895_init(&(sensors[i]), configuration_map[i]); // Initialize EE895 sensor
                    sensors[i].sensor_number = sensor_indices[EE895]++; // Set sensor type index
                    break;
                }
                case CDM7162:
                {
                    print_ser_output(SEVERITY_DEBUG, "MAIN-CDM7162", "Init sensor %i...", i);
                    ret = cdm7162_init(&(sensors[i]), configuration_map[i]); // Initialize CDM7162 sensor
                    sensors[i].sensor_number = sensor_indices[CDM7162]++; // Set sensor type index
                    break;
                }
                case SUNRISE:
                {
                    print_ser_output(SEVERITY_DEBUG, "MAIN-SUNRISE", "Init sensor %i...", i);
                    ret = sunrise_init(&(sensors[i]), configuration_map[i]); // Initialize SUNRISE sensor
                    sensors[i].sensor_number = sensor_indices[SUNRISE]++; // Set sensor type index
                    break;
                }
                case SUNLIGHT:
                {
                    print_ser_output(SEVERITY_DEBUG, "MAIN-SUNLIGHT", "Init sensor %i...", i);
                    ret = sunlight_init(&(sensors[i]), configuration_map[i]); // Initialize SUNLIGHT sensor
                    sensors[i].sensor_number = sensor_indices[SUNLIGHT]++; // Set sensor type index
                    break;
                }
                case SCD30:
                {
                    print_ser_output(SEVERITY_DEBUG, "MAIN-SCD30", "Init sensor %i...", i);
                    ret = scd30_init(&(sensors[i]), configuration_map[i]); // Initialize SCD30 sensor
                    sensors[i].sensor_number = sensor_indices[SCD30]++; // Set sensor type index
                    break;
                }
                case SCD41:
                {
                    print_ser_output(SEVERITY_DEBUG, "MAIN-SCD41", "Init sensor %i...", i);
                    ret = scd41_init(&(sensors[i]), configuration_map[i]); // Initialize SCD41 sensor
                    sensors[i].sensor_number = sensor_indices[SCD41]++; // Set sensor type index
                    break;
                }
                case COZIR_LP3:
                {
                    print_ser_output(SEVERITY_DEBUG, "MAIN-CozIR-LP3", "Init sensor %i...", i);
                    ret = cozir_lp3_init(&(sensors[i]), configuration_map[i]); // Initialize CozIR-LP3 sensor
                    sensors[i].sensor_number = sensor_indices[COZIR_LP3]++; // Set sensor type index
                    break;
                }
                case CM1107N:
                {
                    print_ser_output(SEVERITY_DEBUG, "MAIN-CM1107N", "Init sensor %i...", i);
                    ret = cm1107n_init(&(sensors[i]), configuration_map[i]); // Initialize CM1107N sensor
                    sensors[i].sensor_number = sensor_indices[CM1107N]++; // Set sensor type index
                    break;
                }
                
                default:
                {
                    print_ser_output(SEVERITY_ERROR, "MAIN-SENSOR", "Unknown sensor %i, init abort", i);
                    sensors[i].state = ERROR_UNKNOWN_SENSOR;
                    break;
                }
            }
            if (sensors[i].state == ERROR_UNKNOWN_SENSOR) continue;
            if (!ret) print_ser_output(SEVERITY_INFO, "MAIN-SENSOR", "Init sensor %i success", i);
            if (ret) print_ser_output(SEVERITY_ERROR, "MAIN-SENSOR", "Init sensor %i failed: %i", i, ret);
            sensors[i].state = ret != 0 ? ERROR_SENSOR_INIT_FAILED : ERROR_NO_MEAS;
        }
    }

    set_power_mode(); // Set power control mode
}

void reset_i2c(void)
{
    i2c_deinit(I2C_SENSOR);

    gpio_set_function(I2C_SCL, GPIO_FUNC_SIO);
    gpio_set_dir(I2C_SCL, GPIO_OUT);
    sleep_us(10);
    gpio_put(I2C_SCL, 0);
    sleep_us(100);
    gpio_put(I2C_SCL, 1);
    sleep_us(10);
    gpio_pull_up(I2C_SCL);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    sleep_us(100);

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
        print_ser_output(SEVERITY_INFO, "MAIN-SENSOR", "Measurement start");
        for (int i = 0; i < 8; i++)
        {
            if (sensors[i].state == ERROR_SENSOR_NOT_INITIALIZED) continue; // If not initialized sensor (aka disabled) continue
            sensors[i].meas_state = MEAS_STARTED; // Start measurement
        }
        
        sensor_start_measurement_time = make_timeout_time_us(sensor_read_interval_ms * 1000); // Set another mesurement start in sensor_read_interval_ms time
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

    for (uint8_t i = 0; i < 8; i++) // Iterate sensor
    {
        if ((sensor_timer_vector & (0b1 << i)) && (active_sensors & (0b1 << i))) // If sensor should react to a timer reached
        {
            sensor_timer_vector &= ~(0b1 << i); // Clear timer reached bit
            for (uint8_t j = 0; j < 2; j++)
            {
                if (read_single_sensor(i)) break; // If reading successful break
            }
        }
        if (sensors[i].meas_state == MEAS_FINISHED) sensor_measurement_vector &= ~(0b1 << i); // If measurement completed clear sensor measurement bit
    }

    sensor_measurement_vector &= active_sensors; // All other measurements finished
    sensor_timer_vector &= active_sensors; // All other timers are not reached

    if (!sensor_measurement_vector) // If all measurements finished - turn off power globally if possible
    {
        set_power(false);
    }
    update_display_buffer = true; // Update display
    return;
}

bool read_single_sensor(uint8_t sensor_index)
{
    int32_t ret;
    bool repeat_on_error = true;
    
    reset_i2c();
    if ((ret = mux_enable_sensor(sensor_index)) != 0) // Mux to sensor
    {
        print_ser_output(SEVERITY_ERROR, "MAIN-MUX", "Failed to mux sensor %i: e%i", sensor_index, ret);
        reset_i2c(); // Reset i2c
        common_init_struct(&sensors[sensor_index], sensor_index); // Reset sensor
        sensors[sensor_index].state = ERROR_SENSOR_MUX_FAILED; // Set sensor state to MUX failed
        mux_reset(); // Reset MUX
        return false; // Repeat measurement
    }
    if (sensors[sensor_index].state && // Sensor not initialized
        sensors[sensor_index].state != ERROR_SENSOR_NOT_INITIALIZED && 
        sensors[sensor_index].state != ERROR_NO_MEAS)
    {
        reset_i2c(); // Reset I2C
        common_init_struct(&sensors[sensor_index], sensor_index); // Init sensor struct
        sensors[sensor_index].meas_state = MEAS_STARTED;
        switch (configuration_map[sensor_index]->sensor_type) // Init on sensor type
        {
            case EE895:
            {
                print_ser_output(SEVERITY_DEBUG, "MAIN-EE895", "Init sensor %i...", sensor_index);
                ret = ee895_init(&(sensors[sensor_index]), configuration_map[sensor_index]); // Initialize EE895 sensor
                break;
            }
            case CDM7162:
            {
                print_ser_output(SEVERITY_DEBUG, "MAIN-CDM7162", "Init sensor %i...", sensor_index);
                ret = cdm7162_init(&(sensors[sensor_index]), configuration_map[sensor_index]); // Initialize CDM7162 sensor
                break;
            }
            case SUNRISE:
            {
                print_ser_output(SEVERITY_DEBUG, "MAIN-SUNRISE", "Init sensor %i...", sensor_index);
                ret = sunrise_init(&(sensors[sensor_index]), configuration_map[sensor_index]); // Initialize SUNRISE sensor
                break;
            }
            case SUNLIGHT:
            {
                print_ser_output(SEVERITY_DEBUG, "MAIN-SUNLIGHT", "Init sensor %i...", sensor_index);
                ret = sunlight_init(&(sensors[sensor_index]), configuration_map[sensor_index]); // Initialize SUNLIGHT sensor
                break;
            }
            case SCD30:
            {
                print_ser_output(SEVERITY_DEBUG, "MAIN-SCD30", "Init sensor %i...", sensor_index);
                ret = scd30_init(&(sensors[sensor_index]), configuration_map[sensor_index]); // Initialize SCD30 sensor
                break;
            }
            case SCD41:
            {
                print_ser_output(SEVERITY_DEBUG, "MAIN-SCD41", "Init sensor %i...", sensor_index);
                ret = scd41_init(&(sensors[sensor_index]), configuration_map[sensor_index]); // Initialize SCD41 sensor
                break;
            }
            case COZIR_LP3:
            {
                print_ser_output(SEVERITY_DEBUG, "MAIN-CozIR-LP3", "Init sensor %i...", sensor_index);
                ret = cozir_lp3_init(&(sensors[sensor_index]), configuration_map[sensor_index]); // Initialize CozIR-LP3 sensor
                break;
            }
            case CM1107N:
            {
                print_ser_output(SEVERITY_DEBUG, "MAIN-CM1107N", "Init sensor %i...", sensor_index);
                ret = cm1107n_init(&(sensors[sensor_index]), configuration_map[sensor_index]); // Initialize CM1107N sensor
                break;
            }
            
            default:
            {
                print_ser_output(SEVERITY_ERROR, "MAIN-SENSOR", "Unknown sensor %i, init abort", sensor_index);
                sensors[sensor_index].state = ERROR_UNKNOWN_SENSOR;
                return true; // Unknown sensor - don't repeat measurement
            }
        }
        if (!ret) // Init successful
        {
            print_ser_output(SEVERITY_INFO, "MAIN-SENSOR", "Init sensor %i success", sensor_index);
            sensors[sensor_index].state = ERROR_NO_MEAS;
        }
        else // Init not successful
        {
            print_ser_output(SEVERITY_ERROR, "MAIN-SENSOR", "Init sensor %i failed: %i", sensor_index, ret);
            reset_i2c(); // Reset I2C
            mux_reset(); // Reset MUX
            sleep_ms(10);
            sensors[sensor_index].state = ERROR_SENSOR_INIT_FAILED;
            return false;
        }
    }
    if (sensors[sensor_index].state == SUCCESS || 
        sensors[sensor_index].state == ERROR_NO_MEAS) // If sensor initialized
    {
        switch (configuration_map[sensor_index]->sensor_type) // Get value based on sensor type
        {
            case EE895:
            {
                print_ser_output(SEVERITY_DEBUG, "MAIN-EE895", "Reading sensor %i...", sensor_index);
                ee895_get_value(&(sensors[sensor_index])); // Read EE895 values
                break;
            }
            case CDM7162:
            {
                print_ser_output(SEVERITY_DEBUG, "MAIN-CDM7162", "Reading sensor %i...", sensor_index);
                cdm7162_get_value(&(sensors[sensor_index])); // Read CDM7162 values
                break;
            }
            case SUNRISE:
            {
                print_ser_output(SEVERITY_DEBUG, "MAIN-SUNRISE", "Reading sensor %i...", sensor_index);
                sunrise_get_value(&(sensors[sensor_index])); // Read SUNRISE values
                break;
            }
            case SUNLIGHT:
            {
                print_ser_output(SEVERITY_DEBUG, "MAIN-SUNLIGHT", "Reading sensor %i...", sensor_index);
                sunlight_get_value(&(sensors[sensor_index])); // Read SUNLIGHT values
                break;
            }
            case SCD30:
            {
                print_ser_output(SEVERITY_DEBUG, "MAIN-SCD30", "Reading sensor %i...", sensor_index);
                scd30_get_value(&(sensors[sensor_index])); // Read SCD30 values
                break;
            }
            case SCD41:
            {
                print_ser_output(SEVERITY_DEBUG, "MAIN-SCD41", "Reading sensor %i...", sensor_index);
                scd41_get_value(&(sensors[sensor_index])); // Read SCD41 values
                break;
            }
            case COZIR_LP3:
            {
                print_ser_output(SEVERITY_DEBUG, "MAIN-CozIR-LP3", "Reading sensor %i...", sensor_index);
                cozir_lp3_get_value(&(sensors[sensor_index])); // Read CozIR-LP3 values
                break;
            }
            case CM1107N:
            {
                print_ser_output(SEVERITY_DEBUG, "MAIN-CM1107N", "Reading sensor %i...", sensor_index);
                cm1107n_get_value(&(sensors[sensor_index])); // Read CM1107N values
                break;
            }
            default:
            {
                print_ser_output(SEVERITY_ERROR, "MAIN-SENSOR", "Reading unknown sensor %i", sensor_index);
                sensors[sensor_index].state = ERROR_UNKNOWN_SENSOR; // Unknown sensor
                return true; // Unknown sensor - don't repeat measurement
            }
        }
        if (sensors[sensor_index].state && sensors[sensor_index].state != ERROR_NO_MEAS) // Reading not successful
        {
            print_ser_output(SEVERITY_ERROR, "MAIN-SENSOR", "Reading sensor %i failed: %i", sensor_index, sensors[sensor_index].state);
            sleep_us(100);
            reset_i2c(); // Reset I2C
            mux_reset(); // Reset MUX
            sleep_us(100);
            return false; // Reading failed, repeat measurement
        }
        if (sensors[sensor_index].meas_state == MEAS_FINISHED && sensors[sensor_index].state == SUCCESS && !(sensor_measurement_vector & (0b1 << sensor_index)))
        {
            print_ser_output(SEVERITY_INFO, "MAIN-SENSOR", "Successfully read sensor %i", sensor_index);
        }
    }
    return true; // Sensor successfully read
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

