#include "sensors.h"

#include "common/functions.h"
#include "common/debug.h"
#include "common/i2c_extras.h"
#include "common/constants.h"
#include "../common/serialize.h"
#include "error_codes.h"
#include "common/shared.h"
#include "../sensor_config.h"

#include "ee895/ee895.h"
#include "cdm7162/cdm7162.h"
#include "sunrise/sunrise.h"
#include "sunlight/sunlight.h"
#include "scd30/scd30.h"
#include "scd41/scd41.h"
#include "cozir-lp3/cozir-lp3.h"
#include "cm1107n/cm1107n.h"
#include "mux/mux.h"
#include "power/power.h"
#include "ms5607/ms5607.h"
#include "hyt271/hyt271.h"
#include "../eeprom/eeprom.h"


#include "string.h"
#include "math.h"
#include "pico/mutex.h"
#include "hardware/watchdog.h"

#define EEPROM_SENSOR_ADDR_START 0x00000300
#define EEPROM_SENSOR_CONFIG_LEN 0x100

// Vector of sensors with assigned configuration
static uint8_t active_sensors;

// Timer to start measurement
absolute_time_t sensor_start_measurement_time;

bool sensors_measurement_ready = false;
bool sensors_was_measurement_read = false;

static bool before_measurement = true;

/**
 * @brief Reads sensor configuration from the EEPROM; if reading failed, resets the device, if parsing failed, uses default configuration
 * 
 * @param out_config Read configuration
 * @param sensor_index Index of the sensor
 */
static void sensors_read_config_from_eeprom(sensor_config_t* out_config, uint8_t sensor_index);

/**
 * @brief Sets up sensor structure
 * 
 * @param sensor_index Index of the sensor
 * @return true if sensor structure set up with no errors
 * @return false if an error has occured (invalid sensor type)
 */
static bool sensors_init_sensor_struct(uint8_t sensor_index);

/**
 * @brief Initializes single sensor
 * 
 * @param sensor_index Index of the sensor
 * @return true if init swas successful
 * @return false if init failed
 */
static bool sensors_init(uint8_t sensor_index);

/**
 * @brief Initializes sensor itself according to its type
 * 
 * @param sensor Sensor to initialize
 * @param configuration Configuration to assign to the sensor
 * @param is_first_init Is sensor initialized for the first time
 * @return int32_t Initialization return code:
 * ERROR_UNKNOWN_SENSOR;
 * ERROR_SENSOR_INIT_FAILED;
 * ERROR_SENSOR_MUX_FAILED;
 * SUCCESS;
 */
static int32_t sensors_init_sensor_type(sensor_t* sensor, sensor_config_t* configuration);

/**
 * @brief Attempts to start a new measurement
 * 
 * @return true if measurement successfully started
 * @return false if measurement still running
 */
static bool sensors_check_start_measurement(void);

/**
 * @brief Performs actions during measurement start
 * 
 */
static void sensors_start_measurement(void);

/**
 * @brief Reads measured value from a sensor at a specific sensor_index
 * 
 * @param sensor_index Index of the sensor
 * @return true if sensor read successfully
 * @return false if sensor reading failed
 */
static bool sensors_read(uint8_t sensor_index);

/**
 * @brief Reads sensor according to its type
 * 
 * @param sensor Sensor to read
 */
static void sensors_read_sensor_type(sensor_t* sensor);

/**
 * @brief Reads and verifies sensor configuration
 * 
 * @param sensor_index Index of the sensor
 * @return true if verified successfully
 * @return false if mismatch has been found
 */
static bool sensors_verify_read_config(uint8_t sensor_index);

/**
 * @brief Reads single sensor configuration
 * 
 * @param configuration Output configuration
 * @param sensor_index Sensor index
 */
static int32_t sensors_read_config(sensor_config_t* configuration, uint8_t sensor_index);

/**
 * @brief Compares two sensor configurations
 * 
 * @param left configuration to compare
 * @param right configuration to compare
 * @return true if configurations are the same
 * @return false if configurations differ
 */
static bool sensors_compare_config(sensor_config_t* left, sensor_config_t* right);

/**
 * @brief Sets up multiplexer to access specified sensor
 * 
 * @param sensor_index Index of the sensor to access
 * @return true if multiplexer set
 * @return false if an error on multiplexer has occured; 
 * also sets sensor state in case of error to ERROR_SENSOR_MUX_FAILED
 */
static bool sensors_mux_to_sensor(uint8_t sensor_index);

/**
 * @brief Set the power on globally controlled sensors to [on]
 * 
 * @param on Whether the power should be turned on or off
 * @param startup Whether power should be turned on regardless of sensor power scheme
 */
static void set_power(bool on, bool startup);

/**
 * @brief Set the 5V power to sensors
 * 
 */
static void set_5v(void);

/**
 * @brief Computes pressure compensated co2 value
 * 
 * @param co2_value co2 value before compensation
 * @param pressure current env pressure
 * @return float pressure compensated co2 concentration value
 */
static float sensors_sensor_compensate_pressure(float co2_value, float pressure);



static void sensors_read_config_from_eeprom(sensor_config_t* out_config, uint8_t sensor_index)
{
    int32_t ret;
    uint8_t buffer[EEPROM_SENSOR_CONFIG_LEN];
    ret = eeprom_read(EEPROM_SENSOR_ADDR_START + EEPROM_SENSOR_CONFIG_LEN * sensor_index, 
                        buffer, EEPROM_SENSOR_CONFIG_LEN); // Read config from EEPROM
    if (ret) // Reading failed
    {
        print_ser_output(SEVERITY_FATAL, SOURCE_SENSORS, SOURCE_EEPROM, "Failed to read configuration from EEPROM, resetting device...");
        watchdog_enable(1, 1); // reset
        while (true) tight_loop_contents();
        return;
    }
    ret = serializer_deserialize(out_config, buffer, 0x100); // Parsing read configuration
    if (ret) // If parsing failed
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Failed to parse configuration, using default config");
        memcpy(out_config, &sensor_config_default, sizeof(sensor_config_t)); // Set output configuration to default one
    }
    return;
}

void sensors_init_all()
{
    for (int i = 0; i < 8; i++) // Initialize default structures
    {
        sensors_init_sensor_struct(i);
    }

    init_sensor_i2c(); // Initialize sensor I2C
    mux_init(); // Initialize MUX
    power_reset_all();
    set_5v();
    set_power(true, true);

    watchdog_update();
    sleep_ms(1000); // Sensor power up time (mainly because of CM1107N)
    watchdog_update();
    for (int i = 0; i < CONNECTED_SENSORS; i++)
    {
        sensors_init(i); // Initialize sensor
        watchdog_update(); // Update watchdog
    }
    for (int i = 0; i < CONNECTED_SENSORS; i++)
    {
        if (sensors[i].sensor_type == UNKNOWN || sensors[i].initialized == false) continue;
        for (int j = 0; j < 3; j++)
        {
            watchdog_update();
            if (sensors_verify_read_config(i)) break;
            sleep_ms(1);
        }
    }

    // Initialize MS5607 values
    ms5607.meas_state = MEAS_STARTED;
    ms5607.state = ERROR_NO_MEAS;
    ms5607.pressure = 0;
    ms5607.temperature = 0;
    memset(ms5607.pressure_raw, 0x00, 3);
    memset(ms5607.temperature_raw, 0x00, 3);
    memset(ms5607.prom, 0x00, 16);
    ms5607_get_value(); // Perform MS5607 measurement
    if (ms5607.state != SUCCESS) print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_MS5607, "Failed to read MS5607: %i", ms5607.state);
    watchdog_update();

    // Initialize HYT271 values
    hyt271.meas_state = MEAS_FINISHED;
    hyt271.state = ERROR_NO_MEAS;
    hyt271.humidity = 0;
    hyt271.temperature = 0;
    hyt271.wake_time = at_the_end_of_time;
    memset(hyt271.humidity_raw, 0x00, 2);
    memset(hyt271.temperature_raw, 0x00, 2);

    sensor_start_measurement_time = make_timeout_time_us(global_configuration.meas_int * 1000); // Set measurement start timer
    set_power(false, true);
}

static bool sensors_init_sensor_struct(uint8_t sensor_index)
{
    print_ser_output(SEVERITY_DEBUG, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Setting up structure %i", sensor_index);
    if (sensor_index >= CONNECTED_SENSORS) return false;
    sensor_t* sensor = &sensors[sensor_index];

    common_init_struct(sensor, sensor_index); // Initialize sensor structure

    sensor_config_t config;
    memset(&config, 0x00, sizeof(sensor_config_t));
    sensors_read_config_from_eeprom(&config, sensor_index); // Read sensor config from EEPROM
    sensor->sensor_type = config.sensor_type; // Copy sensor type to sensor structure
    memcpy(&(sensor->config), &config, sizeof(sensor_config_t)); // Assign config
    sensor->config.sensor_active = false; // Set sensor to inactive state

    if (sensors[sensor_index].sensor_type < 0 || sensors[sensor_index].sensor_type >= SENSOR_TYPES) // Check for invalid sensor type
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_NO_SOURCE, 
                            "Unknown sensor at input %x, init abort", sensors[sensor_index].index);
        sensors[sensor_index].state = ERROR_UNKNOWN_SENSOR; // Unknown sensor
        sensors[sensor_index].sensor_type = UNKNOWN;
        return false; // Not initiable
    }
    sensors[sensor_index].sensor_number = config.sensor_ord; // Set sensor type index
    sensors[sensor_index].err_total_counter = 0;
    sensors[sensor_index].config.sensor_active = true;

    switch (sensors[sensor_index].sensor_type) // assign functions
    {
        case EE895:
            sensors[sensor_index].config.sensor_init = ee895_init;
            sensors[sensor_index].config.sensor_get_value = ee895_get_value;
            sensors[sensor_index].config.sensor_read_config = ee895_read_config;
            break;
        case CDM7162:
            sensors[sensor_index].config.sensor_init = cdm7162_init;
            sensors[sensor_index].config.sensor_get_value = cdm7162_get_value;
            sensors[sensor_index].config.sensor_read_config = cdm7162_read_config;
            break;
        case SUNRISE:
            sensors[sensor_index].config.sensor_init = sunrise_init;
            sensors[sensor_index].config.sensor_get_value = sunrise_get_value;
            sensors[sensor_index].config.sensor_read_config = sunrise_read_config;
            break;
        case SUNLIGHT:
            sensors[sensor_index].config.sensor_init = sunlight_init;
            sensors[sensor_index].config.sensor_get_value = sunlight_get_value;
            sensors[sensor_index].config.sensor_read_config = sunlight_read_config;
            break;
        case SCD30:
            sensors[sensor_index].config.sensor_init = scd30_init;
            sensors[sensor_index].config.sensor_get_value = scd30_get_value;
            sensors[sensor_index].config.sensor_read_config = scd30_read_config;
            break;
        case SCD41:
            sensors[sensor_index].config.sensor_init = scd41_init;
            sensors[sensor_index].config.sensor_get_value = scd41_get_value;
            sensors[sensor_index].config.sensor_read_config = scd41_read_config;
            break;
        case COZIR_LP3:
            sensors[sensor_index].config.sensor_init = cozir_lp3_init;
            sensors[sensor_index].config.sensor_get_value = cozir_lp3_get_value;
            sensors[sensor_index].config.sensor_read_config = cozir_lp3_read_config;
            break;
        case CM1107N:
            sensors[sensor_index].config.sensor_init = cm1107n_init;
            sensors[sensor_index].config.sensor_get_value = cm1107n_get_value;
            sensors[sensor_index].config.sensor_read_config = cm1107n_read_config;
            break;
        default:
            sensors[sensor_index].config.sensor_init = NULL;
            sensors[sensor_index].config.sensor_get_value = NULL;
            sensors[sensor_index].config.sensor_read_config = NULL;
            break;
    }

    return true;
}

static bool sensors_init(uint8_t sensor_index)
{
    int32_t ret;
    if (sensor_index >= CONNECTED_SENSORS) return false;
    sensor_t* sensor = &sensors[sensor_index];

    for (int i = 0; i < 2; i++) // Try initialization twice
    {
        sensor->config.sensor_active = true; // Activate sensor
        sensor->config.verified = false; // Configuration not verified
        sensor->initialized = false;

        print_ser_output(SEVERITY_DEBUG, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Initializing sensor %i...", sensor_index);

        ret = sensors_init_sensor_type(sensor, &(sensor->config)); // Initialize sensor

        if (!ret) // Init successful
        {
            print_ser_output(SEVERITY_INFO, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Init sensor %i success", sensor_index);
        }
        else // Error during initialization
        {   
            print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Init sensor %i failed: %i", sensor_index, ret); 
            continue; // Retry initialization
        }

        sensor->initialized = true;
        for (int j = 0; j < 3; j++) // Verify configuration
        {
            watchdog_update();
            if (sensors_verify_read_config(i))
            {
                sensor->config.verified = true;
                break;
            }
            sleep_ms(1);
        }

        return true; // Initialization successful
    }
    return false; // Initialization attempt 2 failed
}

static int32_t sensors_init_sensor_type(sensor_t* sensor, sensor_config_t* configuration)
{
    int32_t ret;
    // pre-init checks
    if (sensor->initialized) return SUCCESS;
    if (sensor->state == ERROR_UNKNOWN_SENSOR) return ERROR_UNKNOWN_SENSOR;

    if (sensor->sensor_type == UNKNOWN || (uint8_t)(sensor->sensor_type) >= SENSOR_TYPES) // Check for valid sensor type
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_NO_SOURCE, 
                         "Unknown sensor at input %x, init abort", sensor->index); // No type match - unknown sensor
        sensor->state = ERROR_UNKNOWN_SENSOR;
        return ERROR_UNKNOWN_SENSOR;
    }

    // init
    print_ser_output(SEVERITY_DEBUG, SOURCE_SENSORS, SOURCE_EE895 + sensor->sensor_type, 
                     "Init sensor to type %x%x...", sensor->sensor_type, sensor->sensor_number);
    if (sensor->config.sensor_init != NULL) // initialize sensor
    {
        if (!sensors_mux_to_sensor(sensor->index)) // Mux to sensor
        {
            print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_MUX, "Failed to mux to %i", sensor->index);
            return ERROR_SENSOR_MUX_FAILED;
        }
        ret = sensor->config.sensor_init(sensor, configuration);
    }
    else 
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_NO_SOURCE, 
                         "Unknown sensor %x?, init abort", sensor->sensor_type); // No type match - unknown sensor
            sensor->state = ERROR_UNKNOWN_SENSOR;
        return ERROR_UNKNOWN_SENSOR;
    }

    // post init
    if (!ret) // Successful init
    {
        sensor->state = ERROR_NO_MEAS;
        return SUCCESS;
    }
    else // Error during init
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_NO_SOURCE, 
                         "Error during sensor init: %i", ret); // error during sensor initialization
        sensor->state = ERROR_SENSOR_INIT_FAILED;
        return ERROR_SENSOR_INIT_FAILED;
    }
}

static void sensors_start_measurement(void)
{
    static int counter = 0;
    bool start_meas = sensors_check_start_measurement(); // Start new measurement
    if (!start_meas && counter++ < 20) 
    {
        print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Cannot start new measurement.");
        sensor_start_measurement_time = make_timeout_time_ms(global_configuration.meas_int / 10);
    }
    else if (start_meas)
    {
        counter = 0; // Reset unsuccessful measurement start attempts
        sensor_start_measurement_time = make_timeout_time_ms(global_configuration.meas_int);
        sensors_was_measurement_read = false; // Sensor measurement is ready to be read
        sensors_measurement_ready = false;
        before_measurement = false; // Not before first measurement anymore
        set_power(true, false);
        for (int i = 0; i < 8; i++) // Initialize measurement cycle variables
        {
            sensors[i].err_iter_counter = 0;
        }
    }
    else // Safety mechanism to unblock measurement after 20 unsuccessfull attempts to start new measurement
    {
        counter = 0; // Reset unsuccessful measurement start attempts
        print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Forcing new measurement");
        if (ms5607.meas_state != MEAS_FINISHED) 
        {
            ms5607.meas_state = MEAS_FINISHED;
            ms5607.state = ERROR_SENSOR_INIT_FAILED;
            print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Pressure sensor blocking measurement");
        }
        if (hyt271.meas_state != MEAS_FINISHED || !is_at_the_end_of_time(hyt271.wake_time))
        {
            hyt271.meas_state = MEAS_FINISHED;
            hyt271.state = ERROR_SENSOR_INIT_FAILED;
            hyt271.wake_time = at_the_end_of_time;
            print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_NO_SOURCE, "T/RH sensor blocking measurement");
        }
        for (int i = 0; i < 8; i++)
        {
            
            if (common_is_measurement_running(&sensors[i])) 
            {
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Sensor %X blocking measurement", i);
                common_measurement_force_stop(&sensors[i]);
                sensors[i].state = ERROR_SENSOR_INIT_FAILED; // Cancel initialization
                sensors[i].err_total_counter++;
            }
        }
    }   
}

void sensors_read_all(void)
{
    static uint8_t sensor_index = 0;
    if (time_reached(sensor_start_measurement_time)) // Should new measurement be started
    {
        sensors_start_measurement();
    }
    if (!sensors_is_measurement_finished()) // If measurement running
    {
        if (ms5607.meas_state != MEAS_FINISHED)
        {
            for (int i = 0; i < 2; i++) // Try pressure measurement twice
            {
                ms5607_get_value(); // Read pressure sensor
                if (!ms5607.state) break; // On no error break
                print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_MS5607, "Failed to read sensor: %i", ms5607.state);
                ms5607.meas_state = i != 1 ? MEAS_STARTED : MEAS_FINISHED; // If on next iteration should try another pressure measurement
                sleep_ms(2);
            }
        }

        if (time_reached(hyt271.wake_time)) // If should read HYT271
        {
            for (int i = 0; i < 2; i++) // Try humidity measurement twice
            {
                hyt271_get_value(); // Read HYT sensor
                if (!hyt271.state) break; // On no error break
                print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_HYT271, "Failed to read sensor: %i", hyt271.state);
                if (++hyt271.err_count < 2) hyt271.meas_state = MEAS_STARTED; // If on next iteration should try another measurement
                sleep_ms(2);
            }
        }

        if (common_should_sensor_operate(&sensors[sensor_index])) // If sensor should react to a timer reached
        {
            watchdog_update(); // Update watchdog - just in case
            if (!sensors_read(sensor_index)) // If reading failed
            {
                sensors[sensor_index].err_iter_counter++; // Increment continuous error counter
            }
        }
        if (sensors[sensor_index].err_iter_counter == 2) common_measurement_force_stop(&sensors[sensor_index]); // Force quit sensor measurement if 2 continuous errors
        sensor_index = (sensor_index + 1) % 8; // Sensor iterator

        if (sensors_is_measurement_finished()) // If all measurements finished - turn off power globally if possible
        {
            set_power(false, false);
            for (int i = 0; i < 8; i++)
            {
                if (sensors[i].state && sensors[i].config.sensor_active) sensors[i].err_total_counter++; // Counter of total errors during measurement run
                print_ser_output(SEVERITY_INFO, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Input: %i, Errors: %i", i, sensors[i].err_total_counter);
            }
            if (!sensors_was_measurement_read && !sensors_measurement_ready) sensors_measurement_ready = true; // Set measurement ready
        }
    }
}

static bool sensors_check_start_measurement(void)
{
    if (sensors_is_measurement_finished()) // Check if all sensors have finished measurement
    {
        print_ser_output(SEVERITY_INFO, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Measurement starting...");
        ms5607.meas_state = MEAS_STARTED; // Start pressure measurement
        for (int i = 0; i < 8; i++) // Start sensors measurement
        {
            if (sensors[i].state != ERROR_UNKNOWN_SENSOR) 
            {
                common_measurement_start(&sensors[i]);
                sensors[i].init_count = 0;
            }
        }       
        hyt271.wake_time = get_absolute_time();
        hyt271.meas_state = MEAS_STARTED;
        return true; 
    }
    return false;
}

static bool sensors_read(uint8_t sensor_index)
{
    int32_t ret;
    if (sensor_index > CONNECTED_SENSORS) return false;
    sensor_t* sensor = &sensors[sensor_index];
    if (sensor->state == ERROR_UNKNOWN_SENSOR) return false;

    for (uint8_t i = 0; i < 2; i++) // Attempts
    {
        if (!sensor->initialized) // Check sensor initialization
        {
            if (!sensors_init(sensor->index))
            {
                print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_EE895 + sensor->sensor_type, "Failed to reinit sensor");
                continue;
            }
        }
        if (!sensor->config.verified) // Check valid configuration
        {
            for (int j = 0; j < 3; j++)
            {
                if (sensors_verify_read_config(i))
                {
                    sensor->config.verified = true;
                    break;
                }
                sleep_ms(1);
            }
            continue;
        }
        if (sensor->state && sensor->state != ERROR_NO_MEAS) 
        {
            sensor->initialized = false; // cancel init
            continue;
        }

        print_ser_output(SEVERITY_DEBUG, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Reading sensor %i...", sensor_index);
        sensors_read_sensor_type(sensor);

        if (sensor->state && sensor->state != ERROR_NO_MEAS) // Reading not successful
        {
            print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Reading sensor %i failed: %i", 
                                sensor_index, sensor->state);
            if (global_configuration.reinit_sensors_on_error) sensors[sensor_index].initialized = false; // Cancel init
            continue;
        }

        // post-read
        if (!common_is_measurement_running(&sensors[sensor_index]) && sensors[sensor_index].state == SUCCESS) // Reading finished successfully
        {
            print_ser_output(SEVERITY_INFO, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Successfully read sensor %i", sensor_index);
            if (sensors[sensor_index].config.ext_pressure_comp && ms5607.pressure != NAN) // Compensate for pressure
            {
                float val = sensors_sensor_compensate_pressure(sensors[sensor_index].co2, ms5607.pressure);
                print_ser_output(SEVERITY_DEBUG, SOURCE_SENSORS, SOURCE_NO_SOURCE, 
                    "Pressure compensation of sensor %i: %.0f -> %.0f", sensor_index, sensors[sensor_index].co2, val);
                sensors[sensor_index].co2 = val;
            }
            else if (ms5607.pressure == NAN) // Pressure measurement failed
            {
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Cannot compute pressure compensation for sensor %i", sensor_index);
            }
            sensors[sensor_index].err_iter_counter = 0;
        }

        return true;
    }
    common_measurement_force_stop(sensor); // Terminate measurement
    return false;
}

static void sensors_read_sensor_type(sensor_t* sensor)
{
    if (sensor->sensor_type < 0 || sensor->sensor_type >= SENSOR_TYPES) // Check for valid sensor type
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_NO_SOURCE, 
            "Unknown sensor %x?, read abort", sensor->sensor_type); // No type match - unknown sensor
        sensor->state = ERROR_UNKNOWN_SENSOR;
        return;
    }
    print_ser_output(SEVERITY_DEBUG, SOURCE_SENSORS, SOURCE_EE895 + sensor->sensor_type, 
        "Reading sensor type %x%x...", sensor->sensor_type, sensor->sensor_number);

    if (!sensors_mux_to_sensor(sensor->index))
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_MUX, "Failed to mux to sensor %i", sensor->index);
        return;
    }
    if (sensor->config.sensor_get_value != NULL) // read measured value
    {
        sensor->config.sensor_get_value(sensor);
    }
    else 
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_NO_SOURCE, 
            "Unknown sensor %x?, read abort", sensor->sensor_type); // No type match - unknown sensor
        sensor->state = ERROR_UNKNOWN_SENSOR;
    }
}

static bool sensors_verify_read_config(uint8_t sensor_index)
{
    int32_t ret;
    sensor_config_t config;
    sensors_mux_to_sensor(sensor_index);
    ret = sensors_read_config(&config, sensor_index); // Read sensor config
    if (!sensors_compare_config(&sensors[sensor_index].config, &config) && !ret) // Compare config with the one set
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Configuration %i mismatch", sensor_index);
        return false;
    }
    else if (!ret)
    {
        print_ser_output(SEVERITY_INFO, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Configuration %i verified", sensor_index);
        sensors[sensor_index].config.verified = true; // Configuration verified
        return true;
    }
    return false;
}

static int32_t sensors_read_config(sensor_config_t* configuration, uint8_t sensor_index)
{
    int32_t ret = 0;
    if (sensors[sensor_index].state == ERROR_UNKNOWN_SENSOR) return ERROR_UNKNOWN_SENSOR;
    if (sensors[sensor_index].state == ERROR_SENSOR_INIT_FAILED || sensors[sensor_index].state == ERROR_SENSOR_NOT_INITIALIZED) 
        return ERROR_SENSOR_NOT_INITIALIZED; // Check if sensor actually initialized
    print_ser_output(SEVERITY_DEBUG, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Reading configuration %i...", sensor_index);

    if (sensors[sensor_index].config.sensor_read_config != NULL) // read sensor configuration
    {
        ret = sensors[sensor_index].config.sensor_read_config(configuration, sensors[sensor_index].config.single_meas_mode);
    }
    else 
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Unknown sensor %i, read config abort...", sensor_index);
        return ERROR_UNKNOWN_SENSOR;
    }

    if (ret) // Error during config reading
    {
        memset(configuration, 0x00, sizeof(sensor_config_t)); // Clear config
        configuration->sensor_type = UNKNOWN; // Unknown sensor type
        configuration->verified = false;
        print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Failed to read config %i: %i", sensor_index, ret);
    }
    else
    {
        print_ser_output(SEVERITY_DEBUG, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Successfully read configuration %i", sensor_index);
        configuration->co2_en = sensors[sensor_index].config.co2_en; // Set some sw parameters
        configuration->temp_en = sensors[sensor_index].config.temp_en;
        configuration->RH_en = sensors[sensor_index].config.RH_en;
        configuration->pressure_en = sensors[sensor_index].config.pressure_en;
        configuration->power_5V = sensors[sensor_index].config.power_5V;
        configuration->power_global_control = sensors[sensor_index].config.power_global_control;
        configuration->verified = false;
    }
    return ret;
}

static bool sensors_compare_config(sensor_config_t* left, sensor_config_t* right)
{
    if (left->sensor_type != right->sensor_type) return false; // Check sensor type mismatch
    switch(left->sensor_type)
    {
        case EE895:
        {
            if (left->meas_period != right->meas_period ||
                left->filter_coeff != right->filter_coeff)
            {
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_EE895, "meas_period: %u, %u", left->meas_period, right->meas_period);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_EE895, "filter_coeff: %u, %u", left->filter_coeff, right->filter_coeff);
                return false;
            }
            return true;
        }
        case CDM7162:
        {
            if (left->enable_PWM_pin != right->enable_PWM_pin ||
                left->PWM_range_high != right->PWM_range_high ||
                left->enable_pressure_comp != right->enable_pressure_comp ||
                left->enable_pressure_comp && (left->pressure != right->pressure) ||
                left->enable_altitude_comp != right->enable_altitude_comp ||
                left->enable_altitude_comp && (left->altitude != right->altitude) ||
                left->enable_abc != right->enable_abc ||
                left->enable_alternate_abc != right->enable_alternate_abc ||
                left->abc_target_value != right->abc_target_value ||
                left->abc_period != right->abc_period ||
                left->alarm_treshold_co2_high != right->alarm_treshold_co2_high ||
                left->alarm_treshold_co2_low != right->alarm_treshold_co2_low)
            {
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_CDM7162, "enable_PWM_pin: %u, %u", left->enable_PWM_pin, right->enable_PWM_pin);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_CDM7162, "PWM_range_high: %u, %u", left->PWM_range_high, right->PWM_range_high);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_CDM7162, "enable_pressure_comp: %u, %u", left->enable_pressure_comp, right->enable_pressure_comp);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_CDM7162, "pressure: %u, %u", left->pressure, right->pressure);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_CDM7162, "enable_altitude_comp: %u, %u", left->enable_altitude_comp, right->enable_altitude_comp);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_CDM7162, "altitude: %u, %u", left->altitude, right->altitude);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_CDM7162, "enable_abc: %u, %u", left->enable_abc, right->enable_abc);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_CDM7162, "enable_alternate_abc: %u, %u", left->enable_alternate_abc, right->enable_alternate_abc);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_CDM7162, "abc_target_value: %u, %u", left->abc_target_value, right->abc_target_value);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_CDM7162, "abc_period: %u, %u", left->abc_period, right->abc_period);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_CDM7162, "alarm_treshold_co2_high: %u, %u", left->alarm_treshold_co2_high, right->alarm_treshold_co2_high);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_CDM7162, "alarm_treshold_co2_low: %u, %u", left->alarm_treshold_co2_low, right->alarm_treshold_co2_low);
                return false;
            }
            return true;
        }
        case SUNRISE:
        {
            if (left->meas_period != right->meas_period ||
                left->single_meas_mode != right->single_meas_mode ||
                left->meas_samples != right->meas_samples ||
                left->enable_static_IIR != right->enable_static_IIR ||
                left->enable_dynamic_IIR != right->enable_dynamic_IIR ||
                left->filter_coeff != right->filter_coeff ||
                left->enable_nRDY != right->enable_nRDY ||
                left->invert_nRDY != right->invert_nRDY ||
                left->enable_pressure_comp != right->enable_pressure_comp ||
                left->enable_abc != right->enable_abc ||
                left->abc_period != right->abc_period ||
                left->abc_target_value != right->abc_target_value)
            {
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SUNRISE, "meas_period: %u, %u", left->meas_period, right->meas_period);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SUNRISE, "single_meas_mode: %u, %u", left->single_meas_mode, right->single_meas_mode);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SUNRISE, "meas_samples: %u, %u", left->meas_samples, right->meas_samples);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SUNRISE, "enable_static_IIR: %u, %u", left->enable_static_IIR, right->enable_static_IIR);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SUNRISE, "enable_dynamic_IIR: %u, %u", left->enable_dynamic_IIR, right->enable_dynamic_IIR);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SUNRISE, "filter_coeff: %u, %u", left->filter_coeff, right->filter_coeff);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SUNRISE, "enable_nRDY: %u, %u", left->enable_nRDY, right->enable_nRDY);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SUNRISE, "invert_nRDY: %u, %u", left->invert_nRDY, right->invert_nRDY);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SUNRISE, "enable_pressure_comp: %u, %u", left->enable_pressure_comp, right->enable_pressure_comp);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SUNRISE, "enable_abc: %u, %u", left->enable_abc, right->enable_abc);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SUNRISE, "abc_period: %u, %u", left->abc_period, right->abc_period);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SUNRISE, "abc_target_value: %u, %u", left->abc_target_value, right->abc_target_value);
                return false;
            }
            return true;
        }
        case SUNLIGHT:
        {
            if (left->meas_period != right->meas_period ||
                left->single_meas_mode != right->single_meas_mode ||
                left->meas_samples != right->meas_samples ||
                left->enable_static_IIR != right->enable_static_IIR ||
                left->enable_dynamic_IIR != right->enable_dynamic_IIR ||
                left->filter_coeff != right->filter_coeff ||
                left->enable_nRDY != right->enable_nRDY ||
                left->invert_nRDY != right->invert_nRDY ||
                left->enable_pressure_comp != right->enable_pressure_comp ||
                left->enable_abc != right->enable_abc ||
                left->abc_period != right->abc_period ||
                left->abc_target_value != right->abc_target_value)
            {
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SUNLIGHT, "meas_period: %u, %u", left->meas_period, right->meas_period);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SUNLIGHT, "single_meas_mode: %u, %u", left->single_meas_mode, right->single_meas_mode);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SUNLIGHT, "meas_samples: %u, %u", left->meas_samples, right->meas_samples);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SUNLIGHT, "enable_static_IIR: %u, %u", left->enable_static_IIR, right->enable_static_IIR);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SUNLIGHT, "enable_dynamic_IIR: %u, %u", left->enable_dynamic_IIR, right->enable_dynamic_IIR);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SUNLIGHT, "filter_coeff: %u, %u", left->filter_coeff, right->filter_coeff);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SUNLIGHT, "enable_nRDY: %u, %u", left->enable_nRDY, right->enable_nRDY);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SUNLIGHT, "invert_nRDY: %u, %u", left->invert_nRDY, right->invert_nRDY);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SUNLIGHT, "enable_pressure_comp: %u, %u", left->enable_pressure_comp, right->enable_pressure_comp);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SUNLIGHT, "enable_abc: %u, %u", left->enable_abc, right->enable_abc);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SUNLIGHT, "abc_period: %u, %u", left->abc_period, right->abc_period);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SUNLIGHT, "abc_target_value: %u, %u", left->abc_target_value, right->abc_target_value);
                return false;
            }
            return true;
        }
        case SCD30:
        {
            if (left->meas_period != right->meas_period ||
                fabs(left->temperature_offset - right->temperature_offset) > 0.01f ||
                left->enable_pressure_comp != right->enable_pressure_comp ||
                left->enable_pressure_comp && (left->pressure != right->pressure) ||
                left->enable_altitude_comp != right->enable_altitude_comp ||
                left->enable_altitude_comp && (left->altitude != right->altitude) ||
                left->enable_abc != right->enable_abc)
            {
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SCD30, "meas_period: %u, %u", left->meas_period, right->meas_period);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SCD30, "temperature_offset: %f, %f", left->temperature_offset, right->temperature_offset);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SCD30, "enable_pressure_comp: %u, %u", left->enable_pressure_comp, right->enable_pressure_comp);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SCD30, "pressure: %u, %u", left->pressure, right->pressure);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SCD30, "enable_altitude_comp: %u, %u", left->enable_altitude_comp, right->enable_altitude_comp);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SCD30, "altitude: %u, %u", left->altitude, right->altitude);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SCD30, "enable_abc: %u, %u", left->enable_abc, right->enable_abc);
                return false;
            }
            return true;
        }
        case SCD41:
        {
            if (left->single_meas_mode != right->single_meas_mode ||
                fabs(left->temperature_offset - right->temperature_offset) > 0.01f ||
                left->enable_pressure_comp != right->enable_pressure_comp ||
                left->enable_pressure_comp && (left->pressure != right->pressure) ||
                left->enable_altitude_comp != right->enable_altitude_comp ||
                left->enable_altitude_comp && (left->altitude != right->altitude) ||
                left->enable_abc != right->enable_abc ||
                left->abc_init_period != right->abc_init_period ||
                left->abc_period != right->abc_period)
            {
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SCD41, "single_meas_mode: %u, %u", left->single_meas_mode, right->single_meas_mode);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SCD41, "temperature_offset: %f, %f", left->temperature_offset, right->temperature_offset);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SCD41, "enable_pressure_comp: %u, %u", left->enable_pressure_comp, right->enable_pressure_comp);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SCD41, "pressure: %u, %u", left->pressure, right->pressure);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SCD41, "enable_altitude_comp: %u, %u", left->enable_altitude_comp, right->enable_altitude_comp);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SCD41, "altitude: %u, %u", left->altitude, right->altitude);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SCD41, "enable_abc: %u, %u", left->enable_abc, right->enable_abc);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SCD41, "abc_init_period: %u, %u", left->abc_init_period, right->abc_init_period);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_SCD41, "abc_period: %u, %u", left->abc_period, right->abc_period);
                return false;
            }
            return true;
        }
        case COZIR_LP3:
        {
            if (left->filter_coeff != right->filter_coeff ||
                left->enable_PWM_pin != right->enable_PWM_pin ||
                left->enable_pressure_comp != right->enable_pressure_comp ||
                left->enable_pressure_comp && (left->pressure != right->pressure) ||
                left->enable_abc != right->enable_abc ||
                left->abc_init_period != right->abc_init_period ||
                left->abc_period != right->abc_period ||
                left->abc_target_value != right->abc_target_value ||
                left->alarm_en != right->alarm_en ||
                left->alarm_treshold_co2_high != right->alarm_treshold_co2_high)
            {
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_COZIR_LP3, "filter_coeff: %u, %u", left->filter_coeff, right->filter_coeff);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_COZIR_LP3, "enable_PWM_pin: %u, %u", left->enable_PWM_pin, right->enable_PWM_pin);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_COZIR_LP3, "enable_pressure_comp: %u, %u", left->enable_pressure_comp, right->enable_pressure_comp);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_COZIR_LP3, "pressure: %u, %u", left->pressure, right->pressure);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_COZIR_LP3, "enable_abc: %u, %u", left->enable_abc, right->enable_abc);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_COZIR_LP3, "abc_init_period: %u, %u", left->abc_init_period, right->abc_init_period);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_COZIR_LP3, "abc_period: %u, %u", left->abc_period, right->abc_period);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_COZIR_LP3, "abc_target_value: %u, %u", left->abc_target_value, right->abc_target_value);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_COZIR_LP3, "alarm_en: %u, %u", left->alarm_en, right->alarm_en);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_COZIR_LP3, "alarm_treshold_co2_high: %u, %u", left->alarm_treshold_co2_high, right->alarm_treshold_co2_high);
                return false;
            }
            return true;
        }
        case CM1107N:
        {
            if (left->enable_abc != right->enable_abc ||
                left->abc_target_value != right->abc_target_value ||
                left->abc_period != right->abc_period)
            { 
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_CM1107N, "enable_abc: %u, %u", left->enable_abc, right->enable_abc);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_CM1107N, "abc_target_value: %u, %u", left->abc_target_value, right->abc_target_value);
                print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_CM1107N, "abc_period: %u, %u", left->abc_period, right->abc_period);
                return false;
            }
            return true;
        }
        default: return false;
    }
}

static bool sensors_mux_to_sensor(uint8_t sensor_index)
{
    int32_t ret;
    if ((ret = mux_enable_sensor(sensors[sensor_index].input_index)) != 0) // Mux to sensor
    { // Mux failed
        print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_MUX, "Failed to mux sensor %i: e%i", sensor_index, ret);
        sensors[sensor_index].state = ERROR_SENSOR_MUX_FAILED;
        reset_i2c(); // Reset I2C
        mux_reset(); // Reset MUX
        sleep_ms(300);
        return false; // Retry initialization
    }
    return true;
}

bool sensors_is_measurement_finished(void)
{
    if (ms5607.meas_state != MEAS_FINISHED) return false;
    if (hyt271.meas_state != MEAS_FINISHED || !is_at_the_end_of_time(hyt271.wake_time)) return false;
    for (int i = 0; i < 8; i++)
    {
        if (!sensors[i].config.sensor_active) continue;
        if (common_is_measurement_running(&sensors[i])) return false;
    }
    return true;
}

static void set_power(bool on, bool startup)
{
    uint8_t power_vector = 0;
    for (int i = 0; i < 8; i++)
    {
        if (!sensors[sensors[i].power_index].config.sensor_active || (sensors[sensors[i].power_index].config.power_continuous && (!startup || !on))) continue; // Sensor inactive or sensor should be powered continuously
        else if (sensors[sensors[i].power_index].config.power_global_control || startup) //  || (startup && !sensors[sensors[i].power_index].config.power_continuous)
        {
            power_vector |= (0b1 << sensors[i].power_index);
        }
    }
    power_en_set_vector_affected_sensors(power_vector, on);
}

static void set_5v(void)
{
    uint8_t power_vector = 0;
    for (int i = 0; i < 8; i++)
    {
        if (sensors[sensors[i].power_index].config.power_5V)
        {
            power_vector |= (0b1 << sensors[i].power_index);
        }
    }
    power_5v_set_vector(power_vector);
}

static float sensors_sensor_compensate_pressure(float co2_value, float pressure)
{
    return co2_value / (0.004026 * pressure / 10 + 0.0000578 * pressure * pressure / 100);
}

