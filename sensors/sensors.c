#include "sensors.h"

#include "../common/functions.h"
#include "../common/debug.h"
#include "../common/i2c_extras.h"
#include "../error_codes.h"
#include "../shared.h"

#include "ee895/ee895.h"
#include "cdm7162/cdm7162.h"
#include "sunrise/sunrise.h"
#include "sunlight/sunlight.h"
#include "scd30/scd30.h"
#include "scd41/scd41.h"
#include "cozir-lp3/cozir-lp3.h"
#include "cm1107n/cm1107n.h"
#include "mux/mux.h"

#include "string.h"
#include "math.h"
#include "pico/mutex.h"

// Must be increased with each new sensor type
#define SENSOR_TYPES 8

sensor_t sensors[8];

sensor_config_t* sensors_config[8];

bool read_config_trigger = false;

uint8_t read_config_number = 0;

// Holds numbers of each sensor type, created for 8 sensor types
static uint8_t sensor_indices[SENSOR_TYPES];

// Vector of sensors with assigned configuration
static uint8_t active_sensors;

// Timer to start measurement
absolute_time_t sensor_start_measurement_time;

// Interval between sensor measurement starts in seconds
uint32_t sensor_measurement_interval_s = 6;


/**
 * @brief Initializes sensor itself according to its type
 * 
 * @param sensor Sensor to initialize
 * @param configuration Configuration to assign to the sensor
 * @param is_first_init Is sensor initialized for the first time
 * @return int32_t Initialization return code
 */
int32_t sensors_init_sensor_type(sensor_t* sensor, sensor_config_t* configuration);

/**
 * @brief Reads sensor according to its type
 * 
 * @param sensor Sensor to read
 */
void sensors_read_sensor_type(sensor_t* sensor);

/**
 * @brief Set the power on globally controlled sensors to [on]
 * 
 * @param on Whether the power should be turned on or off
 */
void set_power(bool on);

/**
 * @brief Attempts to start a new measurement
 * 
 * @return true if measurement successfully started
 * @return false if measurement still running
 */
bool sensors_start_measurement(void);

/**
 * @brief Reads measured values from all sensors
 * 
 */
void sensors_read_all(void);

/**
 * @brief Reads measured value from a sensor at a specific sensor_index
 * 
 * @param sensor_index Index of the sensor
 * @return true if sensor read successfully
 * @return false if sensor reading failed
 */
bool sensors_read(uint8_t sensor_index);

/**
 * @brief Reads all sensors configuration
 * 
 * @param configuration Output configuration
 * @param configuration_count Number of configurations to read
 */
void sensors_read_config_all(sensor_config_t** configuration, uint8_t configuration_count);

/**
 * @brief Reads single sensor configuration
 * 
 * @param configuration Output configuration
 * @param sensor_index Sensor index
 */
void sensors_read_config(sensor_config_t* configuration, uint8_t sensor_index);

/**
 * @brief Compares two sensor configurations
 * 
 * @param left configuration to compare
 * @param right configuration to compare
 * @return true if configurations are the same
 * @return false if configurations differ
 */
bool sensors_compare_config(sensor_config_t* left, sensor_config_t* right);




void sensors_init_all(sensor_config_t** configuration_map, uint8_t config_map_length)
{
    // ToDo: Read config from EEPROM for init, replace configuration map with this new configuration
    for (int i = 0; i < 8; i++) // Try initialize mutexes for sensor configurations
    {
        if (!mutex_is_initialized(&sensors_config_all[i].sensor_config_mutex))
        {
            mutex_init(&sensors_config_all[i].sensor_config_mutex);
        }
    }

    init_sensor_i2c(); // Initialize sensor I2C
    mux_init(); // Initialize MUX

    for (int i = 0; i < 8; i++)
    {
        sensor_indices[i] = 0; // Reset sensor indices
    }
    active_sensors = 0;
    for (int i = 0; i < MIN(8, config_map_length); i++)
    {
        sensors_init(i, configuration_map[i], true); // Initialize sensor
    }
    sensor_start_measurement_time = make_timeout_time_us(sensor_measurement_interval_s * 1000000); // Set measurement start timer
}

bool sensors_init(uint8_t sensor_index, sensor_config_t* configuration, bool is_first_init)
{
    int32_t ret; 
    sensors[sensor_index].state = ERROR_SENSOR_INIT_FAILED;

    for (int i = 0; i < 2; i++) // Try initialization twice
    {
        sleep_ms(10); // Wait some time

        active_sensors &= ~(0b1 << sensor_index); // Set sensor to inactive

        print_ser_output(SEVERITY_DEBUG, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Init structure %i", sensor_index);
        common_init_struct(&sensors[sensor_index], sensor_index); // Initialize sensor structure
        sensors[sensor_index].sensor_type = configuration != NULL ? configuration->sensor_type : UNKNOWN; // Copy sensor type to sensor structure
        
        if (configuration == NULL) return false; // Not initiable

        if (sensors[sensor_index].sensor_type < 0 || sensors[sensor_index].sensor_type >= SENSOR_TYPES) // Check for valid sensor type
        {
            print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_NO_SOURCE, 
                "Unknown sensor %x?, init abort", sensors[sensor_index].sensor_type); // No type match - unknown sensor
            sensors[sensor_index].state = ERROR_UNKNOWN_SENSOR;
            return false; // Not initiable
        }
        if (is_first_init && i == 0)  // First init, first iteration
        {
            sensors[sensor_index].sensor_number = sensor_indices[sensors[sensor_index].sensor_type]; // Set sensor type index
            sensor_indices[sensors[sensor_index].sensor_type]++; // Increment sensor type counter
        }

        if ((ret = mux_enable_sensor(sensors[sensor_index].input_index)) != 0) // Mux to sensor
        {
            active_sensors |= (0b1 << sensor_index); // Set sensor active
            print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_MUX, "Failed to mux sensor %i: e%i", sensor_index, ret); // On invalid MUX
            reset_i2c(); // Reset I2C
            mux_reset(); // Reset MUX
            continue; // Retry initialization
        }

        print_ser_output(SEVERITY_DEBUG, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Initializing sensor %i...", sensor_index);
        ret = sensors_init_sensor_type(&sensors[sensor_index], configuration);

        if (sensors[sensor_index].state == ERROR_UNKNOWN_SENSOR) // Unknown sensor
        {
            reset_i2c(); // Reset I2C
            return false; // Not initiable
        }
        active_sensors |= (0b1 << sensor_index); // Set sensor active
        if (!ret) print_ser_output(SEVERITY_INFO, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Init sensor %i success", sensor_index);
        else 
        {   
            print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Init sensor %i failed: %i", sensor_index, ret); 
            reset_i2c(); // Reset I2C
            mux_reset(); // Reset MUX
            sensors[sensor_index].state = ERROR_SENSOR_INIT_FAILED; // Sensor initialization failed
            continue; // Retry initialization
        }

        sensors[sensor_index].state = ERROR_NO_MEAS; // Sensor successfully initialized

        sensor_config_t config;
        sensors_read_config(&config, sensor_index); // Read sensor config
        if (!sensors_compare_config(sensors[sensor_index].config, &config)) // Compare config with the one set
        {
            print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Configuration %i mismatch", sensor_index);
            if (config.sensor_type != UNKNOWN) // Sensor actually has a configuration
                memcpy(sensors[i].config, &config, sizeof(sensor_config_t)); // Update configuration
        }
        else 
        {
            print_ser_output(SEVERITY_INFO, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Configuration %i verified", sensor_index);
            mutex_enter_timeout_ms(&sensors_config_all[sensor_index].sensor_config_mutex, 1000); // Safe copy configuration
            memcpy(&sensors_config_all[sensor_index], &config, sizeof(sensor_config_t));
            mutex_exit(&sensors_config_all[sensor_index].sensor_config_mutex);
        }
        return true; // Initialization successful
    }
    return false; // Initialization attempt 2 failed
}

int32_t sensors_init_sensor_type(sensor_t* sensor, sensor_config_t* configuration)
{
    int32_t ret;
    if (sensor->sensor_type < 0 || sensor->sensor_type >= SENSOR_TYPES) // Check for valid sensor type
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_NO_SOURCE, 
            "Unknown sensor %x?, init abort", sensor->sensor_type); // No type match - unknown sensor
        sensor->state = ERROR_UNKNOWN_SENSOR;
        return ERROR_UNKNOWN_SENSOR;
    }
    print_ser_output(SEVERITY_DEBUG, SOURCE_SENSORS, SOURCE_EE895 + sensor->sensor_type, 
        "Init sensor to type %x%x...", sensor->sensor_type, sensor->sensor_number);
    switch (sensor->sensor_type) // For sensor type
    {
        case EE895:
        {
            ret = ee895_init(sensor, configuration); // Initialize EE895 sensor
            break;
        }
        case CDM7162:
        {
            ret = cdm7162_init(sensor, configuration); // Initialize CDM7162 sensor
            break;
        }
        case SUNRISE:
        {
            ret = sunrise_init(sensor, configuration); // Initialize SUNRISE sensor
            break;
        }
        case SUNLIGHT:
        {
            ret = sunlight_init(sensor, configuration); // Initialize SUNLIGHT sensor
            break;
        }
        case SCD30:
        {
            ret = scd30_init(sensor, configuration); // Initialize SCD30 sensor
            break;
        }
        case SCD41:
        {
            ret = scd41_init(sensor, configuration); // Initialize SCD41 sensor
            break;
        }
        case COZIR_LP3:
        {
            ret = cozir_lp3_init(sensor, configuration); // Initialize CozIR-LP3 sensor
            break;
        }
        case CM1107N:
        {
            ret = cm1107n_init(sensor, configuration); // Initialize CM1107N sensor
            break;
        }
        default:
        {
            print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_NO_SOURCE, 
                "Unknown sensor %x?, init abort", sensor->sensor_type); // No type match - unknown sensor
            sensor->state = ERROR_UNKNOWN_SENSOR;
            return ERROR_UNKNOWN_SENSOR;
        }
    }
    return ret;
}

void sensors_read_all(void)
{
    int32_t ret = -99; // random default return value

    for (uint8_t i = 0; i < 8; i++) // Iterate sensor
    {
        if (time_reached(sensors[i].wake_time) && (active_sensors & (0b1 << i))) // If sensor should react to a timer reached
        {
            for (uint8_t j = 0; j < 2; j++)
            {
                if (sensors_read(i)) break; // If reading successful break
                sleep_ms(10);
            }
        }
    }

    if (sensors_is_measurement_finished()) // If all measurements finished - turn off power globally if possible
    {
        set_power(false);
    }
    return;
}

bool sensors_read(uint8_t sensor_index)
{
    int32_t ret;
    for (uint8_t j = 0; j < 2; j++)
    {
        sleep_ms(10);

        if ((ret = mux_enable_sensor(sensors[sensor_index].input_index)) != 0) // Mux to sensor
        {
            print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_MUX, "Failed to mux sensor %i: e%i", sensor_index, ret);
            reset_i2c(); // Reset i2c
            common_init_struct(&sensors[sensor_index], sensor_index); // Reset sensor
            sensors[sensor_index].state = ERROR_SENSOR_MUX_FAILED; // Set sensor state to MUX failed
            mux_reset(); // Reset MUX
            continue; // Repeat measurement
        }
        
        if (sensors[sensor_index].state && // Sensor not initialized
            sensors[sensor_index].state != ERROR_SENSOR_NOT_INITIALIZED && 
            sensors[sensor_index].state != ERROR_NO_MEAS)
        {
            if (!sensors_init(sensor_index, sensors[sensor_index].config, false)) continue; // Initialize sensor
            sensors[sensor_index].meas_state = MEAS_STARTED;
        }

        if (sensors[sensor_index].state == SUCCESS || 
            sensors[sensor_index].state == ERROR_NO_MEAS) // If sensor initialized
        {
            print_ser_output(SEVERITY_DEBUG, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Reading sensor %i...", sensor_index);
            sensors_read_sensor_type(&sensors[sensor_index]); // Read sensor type

            if (sensors[sensor_index].state && sensors[sensor_index].state != ERROR_NO_MEAS) // Reading not successful
            {
                print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Reading sensor %i failed: %i", sensor_index, sensors[sensor_index].state);
                reset_i2c(); // Reset I2C
                mux_reset(); // Reset MUX
                continue; // Reading failed, repeat measurement
            }
            if (sensors[sensor_index].meas_state == MEAS_FINISHED && sensors[sensor_index].state == SUCCESS && is_at_the_end_of_time(sensors[sensor_index].wake_time))
            {
                print_ser_output(SEVERITY_INFO, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Successfully read sensor %i", sensor_index);
            }
            return true; // Sensor successfully read
        }
    }
    return false;
}

void sensors_read_sensor_type(sensor_t* sensor)
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
    switch (sensor->sensor_type) // Get value based on sensor type
    {
        case EE895:
        {
            ee895_get_value(sensor); // Read EE895 values
            break;
        }
        case CDM7162:
        {
            cdm7162_get_value(sensor); // Read CDM7162 values
            break;
        }
        case SUNRISE:
        {
            sunrise_get_value(sensor); // Read SUNRISE values
            break;
        }
        case SUNLIGHT:
        {
            sunlight_get_value(sensor); // Read SUNLIGHT values
            break;
        }
        case SCD30:
        {
            scd30_get_value(sensor); // Read SCD30 values
            break;
        }
        case SCD41:
        {
            scd41_get_value(sensor); // Read SCD41 values
            break;
        }
        case COZIR_LP3:
        {
            cozir_lp3_get_value(sensor); // Read CozIR-LP3 values
            break;
        }
        case CM1107N:
        {
            cm1107n_get_value(sensor); // Read CM1107N values
            break;
        }
        default:
        {
            print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_NO_SOURCE, 
                "Unknown sensor %x?, read abort", sensor->sensor_type); // No type match - unknown sensor
            sensor->state = ERROR_UNKNOWN_SENSOR;
            break;
        }
    }
}

bool sensors_is_measurement_finished(void)
{
    for (int i = 0; i < 8; i++)
    {
        if (sensors[i].meas_state != MEAS_FINISHED || !is_at_the_end_of_time(sensors[i].wake_time)) return false;
    }
    return true;
}

bool sensors_start_measurement(void)
{
    if (sensors_is_measurement_finished())
    {
        print_ser_output(SEVERITY_INFO, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Measurement starting...");
        for (int i = 0; i < 8; i++)
        {
            if (sensors[i].state != ERROR_SENSOR_NOT_INITIALIZED) 
            {
                sensors[i].meas_state = MEAS_STARTED;
                sensors[i].wake_time = get_absolute_time();
            }
        }       
        return true; 
    }
    return false;
}

void set_power(bool on)
{
    uint8_t power_vector = 0;
    for (int i = 0; i < 8; i++)
    {
        if (sensors[sensors[i].power_index].config->power_global_control)
        {
            power_vector |= (0b1 << sensors[i].power_index);
        }
    }
    // read vector
    uint8_t read_vector;
    if (on) read_vector |= power_vector;
    else read_vector &= ~(power_vector);
    // write vector
}

void sensors_read_sensors(void)
{
    if (time_reached(sensor_start_measurement_time)) // Should new measurement be started
    {
        bool out = sensors_start_measurement(); // Start new measurement
        if (!out) 
        {
            print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Cannot start new measurement.");
            sensor_start_measurement_time = make_timeout_time_ms(100);
        }
        else sensor_start_measurement_time = make_timeout_time_us(sensor_measurement_interval_s * 1000000);
    }
    if (!sensors_is_measurement_finished()) // If measurement running
    {
        sensors_read_all(); // Read all sensors
    }
}

void sensors_read_config_all(sensor_config_t** configuration, uint8_t configuration_count)
{
    for (int i = 0; i < MIN(configuration_count, 8); i++)
    {
        memset(configuration[i], 0x00, sizeof(sensor_config_t));
        sensors_read_config(configuration[i], i);
    }
}

void sensors_read_config(sensor_config_t* configuration, uint8_t sensor_index)
{
    int32_t ret = 0;
    if (sensors[sensor_index].state == ERROR_SENSOR_INIT_FAILED || sensors[sensor_index].state == ERROR_SENSOR_NOT_INITIALIZED) return;
    print_ser_output(SEVERITY_DEBUG, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Reading configuration %i...", sensor_index);
    switch(sensors[sensor_index].sensor_type)
    {
        case EE895:
            ret = ee895_read_config(configuration);
            break;
        case CDM7162:
            ret = cdm7162_read_config(configuration);
            break;
        case SUNRISE:
            ret = sunrise_read_config(configuration);
            break;
        case SUNLIGHT:
            ret = sunlight_read_config(configuration);
            break;
        case SCD30:
            ret = scd30_read_config(configuration);
            break;
        case SCD41:
            ret = scd41_read_config(configuration, sensors[sensor_index].config->single_meas_mode);
            break;
        case COZIR_LP3:
            ret = cozir_lp3_read_config(configuration);
            break;
        case CM1107N:
            ret = cm1107n_read_config(configuration);
            break;
        default:
            ret = ERROR_UNKNOWN_SENSOR;
            print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Unknown sensor %i, read config abort...", sensor_index);
            return;
    }
    if (ret)
    {
        memset(configuration, 0x00, sizeof(sensor_config_t));
        configuration->sensor_type = UNKNOWN;
        print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Failed to read config %i: %i", sensor_index, ret);
    }
    else
    {
        print_ser_output(SEVERITY_INFO, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Successfully read configuration %i", sensor_index);
        configuration->co2_en = sensors[sensor_index].config->co2_en;
        configuration->temp_en = sensors[sensor_index].config->temp_en;
        configuration->RH_en = sensors[sensor_index].config->RH_en;
        configuration->pressure_en = sensors[sensor_index].config->pressure_en;
        configuration->power_5V = sensors[sensor_index].config->power_5V;
        configuration->power_global_control = sensors[sensor_index].config->power_global_control;
    }
    return;
}

bool sensors_compare_config(sensor_config_t* left, sensor_config_t* right)
{
    if (left->sensor_type != right->sensor_type) return false;
    switch(left->sensor_type)
    {
        case EE895:
        {
            if (left->meas_period != right->meas_period ||
                left->single_meas_mode != right->single_meas_mode ||
                left->filter_coeff != right->filter_coeff ||
                left->co2_offset != right->co2_offset) return false;
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
                left->alarm_treshold_co2_low != right->alarm_treshold_co2_low) return false;
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
                left->enable_pressure_comp && (left->pressure != right->pressure) ||
                left->enable_abc != right->enable_abc ||
                left->abc_period != right->abc_period ||
                left->abc_target_value != right->abc_target_value) return false;
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
                left->enable_pressure_comp && (left->pressure != right->pressure) ||
                left->enable_abc != right->enable_abc ||
                left->abc_period != right->abc_period ||
                left->abc_target_value != right->abc_target_value) return false;
            return true;
        }
        case SCD30:
        {
            if (left->meas_period != right->meas_period ||
                left->temperature_offset != right->temperature_offset ||
                left->enable_pressure_comp != right->enable_pressure_comp ||
                left->enable_pressure_comp && (left->pressure != right->pressure) ||
                left->enable_altitude_comp != right->enable_altitude_comp ||
                left->enable_altitude_comp && (left->altitude != right->altitude) ||
                left->enable_abc != right->enable_abc) return false;
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
                left->abc_period != right->abc_period) return false;
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
                left->alarm_treshold_co2_high != right->alarm_treshold_co2_high) return false;
            return true;
        }
        case CM1107N:
        {
            if (left->enable_abc != right->enable_abc ||
                left->abc_target_value != right->abc_target_value ||
                left->abc_period != right->abc_period) return false;
            return true;
        }
        default: return false;
    }
}



