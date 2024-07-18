#include "sensors.h"

#include "../common/functions.h"
#include "../common/debug.h"
#include "../error_codes.h"

#include "ee895/ee895.h"
#include "cdm7162/cdm7162.h"
#include "sunrise/sunrise.h"
#include "sunlight/sunlight.h"
#include "scd30/scd30.h"
#include "scd41/scd41.h"
#include "cozir-lp3/cozir-lp3.h"
#include "cm1107n/cm1107n.h"
#include "mux/mux.h"

#define MAX(A, B) A > B ? A : B
#define MIN(A, B) A > B ? B : A

// Must be increased with each new sensor type
#define SENSOR_TYPES 8

// Holds numbers of each sensor type, created for 8 sensor types
static uint8_t sensor_indices[SENSOR_TYPES];

// Vector of sensors with assigned configuration
static uint8_t active_sensors;

static uint8_t sensor_timer_vector;

static uint8_t sensor_measurement_vector;

/**
 * @brief Initializes sensor itself according to its type
 * 
 * @param sensor Sensor to initialize
 * @param configuration Configuration to assign to the sensor
 * @param is_first_init Is sensor initialized for the first time
 * @return int32_t Initialization return code
 */
int32_t sensors_init_sensor_type(sensor_t* sensor, sensor_config_t* configuration, bool is_first_init);

/**
 * @brief Updates sensor timer vector
 * 
 */
void sensors_update_timer_vector(void);

void sensors_init_all(sensor_config_t** configuration_map, uint8_t config_map_length)
{
    for (int i = 0; i < 8; i++)
    {
        sensor_indices[i] = 0; // Reset sensor indices
    }
    active_sensors = 0;
    for (int i = 0; i < MIN(8, config_map_length); i++)
    {
        sensors_init(i, configuration_map[i], true);
        active_sensors |= configuration_map[i] != NULL ? (0b1 << i) : 0;
    }
}

void sensors_init(uint8_t sensor_index, sensor_config_t* configuration, bool is_first_init)
{
    int32_t ret; 

    for (int i = 0; i < 2; i++)
    {
        sleep_ms(10); // Wait some time

        print_ser_output(SEVERITY_DEBUG, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Init structure %i", sensor_index);
        common_init_struct(&sensors[sensor_index], sensor_index); // Initialize sensor structure
        sensors[sensor_index].sensor_type = configuration != NULL ? configuration->sensor_type : UNKNOWN; // Copy sensor type to sensor structure
        
        if (configuration == NULL) return; // Check valid configuration

        if ((ret = mux_enable_sensor(sensor_index)) != 0) // Mux to sensor
        {
            print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_MUX, "Failed to mux sensor %i: e%i", sensor_index, ret); // On invalid MUX
            reset_i2c(); // Reset I2C
            mux_reset(); // Reset MUX
            continue; // Retry initialization
        }

        print_ser_output(SEVERITY_DEBUG, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Initializing sensor %i...", sensor_index);
        ret = sensors_init_sensor_type(&sensors[sensor_index], configuration, is_first_init);

        if (sensors[sensor_index].state == ERROR_UNKNOWN_SENSOR) // Unknown sensor
        {
            reset_i2c(); // Reset I2C
            return; // Stop initialization attempts
        }
        if (!ret) print_ser_output(SEVERITY_INFO, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Init sensor %i success", sensor_index);
        else 
        {   
            print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Init sensor %i failed: %i", sensor_index, ret); 
            reset_i2c(); // Reset I2C
            sensors[sensor_index].state = ERROR_SENSOR_INIT_FAILED; // Sensor initialization failed
            continue; // Retry initialization
        }

        sensors[sensor_index].state = ERROR_NO_MEAS; // Sensor successfully initialized
        return; // Stop initialization attempts
    }
}

int32_t sensors_init_sensor_type(sensor_t* sensor, sensor_config_t* configuration, bool is_first_init)
{
    int32_t ret;
    if (sensor->sensor_type < 0 || sensor->sensor_type >= SENSOR_TYPES) // Check for valid sensor type
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_NO_SOURCE, 
            "Unknown sensor %x?, init abort", sensor->sensor_type); // No type match - unknown sensor
        sensor->state = ERROR_UNKNOWN_SENSOR;
        return;
    }
    print_ser_output(SEVERITY_DEBUG, SOURCE_SENSORS, SOURCE_EE895 + sensor->sensor_type, 
        "Init sensor to type %x%x...", sensor->sensor_type, sensor_indices[sensor->sensor_type]);
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
            break;
        }
    }
    if (is_first_init) sensor->sensor_number = sensor_indices[sensor->sensor_type]++; // Set sensor type index
}

void sensors_read_all(void)
{
    sensors_update_timer_vector();
    int32_t ret = -99; // random default return value

    for (uint8_t i = 0; i < 8; i++) // Iterate sensor
    {
        if ((sensor_timer_vector & (0b1 << i)) && (active_sensors & (0b1 << i))) // If sensor should react to a timer reached
        {
            sensor_timer_vector &= ~(0b1 << i); // Clear timer reached bit
            for (uint8_t j = 0; j < 2; j++)
            {
                if (sensors_read(i)) break; // If reading successful break
                sleep_ms(10);
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
    return;
}

bool sensors_read(uint8_t sensor_index)
{

}

void sensors_update_timer_vector(void)
{

}


