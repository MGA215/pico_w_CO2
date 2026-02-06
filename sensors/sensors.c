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
#include "ee872/ee872.h"
#include "../eeprom/eeprom.h"
#include "generic_co2/generic_co2.h"


#include "string.h"
#include "math.h"
#include "pico/mutex.h"
#include "hardware/watchdog.h"

#define EEPROM_SENSOR_ADDR_START 0x00000300
#define EEPROM_SENSOR_CONFIG_LEN 0x100

#define N_INIT_TRIES 2
#define N_VERIFY_TRIES 2
#define N_READ_TRIES 2

bool sensors_measurement_ready = false;
bool sensors_was_measurement_read = false;

absolute_time_t sensor_start_measurement_time;

/**
 * @brief Reads sensor configuration from the EEPROM; if reading failed, resets the device
 * 
 * @param out_config Read configuration
 * @param sensor_index Index of the sensor
 */
static void sensors_read_config_from_eeprom(sensor_t* sensor);

/**
 * @brief Sets up sensor structure
 * 
 * @param sensor_index Index of the sensor
 */
static void sensors_init_sensor_struct(uint8_t sensor_index);

/**
 * @brief Reads single sensor configuration
 * 
 * @param configuration Output configuration
 * @param sensor_index Sensor index
 */
static int32_t sensors_read_config(sensor_config_t* configuration, sensor_t* sensor);

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
 * @return int32_t error code (SUCCESS or ERROR_SENSOR_MUX_FAILED)
 */
static int32_t sensors_mux_to_sensor(uint8_t sensor_index);

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

static void sensors_sensor_init(sensor_t* sensor);
static void sensors_sensor_verify(sensor_t* sensor);
static void sensors_sensor_run(sensor_t* sensor);
static void sensors_sensor_run_measurement(sensor_t* sensor);
void sensors_init_trhp_sensor_struct(sensor_t* sensor, sensor_type_e sensor_type);
static void sensors_run_trhp_measurement(sensor_t* sensor);
static void sensors_on_measurement_finish(void);
static void sensors_start_measurement(void);
bool sensors_is_measurement_finished(void);




void sensors_init()
{
    for (int i = 0; i < CONNECTED_SENSORS; i++)
    {
        sensors_init_sensor_struct(i);
    }
    mux_init(); // Initialize MUX
    power_reset_all();
    set_5v();
    set_power(true, true);
    
    sensors_init_trhp_sensor_struct(&ms5607, MS5607); // Initialize MS5607 struct
    watchdog_update();
    
    sensors_init_trhp_sensor_struct(&hyt271, HYT271); // Initialize HYT271 struct

    // mux_init(); // Initialize MUX
    // power_5v_set_vector(0);
    // power_en_set_vector_affected_sensors(0xFF, true);

    // mux_enable_sensor(6);
    // int32_t ret = generic_co2_write_float(0xB0, 1.05f);
    // print_ser_output(SEVERITY_FATAL, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Write return value: %i", ret);
    // float val;
    // while (true)
    // {
    //     uint16_t status_buffer[2];
    //     int32_t ret = generic_co2_read(0x10, 2, status_buffer);
    //     uint32_t status = status_buffer[1] << 16 | status_buffer[0];
    //     print_ser_output(SEVERITY_FATAL, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Status: %08X, return value: %i", status, ret);
    //     ret = generic_co2_read_float(0x0C, &val);
    //     print_ser_output(SEVERITY_FATAL, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Read pressure: %f hPa, return value: %i", val, ret);
    //     ret = generic_co2_read_float(0x18, &val);
    //     print_ser_output(SEVERITY_FATAL, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Read temperature: %f degC, return value: %i", val, ret);
    //     // sleep_ms(5000);
    // }


    watchdog_update();
}

void sensors_init_trhp_sensor_struct(sensor_t* sensor, sensor_type_e sensor_type)
{
    print_ser_output(SEVERITY_DEBUG, SOURCE_SENSORS, SOURCE_MS5607 + sensor_type - MS5607, 
        "Setting up TRHP structure type %i", sensor_type);
    common_init_struct(sensor, 255);
    sensor->sensor_type = sensor_type;
    sensor->config.sensor_active = true; // Activate sensor
    switch (sensor_type)
    {
        case MS5607:
            sensor->functions = &ms5607_functions;
            break;
        case HYT271:
            sensor->functions = &hyt271_functions;
            break;
        default:
            sensor->functions = NULL;
            print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_NO_SOURCE, 
                "Failed to assign function to sensor type %i", sensor_type);
            sensor->error_state = ERROR_SENSOR_UNKNOWN_SENSOR;
            break;
    }
}

void sensors_run()
{
    static uint8_t sensor_index = 0;
    sensor_t* sensor = &sensors[sensor_index];
    sensor_state_e sensor_state = sensor->sensor_state;

    if (sensor->error_state != ERROR_SENSOR_UNKNOWN_SENSOR)
    {
        switch (sensor->sensor_state)
        {
            case NOT_INITIALIZED:
                sensors_sensor_init(sensor);
                if (sensor->error_state == STATE_OK) 
                {
                    sensor_state = INITIALIZED;
                } 
                break;
            case INITIALIZED:
                sensors_sensor_verify(sensor);
                if (sensor->error_state == STATE_OK) 
                {
                    sensor_state = SENSOR_OK;
                } 
                break;
            case SENSOR_OK:
                sensors_sensor_run(sensor);
                break;
        }
    }

    if (sensor->error_state != 0 && sensor->sensor_state != NOT_INITIALIZED) // On sensor error reinitialize
    {
        common_measurement_force_stop(sensor);
        if (global_configuration.reinit_sensors_on_error)
        {
            sensor_state = NOT_INITIALIZED;
        }
    }

    sensor->sensor_state = sensor_state;

    sensor_index = (++sensor_index) % CONNECTED_SENSORS; // increment
    return;
}

static void sensors_sensor_init(sensor_t* sensor)
{
    if (!common_should_sensor_operate(sensor)) return;
    print_ser_output(SEVERITY_DEBUG, SOURCE_SENSORS, SOURCE_EE895 + sensor->sensor_type, 
        "Initializing sensor %i...", sensor->index);

    for (int i = 0; i < N_INIT_TRIES; i++)
    {
        if (sensor->error_state == ERROR_SENSOR_CONFIG_PARSING_FAILED) // Read configuration from EEPROM
        {
            sensors_read_config_from_eeprom(sensor);
            if (sensor->error_state == ERROR_SENSOR_CONFIG_PARSING_FAILED) continue;
        }

        sensor->error_state = sensors_mux_to_sensor(sensor->index);
        if (sensor->error_state) continue;
        if (sensor->functions == NULL || sensor->functions->sensor_init == NULL) // Nonexistent init function
        {
            print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_NO_SOURCE, 
                "Unknown init function on sensor %i", sensor->index);
            sensor->error_state = ERROR_SENSOR_UNKNOWN_SENSOR;
            return;
        }
        sensor->functions->sensor_init(sensor); // Initialize sensor

        if (!sensor->internal_error_state) // Init successful
        {
            print_ser_output(SEVERITY_INFO, SOURCE_SENSORS, SOURCE_EE895 + sensor->sensor_type, 
                "Init sensor %i success", sensor->index);
            common_measurement_force_stop(sensor);
            sensor->error_state = STATE_OK;
            return;
        }
    }
    sensor->error_state = ERROR_SENSOR_INIT_FAILED;
    sensor->err_total_counter++;
    print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_EE895 + sensor->sensor_type, 
        "Failed to initialize sensor %i: %i, internal %i", sensor->index, sensor->error_state, sensor->internal_error_state);
    common_disable_sensor_for_ms(sensor, global_configuration.meas_int_ms); // Disable sensor for 1 measurement period
}

static void sensors_sensor_verify(sensor_t* sensor)
{
    if (sensor->sensor_type == EE872 || sensor->sensor_type == GENERIC_CO2) // on UART type sensor - temporary!!!
    {
        sensor->error_state = STATE_OK;
        return;
    }

    print_ser_output(SEVERITY_DEBUG, SOURCE_SENSORS, SOURCE_EE895 + sensor->sensor_type, 
        "Verifying sensor %i configuration...", sensor->index);

    for (int i = 0; i < N_VERIFY_TRIES; i++)
    {
        sensor->error_state = sensors_mux_to_sensor(sensor->index);
        if (sensor->error_state)
        {
            continue;
        }

        sensor_config_t read_config;
        if (sensors_read_config(&read_config, sensor)) continue; // Read sensor configuration

        if (sensors_compare_config(&read_config, &(sensor->config))) // Verify sensor configuration
        {
            sensor->error_state = STATE_OK;
            print_ser_output(SEVERITY_INFO, SOURCE_SENSORS, SOURCE_EE895 + sensor->sensor_type, 
                "Sensor %i verified", sensor->index);
                return;
        }
    }
    print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_EE895 + sensor->sensor_type, 
        "Failed to verify configuration %i", sensor->index);
    sensor->error_state = ERROR_CONFIG_VERIFICATION_FAILED;
    sensor->err_total_counter++;
}

static void sensors_sensor_run(sensor_t* sensor)
{
    if (time_reached(sensor_start_measurement_time)) // Initialize measurement
    {
        if (!sensors_is_measurement_finished()) // Check if all measurement finished
        {
            static uint8_t iterator = 0;

            sensor_start_measurement_time = make_timeout_time_us(1000 * (uint64_t)global_configuration.meas_int_ms / 10); // Add 1/10 measurement interval

            if (iterator++ >= 20) // Check for total double delay
            {
                for (int i = 0; i < CONNECTED_SENSORS; i++) // Force all sensors to stop
                {
                    common_measurement_force_stop(&sensors[i]);
                }
            }
            iterator %= 20;
        }
        
        if (sensors_is_measurement_finished())
        {
            sensors_start_measurement();
        }
    }

    if (common_should_sensor_operate(sensor)) // Should sensor react
    {
        sensors_sensor_run_measurement(sensor);
    }

    if (common_should_sensor_operate(&ms5607)) // Should pressure sensor react
    {
        sensors_run_trhp_measurement(&ms5607);
    }

    if (common_should_sensor_operate(&hyt271)) // Should TRH sensor react
    {
        sensors_run_trhp_measurement(&hyt271);
    }

    if (sensors_is_measurement_finished() && !sensors_measurement_ready) // On measurement finished - single operation
    {
        sensors_on_measurement_finish();
        if (!sensors_was_measurement_read) sensors_measurement_ready = true; // Set measurement ready
    }    
}

static void sensors_run_trhp_measurement(sensor_t* sensor)
{
    print_ser_output(SEVERITY_DEBUG, SOURCE_SENSORS, SOURCE_MS5607 + sensor->sensor_type - MS5607, 
        "Reading sensor");
    for (int i = 0; i < 2; i++) // Try measurement twice
    {
        sensor->functions->sensor_get_value(sensor);

        if (!sensor->internal_error_state) // On no error
        {
            sensor->error_state = STATE_OK;
            return;
        }
        if (i == 0) // on first iteration
        {
            common_measurement_start(sensor); // Try anothoer measurement
        }
        sleep_ms(2);
    }
    sensor->err_total_counter++;
    sensor->error_state = ERROR_SENSOR_READING_FAILED;
    print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_MS5607 + sensor->sensor_type - MS5607,
        "Failed to read sensor: %i, internal %i", sensor->error_state, sensor->internal_error_state);
}

static void sensors_sensor_run_measurement(sensor_t* sensor)
{
    print_ser_output(SEVERITY_DEBUG, SOURCE_SENSORS, SOURCE_EE895 + sensor->sensor_type,
        "Reading sensor %i", sensor->index);

    for (int i = 0; i < N_READ_TRIES; i++)
    {
        if (sensor->config.sensor_IIC)
        {
            sensor->error_state = sensors_mux_to_sensor(sensor->index);
            if (sensor->error_state) continue;
        }

        if (sensor->functions == NULL || sensor->functions->sensor_get_value == NULL) // Nonexistent init function
        {
            print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_NO_SOURCE, 
                "Unknown get_value function on sensor %i", sensor->index);
            sensor->error_state = ERROR_SENSOR_UNKNOWN_SENSOR;
            return;
        }

        sensor->functions->sensor_get_value(sensor);

        if (!sensor->internal_error_state)
        {
            sensor->error_state = STATE_OK;
            return;
        }
    }
    sensor->error_state = ERROR_SENSOR_READING_FAILED;
    sensor->err_total_counter++;
    print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_EE895 + sensor->sensor_type,
        "Failed to read sensor %i: %i, internal %i", sensor->index, sensor->error_state, sensor->internal_error_state);
}

static void sensors_init_sensor_struct(uint8_t sensor_index)
{
    print_ser_output(SEVERITY_DEBUG, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Setting up structure %i", sensor_index);
    if (sensor_index >= CONNECTED_SENSORS) return;
    sensor_t* sensor = &sensors[sensor_index];

    common_init_struct(sensor, sensor_index); // Initialize sensor structure

    sensors_read_config_from_eeprom(sensor); // Read sensor config from EEPROM

    if (sensor->sensor_type < 0 || sensor->sensor_type >= SENSOR_TYPES) // Check for invalid sensor type
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_NO_SOURCE, 
                            "Unknown sensor at input %x, init abort", sensor->index);
        sensor->error_state = ERROR_SENSOR_UNKNOWN_SENSOR; // Unknown sensor
        sensor->sensor_type = UNKNOWN;
    }
}

static void sensors_read_config_from_eeprom(sensor_t* sensor)
{
    int32_t ret;
    uint8_t buffer[EEPROM_SENSOR_CONFIG_LEN];
    ret = eeprom_read(EEPROM_SENSOR_ADDR_START + EEPROM_SENSOR_CONFIG_LEN * sensor->index, 
                        buffer, EEPROM_SENSOR_CONFIG_LEN); // Read config from EEPROM
    if (ret) // Reading failed
    {
        print_ser_output(SEVERITY_FATAL, SOURCE_SENSORS, SOURCE_EEPROM, "Failed to read configuration %i from EEPROM, resetting device...", sensor->index);
        watchdog_enable(1, 1); // reset
        while (true) tight_loop_contents();
        return;
    }

    sensor_config_t config;
    ret = serializer_deserialize(&config, buffer, EEPROM_SENSOR_CONFIG_LEN); // Parsing read configuration
    if (ret) // If parsing failed
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Failed to parse configuration on sensor %i", sensor->index);
        sensor->error_state = ERROR_SENSOR_CONFIG_PARSING_FAILED;
        return;
    }
    memcpy(&(sensor->config), &config, sizeof(sensor_config_t)); // Assign configuration
    sensor->sensor_type = sensor->config.sensor_type;
    sensor->sensor_number = sensor->config.sensor_ord; // Set sensor type index (for differentiating same type sensors)

    switch (sensor->sensor_type) // assign functions
    {
        case EE895:
            if (sensor->config.sensor_IIC) sensor->functions = &ee895_functions_i2c;
            else sensor->functions = NULL;
            break;
        case CDM7162:
            if (sensor->config.sensor_IIC) sensor->functions = &cdm7162_functions;
            else sensor->functions = NULL;
            break;
        case SUNRISE:
            if (sensor->config.sensor_IIC) sensor->functions = &sunrise_functions;
            else sensor->functions = NULL;
            break;
        case SUNLIGHT:
            if (sensor->config.sensor_IIC) sensor->functions = &sunlight_functions;
            else sensor->functions = NULL;
            break;
        case SCD30:
            if (sensor->config.sensor_IIC) sensor->functions = &scd30_functions;
            else sensor->functions = NULL;
            break;
        case SCD41:
            if (sensor->config.sensor_IIC) sensor->functions = &scd41_functions;
            else sensor->functions = NULL;
            break;
        case COZIR_LP3:
            if (sensor->config.sensor_IIC) sensor->functions = &cozir_lp3_functions;
            else sensor->functions = NULL;
            break;
        case CM1107N:
            if (sensor->config.sensor_IIC) sensor->functions = &cm1107n_functions;
            else sensor->functions = NULL;
            break;
        case EE872:
            if (sensor->config.sensor_IIC) sensor->functions = NULL;
            else sensor->functions = &ee872_functions_uart;
            break;
        case GENERIC_CO2:
            sensor->functions = &generic_co2_functions;
        default:
            sensor->functions = NULL;
            break;
    }

    return;
}

static int32_t sensors_mux_to_sensor(uint8_t sensor_index)
{
    int32_t ret;
    if ((ret = mux_enable_sensor(sensors[sensor_index].input_index)) != 0) // Mux to sensor
    { // Mux failed
        sensors[sensor_index].internal_error_state = ret;
        print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_MUX, "Failed to mux sensor %i: e%i", sensor_index, ret);
        reset_i2c(); // Reset I2C
        mux_reset(); // Reset MUX
        // sleep_ms(300);
        return ERROR_SENSOR_MUX_FAILED;
    }
    return SUCCESS;
}

static int32_t sensors_read_config(sensor_config_t* configuration, sensor_t* sensor)
{
    int32_t ret;

    print_ser_output(SEVERITY_DEBUG, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Reading configuration %i...", sensor->index);

    if (sensor->functions != NULL && sensor->functions->sensor_read_config != NULL) // read sensor configuration
    {
        ret = sensor->functions->sensor_read_config(configuration, sensor->config.single_meas_mode);
    }
    else 
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_NO_SOURCE, 
            "Unknown read config function on sensor %i", sensor->index);
        sensor->error_state = ERROR_SENSOR_UNKNOWN_SENSOR;
        return ERROR_SENSOR_UNKNOWN_SENSOR;
    }

    if (ret) // Error during config reading
    {
        memset(configuration, 0x00, sizeof(sensor_config_t)); // Clear config
        configuration->sensor_type = UNKNOWN; // Unknown sensor type
        print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Failed to read config %i: %i", sensor->index, ret);
    }
    else // Assign power & channel variables
    {
        print_ser_output(SEVERITY_DEBUG, SOURCE_SENSORS, SOURCE_NO_SOURCE, "Successfully read configuration %i", sensor->index);
        configuration->co2_en = sensor->config.co2_en; // Set some sw parameters
        configuration->temp_en = sensor->config.temp_en;
        configuration->RH_en = sensor->config.RH_en;
        configuration->pressure_en = sensor->config.pressure_en;
        configuration->power_5V = sensor->config.power_5V;
        configuration->power_global_control = sensor->config.power_global_control;
    }
    return ret;
}

static bool sensors_compare_config(sensor_config_t* left, sensor_config_t* right)
{
    if (left->sensor_type != right->sensor_type) // Check sensor type mismatch
    {
        print_ser_output(SEVERITY_WARN, SOURCE_SENSORS, SOURCE_NO_SOURCE, 
            "Config sensor type mismatch: left %i, right %i", left->sensor_type, right->sensor_type);
        return false;
    }

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
        default: return true;
    }
}

bool sensors_is_measurement_finished(void)
{
    if (common_is_measurement_running(&ms5607)) return false;
    if (common_is_measurement_running(&hyt271)) return false;
    for (int i = 0; i < CONNECTED_SENSORS; i++)
    {
        if (!sensors[i].config.sensor_active) continue;
        if (common_is_measurement_running(&sensors[i])) return false;
    }
    return true;
}

static void set_power(bool on, bool startup)
{
    uint8_t power_vector = 0;
    for (int i = 0; i < CONNECTED_SENSORS; i++)
    {
        sensor_t* sensor = &sensors[sensors[i].power_index];
        if (!sensor->config.sensor_active || (sensor->config.power_continuous && (!startup || !on))) continue; // Sensor inactive or sensor should be powered continuously
        else if (sensor->config.power_global_control || startup) //  || (startup && !sensors[sensors[i].power_index].config.power_continuous)
        {
            power_vector |= (0b1 << sensors[i].power_index);
        }
    }
    power_en_set_vector_affected_sensors(power_vector, on);
}

static void set_5v(void)
{
    uint8_t power_vector = 0;
    for (int i = 0; i < CONNECTED_SENSORS; i++)
    {
        if (sensors[sensors[i].power_index].config.power_5V)
        {
            power_vector |= (0b1 << sensors[i].power_index);
        }
    }
    power_5v_set_vector(power_vector);
}

static void sensors_start_measurement(void)
{
    static uint32_t n_measurement;

    print_ser_output(SEVERITY_INFO, SOURCE_SENSORS, SOURCE_NO_SOURCE, 
        "Starting measurement no. %i", ++n_measurement);
    common_measurement_start(&ms5607);
    common_measurement_start(&hyt271);
    for (int i = 0; i < CONNECTED_SENSORS; i++)
    {
        if (sensors[i].error_state == ERROR_SENSOR_UNKNOWN_SENSOR) continue;
        common_measurement_start(&sensors[i]);
    }
    sensor_start_measurement_time = make_timeout_time_us(1000 * (uint64_t)global_configuration.meas_int_ms);
    sensors_was_measurement_read = false;
    sensors_measurement_ready = false;
}

static void sensors_on_measurement_finish(void)
{
    set_power(false, false);
    for (int i = 0; i < CONNECTED_SENSORS; i++)
    {
        if (!sensors[i].config.sensor_active) continue;
        if (sensors[i].config.ext_pressure_comp && ms5607.pressure != NAN && sensors[i].error_state == STATE_OK) // Compensate for pressure
        {
            float val = sensors_sensor_compensate_pressure(sensors[i].co2, ms5607.pressure);
            print_ser_output(SEVERITY_DEBUG, SOURCE_SENSORS, SOURCE_EE895 + sensors[i].sensor_type, 
                "Pressure compensation of sensor %i: %.0f -> %.0f", sensors[i].index, sensors[i].co2, val);
            sensors[i].co2 = val;
        }
        print_ser_output(SEVERITY_INFO, SOURCE_SENSORS, SOURCE_EE895 + sensors[i].sensor_type, 
            "Input: %i, Errors: %i", i, sensors[i].err_total_counter);
    }
}

static float sensors_sensor_compensate_pressure(float co2_value, float pressure)
{
    return co2_value / (0.004026 * pressure / 10 + 0.0000578 * pressure * pressure / 100);
}
