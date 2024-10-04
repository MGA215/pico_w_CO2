/**
 * @file structs.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Defines main structs and enums used in the project
 * @version 0.1
 * @date 2024-06-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __STRUCTS_H__
#define __STRUCTS_H__

#include "pico/stdlib.h"
#include "constants.h"
#include "pico/mutex.h"
// #if FULL_BUILD
// #include "../service_comm/service_comm.h"
// #endif

typedef enum sensor_type
{
    UNKNOWN = -1,
    EE895 = 0,
    CDM7162 = 1,
    SUNRISE = 2,
    SUNLIGHT = 3,
    SCD30 = 4,
    SCD41 = 5,
    COZIR_LP3 = 6,
    CM1107N = 7
} sensor_type_e;

typedef enum meas_state_fsm
{
    MEAS_FINISHED = 0,
    MEAS_STARTED = 1,
    MEAS_READ_MODE = 2,
    MEAS_READ_VALUE = 3,
    MEAS_READ_STATUS = 4,
    MEAS_TRIGGER_SINGLE_MEAS = 5
} meas_state_e;

typedef struct sensor_config
{
    // Measurement
    uint16_t meas_period; // Set measurement period in s (default 15 for EE895)                                 EE895, SUNRISE, SUNLIGHT, SCD30
    bool single_meas_mode; // Enable single masurement mode                                                     EE895, SUNRISE, SUNLIGHT, SCD41
    uint16_t filter_coeff; // Set filter coefficient (default 4 EE895, 16 CozIR (1 .. 255), SUN* 4 (2 .. 10))   EE895, CozIR-LP3, SUNRISE, SUNLIGHT
    uint16_t meas_samples; // Set number of samples to measure (default 4)                                      SUNRISE, SUNLIGHT
    bool enable_static_IIR; // Enable static IIR filtering (possibly smoothing)                                 SUNRISE, SUNLIGHT
    bool enable_dynamic_IIR; // Enable dynamic IIR filtering (reacting to spikes, works if static IIR is on)    SUNRISE, SUNLIGHT
    bool co2_en; // Sensor measures CO2
    bool temp_en; // Sensor measures temperature
    bool RH_en; // Sensor measures humidity
    bool pressure_en; // Sensor measures pressure

    // Offsets
    float temperature_offset; // Set temperature offset                                                         SCD30, SCD41

    // Pins, PWM
    bool enable_PWM_pin; // Output 1 kHz PWM with duty proportional to CO2 concentration                        CDM7162, CozIR-LP3
    bool PWM_range_high; // CO2 = PWM high (us) * 5 if true, else * 2                                           CDM7162
    bool enable_nRDY; // Output measurement done state on nRDY pin                                              SUNRISE, SUNLIGHT
    bool invert_nRDY; // Invert nRDY pin logic                                                                  SUNRISE, SUNLIGHT

    // Pressure/altitude compensation
    bool enable_pressure_comp; // Enable pressure compensation                                                  CDM7162, SUNRISE, SUNLIGHT, SCD30, SCD41, CozIR-LP3
    // CDM7162: pressure in range 800 .. 1055
    uint16_t pressure; // Pressure value                                                                        CDM7162, SUNRISE, SUNLIGHT, SCD30, SCD41, CozIR-LP3
    bool enable_altitude_comp; // Enable altitude compensation                                                  CDM7162, SCD30, SCD41
    uint16_t altitude; // Altitude value                                                                        CDM7162, SCD30, SCD41

    // Calibration
    bool enable_abc; // Enable ABC calibration                                                                  SUNRISE, SUNLIGHT, SCD30, SCD41, CozIR-LP3
    bool enable_alternate_abc; // Enable ABC alternate algorithm                                                CDM7162
    // CDM7162 abc_target_value range: 300 .. 555 ppm
    uint16_t abc_init_period; // Initial ABC calibration period                                                 SCD41, CozIR-LP3
    uint16_t abc_period; // Set ABC calibration period (set 0 to disable for SUN*)                              SUNRISE, SUNLIGHT, SCD41, CozIR-LP3
    uint16_t abc_target_value; // Set ABC calibration target value (default 400 ppm)                            SUNRISE, SUNLIGHT, CozIR-LP3
    
    // Alarm
    bool alarm_en; // Enable alarm                                                                              CozIR-LP3
    // CDM7162 alarm range: 0 .. 2550 ppm; alarm uses hysteresis
    uint16_t alarm_treshold_co2_high; // Alarm high treshold                                                    CDM7162, CozIR-LP3
    uint16_t alarm_treshold_co2_low; // Alarm low treshold                                                      CDM7162

    // Power
    bool power_global_control; // Power controlled globally
    bool power_5V; // Sensor requires 5 V
    sensor_type_e sensor_type; // Type of sensor the configuration is written for
    uint8_t sensor_ord; // Sensor type no
    bool power_continuous; // True if sensor powered continuously
    bool power_12V; // Sensor voltage 12 V
    bool sensor_IIC; // true if I2C comm, false for UART comm
    bool ext_pressure_comp; // If true measured value is compensated using the on-board pressure sensor
    bool sensor_active; // Whether the sensor should be accessed or not

    uint8_t sensor_on_off; // When power disconnecting, get measured value (0x00 ... single value after sensor_power_up_time)
    uint32_t sensor_power_up_time; // Time for the sensor to stabilize while power disconnecting before taking measurement, min 3 s

    // 
    bool verified; // Is configuration verified
} sensor_config_t;

typedef struct sensor
{
    float co2;
    float temperature;
    float pressure;
    float humidity;
    int32_t state;
    int32_t timeout_iterator;
    uint8_t measurement_iterator;
    meas_state_e meas_state;
    absolute_time_t wake_time;
    sensor_config_t config;
    sensor_type_e sensor_type;
    uint8_t input_index; // Index of the input connector
    uint8_t power_index; // Index in the power vector
    uint8_t state_reg[28]; //                                                                                   SUNRISE, SUNLIGHT
    uint8_t sensor_number; // Index of the sensor of a type
    uint8_t init_count; // Counter of initializations in single measurement cycle
    uint8_t index; // Index of the sensor on the input - not converted to input indices that are moved around
    uint32_t err_total_counter; // Counter of total errors during the run
    uint8_t err_iter_counter; // Counts 0 to 2, if value reaches 2 measurement is evaluated as error
    bool initialized;
} sensor_t;

typedef struct ms5607
{
    uint16_t prom[16];
    float pressure;
    float temperature;
    uint8_t pressure_raw[3];
    uint8_t temperature_raw[3];
    meas_state_e meas_state;
    int32_t state;
} ms5607_t;

typedef struct hyt271
{
    float temperature;
    float humidity;
    uint8_t temperature_raw[2];
    uint8_t humidity_raw[2];
    int32_t state;
    meas_state_e meas_state;
    absolute_time_t wake_time;
    uint8_t err_count;
} hyt271_t;

typedef struct soap_data
{
    uint8_t data[MAX_SOAP_SIZE];
    uint16_t data_len;
    mutex_t data_mutex;
} soap_data_t;

typedef struct
{
    uint8_t command[CONFIG_RECVD_BUFFER_SIZE];
    uint8_t response[CONFIG_SEND_BUFFER_SIZE];
    uint32_t command_len;
    uint32_t response_len;
    mutex_t command_mutex;
    mutex_t response_mutex;
    bool command_rdy;
    bool response_rdy;
    bool response_sent;
    uint16_t data_len;
#ifdef __SERVICE_COMM_H__
    service_message_t message;
#endif
    uint8_t err;
} service_comm_data_t;

typedef enum {
    SERVICE_COMM_UART = 0,
    SERVICE_COMM_TCP_SERVER = 1
} service_comm_source_e;

typedef enum 
{
    MEASURED_VALUE_CO2 = 0,
    MEASURED_VALUE_T = 1,
    MEASURED_VALUE_RH = 2,
    MEASURED_VALUE_P = 3,
    MEASURED_VALUE_NUMBER = 4,
} measured_value_type_e;

typedef struct 
{
    sensor_type_e sensor_type;
    uint8_t sensor_type_num;
    measured_value_type_e measured_value_type;
    float measured_value;
    bool channel_active;
    sensor_t* sensor;
} message_channel;

typedef struct
{
    uint8_t channel_name[16];
    bool channel_active;
    measured_value_type_e measured_value_type;
    float* measured_value;
    int32_t* state;
} message_channel_general_t;

typedef enum
{
    SERVICE_MODE_DISABLED = 0,
    SERVICE_MODE_UART = 1,
    SERVICE_MODE_ETHERNET = 2
} service_mode_source_e;

typedef struct
{
    uint32_t ser_num;
    uint32_t ser_num_aux;
    uint8_t channel_act[16];
    uint8_t channel_idx[16];
    uint8_t channel_quant[16];
    uint8_t sta_security;
    uint8_t wlan_mode;
    uint8_t host_name[32];
    uint8_t sta_ssid[32];
    uint8_t sta_password[32];
    uint8_t sta_ip[16];
    uint8_t sta_gw[16];
    uint8_t sta_mask[16];
    uint8_t sta_dns[16];
    uint8_t soap_ip[32];
    uint8_t soap_path[32];
    uint16_t soap_port;
    uint8_t cloud_ip[32];
    uint8_t cloud_path[32];
    uint16_t cloud_port;
    uint32_t meas_int;
    uint32_t soap_int;
    uint8_t soap_mode;
    uint8_t device_desc[16];
    uint8_t aux_msg;
    bool reinit_sensors_on_error;
} global_config_t;

#endif