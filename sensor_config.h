#ifndef CONFIG
#define CONFIG

#include "common/structs.h"
static bool global_power = true;

static sensor_config_t sensor_config_0 = {
    .sensor_type = EE895,
    .meas_period = 15,
    .single_meas_mode = false,
    .filter_coeff = 4,
    .co2_offset = 0,
    .co2_en = true,
    .temp_en = true,
    .RH_en = false,
    .pressure_en = true,
    .power_5V = false,
};

static sensor_config_t sensor_config_1 = {
    .sensor_type = CDM7162,
    .enable_PWM_pin = false,
    .PWM_range_high = false,
    .enable_pressure_comp = false,
    .pressure = 0,
    .enable_altitude_comp = false,
    .altitude = 0,
    .long_term_adj_1 = false,
    .long_term_adj_2 = false,
    .target_LTA = 400,
    .period_LTA = 255,
    .alarm_treshold_co2_high = 255,
    .alarm_treshold_co2_low = 0,
    .co2_en = true,
    .temp_en = false,
    .RH_en = false,
    .pressure_en = false,
    .power_5V = false,
};

static sensor_config_t sensor_config_2 = {
    .sensor_type = SUNRISE,
    .meas_period = 14,
    .single_meas_mode = false,
    .meas_samples = 8,
    .enable_static_IIR = true,
    .enable_dynamic_IIR = true,
    .static_IIR_filter_coeff = 4,
    .enable_nRDY = false,
    .invert_nRDY = false,
    .enable_pressure_comp = false,
    .pressure = 0,
    .enable_abc = false,
    .abc_period = 0,
    .abc_target_value = 400,
    .co2_en = true,
    .temp_en = true,
    .RH_en = false,
    .pressure_en = false,
    .power_5V = false,
};

static sensor_config_t sensor_config_3 = {
    .sensor_type = SUNRISE,
    .meas_period = 14,
    .single_meas_mode = false,
    .meas_samples = 8,
    .enable_static_IIR = true,
    .enable_dynamic_IIR = true,
    .static_IIR_filter_coeff = 4,
    .enable_nRDY = false,
    .invert_nRDY = false,
    .enable_pressure_comp = false,
    .pressure = 0,
    .enable_abc = false,
    .abc_period = 0,
    .abc_target_value = 400,
    .co2_en = true,
    .temp_en = true,
    .RH_en = false,
    .pressure_en = false,
    .power_5V = false,
};

static sensor_config_t sensor_config_4 = {
    .sensor_type = SCD30,
    .meas_period = 2,
    .temperature_offset = 0,
    .enable_pressure_comp = false,
    .pressure = 0,
    .enable_altitude_comp = false,
    .altitude = 0,
    .enable_abc = false,
    .co2_en = true,
    .temp_en = true,
    .RH_en = true,
    .pressure_en = false,
    .power_5V = false,
};

static sensor_config_t sensor_config_5 = {
    .sensor_type = SCD41,
    .single_meas_mode = false,
    .temperature_offset = 4,
    .enable_pressure_comp = false,
    .pressure = 1013,
    .enable_altitude_comp = false,
    .altitude = 0,
    .enable_abc = false,
    .abc_init_period = 65535,
    .abc_period = 65535,
    .co2_en = true,
    .temp_en = true,
    .RH_en = true,
    .pressure_en = false,
    .power_5V = false,
};

static sensor_config_t sensor_config_6 = {
    .sensor_type = COZIR_LP3,
    .digital_filter = 16,
    .enable_PWM_pin = false,
    .enable_pressure_comp = false,
    .pressure = 1013,
    .enable_abc = false,
    .abc_init_period = 65535,
    .abc_period = 65535,
    .abc_target_value = 400,
    .alarm_en = false,
    .alarm_treshold_co2_high = 0,
    .co2_en = true,
    .temp_en = false,
    .RH_en = false,
    .pressure_en = false,
    .power_5V = false,
};
#endif