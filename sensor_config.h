static bool global_power = true;

sunrise_config_t sensor_sunrise_config = {
    .abc_period = 0,
    .abc_target_value = 400,
    .enable_ABC = false,
    .enable_dynamic_IIR = true,
    .enable_nRDY = false,
    .enable_pressure_comp = false,
    .enable_static_IIR = true,
    .invert_nRDY = false,
    .meas_period = 14,
    .meas_samples = 8,
    .single_meas_mode = false,
};

sunlight_config_t sensor_sunlight_config = {
    .abc_period = 0,
    .abc_target_value = 400,
    .enable_ABC = false,
    .enable_dynamic_IIR = true,
    .enable_nRDY = false,
    .enable_pressure_comp = false,
    .enable_static_IIR = true,
    .invert_nRDY = false,
    .meas_period = 14,
    .meas_samples = 8,
    .single_meas_mode = false,
};

ee895_config_t sensor_ee895_config = {
    .offset = 0,
    .filter_coeff = 4,
    .meas_period = 15,
    .single_meas_mode = false,
};

cdm7162_config_t sensor_cdm7162_config = {
    .enable_PWM_pin = false,
    .long_term_adj_1 = false,
    .long_term_adj_2 = false,
    .pressure_corr = false,
    .PWM_range_high = false,
};