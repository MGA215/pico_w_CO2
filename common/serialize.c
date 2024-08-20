#include "serialize.h"
#include "common_include.h"
#include "../sensor_config.h"
#include "string.h"

/**
 * @brief Checks for out of range values for given sensors
 * 
 * @param config Configuration of the sensor
 * @return int32_t Return code
 */
static inline int32_t serializer_check_values(sensor_config_t* config);

int32_t serializer_serialize(sensor_config_t* config, uint8_t* serialized, uint16_t len)
{
    if (len < 0x100) return ERROR_SERIALIZATION_BUFFER_LEN;

    uint8_t power_vector = 0;
    uint8_t measurement_vector = 0;
    uint8_t pin_vector = 0;
    uint8_t comp_cal_vector = 0;
    uint8_t buf[0x100];
    memset(buf, 0x00, 0x100);

    buf[0] = (uint8_t)config->sensor_type;

    if (config->power_5V) power_vector |= (0b1 << 0);
    if (config->power_global_control) power_vector |= (0b1 << 1);
    buf[1] = power_vector;

    buf[2] = (config->meas_period & 0x00FF) >> 0;
    buf[3] = (config->meas_period & 0xFF00) >> 8;

    buf[4] = (config->filter_coeff & 0x00FF) >> 0;
    buf[5] = (config->filter_coeff & 0xFF00) >> 8;

    buf[6] = (config->meas_samples & 0x00FF) >> 0;
    buf[7] = (config->meas_samples & 0xFF00) >> 8;

    if (config->co2_en) measurement_vector |= (0b1 << 0);
    if (config->temp_en) measurement_vector |= (0b1 << 1);
    if (config->RH_en) measurement_vector |= (0b1 << 2);
    if (config->pressure_en) measurement_vector |= (0b1 << 3);
    if (config->enable_static_IIR) measurement_vector |= (0b1 << 4);
    if (config->enable_dynamic_IIR) measurement_vector |= (0b1 << 5);
    if (config->single_meas_mode) measurement_vector |= (0b1 << 6);
    buf[8] = measurement_vector;

    buf[9] = (config->co2_offset & 0x00FF) >> 0;
    buf[10] = (config->co2_offset & 0xFF00) >> 8;
    uint32_t temp = float2byte(config->temperature_offset);
    buf[11] = (temp & 0x000000FF) >> 0;
    buf[12] = (temp & 0x0000FF00) >> 8;
    buf[13] = (temp & 0x00FF0000) >> 16;
    buf[14] = (temp & 0xFF000000) >> 24;

    if (config->alarm_en) pin_vector |= (0b1 << 0);
    if (config->enable_nRDY) pin_vector |= (0b1 << 3);
    if (config->invert_nRDY) pin_vector |= (0b1 << 4);
    if (config->PWM_range_high) pin_vector |= (0b1 << 5);
    if (config->enable_PWM_pin) pin_vector |= (0b1 << 6);
    buf[15] = pin_vector;

    buf[16] = (config->pressure & 0x00FF) >> 0;
    buf[17] = (config->pressure & 0xFF00) >> 8;

    buf[18] = (config->altitude & 0x00FF) >> 0;
    buf[19] = (config->altitude & 0xFF00) >> 8;

    buf[20] = (config->alarm_treshold_co2_high & 0x00FF) >> 0;
    buf[21] = (config->alarm_treshold_co2_high & 0xFF00) >> 8;

    buf[22] = (config->alarm_treshold_co2_low & 0x00FF) >> 0;
    buf[23] = (config->alarm_treshold_co2_low & 0xFF00) >> 8;

    if (config->enable_abc) comp_cal_vector |= (0b1 << 0);
    if (config->enable_alternate_abc) comp_cal_vector |= (0b1 << 1);
    if (config->enable_altitude_comp) comp_cal_vector |= (0b1 << 6);
    if (config->enable_pressure_comp) comp_cal_vector |= (0b1 << 7);
    buf[24] = comp_cal_vector;

    buf[25] = (config->abc_target_value & 0x00FF) >> 0;
    buf[26] = (config->abc_target_value & 0xFF00) >> 8;

    buf[27] = (config->abc_period & 0x00FF) >> 0;
    buf[28] = (config->abc_period & 0xFF00) >> 8;

    buf[29] = (config->abc_init_period & 0x00FF) >> 0;
    buf[30] = (config->abc_init_period & 0xFF00) >> 8;

    buf[31] = 0;

    memcpy(serialized, buf, 0x100);

    return SUCCESS;
}

int32_t serializer_deserialize(sensor_config_t* config, uint8_t* serialized, uint16_t len)
{
    if (len < 0x100) return ERROR_SERIALIZATION_BUFFER_LEN;

    uint8_t buf[0x100];
    memcpy(buf, serialized, 0x100);

    memset(config, 0x00, sizeof(sensor_config_t));

    if (buf[0] > 7 && serialized[0] < 255) return ERROR_DESERIALIZATION_FAILURE; // These boundaries might error when new sensor added (ID >= 8)
    config->sensor_type = (sensor_type_e)buf[0];

    config->sensor_ord = buf[1];

    config->sensor_active = (bool)((buf[2] & (0b1 << 0)) >> 0);
    config->power_5V = (bool)((buf[2] & (0b1 << 1)) >> 1);
    config->power_control = (bool)((buf[2] & (0b1 << 2)) >> 2);
    config->power_global_control = (bool)((buf[2] & (0b1 << 3)) >> 3);
    config->power_12V = (bool)((buf[2] & (0b1 << 4)) >> 4);
    config->sensor_IIC = (bool)((buf[2] & (0b1 << 5)) >> 5);
    config->ext_pressure_comp = (bool)((buf[2] & (0b1 << 6)) >> 6);

    config->single_meas_mode = buf[3];
    config->sensor_power_up_time = buf[4];

    config->meas_period |= buf[0x10] << 0;
    config->meas_period |= buf[0x11] << 8;

    config->filter_coeff |= buf[0x12] << 0;
    config->filter_coeff |= buf[0x13] << 8;

    config->meas_samples |= buf[0x14] << 0;
    config->meas_samples |= buf[0x15] << 8;

    config->co2_offset |= buf[0x16] << 0;
    config->co2_offset |= buf[0x17] << 8;

    config->pressure |= buf[0x18] << 0;
    config->pressure |= buf[0x19] << 8;

    config->altitude |= buf[0x1A] << 0;
    config->altitude |= buf[0x1B] << 8;

    config->abc_init_period |= buf[0x1C] << 0;
    config->abc_init_period |= buf[0x1D] << 8;
    
    config->abc_period |= buf[0x1E] << 0;
    config->abc_period |= buf[0x1F] << 8;

    config->abc_target_value |= buf[0x20] << 0;
    config->abc_target_value |= buf[0x21] << 8;

    config->alarm_treshold_co2_high |= buf[0x22] << 0;
    config->alarm_treshold_co2_high |= buf[0x23] << 8;

    config->alarm_treshold_co2_low |= buf[0x24] << 0;
    config->alarm_treshold_co2_low |= buf[0x25] << 8;

    uint32_t temp = 0;
    temp |= buf[0x26] << 0;
    temp |= buf[0x27] << 8;
    temp |= buf[0x28] << 16;
    temp |= buf[0x29] << 24;
    config->temperature_offset = byte2float(temp);
    
    config->enable_static_IIR = (bool)((buf[0x2A] & (0b1 << 0)) >> 0);
    config->enable_dynamic_IIR = (bool)((buf[0x2A] & (0b1 << 1)) >> 1);
    config->enable_PWM_pin = (bool)((buf[0x2A] & (0b1 << 2)) >> 2);
    config->PWM_range_high = (bool)((buf[0x2A] & (0b1 << 3)) >> 3);
    config->enable_nRDY = (bool)((buf[0x2A] & (0b1 << 4)) >> 4);
    config->invert_nRDY = (bool)((buf[0x2A] & (0b1 << 5)) >> 5);
    config->enable_pressure_comp = (bool)((buf[0x2A] & (0b1 << 6)) >> 6);
    config->enable_altitude_comp = (bool)((buf[0x2A] & (0b1 << 7)) >> 7);

    config->enable_abc = (bool)((buf[0x2B] & (0b1 << 0)) >> 0);
    config->enable_alternate_abc = (bool)((buf[0x2B] & (0b1 << 1)) >> 1);
    config->alarm_en = (bool)((buf[0x2B] & (0b1 << 2)) >> 2);

    switch (config->sensor_type)
    {
        case EE895:
        {
            config->co2_en = true;
            config->temp_en = true;
            config->RH_en = false;
            config->pressure_en = true;
            break;
        }
        case CDM7162:
        {
            config->co2_en = true;
            config->temp_en = false;
            config->RH_en = false;
            config->pressure_en = false;
            break;
        }
        case SUNRISE:
        {
            config->co2_en = true;
            config->temp_en = true;
            config->RH_en = false;
            config->pressure_en = false;
            break;
        }
        case SUNLIGHT:
        {
            config->co2_en = true;
            config->temp_en = true;
            config->RH_en = false;
            config->pressure_en = false;
            break;
        }
        case SCD30:
        {
            config->co2_en = true;
            config->temp_en = true;
            config->RH_en = true;
            config->pressure_en = false;
            break;
        }
        case SCD41:
        {
            config->co2_en = true;
            config->temp_en = true;
            config->RH_en = true;
            config->pressure_en = false;
            break;
        }
        case COZIR_LP3:
        {
            config->co2_en = true;
            config->temp_en = false;
            config->RH_en = false;
            config->pressure_en = false;
            break;
        }
        case CM1107N:
        {
            config->co2_en = true;
            config->temp_en = false;
            config->RH_en = false;
            config->pressure_en = false;
            break;
        }
        default:
        {
            config->co2_en = false;
            config->temp_en = false;
            config->RH_en = false;
            config->pressure_en = false;
            break;
        }
    }

    return serializer_check_values(config);
}

static inline int32_t serializer_check_values(sensor_config_t* config)
{
    if (config->sensor_power_up_time < 3) config->sensor_power_up_time = 3;
    switch(config->sensor_type)
    {
        case EE895:
        {
            if (config->meas_period < 10 || config->meas_period > 3600) return ERROR_DESERIALIZATION_VALUE_OUT_OF_RAGE;
            if (config->filter_coeff < 1 || config->filter_coeff > 20) return ERROR_DESERIALIZATION_VALUE_OUT_OF_RAGE;
            break;
        }
        case CDM7162:
        {
            if (config->pressure < 800 || config->pressure > 1055) return ERROR_DESERIALIZATION_VALUE_OUT_OF_RAGE;
            if (config->altitude > 2550) return ERROR_DESERIALIZATION_VALUE_OUT_OF_RAGE;
            if (config->abc_target_value < 300 || config->abc_target_value > 555) return ERROR_DESERIALIZATION_VALUE_OUT_OF_RAGE;
            if (config->abc_period > (30 * 64)) return ERROR_DESERIALIZATION_VALUE_OUT_OF_RAGE;
            if (config->alarm_treshold_co2_high > 2550) return ERROR_DESERIALIZATION_VALUE_OUT_OF_RAGE;
            if (config->alarm_treshold_co2_low > 2550) return ERROR_DESERIALIZATION_VALUE_OUT_OF_RAGE;
            break;
        }
        case SUNRISE:
        case SUNLIGHT:
        {
            if (config->meas_period < 2 || config->meas_period > 65534) return ERROR_DESERIALIZATION_VALUE_OUT_OF_RAGE;
            if (config->meas_samples < 1 || config->meas_samples > 1024) return ERROR_DESERIALIZATION_VALUE_OUT_OF_RAGE;
            if (config->filter_coeff < 2 || config->filter_coeff > 10) return ERROR_DESERIALIZATION_VALUE_OUT_OF_RAGE;
            if ((config->pressure < 300 || config->pressure > 1300) && config->pressure != 0) return ERROR_DESERIALIZATION_VALUE_OUT_OF_RAGE;
            break;
        }
        case SCD30:
        {
            if (config->pressure != 0 && (config->pressure < 700 || config->pressure > 1400)) return ERROR_DESERIALIZATION_VALUE_OUT_OF_RAGE;
            if (config->meas_period < 2 || config->meas_period > 1600) return ERROR_DESERIALIZATION_VALUE_OUT_OF_RAGE;
            break;
        }
        case SCD41:
        {
            if (config->altitude < 0 || config->altitude > 3000) return ERROR_DESERIALIZATION_VALUE_OUT_OF_RAGE;
            if (config->pressure < 700 || config->pressure > 1200) return ERROR_DESERIALIZATION_VALUE_OUT_OF_RAGE;
            break;
        }
        case COZIR_LP3:
        {
            if (config->filter_coeff < 1 || config->filter_coeff > 255) return ERROR_DESERIALIZATION_VALUE_OUT_OF_RAGE;
            if ((config->pressure < 697 || config->pressure > 1050) && config->pressure != 0) return ERROR_DESERIALIZATION_VALUE_OUT_OF_RAGE;
            if (config->alarm_treshold_co2_high > 20000) return ERROR_DESERIALIZATION_VALUE_OUT_OF_RAGE;
            break;
        }
        case CM1107N:
        {
            if (config->abc_period < 1 || config->abc_period > 30) return ERROR_DESERIALIZATION_VALUE_OUT_OF_RAGE;
            if (config->abc_target_value < 400 || config->abc_target_value > 1500) return ERROR_DESERIALIZATION_VALUE_OUT_OF_RAGE;
            break;
        }
        default: return SUCCESS;
    }
    return SUCCESS;
}