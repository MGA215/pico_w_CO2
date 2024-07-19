#include "serialize.h"
#include "../common_include.h"
#include "../sensor_config.h"
#include "string.h"

/**
 * @brief Checks for out of range values for given sensors
 * 
 * @param config Configuration of the sensor
 * @return int32_t Return code
 */
static inline int32_t serializer_check_values(sensor_config_t* config);

int32_t serializer_serialize(sensor_config_t* config, uint8_t* serialized, uint8_t len)
{
    if (len < SERIALIZE_BUFFER_LEN) return ERROR_SERIALIZATION_BUFFER_LEN;

    uint8_t power_vector = 0;
    uint8_t measurement_vector = 0;
    uint8_t pin_vector = 0;
    uint8_t comp_cal_vector = 0;
    uint8_t buf[SERIALIZE_BUFFER_LEN];
    memset(buf, 0x00, SERIALIZE_BUFFER_LEN);

    buf[0] = (uint8_t)config->sensor_type;

    if (config->power_5V) power_vector |= (0b1 << 0);
    if (config->power_global_control) power_vector |= (0b1 << 1);
    buf[1] = power_vector;

    *((uint16_t*)&buf[2]) = config->meas_period;
    *((uint16_t*)&buf[4]) = config->filter_coeff;
    *((uint16_t*)&buf[6]) = config->meas_samples;

    if (config->co2_en) measurement_vector |= (0b1 << 0);
    if (config->temp_en) measurement_vector |= (0b1 << 1);
    if (config->RH_en) measurement_vector |= (0b1 << 2);
    if (config->pressure_en) measurement_vector |= (0b1 << 3);
    if (config->enable_static_IIR) measurement_vector |= (0b1 << 4);
    if (config->enable_dynamic_IIR) measurement_vector |= (0b1 << 5);
    if (config->single_meas_mode) measurement_vector |= (0b1 << 6);
    buf[8] = measurement_vector;

    *((int16_t*)&buf[9]) = config->co2_offset;
    *((uint32_t*)&buf[11]) = float2byte(config->temperature_offset);

    if (config->alarm_en) pin_vector |= (0b1 << 0);
    if (config->enable_nRDY) pin_vector |= (0b1 << 3);
    if (config->invert_nRDY) pin_vector |= (0b1 << 4);
    if (config->PWM_range_high) pin_vector |= (0b1 << 5);
    if (config->enable_PWM_pin) pin_vector |= (0b1 << 6);
    buf[15] = pin_vector;

    *((uint16_t*)&buf[16]) = config->pressure;
    *((uint16_t*)&buf[18]) = config->altitude;
    *((uint16_t*)&buf[20]) = config->alarm_treshold_co2_high;
    *((uint16_t*)&buf[22]) = config->alarm_treshold_co2_low;

    if (config->enable_abc) comp_cal_vector |= (0b1 << 0);
    if (config->enable_alternate_abc) comp_cal_vector |= (0b1 << 1);
    if (config->enable_altitude_comp) comp_cal_vector |= (0b1 << 6);
    if (config->enable_pressure_comp) comp_cal_vector |= (0b1 << 7);
    buf[23] = comp_cal_vector;

    *((uint16_t*)&buf[25]) = config->abc_target_value;
    *((uint16_t*)&buf[27]) = config->abc_period;
    *((uint16_t*)&buf[29]) = config->abc_init_period;

    buf[31] = 0;

    memcpy(serialized, buf, SERIALIZE_BUFFER_LEN);

    return SUCCESS;
}

int32_t serializer_deserialize(sensor_config_t* config, uint8_t* serialized, uint8_t len)
{
    if (len < SERIALIZE_BUFFER_LEN) return ERROR_SERIALIZATION_BUFFER_LEN;

    uint8_t power_vector = 0;
    uint8_t measurement_vector = 0;
    uint8_t pin_vector = 0;
    uint8_t comp_cal_vector = 0;

    uint8_t buf[SERIALIZE_BUFFER_LEN];
    memcpy(buf, serialized, SERIALIZE_BUFFER_LEN);

    if (buf[0] > 7 && serialized[0] < 255) return ERROR_DESERIALIZATION_FAILURE;
    config->sensor_type = (sensor_type_e)buf[0];

    config->power_5V = (bool)((buf[1] & (0b1 << 0)) >> 0);
    config->power_global_control = (bool)((buf[1] & (0b1 << 1)) >> 1);

    config->meas_period = *((uint16_t*)&buf[2]);
    config->filter_coeff = *((uint16_t*)&buf[4]);
    config->meas_samples = *((uint16_t*)&buf[6]);

    config->co2_en = (bool)((buf[8] & (0b1 << 0)) >> 0);
    config->temp_en = (bool)((buf[8] & (0b1 << 1)) >> 1);
    config->RH_en = (bool)((buf[8] & (0b1 << 2)) >> 2);
    config->pressure_en = (bool)((buf[8] & (0b1 << 3)) >> 3);
    config->enable_static_IIR = (bool)((buf[8] & (0b1 << 4)) >> 4);
    config->enable_dynamic_IIR = (bool)((buf[8] & (0b1 << 5)) >> 5);
    config->single_meas_mode = (bool)((buf[8] & (0b1 << 6)) >> 6);

    config->co2_offset = *((int16_t*)&buf[9]);
    config->temperature_offset = byte2float(*((uint32_t*)&buf[11]));

    config->alarm_en = (bool)((buf[15] & (0b1 << 0)) >> 0);
    config->enable_nRDY = (bool)((buf[15] & (0b1 << 3)) >> 3);
    config->invert_nRDY = (bool)((buf[15] & (0b1 << 4)) >> 4);
    config->PWM_range_high = (bool)((buf[15] & (0b1 << 5)) >> 5);
    config->enable_PWM_pin = (bool)((buf[15] & (0b1 << 6)) >> 6);

    config->pressure = *((uint16_t*)&buf[16]);
    config->altitude = *((uint16_t*)&buf[18]);
    config->alarm_treshold_co2_high = *((uint16_t*)&buf[20]);
    config->alarm_treshold_co2_low = *((uint16_t*)&buf[22]);

    config->enable_abc = (bool)((buf[23] & (0b1 << 0)) >> 0);
    config->enable_alternate_abc = (bool)((buf[23] & (0b1 << 1)) >> 1);
    config->enable_altitude_comp = (bool)((buf[23] & (0b1 << 6)) >> 6);
    config->enable_pressure_comp = (bool)((buf[23] & (0b1 << 7)) >> 7);

    config->abc_target_value = *((uint16_t*)&buf[25]);
    config->abc_period = *((uint16_t*)&buf[27]);
    config->abc_init_period = *((uint16_t*)&buf[29]);

    return serializer_check_values(config);
}

static inline int32_t serializer_check_values(sensor_config_t* config)
{
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
            if (config->pressure < 300 || config->pressure > 1300) return ERROR_DESERIALIZATION_VALUE_OUT_OF_RAGE;
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
            if (config->pressure < 697 || config->pressure > 1050) return ERROR_DESERIALIZATION_VALUE_OUT_OF_RAGE;
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

/* Serializing map
 * 
 * sensor_type
 * 0 | 0 | 0 | 0 | 0 | 0 | power_global_control (read only) | power_5V                                                      // Power vector
 * meas_period LSB
 * meas_period MSB
 * filter_coeff LSB
 * filter_coeff MSB
 * meas_samples LSB
 * meas_samples MSB
 * 0 | single_meas_mode | enable_dynamic_IIR | enable_static_IIR | pressure_en | RH_en | temp_en | co2_en       // measurement vector
 * co2_offset LSB
 * co2_offset MSB
 * temperature_offset LLSB
 * temperature_offset MLSB
 * temperature_offset LMSB
 * temperature_offset MMSB
 * 0 | enable_PWM_pin | PWM_range_high | invert_nRDY | enable_nRDY | 0 | 0 | alarm_en                           // pin vector
 * pressure LSB
 * pressure MSB
 * altitude LSB
 * altitude MSB
 * alarm_treshold_co2_high LSB
 * alarm_treshold_co2_high MSB
 * alarm_treshold_co2_low LSB
 * alarm_treshold_co2_low MSB
 * enable_pressure_comp | enable_altitude_comp | 0 | 0 | 0 | 0 | enable_alternate_abc | enable_abc              // compensation & calibration vector
 * abc_target_value LSB
 * abc_target_value MSB
 * abc_period LSB
 * abc_period MSB
 * abc_init_period LSB
 * abc_init_period MSB
 * 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0                                                                                // Empty vector
*/ 