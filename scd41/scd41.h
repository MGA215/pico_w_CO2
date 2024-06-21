#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "../error_codes.h"
#include "../common/functions.h"
#include "string.h"
#include <math.h>

typedef enum scd41_meas_state
{
    SCD41_MEAS_FINISHED = 0,
    SCD41_MEAS_START = 1,
    SCD41_READ_MODE = 2,
    SCD41_WRITE_MEAS_CMD = 3,
    SCD41_READ_STATUS = 4,
    SCD41_READ_VALUE = 5
} scd41_meas_state_e;

typedef struct scd41_config
{
    bool pressure_comp;
    uint16_t pressure;
    bool enable_autocal;
    float temperature_offset;
    bool enable_altitude_comp;
    uint16_t altitude;
    bool enable_single_meas;
    bool power_global_control; // Global power control
} scd41_config_t;



typedef struct scd41
{
    float co2;
    float temperature;
    float humidity;
    int state;
    scd41_meas_state_e meas_state;
    absolute_time_t wake_time; // Time of next action
    scd41_config_t* config;
} scd41_t;


int32_t scd41_read(uint16_t command, uint16_t* buf, uint32_t len);

int32_t scd41_write_value(uint16_t command, uint16_t value);

int32_t scd41_write_command(uint16_t command);

void scd41_get_value(scd41_t* scd41);

void scd41_power(scd41_t* scd41, bool on);

int32_t scd41_init(scd41_t* scd41, scd41_config_t* config);

int32_t scd41_read_config(scd41_config_t* config);

void scd41_init_struct(scd41_t* scd41);

void scd41_reset(void);