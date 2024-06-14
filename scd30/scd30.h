#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "../error_codes.h"
#include "../common/functions.h"
#include "string.h"
#include <math.h>

typedef enum scd30_meas_state
{
    SCD30_MEAS_FINISHED = 0,
    SCD30_MEAS_START = 1,
    SCD30_READ_STATUS = 2,
    SCD30_READ_VALUE = 3
} scd30_meas_state_e;

typedef struct scd30_config
{
    bool pressure_comp;
    uint16_t pressure;
    bool enable_autocal;
    uint16_t autocal_value;
    uint16_t meas_period;
    float temperature_offset;
    bool enable_altitude_comp;
    uint16_t altitude;
    bool power_global_control; // Global power control
} scd30_config_t;



typedef struct scd30
{
    float co2;
    float temperature;
    float humidity;
    int state;
    scd30_meas_state_e meas_state;
    absolute_time_t wake_time; // Time of next action
    //
    scd30_config_t* config;
} scd30_t;


int32_t scd30_read(uint16_t command, uint16_t* buf, uint32_t len);

int32_t scd30_write_value(uint16_t command, uint16_t value);

int32_t scd30_write_command(uint16_t command);

void scd30_get_value(scd30_t* scd30);

void scd30_power(scd30_t* scd30, bool on);

int32_t scd30_init(scd30_t* scd30, scd30_config_t* config);

int32_t scd30_read_config(scd30_config_t* config);

void scd30_init_struct(scd30_t* scd30);