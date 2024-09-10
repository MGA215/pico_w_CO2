#include "shared.h"
#include "../soap/soap_channels.h"

global_config_t global_configuration;

sensor_t sensors[8];
service_comm_data_t config_data = {.command_rdy = false, .response_rdy = false};

ms5607_t ms5607;
hyt271_t hyt271;

soap_data_t soap_data[2];

// SOAP sensor channels
message_channel channels1[16]; // First 16 channels
message_channel_general_t* channels2[16] = { // Second 16 channels
    &channel_general_default,
    &channel_general_default,
    &channel_general_default,
    &channel_general_default,
    &channel_general_default,
    &channel_general_default,
    &channel_general_default,
    &channel_general_default,
    &channel_general_default,
    &channel_general_default,
    &channel_general_default,
    &channel_general_default,
    &channel_general_default,
    &channel_general_default,
    &channel_general_default,
    &channel_general_default
}; 

uint8_t datetime_str[30] = {0};
datetime_t datetime;

float cozir_unfiltered = 0;

service_mode_source_e service_mode = SERVICE_MODE_DISABLED;