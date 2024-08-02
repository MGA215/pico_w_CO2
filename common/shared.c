#include "shared.h"

sensor_t sensors[8];
service_comm_data_t config_data = {.command_rdy = false, .response_rdy = false};

ms5607_t ms5607;
hyt271_t hyt271;

soap_data_t soap_data1;

bool service_mode = false;

// SOAP sensor channels
message_channel* channels1[16]; // First 16 channels
uint8_t channels1_len; // Number of channels in first buffer
message_channel* channels2[16]; // Second 16 channels
uint8_t channels2_len; // Number of channels in second buffer

uint8_t datetime_str[30] = {0};
datetime_t datetime;