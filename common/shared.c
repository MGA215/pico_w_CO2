#include "shared.h"
#include "structs.h"

sensor_t sensors[8];
config_data_t config_data = {.command_rdy = false, .response_rdy = false};

soap_data_t soap_data1;
soap_data_t soap_data2;

bool service_mode = false;

// SOAP sensor channels
message_channel* channels1[16]; // First 16 channels
uint8_t channels1_len; // Number of channels in first buffer
message_channel* channels2[16]; // Second 16 channels
uint8_t channels2_len; // Number of channels in second buffer