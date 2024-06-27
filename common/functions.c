#include "functions.h"
float byte2float(uint32_t byte_value)
{
    uint8_t* bytes = (uint8_t*)&byte_value;
    float output;

    *((uint8_t*)(&output) + 3) = bytes[0];
    *((uint8_t*)(&output) + 2) = bytes[1];
    *((uint8_t*)(&output) + 1) = bytes[2];
    *((uint8_t*)(&output) + 0) = bytes[3];

    return output;
}

uint16_t ntoh16(uint16_t network)
{
    return (network >> 8) | ((network & 0x00FF) << 8);
}

uint32_t ntoh32(uint32_t network)
{
    return (network >> 16) | ((network & 0x0000FFFF) << 16);
}

void common_init_struct(sensor_t* sensor)
{
    
}