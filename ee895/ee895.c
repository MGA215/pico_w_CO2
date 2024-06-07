#include "ee895.h"

#define EE895_I2C i2c0

#define EE895_ADDR 0x5F
#define EE895_MAX_REG_READ 8

// CO2 range
#define CO2_MIN_RANGE           0.0
#define CO2_MAX_RANGE           10000.0

// temperature range
#define T_MIN_RANGE            -40.0
#define T_MAX_RANGE             60.0

// Register addresses - float
#define REG_T_C_FLOAT           (0x03EA)
#define REG_T_F_FLOAT           (0x03EC)
#define REG_T_K_FLOAT           (0x03F0)
#define REG_CO2_AVG_FLOAT       (0x0424)
#define REG_CO2_RAW_FLOAT       (0x0426)
#define REG_CO2_AVG_NPC_FLOAT   (0x0428)
#define REG_CO2_RAW_NPC_FLOAT   (0x042A)
#define REG_P_MBAR_FLOAT        (0x04B0)
#define REG_P_PSI_FLOAT         (0x04B2)
// Register addresses - int16
#define REG_T_C_INT16           (0x0FA1)
#define REG_T_F_INT16           (0x0FA2)
#define REG_T_K_INT16           (0x0FA4)
#define REG_CO2_AVG_INT16       (0x0FBE)
#define REG_CO2_RAW_INT16       (0x0FBF)
#define REG_CO2_AVG_NPC_INT16   (0x0FC0)
#define REG_CO2_RAW_NPC_INT16   (0x0FC1)
#define REG_P_MBAR_INT16        (0x1004)
#define REG_P_PSI_INT16         (0x1005)
// User register addresses
#define REG_USER1               (0x16A8)
#define REG_USER2               (0x16A9)
// Sensor information
#define REG_SER_NR              (0x0000)
#define REG_FW_VER              (0x0008)
#define REG_FW_NAME             (0x0009)
// Sensor general settings
#define REG_MEAS_MODE           (0x01F8)
#define REG_STATUS              (0x01F9)
#define REG_MEAS_TRIGGER        (0x01FA)
#define REG_STATUS_DETAIL       (0x0258)
// Sensor CO2 settings
#define REG_MEAS_INTERVAL       (0x1450)
#define REG_MEAS_FILTER         (0x1451)
#define REG_MEAS_OFFSET         (0x1452)


/**
 * @brief converts byte representation of a float to float
 * 
 * @param byte_value bytes of the float
 * @return float output value
 */
float byte2float(uint32_t byte_value);

/**
 * @brief Converts 16-bit value to the other endian
 * 
 * @param network Value to be converted
 * @return uint16_t Converted value
 */
uint16_t ntoh16(uint16_t network);

/**
 * @brief Swaps halves of a 32-bit value
 * 
 * @param network Value to be converted
 * @return uint16_t Converted value
 */
uint32_t ntoh32(uint32_t network);

/**
 * @brief Computes Modbus CRC for specified buffer
 * 
 * @param buf Buffer the CRC is computed from
 * @param len Length of the buffer
 * @return uint16_t CRC value
 */
uint16_t ee895_modbus_crc(uint8_t* buf, uint32_t len);

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

uint16_t ee895_modbus_crc(uint8_t* buf, uint32_t len)
{
    uint16_t crc = 0xFFFF;
    uint32_t i;

    for (i = 0; i < len; i++) 
    {
        crc ^= (uint16_t)buf[i];

        for (int i = 8; i != 0; i--) 
        {
            if ((crc & 0x0001) != 0) 
            {
                crc >>= 1;
                crc ^= 0xA001;
            } 
            else 
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

int32_t ee895_read(uint16_t addr, uint16_t nreg, uint8_t* buf)
{
    int32_t ret;
    uint8_t commandBuffer[(EE895_MAX_REG_READ * 2) + 8];
    if (nreg < 1 || nreg > 8) return EE895_ERROR_NREG_REG;

    commandBuffer[0] = EE895_ADDR; // Slave address
    commandBuffer[1] = 0x03; // Read multiple holding registers
    *((uint16_t*)&commandBuffer[2]) = ntoh16(addr); // Convert reg address to big endian
    *((uint16_t*)&commandBuffer[4]) = ntoh16(nreg); // Convert number of registers to big endian
    *((uint16_t*)&commandBuffer[6]) = ee895_modbus_crc(commandBuffer, 6); // CRC computation

    if ((ret = i2c_write_blocking(EE895_I2C, EE895_ADDR, &commandBuffer[1], 7, true)) < 0) return ret; // Write to slave
    busy_wait_ms(2);

    if ((ret = i2c_read_blocking(EE895_I2C, EE895_ADDR, &commandBuffer[1], nreg * 2 + 4, false)) < 0) return ret; // Read from slave
    if (commandBuffer[1] != 0x03 || commandBuffer[2] != 2 * nreg) return EE895_ERROR_READ_RESP; // Check valid command & number of registers

    if (ee895_modbus_crc(commandBuffer, nreg * 2 + 5) != 0) return EE895_ERROR_INVALID_CRC; // Check CRC
    memcpy(buf, &commandBuffer[3], nreg * 2); // Copy to output buffer
    return 0;
}

int32_t ee895_write(uint16_t addr, uint16_t value)
{
    int32_t ret;
    uint8_t commandBuffer[2 * EE895_MAX_REG_READ + 8];

    commandBuffer[0] = EE895_ADDR; // Slave address
    commandBuffer[1] = 0x03; // Read multiple holding registers
    *((uint16_t*)&commandBuffer[2]) = ntoh16(addr); // Convert reg address to big endian
    *((uint16_t*)&commandBuffer[4]) = ntoh16(value); // Convert number of registers to big endian
    *((uint16_t*)&commandBuffer[6]) = ee895_modbus_crc(commandBuffer, 6); // CRC computation

    if ((ret = i2c_write_blocking(EE895_I2C, EE895_ADDR, &commandBuffer[1], 7, false)) < 0) return ret; // Write to slave
    busy_wait_ms(2);

    memset(&commandBuffer[1], 0x00, 7);
    if ((ret = i2c_read_blocking(EE895_I2C, EE895_ADDR, &commandBuffer[1], 6, false)) < 0) return ret; // Read from slave

    if (commandBuffer[1] != 0x06 || (*((uint16_t*)&commandBuffer[4])) != value) return EE895_ERROR_WRITE_RESP; // Check valid command & value
    if (ee895_modbus_crc(commandBuffer, 8) != 0) return EE895_ERROR_INVALID_CRC; // Check CRC

    return 0;
}

int32_t ee895_get_value(float* co2, float* temperature, float* pressure)
{
    uint8_t tempBuffer[4] = {0};
    int32_t i, ret;

    busy_wait_ms(750);

    i = 0;
    while (true)
    {
        ret = ee895_read(REG_STATUS, 1, tempBuffer);
        if (ret != 0) 
        {
            *co2 = NAN;
            *temperature = NAN;
            *pressure = NAN;
            return ret;
        }
        if (tempBuffer[1] & 0x01) break;
        busy_wait_ms(25);
        if (i++ > 20)
        {
            *co2 = NAN;
            *temperature = NAN;
            *pressure = NAN;
            return EE895_ERROR_DATA_READY_TIMEOUT;
        }
    }
    ret = ee895_read(REG_T_C_FLOAT, 2, tempBuffer);
    if (ret != 0) 
    {
        *temperature = NAN;
        *pressure = NAN;
        return ret;
    }
    *((uint32_t*)&tempBuffer[0]) = ntoh32(*((uint32_t*)&tempBuffer[0]));
    float val = byte2float(*((uint32_t*)&tempBuffer[0]));
    if (val < T_MIN_RANGE || val > T_MAX_RANGE)
    {
        *temperature = NAN;
        *pressure = NAN;
        return EE895_ERROR_RANGE;
    }
    *temperature = val;

    ret = ee895_read(REG_CO2_RAW_FLOAT, 2, tempBuffer);
    if (ret != 0) 
    {
        *co2 = NAN;
        *temperature = NAN;
        *pressure = NAN;
        return ret;
    }
    *((uint32_t*)&tempBuffer[0]) = ntoh32(*((uint32_t*)&tempBuffer[0]));
    val = byte2float(*((uint32_t*)&tempBuffer[0]));
    if (val < CO2_MIN_RANGE || val > CO2_MAX_RANGE)
    {
        *co2 = NAN;
        *temperature = NAN;
        *pressure = NAN;
        return EE895_ERROR_RANGE;
    }
    *co2 = val;

    ret = ee895_read(REG_P_MBAR_FLOAT, 2, tempBuffer);
    if (ret != 0) 
    {
        *pressure = NAN;
        return ret;
    }
    *((uint32_t*)&tempBuffer[0]) = ntoh32(*((uint32_t*)&tempBuffer[0]));
    val = byte2float(*((uint32_t*)&tempBuffer[0]));
    *pressure = val;
    return 0;
}

int32_t ee895_read_reg(uint16_t addr, uint16_t nreg, uint8_t* buf)
{
    int32_t ret;
    busy_wait_ms(250);

    // u vybranych registru pridej cekani nez bude hodnota zmerena
    if (((addr >= REG_T_C_FLOAT) || ((addr + nreg) >= REG_T_C_FLOAT)) && (addr <= REG_P_PSI_FLOAT)) {
        busy_wait_ms(750);
    }
    if (((addr >= REG_T_C_INT16) || ((addr + nreg) >= REG_T_C_INT16)) && (addr <= REG_P_PSI_INT16)) {
        busy_wait_ms(750);
    }

    ret = ee895_read(addr, nreg, buf);
    return ret;
}

int32_t ee895_write_reg(uint16_t addr, uint16_t value)
{
    int32_t ret;
    busy_wait_ms(250);
    ret = ee895_write(addr, value);
    busy_wait_ms(10);
    return ret;
}

