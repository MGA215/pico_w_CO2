#include "ms5607.h"
#include "hardware/i2c.h"
#include "common/constants.h"
#include "common/debug.h"
#include "error_codes.h"
#include "common/shared.h"
#include "string.h"
#include "math.h"

#define MS5607_ADDR 0x77

#define CMD_RESET             0x1E
#define CMD_READ_ADC          0x00
#define CMD_CONV_P            0x48
#define CMD_CONV_T            0x58
#define CMD_PROM_READ         0xA0

#define PROM_SIZE             8

#define PROM_READ_REPEAT      3

static uint8_t prom_read = 0;

/**
 * @brief Calculates the CRC4 value for the MS5607
 * 
 * @param prom_buffer PROM registers of the MS5607
 * @return unsigned char CRC4
 */
static unsigned char msCalcCrc4(uint16_t* prom_buffer);



static unsigned char msCalcCrc4(uint16_t* prom_buffer) {

  int32_t cnt;        // simple counter
  uint32_t n_rem;     // crc reminder
  uint32_t crc_read;  // original value of the crc
  uint8_t n_bit;

  n_rem = 0x00;
  //save read CRC
  crc_read=prom_buffer[7];
  //CRC byte is replaced by 0
  prom_buffer[7]=(0xFF00 & (prom_buffer[7]));

  // operation is performed on bytes
  for (cnt = 0; cnt < 16; cnt++) {
    // choose LSB or MSB
    if (cnt%2==1) n_rem ^= (unsigned short) ((prom_buffer[cnt>>1]) & 0x00FF);
    else n_rem ^= (unsigned short) (prom_buffer[cnt>>1]>>8);
    for (n_bit = 8; n_bit > 0; n_bit--) {
      if (n_rem & (0x8000)) {
        n_rem = (n_rem << 1) ^ 0x3000;
      } else {
        n_rem = (n_rem << 1);
      }
    }
  }
  // final 4-bit reminder is CRC code
  n_rem= (0x000F & (n_rem >> 12));
  // restore the crc_read to its original place
  prom_buffer[7]=crc_read;
  return (n_rem ^ 0x00);
}

int32_t ms5607_get_prom_const(uint16_t* prom_buffer, uint8_t buffer_len)
{
    if (buffer_len < PROM_SIZE) return MS5607_ERROR_SHORT_BUFFER;
    uint8_t temp_buffer[2];
    int32_t ret, zr = 1;
    temp_buffer[0] = CMD_RESET;

    if ((ret = i2c_write_timeout_us(I2C_DEVICE, MS5607_ADDR, temp_buffer, 1, false, I2C_TIMEOUT_US)) < 0) // Reset sensor
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_MS5607, "Failed to reset MS5607 sensor: %i", ret);
        return ret;
    }
    sleep_ms(5);

    for (int i = 0; i < PROM_SIZE; i++)
    {
        temp_buffer[0] = CMD_PROM_READ + i * 2; // PROM address to read from
        if ((ret = i2c_write_timeout_us(I2C_DEVICE, MS5607_ADDR, temp_buffer, 1, false, I2C_TIMEOUT_US)) < 0) // Set address to sensor
        {
            print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_MS5607, "Failed to write PROM address for reading: %i", ret);
            return ret;
        }
        if ((ret = i2c_read_timeout_us(I2C_DEVICE, MS5607_ADDR, temp_buffer, 2, false, I2C_TIMEOUT_US)) < 0) // Read from address
        {
            print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_MS5607, "Failed to read from PROM address: %i", ret);
            return ret;
        }
        prom_buffer[i] = 256 * temp_buffer[0] + temp_buffer[1];
        if (prom_buffer[i] != 0x00) zr = 0;
    }

    if (zr == 1) return MS5607_ERROR_PROM_ZERO; // Check for any zero words
    if (msCalcCrc4(prom_buffer) != (prom_buffer[7] & 0x000F)) return MS5607_ERROR_INVALID_CRC; // Check CRC

    return SUCCESS;
}

int32_t ms5607_get_adc_val(int32_t channel, uint8_t* buffer, uint8_t buffer_length)
{
    if (buffer_length < 3) return MS5607_ERROR_SHORT_BUFFER;
    int32_t ret;
    buffer[0] = (channel == 0) ? CMD_CONV_P : CMD_CONV_T; // Set command according to the channel

    if ((ret = i2c_write_timeout_us(I2C_DEVICE, MS5607_ADDR, buffer, 1, false, I2C_TIMEOUT_US)) < 0) // Start conversion
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_MS5607, "Failed to write address for ADC reading: %i");
        return ret;
    }
    sleep_ms(10);

    buffer[0] = CMD_READ_ADC;
    if ((ret = i2c_write_timeout_us(I2C_DEVICE, MS5607_ADDR, buffer, 1, false, I2C_TIMEOUT_US)) < 0) // Set address for reading the ADC value
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_MS5607, "Failed to write address for ADC reading: %i");
        return ret;
    }
    if ((ret = i2c_read_timeout_us(I2C_DEVICE, MS5607_ADDR, buffer, 3, false, I2C_TIMEOUT_US)) < 0) // Set address for reading the ADC value
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_MS5607, "Failed to write address for ADC reading: %i");
        return ret;
    }

    return SUCCESS;
}

void ms5607_get_value(void)
{
    if (ms5607.meas_state != MEAS_STARTED) return;
    int32_t i, ret;
    uint32_t d1, d2;
    uint8_t temp_buffer[3];
    double temp, press, off, sens, off2, sens2, td, t2;

    // Read PROM if not already read
    if (prom_read == 0) 
    {
        // Read PROM repeat
        for (i = 0; i < PROM_READ_REPEAT; i++) 
        {
            if ((ret = ms5607_get_prom_const(ms5607.prom, 8)) == 0) break; // Read PROM
        }
        if (i == PROM_READ_REPEAT) // Reading PROM failed
        {
            ms5607.pressure = NAN; // Reset all variables
            ms5607.temperature = NAN;
            memset(ms5607.pressure_raw, 0x00, 3);
            memset(ms5607.temperature_raw, 0x00, 3);
            ms5607.state = ret;
            ms5607.meas_state = MEAS_FINISHED;
            return;
        }
        prom_read = 1; // PROM read successfully
    }

    // Convert pressure
    if ((ret = ms5607_get_adc_val(0, ms5607.pressure_raw, 3)) != 0) 
    {
        ms5607.pressure = NAN; // Reset all variables
        ms5607.temperature = NAN;
        memset(ms5607.pressure_raw, 0x00, 3);
        memset(ms5607.temperature_raw, 0x00, 3);
        print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_MS5607, "Failed to convert pressure");
        ms5607.state = ret;
        ms5607.meas_state = MEAS_FINISHED;
        return;
    }
    d1 = ms5607.pressure_raw[0] * 65536 + ms5607.pressure_raw[1] * 256 + ms5607.pressure_raw[2];

    // Convert temperature
    if ((ret = ms5607_get_adc_val(1, ms5607.temperature_raw, 3)) != 0) 
    {
        ms5607.pressure = NAN; // Reset variables
        ms5607.temperature = NAN;
        memset(ms5607.temperature_raw, 0x00, 3);
        print_ser_output(SEVERITY_ERROR, SOURCE_SENSORS, SOURCE_MS5607, "Failed to convert temperature");
        ms5607.state = ret;
        ms5607.meas_state = MEAS_FINISHED;
        return;
    }
    d2 = ms5607.temperature_raw[0] * 65536 + ms5607.temperature_raw[1] * 256 + ms5607.temperature_raw[2];

    // 1st order calculation
    td   = d2 - ((double)ms5607.prom[5]) * 256.0;
    temp = 2000.0 + (td * ((double)ms5607.prom[6])) / 8388608.0;
    off  = ((double)ms5607.prom[2]) * 131072.0 + (((double)ms5607.prom[4]) * td) / 64.0;
    sens = ((double)ms5607.prom[1]) * 65536.0  + (((double)ms5607.prom[3]) * td) / 128.0;

    // 2nd order calculation
    t2    = 0.0;
    off2  = 0.0;
    sens2 = 0.0;
    if (temp < 2000.0) 
    {
        t2    = td * td / 2147483648.0;
        off2  = 61 * (temp - 2000.0) * (temp - 2000.0) / 16.0;
        sens2 = 2 * (temp - 2000.0) * (temp - 2000.0);
        if (temp < -1500.0) 
        {
            off2  = off2  + 15 * (temp + 1500.0) * (temp + 1500.0);
            sens2 = sens2 + 8 * (temp + 1500.0) * (temp + 1500.0);
        }
    }
    temp = (temp - t2) / 100.0;
    off  = off  - off2;
    sens = sens - sens2;
    // Final pressure calculation
    press = ((d1 * sens / 2097152.0 - off) / 32768.0) / 100.0;

    // Range check
    if ((press < MS_PRESS_MIN) || (temp < MS_TEMP_MIN)) {
        ms5607.pressure = NAN; // Reset variables
        ms5607.temperature = NAN;
        ms5607.state = MS5607_ERROR_VALUE_LOW;
        ms5607.meas_state = MEAS_FINISHED;
        return;
    }
    if ((press > MS_PRESS_MAX) || (temp > MS_TEMP_MAX)) {
        ms5607.pressure = NAN; // Reset variables
        ms5607.temperature = NAN;
        ms5607.state = MS5607_ERROR_VALUE_HIGH;
        ms5607.meas_state = MEAS_FINISHED;
        return;
    }

    ms5607.pressure = (float)press;
    ms5607.temperature = (float)temp;
    ms5607.meas_state = MEAS_FINISHED;
    ms5607.state = SUCCESS;

    return;
}
