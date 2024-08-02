#include "pico-ds3231/lib/include/ds3231.h"
#include "pico/stdlib.h"
#include "common/constants.h"
#include "error_codes.h"
#include "common/shared.h"
#include "timeconst.h"

// RTC structure
static ds3231_rtc_t rtc;

static absolute_time_t rtc_time;

static uint16_t rtc_read_interval_ms = 250;

/**
 * @brief Converts datetime format to a string YYYY-MM-DD hh-mm-ss
 * 
 * @param buf output buffer containing the datetime string
 * @param buf_size max size of the buffer
 * @param dt datetime struct read from RTC module
 * @return int negative if conversion failed, otherwise number of written characters (except null byte)
 */
static int datetime2str(char *buf, uint8_t buf_size, const ds3231_datetime_t *dt);


void rtc_init(void)
{
    ds3231_init(I2C_DEVICE, I2C_DEVICE_SDA, I2C_DEVICE_SCL, &rtc); // Initialize DS3231 module
    rtc_time = make_timeout_time_ms(rtc_read_interval_ms); // Read time in 250 ms
}

static int datetime2str(char *buf, uint8_t buf_size, const ds3231_datetime_t *dt)
{
    return snprintf(buf, buf_size, "%02u.%02u.%04u %02u:%02u:%02u", dt->day, dt->month, dt->year, 
        dt->hour, dt->minutes, dt->seconds); // Conversion of the datetime struct to date time string
}

bool rtc_update(void)
{
    if (time_reached(rtc_time)) // Check if RTC should be read
    {
        rtc_time = make_timeout_time_ms(rtc_read_interval_ms); // Make new timeout
        ds3231_datetime_t dt;
        ds3231_get_datetime(&dt, &rtc); // read datetime
        if (datetime.year == dt.year && datetime.month == dt.month &&
            datetime.dotw == dt.dotw && datetime.day == dt.day &&
            datetime.hour == dt.hour && datetime.min == dt.minutes && datetime.sec == dt.seconds) return false; // Check for the same time
        datetime2str(datetime_str, 30, &dt); // convert datetime to string
        datetime.year = dt.year; // Update time
        datetime.month = dt.month;
        datetime.day = dt.day;
        datetime.dotw = dt.dotw;
        datetime.hour = dt.hour;
        datetime.min = dt.minutes;
        datetime.sec = dt.seconds;
        return true; // Datetime changed
    }
    return false; // Datetime not changed
}

int32_t rtc_set_datetime(uint16_t year, uint8_t month, uint8_t day, uint8_t dotw, uint8_t hour, uint8_t min, uint8_t sec)
{
    if (year < 2000 || year > 2099 ||
        month > 12 || month == 0 || 
        day > 31 || day == 0 || 
        dotw > 7 || dotw == 0 ||
        hour > 23 || min > 60 || sec > 60) return ERROR_RTC_INVALID_DATETIME;
    
    ds3231_datetime_t datetime = {
        .year = year,
        .month = month,
        .dotw = dotw,
        .day = day,
        .hour = hour,
        .minutes = min,
        .seconds = sec
    };
    ds3231_set_datetime(&datetime, &rtc); // refresh datetime
    return SUCCESS;
}

uint32_t rtc_get_time_bytes(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec)
{
    if (year < 2000 || year > 2099) return 0;
    return rok_sec[year - 2000] + mesic_sec[month] + den_sec[day] + hod_sec[hour] + min_sec[min] + sec;
}

uint32_t rtc_get_current_time_bytes(void)
{
    return rtc_get_time_bytes(datetime.year, datetime.month, datetime.day, datetime.hour, datetime.min, datetime.sec);
}