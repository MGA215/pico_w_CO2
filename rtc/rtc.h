/**
 * @file rtc.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief 
 * @version 0.1
 * @date 2024-08-01
 * Module for interfacing with the pico-ds3231 library
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __RTC_H__
#define __RTC_H__

extern datetime_t datetime;
extern uint8_t datetime_str[30];

/**
 * @brief Initializes the RTC structures
 * 
 */
extern void rtc_init(void);

/**
 * @brief Updates the RTC string and datetime
 * 
 * @return true if time has changed
 * @return false if time has not changed
 */
extern bool rtc_update(void);

/**
 * @brief Sets the RTC date and time
 * 
 * @param year Year to set
 * @param month Month to set
 * @param day Day to set
 * @param dotw Day of the week to set
 * @param hour Hour to set
 * @param min Minute to set
 * @param sec Second to set
 * @return int32_t Error code
 */
int32_t rtc_set_datetime(uint16_t year, uint8_t month, uint8_t day, uint8_t dotw, uint8_t hour, uint8_t min, uint8_t sec);

/**
 * @brief Converts datetime to bytes
 * 
 * @param year Year (between 200 and 2099 incl)
 * @param month Month
 * @param day Day
 * @param hour Hour
 * @param min Min
 * @param sec Sec
 * @return uint32_t Time representation in bytes
 */
uint32_t rtc_get_time_bytes(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec);

/**
 * @brief Get the current time in byte float representation
 * 
 * @return uint32_t 
 */
uint32_t rtc_get_current_time_bytes(void);

#endif