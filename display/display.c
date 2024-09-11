#include "display.h"
#include "gfx_pack/gfx_pack.h"
#include "common/constants.h"
#include "common/shared.h"
#include "pico/printf.h"
#include "string.h"
#include "pico/stdlib.h"
#include "common/debug.h"
#include "cyw43.h"
#include "common/functions.h"
#include "soap/soap_channels.h"
#include "wifi/tcp_client.h"
#include "wifi/tcp_server.h"
#include "wifi/wifi.h"
#include "malloc.h"
#include "rtc/rtc.h"

#define STATUS_ITEMS 12
#define EE895_ITEMS 12
#define CDM7162_ITEMS 20
#define SUNRISE_ITEMS 22
#define SUNLIGHT_ITEMS 22
#define SCD30_ITEMS 14
#define SCD41_ITEMS 15
#define COZIR_LP3_ITEMS 18
#define CM1107N_ITEMS 13

typedef enum display_row
{
    DISPLAY_NONE = 0,
    SENSORS = 1,
    DEBUG = 2,
    GLOBAL_SETTINGS = 3,
    STATUS = 4,
    CHANNELS = 5,
} display_row_e;

typedef enum display_sensors
{
    SENSORS_NONE = 0,
    SENSOR_0 = 1,
    SENSOR_1 = 2,
    SENSOR_2 = 3,
    SENSOR_3 = 4,
    SENSOR_4 = 5,
    SENSOR_5 = 6,
    SENSOR_6 = 7,
    SENSOR_7 = 8,
    SENSOR_0_CONFIG = 11,
    SENSOR_1_CONFIG = 12,
    SENSOR_2_CONFIG = 13,
    SENSOR_3_CONFIG = 14,
    SENSOR_4_CONFIG = 15,
    SENSOR_5_CONFIG = 16,
    SENSOR_6_CONFIG = 17,
    SENSOR_7_CONFIG = 18,
} display_sensors_e;

// Display row being shown
static display_row_e row;

// Display sensor
static display_sensors_e sensor;

// Time between brightness updates
static absolute_time_t brightness_update_timer;

// Time in ms how often can be display brightness updated
static uint32_t brightness_update_time_ms = 100;

// Time value to check if display & button update should be performed
absolute_time_t process_update_time;

// Display brightness
static uint8_t display_brightness = 30;

// Previous state of buttons - used to determine button press (single action)
static uint8_t buttons_prev_state = 0;

// Force display update from inside - button pushed, RTC time changed, etc.
static bool update_display_buffer = false;

// Selected display row
static uint8_t row_select = 0;

// Offset of the current page
static uint8_t page_offset = 0;

// Offset of the currently selected item
static uint8_t item_offset = 0;

// Boot time string
static uint8_t boot_time_str[30];

// FW version string
static uint8_t fw_version_str[16];




extern uint8_t debug; // Global max debug level

extern uint8_t debug_main_init; // Debug levels
extern uint8_t debug_main_loop;
extern uint8_t debug_sensors;
extern uint8_t debug_soap;
extern uint8_t debug_rtc;
extern uint8_t debug_display;
extern uint8_t debug_gfx;
extern uint8_t debug_ram;
extern uint8_t debug_service_comm;
extern uint8_t debug_eeprom;

extern uint8_t debug_mux;
extern uint8_t debug_power;
extern uint8_t debug_ms5607;
extern uint8_t debug_hyt271;
extern uint8_t debug_ee895;
extern uint8_t debug_cdm7162;
extern uint8_t debug_sunrise;
extern uint8_t debug_sunlight;
extern uint8_t debug_scd30;
extern uint8_t debug_scd41;
extern uint8_t debug_cozir_lp3;
extern uint8_t debug_cm1107n;

extern uint8_t debug_wifi;
extern uint8_t debug_tcp_client;
extern uint8_t debug_tcp_server;
extern uint8_t debug_tcp_dns;

// Time the last message was sent
extern uint8_t last_message_time[32];

// Time new measurement cycle should start
extern absolute_time_t sensor_start_measurement_time;

// Error of the last message sent
extern uint8_t last_message_error;

// Time new message should be sent
extern absolute_time_t send_data_time;


static void display_buttons(void);
static void display_on_button_a();
static void display_on_button_b();
static void display_on_button_c();
static void display_on_button_d();
static void display_on_button_e();
static void display_on_button_c_pressing(void);
static void display_on_button_d_pressing(void);
static void write_display(void);
static void get_sensor_name_string(sensor_t* sensor, uint8_t* buf, uint8_t len);
static void write_display_sensor(uint8_t* sensor_name, int state, 
        bool co2, float co2_value, 
        bool temp, float temp_value, 
        bool pressure, float pressure_value,
        bool humidity, float humidity_value);
static void display_get_debug_str(uint8_t* buf, uint8_t len, uint8_t debug_severity);
static uint64_t getFreeHeap(void);
static void boot_time(void);


void display_init(void)
{
    rtc_init();
    if (strlen(boot_time_str) == 0) boot_time();
    snprintf(fw_version_str, 16, "%u.%u.%u.%u", (uint8_t)FW_VERSION_MAJOR, (uint8_t)FW_VERSION_MINOR, (uint8_t)FW_VERSION_PATCH, (uint8_t)FW_VERSION_BUILD);
    update_display_buffer = true;
    row = DISPLAY_NONE;
    row_select = 0;
    brightness_update_timer = make_timeout_time_ms(1);
    process_update_time = make_timeout_time_ms(display_interval); // Set display & input checking interval
    gfx_pack_init((uint8_t)((uint16_t)(display_brightness) * 255 / 100));

    rtc_update();
}

static void boot_time(void)
{
    rtc_update();
    memcpy(boot_time_str, datetime_str, 30);
    return;
}

void display_update(bool force_update)
{
    if (force_update || time_reached(process_update_time))
    {
        rtc_update();
        display_buttons();
        write_display();
        gfx_pack_update();
        gfx_pack_set_backlight((uint8_t)((uint16_t)(display_brightness) * 255 / 100));
        process_update_time = make_timeout_time_us(display_interval * 1000); // Wait for next update
    }
}

static void display_buttons(void)
{
    if (gfx_pack_read_button(GFX_PACK_BUTTON_A)) // Button A down
    {
        if ((buttons_prev_state & (0b1 << 0)) == 0) // Button A pressed - single action
        {
            buttons_prev_state |= (0b1 << 0);
            display_on_button_a();
        }
        // Button A down - repeat action
    }
    else // Button A up
    {
        buttons_prev_state &= ~(0b1 << 0);
    }

    if (gfx_pack_read_button(GFX_PACK_BUTTON_B)) // Button B down
    {
        if ((buttons_prev_state & (0b1 << 1)) == 0) // Button B pressed - single action
        {
            buttons_prev_state |= (0b1 << 1);
            display_on_button_b();
        }
        // Button B down - repeat action
    }
    else // Button B up
    {
        buttons_prev_state &= ~(0b1 << 1);
    }

    if (gfx_pack_read_button(GFX_PACK_BUTTON_C)) // Button C down
    {
        if ((buttons_prev_state & (0b1 << 2)) == 0) // Button C pressed - single action
        {
            buttons_prev_state |= (0b1 << 2);
            display_on_button_c();
        }
        // Button C down - repeat action
        display_on_button_c_pressing();
    }
    else // Button C up
    {
        buttons_prev_state &= ~(0b1 << 2);
    }

    if (gfx_pack_read_button(GFX_PACK_BUTTON_D)) // Button D down
    {
        if ((buttons_prev_state & (0b1 << 3)) == 0) // Button D pressed - single action
        {
            buttons_prev_state |= (0b1 << 3);
            display_on_button_d();
        }
        // Button D down - repeat action
        display_on_button_d_pressing();
    }
    else // Button D up
    {
        buttons_prev_state &= ~(0b1 << 3);
    }

    if (gfx_pack_read_button(GFX_PACK_BUTTON_E)) // Button E down
    {
        if ((buttons_prev_state & (0b1 << 4)) == 0) // Button E pressed - single action
        {
            buttons_prev_state |= (0b1 << 4);
            display_on_button_e();
        }
        // Button E down - repeat action
    }
    else // Button E up
    {
        buttons_prev_state &= ~(0b1 << 4);
    }

    return;
}

static void display_on_button_a()
{
    update_display_buffer = true;
    switch(row)
    {
        case DISPLAY_NONE: return;
        case SENSORS:
        {
            if (sensor <= SENSOR_7) // If showing sensor
            {
                sensor = SENSORS_NONE;
                row = DISPLAY_NONE;
            }
            else sensor -= 10; // Else show sensor
            break;
        }
        case DEBUG:
        {
            row = DISPLAY_NONE; // Return from debug menu
            break;
        }
        case GLOBAL_SETTINGS:
        {
            row = DISPLAY_NONE; // Return from global settings
            break;
        }
        case STATUS:
        {
            row = DISPLAY_NONE; // Return from status screen
            break;
        }
        case CHANNELS:
        {
            row = DISPLAY_NONE; // Return from channels screen
            break;
        }
    }
    row_select = 0; // Reset row select
    page_offset = 0; // Reset page offset
    item_offset = 0; // Reset item offset
}

static void display_on_button_b()
{
    update_display_buffer = true;
    switch(row)
    {
        case DISPLAY_NONE:
        {
            if (page_offset >= 5 - 4)
            {
                page_offset = 0;
                row_select = 0;
            }
            else if (row_select == 4) page_offset++;
            else row_select++;
            break;
        }
        case SENSORS:
        {
            if (sensor <= SENSOR_7 && sensor != SENSORS_NONE) sensor = (sensor % 8) + 1;
            else 
            {
                switch (sensors[sensor - 10 - 1].sensor_type)
                {
                    case EE895:
                    {
                        if (EE895_ITEMS <= 5)
                        {
                            row_select = (row_select + 1) % EE895_ITEMS;
                        }
                        else 
                        {
                            if (page_offset >= EE895_ITEMS - 5)
                            {
                                page_offset = 0;
                                row_select = 0;
                            }
                            else if (row_select == 4) page_offset++;
                            else row_select++;
                        }
                        break;
                    }
                    case CDM7162:
                    {
                        if (CDM7162_ITEMS <= 5)
                        {
                            row_select = (row_select + 1) % CDM7162_ITEMS;
                        }
                        else 
                        {
                            if (page_offset >= CDM7162_ITEMS - 5)
                            {
                                page_offset = 0;
                                row_select = 0;
                            }
                            else if (row_select == 4) page_offset++;
                            else row_select++;
                        }
                        break;
                    }
                    case SUNRISE:
                    {
                        if (SUNRISE_ITEMS <= 5)
                        {
                            row_select = (row_select + 1) % SUNRISE_ITEMS;
                        }
                        else 
                        {
                            if (page_offset >= SUNRISE_ITEMS - 5)
                            {
                                page_offset = 0;
                                row_select = 0;
                            }
                            else if (row_select == 4) page_offset++;
                            else row_select++;
                        }
                        break;
                    }
                    case SUNLIGHT:
                    {
                        if (SUNLIGHT_ITEMS <= 5)
                        {
                            row_select = (row_select + 1) % SUNLIGHT_ITEMS;
                        }
                        else 
                        {
                            if (page_offset >= SUNLIGHT_ITEMS - 5)
                            {
                                page_offset = 0;
                                row_select = 0;
                            }
                            else if (row_select == 4) page_offset++;
                            else row_select++;
                        }
                        break;
                    }
                    case SCD30:
                    {
                        if (SCD30_ITEMS <= 5)
                        {
                            row_select = (row_select + 1) % SCD30_ITEMS;
                        }
                        else 
                        {
                            if (page_offset >= SCD30_ITEMS - 5)
                            {
                                page_offset = 0;
                                row_select = 0;
                            }
                            else if (row_select == 4) page_offset++;
                            else row_select++;
                        }
                        break;
                    }
                    case SCD41:
                    {
                        if (SCD41_ITEMS <= 5)
                        {
                            row_select = (row_select + 1) % SCD41_ITEMS;
                        }
                        else 
                        {
                            if (page_offset >= SCD41_ITEMS - 5)
                            {
                                page_offset = 0;
                                row_select = 0;
                            }
                            else if (row_select == 4) page_offset++;
                            else row_select++;
                        }
                        break;
                    }
                    case COZIR_LP3:
                    {
                        if (COZIR_LP3_ITEMS <= 5)
                        {
                            row_select = (row_select + 1) % COZIR_LP3_ITEMS;
                        }
                        else 
                        {
                            if (page_offset >= COZIR_LP3_ITEMS - 5)
                            {
                                page_offset = 0;
                                row_select = 0;
                            }
                            else if (row_select == 4) page_offset++;
                            else row_select++;
                        }
                        break;
                    }
                    case CM1107N:
                    {
                        if (CM1107N_ITEMS <= 5)
                        {
                            row_select = (row_select + 1) % CM1107N_ITEMS;
                        }
                        else 
                        {
                            if (page_offset >= CM1107N_ITEMS - 5)
                            {
                                page_offset = 0;
                                row_select = 0;
                            }
                            else if (row_select == 4) page_offset++;
                            else row_select++;
                        }
                        break;
                    }
                    default: break;
                }
                break;
            }
            break;
        }
        case DEBUG:
        {
            if (page_offset >= 25 - 4)
            {
                page_offset = 0;
                row_select = 0;
            }
            else if (row_select == 4) page_offset++;
            else row_select++;
            break;
        }
        case GLOBAL_SETTINGS:
        {
            if (page_offset >= 9 - 4)
            {
                page_offset = 0;
                row_select = 0;
            }
            else if (row_select == 4) page_offset++;
            else row_select++;
            break;
        }
        case STATUS:
        {
            if (page_offset >= STATUS_ITEMS - 5)
            {
                page_offset = 0;
                row_select = 0;
            }
            else if (row_select == 4) page_offset++;
            else row_select++;
            break;
        }
        case CHANNELS:
        {
            if (page_offset >= 32 - 4)
            {
                page_offset = 0;
                row_select = 0;
            }
            else if (row_select == 3) page_offset++;
            else row_select++;
            break;
        }
        default: break;
    }
    item_offset = 0; // Reset item offset
    return;
}

static void display_on_button_c()
{
    update_display_buffer = true;
    switch(row)
    {
        case DISPLAY_NONE:
        {
            if (row_select == 1 && debug > 0 && service_mode != SERVICE_MODE_UART) debug--;
            break;
        }
        case DEBUG:
        {
            switch (page_offset + row_select)
            {
                case 0:
                {
                    if (debug_main_init > 0 && service_mode != SERVICE_MODE_UART) debug_main_init--;
                    break;
                }
                case 1:
                {
                    if (debug_main_loop > 0 && service_mode != SERVICE_MODE_UART) debug_main_loop--;
                    break;
                }
                case 2:
                {
                    if (debug_sensors > 0 && service_mode != SERVICE_MODE_UART) debug_sensors--;
                    break;
                }
                case 3:
                {
                    if (debug_soap > 0 && service_mode != SERVICE_MODE_UART) debug_soap--;
                    break;
                }
                case 4:
                {
                    if (debug_rtc > 0 && service_mode != SERVICE_MODE_UART) debug_rtc--;
                    break;
                }
                case 5:
                {
                    if (debug_display > 0 && service_mode != SERVICE_MODE_UART) debug_display--;
                    break;
                }
                case 6:
                {
                    if (debug_gfx > 0 && service_mode != SERVICE_MODE_UART) debug_gfx--;
                    break;
                }
                case 7:
                {
                    if (debug_ram > 0 && service_mode != SERVICE_MODE_UART) debug_ram--;
                    break;
                }
                case 8:
                {
                    if (debug_service_comm > 0 && service_mode != SERVICE_MODE_UART) debug_service_comm--;
                    break;
                }
                case 9:
                {
                    if (debug_eeprom > 0 && service_mode != SERVICE_MODE_UART) debug_eeprom--;
                    break;
                }
                case 10:
                {
                    if (debug_mux > 0 && service_mode != SERVICE_MODE_UART) debug_mux--;
                    break;
                }
                case 11:
                {
                    if (debug_power > 0 && service_mode != SERVICE_MODE_UART) debug_power--;
                    break;
                }
                case 12:
                {
                    if (debug_ms5607 > 0 && service_mode != SERVICE_MODE_UART) debug_ms5607--;
                    break;
                }
                case 13:
                {
                    if (debug_hyt271 > 0 && service_mode != SERVICE_MODE_UART) debug_hyt271--;
                    break;
                }
                case 14:
                {
                    if (debug_ee895 > 0 && service_mode != SERVICE_MODE_UART) debug_ee895--;
                    break;
                }
                case 15:
                {
                    if (debug_cdm7162 > 0 && service_mode != SERVICE_MODE_UART) debug_cdm7162--;
                    break;
                }
                case 16:
                {
                    if (debug_sunrise > 0 && service_mode != SERVICE_MODE_UART) debug_sunrise--;
                    break;
                }
                case 17:
                {
                    if (debug_sunlight > 0 && service_mode != SERVICE_MODE_UART) debug_sunlight--;
                    break;
                }
                case 18:
                {
                    if (debug_scd30 > 0 && service_mode != SERVICE_MODE_UART) debug_scd30--;
                    break;
                }
                case 19:
                {
                    if (debug_scd41 > 0 && service_mode != SERVICE_MODE_UART) debug_scd41--;
                    break;
                }
                case 20:
                {
                    if (debug_cozir_lp3 > 0 && service_mode != SERVICE_MODE_UART) debug_cozir_lp3--;
                    break;
                }
                case 21:
                {
                    if (debug_cm1107n > 0 && service_mode != SERVICE_MODE_UART) debug_cm1107n--;
                    break;
                }
                case 22:
                {
                    if (debug_wifi > 0 && service_mode != SERVICE_MODE_UART) debug_wifi--;
                    break;
                }
                case 23:
                {
                    if (debug_tcp_client > 0 && service_mode != SERVICE_MODE_UART) debug_tcp_client--;
                    break;
                }
                case 24:
                {
                    if (debug_tcp_dns > 0 && service_mode != SERVICE_MODE_UART) debug_tcp_dns--;
                    break;
                }
                case 25:
                {
                    if (debug_tcp_server > 0 && service_mode != SERVICE_MODE_UART) debug_tcp_server--;
                    break;
                }
                default:
                {
                    break;
                }
            }
            break;
        }
        case GLOBAL_SETTINGS:
        {
            switch (page_offset + row_select)
            {
                case 2:
                {
                    if (item_offset >= 1) item_offset--;
                    break;
                }
            }
            break;
        }
        case STATUS:
        {
            switch (page_offset + row_select)
            {
                case 3:
                case 9:
                case 10:
                {
                    if (item_offset >= 1) item_offset--;
                    break;
                }
            }
            break;
        }
    }
}

static void display_on_button_d()
{
    update_display_buffer = true;
    switch(row)
    {
        case DISPLAY_NONE:
        {
            if (row_select == 1 && debug < 6 && service_mode != SERVICE_MODE_UART) debug++;
            break;
        }
        case DEBUG:
        {
            switch (page_offset + row_select)
            {
                case 0:
                {
                    if (debug_main_init < 6 && service_mode != SERVICE_MODE_UART) debug_main_init++;
                    break;
                }
                case 1:
                {
                    if (debug_main_loop < 6 && service_mode != SERVICE_MODE_UART) debug_main_loop++;
                    break;
                }
                case 2:
                {
                    if (debug_sensors < 6 && service_mode != SERVICE_MODE_UART) debug_sensors++;
                    break;
                }
                case 3:
                {
                    if (debug_soap < 6 && service_mode != SERVICE_MODE_UART) debug_soap++;
                    break;
                }
                case 4:
                {
                    if (debug_rtc < 6 && service_mode != SERVICE_MODE_UART) debug_rtc++;
                    break;
                }
                case 5:
                {
                    if (debug_display < 6 && service_mode != SERVICE_MODE_UART) debug_display++;
                    break;
                }
                case 6:
                {
                    if (debug_gfx < 6 && service_mode != SERVICE_MODE_UART) debug_gfx++;
                    break;
                }
                case 7:
                {
                    if (debug_ram < 6 && service_mode != SERVICE_MODE_UART) debug_ram++;
                    break;
                }
                case 8:
                {
                    if (debug_service_comm < 6 && service_mode != SERVICE_MODE_UART) debug_service_comm++;
                    break;
                }
                case 9:
                {
                    if (debug_eeprom < 6 && service_mode != SERVICE_MODE_UART) debug_eeprom++;
                    break;
                }
                case 10:
                {
                    if (debug_mux < 6 && service_mode != SERVICE_MODE_UART) debug_mux++;
                    break;
                }
                case 11:
                {
                    if (debug_power < 6 && service_mode != SERVICE_MODE_UART) debug_power++;
                    break;
                }
                case 12:
                {
                    if (debug_ms5607 < 6 && service_mode != SERVICE_MODE_UART) debug_ms5607++;
                    break;
                }
                case 13:
                {
                    if (debug_hyt271 < 6 && service_mode != SERVICE_MODE_UART) debug_hyt271++;
                    break;
                }
                case 14:
                {
                    if (debug_ee895 < 6 && service_mode != SERVICE_MODE_UART) debug_ee895++;
                    break;
                }
                case 15:
                {
                    if (debug_cdm7162 < 6 && service_mode != SERVICE_MODE_UART) debug_cdm7162++;
                    break;
                }
                case 16:
                {
                    if (debug_sunrise < 6 && service_mode != SERVICE_MODE_UART) debug_sunrise++;
                    break;
                }
                case 17:
                {
                    if (debug_sunlight < 6 && service_mode != SERVICE_MODE_UART) debug_sunlight++;
                    break;
                }
                case 18:
                {
                    if (debug_scd30 < 6 && service_mode != SERVICE_MODE_UART) debug_scd30++;
                    break;
                }
                case 19:
                {
                    if (debug_scd41 < 6 && service_mode != SERVICE_MODE_UART) debug_scd41++;
                    break;
                }
                case 20:
                {
                    if (debug_cozir_lp3 < 6 && service_mode != SERVICE_MODE_UART) debug_cozir_lp3++;
                    break;
                }
                case 21:
                {
                    if (debug_cm1107n < 6 && service_mode != SERVICE_MODE_UART) debug_cm1107n++;
                    break;
                }
                case 22:
                {
                    if (debug_wifi < 6 && service_mode != SERVICE_MODE_UART) debug_wifi++;
                    break;
                }
                case 23:
                {
                    if (debug_tcp_client < 6 && service_mode != SERVICE_MODE_UART) debug_tcp_client++;
                    break;
                }
                case 24:
                {
                    if (debug_tcp_dns < 6 && service_mode != SERVICE_MODE_UART) debug_tcp_dns++;
                    break;
                }
                case 25:
                {
                    if (debug_tcp_server < 6 && service_mode != SERVICE_MODE_UART) debug_tcp_server++;
                    break;
                }
                default:
                {
                    break;
                }
            }
            break;
        }
        case GLOBAL_SETTINGS:
        {
            switch (page_offset + row_select)
            {
                case 2:
                {
                    if ((int)item_offset < ((int)strlen(global_configuration.sta_ssid) - 21 + 6)) item_offset++; // Offset, 21 ... display width in chars, 6 ... length of prefix SSID:_
                    break;
                }
            }
            break;
        }
        case STATUS:
        {
            switch (page_offset + row_select)
            {
                case 3:
                {
                    if ((int)item_offset < ((int)strlen(last_message_time) - 21 + 10)) item_offset++; // Offset, 21 ... display width in chars, 10 ... length of prefix LAST MSG:_
                    break;
                }
                case 9:
                {
                    if ((int)item_offset < ((int)strlen(boot_time_str) - 21 + 11)) item_offset++; // Offset, 21 ... display width in chars, 11 ... length of prefix BOOT TIME:_
                    break;
                }
                case 10:
                {
                    uint8_t buf[32];
                    snprintf(buf, 32, "%i", to_ms_since_boot(get_absolute_time()) / 1000);
                    if ((int)item_offset < ((int)strlen(buf) - 21 + 14)) item_offset++; // Offset, 21 ... display width in chars, 14 ... length of prefix SINCE BOOT:_ _s
                    break;
                }
                case 11:
                {
                    if ((int)item_offset < ((int)strlen(fw_version_str) - 21 + 12)) item_offset++; // Offset, 21 ... display width in chars, 11 ... length of prefix FW VERSION:_
                    break;
                }
            }
            break;
        }
    }
}

static void display_on_button_e()
{
    update_display_buffer = true;
    switch(row)
    {
        case DISPLAY_NONE:
            switch(row_select) // Main menu
            {
                case 0: row = SENSORS; sensor = SENSOR_0; break;
                case 1: row = DEBUG; break;
                case 2: row = GLOBAL_SETTINGS; break;
                case 3: row = STATUS; break;
                case 4: row = CHANNELS; break;
                default: return;
            }
            break;
        case SENSORS: // On sensor
            if (sensor <= SENSOR_7 && sensor != SENSORS_NONE) sensor += 10; // On sensor (not sensor config)
            else return;
            break;
        default: return;
    }
    row_select = 0; // Reset row select
    page_offset = 0; // Reset page offset
    item_offset = 0; // Reset item offset
}

static void display_on_button_c_pressing(void)
{
    if (time_reached(brightness_update_timer) && row == DISPLAY_NONE && row_select + page_offset == 5)
    {
        if (display_brightness < 2) return;
        display_brightness -= 2;
        brightness_update_timer = make_timeout_time_ms(brightness_update_time_ms);
    }
}

static void display_on_button_d_pressing(void)
{
    if (time_reached(brightness_update_timer) && row == DISPLAY_NONE && row_select + page_offset == 5)
    {
        if (display_brightness > 98) return;
        display_brightness += 2;
        brightness_update_timer = make_timeout_time_ms(brightness_update_time_ms);
    }
}

static void write_display(void)
{
    gfx_pack_clear_display(); // Clear display
    point_t position = {.x = 0, .y = 0}; // position of datetime string on display
    gfx_pack_write_text(&position, (char*)datetime_str); // write datetime string to display

    switch(row)
    {
        case DISPLAY_NONE:
        {
            uint8_t buf[32];
            for (int i = 0; i < 5; i++)
            {
                switch(page_offset + i)
                {
                    case 0:
                    {
                        snprintf(buf, 32, "SENSORS");
                        break;
                    }
                    case 1:
                    {
                        uint8_t debug_sev_str[6];
                        display_get_debug_str(debug_sev_str, 6, debug);
                        snprintf(buf, 32, "DEBUG LEVEL: %s", debug_sev_str);
                        break;
                    }
                    case 2:
                    {
                        snprintf(buf, 32, "GLOBAL SETTINGS");
                        break;
                    }
                    case 3:
                    {
                        snprintf(buf, 32, "STATUS");
                        break;
                    }
                    case 4:
                    {
                        snprintf(buf, 32, "CHANNELS");
                        break;
                    }
                    case 5:
                    {
                        snprintf(buf, 32, "BRIGHTNESS: %i", display_brightness);
                        break;
                    }
                    default: break;
                }
                position.x = 0;
                position.y = i + 1;
                gfx_pack_write_text(&position, buf);
            }
            gfx_pack_invert_row_color(row_select + 1);
            break;
        }
        case SENSORS:
        {
            if (sensor <= SENSOR_7 && sensor != SENSORS_NONE)
            {
                uint8_t display_sensor = sensor - 1;
                uint8_t sensor_name[24];
                memset(sensor_name, 0x00, 24);
                get_sensor_name_string(&sensors[display_sensor], sensor_name, 24);

                if (strncmp(sensor_name, "\0", 1) == 0) // If no sensor found
                {
                    position.x = 0;
                    position.y = 1;
                    snprintf(sensor_name, 18, "ERR_NO_SENSOR %i", display_sensor); // Write error no sensor to display
                    gfx_pack_write_text(&position, sensor_name);
                }
                else write_display_sensor(sensor_name, sensors[display_sensor].state, 
                        sensors[display_sensor].config.co2_en, sensors[display_sensor].co2,
                        sensors[display_sensor].config.temp_en, sensors[display_sensor].temperature,
                        sensors[display_sensor].config.pressure_en, sensors[display_sensor].pressure,
                        sensors[display_sensor].config.RH_en, sensors[display_sensor].humidity); // Write sensor readings to the display

                snprintf(sensor_name, 24, "ERRORS: %i", sensors[display_sensor].err_total_counter);
                position.x = 0;
                position.y = 5;
                gfx_pack_write_text(&position, sensor_name); // Write number of errors
            }
            else 
            {
                uint8_t buf[32];
                uint8_t sensor_index = sensor - 10 - 1;
                switch(sensors[sensor_index].sensor_type)
                {
                    case EE895:
                    {
                        for (int i = 0; i < 5; i++)
                        {
                            switch (page_offset + i)
                            {
                                case 0:
                                {
                                    snprintf(buf, 32, "SENSOR NO: %x%x", sensors[sensor_index].config.sensor_type, sensors[sensor_index].config.sensor_ord);
                                    break;
                                }
                                case 1:
                                {
                                    snprintf(buf, 32, "INPUT: %i", sensors[sensor_index].index);
                                    break;
                                }
                                case 2:
                                {
                                    snprintf(buf, 32, "INTERFACE: %s", sensors[sensor_index].config.sensor_IIC ? "IIC" : "UART");
                                    break;
                                }
                                case 3:
                                {
                                    snprintf(buf, 32, "POWER: %s", sensors[sensor_index].config.sensor_active ? "ENABLED" : "DISABLED");
                                    break;
                                }
                                case 4:
                                {
                                    snprintf(buf, 32, "VOLTAGE: %s", sensors[sensor_index].config.power_5V ? "5 V" : "3.3 V");
                                    break;
                                }
                                case 5:
                                {
                                    snprintf(buf, 32, "AUX 12 V: %s", sensors[sensor_index].config.power_12V ? "ENABLED" : "DISABLED");
                                    break;
                                }
                                case 6:
                                {
                                    snprintf(buf, 32, "PWR CONN: %s", sensors[sensor_index].config.power_continuous ? "CONT" : "ON MEAS");
                                    break;
                                }
                                case 7:
                                {
                                    snprintf(buf, 32, "PWR CTRL: %s", sensors[sensor_index].config.power_global_control ? "GLOBAL" : "SENSOR");
                                    break;
                                }
                                case 8:
                                {
                                    snprintf(buf, 32, "MANUAL P COMP: %s", sensors[sensor_index].config.ext_pressure_comp ? "TRUE" : "FALSE");
                                    break;
                                }
                                case 9:
                                {
                                    snprintf(buf, 32, "POWER UP TIME: %u s", sensors[sensor_index].config.sensor_power_up_time);
                                    break;
                                }
                                case 10:
                                {
                                    snprintf(buf, 32, "MEAS PERIOD: %u s", sensors[sensor_index].config.meas_period);
                                    break;
                                }
                                case 11:
                                {
                                    snprintf(buf, 32, "FILTER COEFF: %u", sensors[sensor_index].config.filter_coeff);
                                    break;
                                }
                                default: break;
                            }
                            position.x = 0;
                            position.y = i + 1;
                            gfx_pack_write_text(&position, buf);
                            memset(buf, 0x00, 32);
                        }
                        gfx_pack_invert_row_color(row_select + 1);
                        break;
                    }
                    case CDM7162:
                    {
                        for (int i = 0; i < 5; i++)
                        {
                            switch (page_offset + i)
                            {
                                case 0:
                                {
                                    snprintf(buf, 32, "SENSOR NO: %x%x", sensors[sensor_index].config.sensor_type, sensors[sensor_index].config.sensor_ord);
                                    break;
                                }
                                case 1:
                                {
                                    snprintf(buf, 32, "INPUT: %i", sensors[sensor_index].index);
                                    break;
                                }
                                case 2:
                                {
                                    snprintf(buf, 32, "INTERFACE: %s", sensors[sensor_index].config.sensor_IIC ? "IIC" : "UART");
                                    break;
                                }
                                case 3:
                                {
                                    snprintf(buf, 32, "POWER: %s", sensors[sensor_index].config.sensor_active ? "ENABLED" : "DISABLED");
                                    break;
                                }
                                case 4:
                                {
                                    snprintf(buf, 32, "VOLTAGE: %s", sensors[sensor_index].config.power_5V ? "5 V" : "3.3 V");
                                    break;
                                }
                                case 5:
                                {
                                    snprintf(buf, 32, "AUX 12 V: %s", sensors[sensor_index].config.power_12V ? "ENABLED" : "DISABLED");
                                    break;
                                }
                                case 6:
                                {
                                    snprintf(buf, 32, "PWR CONN: %s", sensors[sensor_index].config.power_continuous ? "CONT" : "ON MEAS");
                                    break;
                                }
                                case 7:
                                {
                                    snprintf(buf, 32, "PWR CTRL: %s", sensors[sensor_index].config.power_global_control ? "GLOBAL" : "SENSOR");
                                    break;
                                }
                                case 8:
                                {
                                    snprintf(buf, 32, "MANUAL P COMP: %s", sensors[sensor_index].config.ext_pressure_comp ? "TRUE" : "FALSE");
                                    break;
                                }
                                case 9:
                                {
                                    snprintf(buf, 32, "POWER UP TIME: %u s", sensors[sensor_index].config.sensor_power_up_time);
                                    break;
                                }
                                case 10:
                                {
                                    uint8_t buf2[16];
                                    if (sensors[sensor_index].config.pressure_en)
                                        snprintf(buf2, 16, "%i hPa", sensors[sensor_index].config.pressure);
                                    else snprintf(buf2, 16, "-");
                                    snprintf(buf, 32, "PRESS_COMP: %s", buf2);
                                    break;
                                }
                                case 11:
                                {
                                    uint8_t buf2[16];
                                    if (sensors[sensor_index].config.enable_altitude_comp)
                                        snprintf(buf2, 16, "%i m", sensors[sensor_index].config.altitude);
                                    else snprintf(buf2, 16, "-");
                                    snprintf(buf, 32, "ALTITUDE: %s", buf2);
                                    break;
                                }
                                case 12:
                                {
                                    snprintf(buf, 32, "ABC: %s", sensors[sensor_index].config.enable_abc ? "ENABLED" : "DISABLED");
                                    break;
                                }
                                case 13:
                                {
                                    snprintf(buf, 32, "ALT ABC: %s", sensors[sensor_index].config.enable_alternate_abc ? "ENABLED" : "DISABLED");
                                    break;
                                }
                                case 14:
                                {
                                    snprintf(buf, 32, "ABC PER: %u h", sensors[sensor_index].config.abc_period);
                                    break;
                                }
                                case 15:
                                {
                                    snprintf(buf, 32, "ABC TARGET: %i ppm", sensors[sensor_index].config.abc_target_value);
                                    break;
                                }
                                case 16:
                                {
                                    snprintf(buf, 32, "ALARM HI: %i ppm", sensors[sensor_index].config.alarm_treshold_co2_high);
                                    break;
                                }
                                case 17:
                                {
                                    snprintf(buf, 32, "ALARM LO: %i ppm", sensors[sensor_index].config.alarm_treshold_co2_low);
                                    break;
                                }
                                case 18:
                                {
                                    snprintf(buf, 32, "PWM: %s", sensors[sensor_index].config.enable_PWM_pin ? "ENABLED" : "DISABLED");
                                    break;
                                }
                                case 19:
                                {
                                    snprintf(buf, 32, "PWM RANGE: %s", sensors[sensor_index].config.PWM_range_high ? "HIGH" : "LOW");
                                    break;
                                }
                                default: break;
                            }
                            position.x = 0;
                            position.y = i + 1;
                            gfx_pack_write_text(&position, buf);
                            memset(buf, 0x00, 32);
                        }
                        gfx_pack_invert_row_color(row_select + 1);
                        break;
                    }
                    case SUNRISE:
                    {
                        for (int i = 0; i < 5; i++)
                        {
                            switch (page_offset + i)
                            {
                                case 0:
                                {
                                    snprintf(buf, 32, "SENSOR NO: %x%x", sensors[sensor_index].config.sensor_type, sensors[sensor_index].config.sensor_ord);
                                    break;
                                }
                                case 1:
                                {
                                    snprintf(buf, 32, "INPUT: %i", sensors[sensor_index].index);
                                    break;
                                }
                                case 2:
                                {
                                    snprintf(buf, 32, "INTERFACE: %s", sensors[sensor_index].config.sensor_IIC ? "IIC" : "UART");
                                    break;
                                }
                                case 3:
                                {
                                    snprintf(buf, 32, "POWER: %s", sensors[sensor_index].config.sensor_active ? "ENABLED" : "DISABLED");
                                    break;
                                }
                                case 4:
                                {
                                    snprintf(buf, 32, "VOLTAGE: %s", sensors[sensor_index].config.power_5V ? "5 V" : "3.3 V");
                                    break;
                                }
                                case 5:
                                {
                                    snprintf(buf, 32, "AUX 12 V: %s", sensors[sensor_index].config.power_12V ? "ENABLED" : "DISABLED");
                                    break;
                                }
                                case 6:
                                {
                                    snprintf(buf, 32, "PWR CONN: %s", sensors[sensor_index].config.power_continuous ? "CONT" : "ON MEAS");
                                    break;
                                }
                                case 7:
                                {
                                    snprintf(buf, 32, "PWR CTRL: %s", sensors[sensor_index].config.power_global_control ? "GLOBAL" : "SENSOR");
                                    break;
                                }
                                case 8:
                                {
                                    snprintf(buf, 32, "MANUAL P COMP: %s", sensors[sensor_index].config.ext_pressure_comp ? "TRUE" : "FALSE");
                                    break;
                                }
                                case 9:
                                {
                                    snprintf(buf, 32, "POWER UP TIME: %u s", sensors[sensor_index].config.sensor_power_up_time);
                                    break;
                                }
                                case 10:
                                {
                                    snprintf(buf, 32, "MEAS PERIOD: %u s", sensors[sensor_index].config.meas_period);
                                    break;
                                }
                                case 11:
                                {
                                    snprintf(buf, 32, "FILTER COEFF: %u", sensors[sensor_index].config.filter_coeff);
                                    break;
                                }
                                case 12:
                                {
                                    snprintf(buf, 32, "MEAS SAMPLES: %u", sensors[sensor_index].config.meas_samples);
                                    break;
                                }
                                case 13:
                                {
                                    snprintf(buf, 32, "STATIC IIR: %s", sensors[sensor_index].config.enable_static_IIR ? "ENABLED" : "DISABLED");
                                    break;
                                }
                                case 14:
                                {
                                    snprintf(buf, 32, "DYN IIR: %s", sensors[sensor_index].config.enable_dynamic_IIR ? "ENABLED" : "DISABLED");
                                    break;
                                }
                                case 15:
                                {
                                    uint8_t buf2[16];
                                    if (sensors[sensor_index].config.pressure_en)
                                        snprintf(buf2, 16, "%i hPa", sensors[sensor_index].config.pressure);
                                    else snprintf(buf2, 16, "-");
                                    snprintf(buf, 32, "PRESS_COMP: %s", buf2);
                                    break;
                                }
                                case 16:
                                {
                                    snprintf(buf, 32, "ALT ABC: %s", sensors[sensor_index].config.enable_alternate_abc ? "ENABLED" : "DISABLED");
                                    break;
                                }
                                case 17:
                                {
                                    snprintf(buf, 32, "ABC INIT PER: %u h", sensors[sensor_index].config.abc_init_period);
                                    break;
                                }
                                case 18:
                                {
                                    snprintf(buf, 32, "ABC PER: %u h", sensors[sensor_index].config.abc_period);
                                    break;
                                }
                                case 19:
                                {
                                    snprintf(buf, 32, "ABC TARGET: %i ppm", sensors[sensor_index].config.abc_target_value);
                                    break;
                                }
                                case 20:
                                {
                                    snprintf(buf, 32, "nRDY PIN: %s", sensors[sensor_index].config.enable_nRDY ? "ENABLED" : "DISABLED");
                                    break;
                                }
                                case 21:
                                {
                                    snprintf(buf, 32, "nRDY DIR: %s", sensors[sensor_index].config.invert_nRDY ? "INVERTED" : "NORMAL");
                                    break;
                                }
                                default: break;
                            }
                            position.x = 0;
                            position.y = i + 1;
                            gfx_pack_write_text(&position, buf);
                            memset(buf, 0x00, 32);
                        }
                        gfx_pack_invert_row_color(row_select + 1);
                        break;
                    }
                    case SUNLIGHT:
                    {
                        for (int i = 0; i < 5; i++)
                        {
                            switch (page_offset + i)
                            {
                                case 0:
                                {
                                    snprintf(buf, 32, "SENSOR NO: %x%x", sensors[sensor_index].config.sensor_type, sensors[sensor_index].config.sensor_ord);
                                    break;
                                }
                                case 1:
                                {
                                    snprintf(buf, 32, "INPUT: %i", sensors[sensor_index].index);
                                    break;
                                }
                                case 2:
                                {
                                    snprintf(buf, 32, "INTERFACE: %s", sensors[sensor_index].config.sensor_IIC ? "IIC" : "UART");
                                    break;
                                }
                                case 3:
                                {
                                    snprintf(buf, 32, "POWER: %s", sensors[sensor_index].config.sensor_active ? "ENABLED" : "DISABLED");
                                    break;
                                }
                                case 4:
                                {
                                    snprintf(buf, 32, "VOLTAGE: %s", sensors[sensor_index].config.power_5V ? "5 V" : "3.3 V");
                                    break;
                                }
                                case 5:
                                {
                                    snprintf(buf, 32, "AUX 12 V: %s", sensors[sensor_index].config.power_12V ? "ENABLED" : "DISABLED");
                                    break;
                                }
                                case 6:
                                {
                                    snprintf(buf, 32, "PWR CONN: %s", sensors[sensor_index].config.power_continuous ? "CONT" : "ON MEAS");
                                    break;
                                }
                                case 7:
                                {
                                    snprintf(buf, 32, "PWR CTRL: %s", sensors[sensor_index].config.power_global_control ? "GLOBAL" : "SENSOR");
                                    break;
                                }
                                case 8:
                                {
                                    snprintf(buf, 32, "MANUAL P COMP: %s", sensors[sensor_index].config.ext_pressure_comp ? "TRUE" : "FALSE");
                                    break;
                                }
                                case 9:
                                {
                                    snprintf(buf, 32, "POWER UP TIME: %u s", sensors[sensor_index].config.sensor_power_up_time);
                                    break;
                                }
                                case 10:
                                {
                                    snprintf(buf, 32, "MEAS PERIOD: %u s", sensors[sensor_index].config.meas_period);
                                    break;
                                }
                                case 11:
                                {
                                    snprintf(buf, 32, "FILTER COEFF: %u", sensors[sensor_index].config.filter_coeff);
                                    break;
                                }
                                case 12:
                                {
                                    snprintf(buf, 32, "MEAS SAMPLES: %u", sensors[sensor_index].config.meas_samples);
                                    break;
                                }
                                case 13:
                                {
                                    snprintf(buf, 32, "STATIC IIR: %s", sensors[sensor_index].config.enable_static_IIR ? "ENABLED" : "DISABLED");
                                    break;
                                }
                                case 14:
                                {
                                    snprintf(buf, 32, "DYN IIR: %s", sensors[sensor_index].config.enable_dynamic_IIR ? "ENABLED" : "DISABLED");
                                    break;
                                }
                                case 15:
                                {
                                    uint8_t buf2[16];
                                    if (sensors[sensor_index].config.pressure_en)
                                        snprintf(buf2, 16, "%i hPa", sensors[sensor_index].config.pressure);
                                    else snprintf(buf2, 16, "-");
                                    snprintf(buf, 32, "PRESS_COMP: %s", buf2);
                                    break;
                                }
                                case 16:
                                {
                                    snprintf(buf, 32, "ABC: %s", sensors[sensor_index].config.enable_abc ? "ENABLED" : "DISABLED");
                                    break;
                                }
                                case 17:
                                {
                                    snprintf(buf, 32, "ABC INIT PER: %u h", sensors[sensor_index].config.abc_init_period);
                                    break;
                                }
                                case 18:
                                {
                                    snprintf(buf, 32, "ABC PER: %u h", sensors[sensor_index].config.abc_period);
                                    break;
                                }
                                case 19:
                                {
                                    snprintf(buf, 32, "ABC TARGET: %i ppm", sensors[sensor_index].config.abc_target_value);
                                    break;
                                }
                                case 20:
                                {
                                    snprintf(buf, 32, "nRDY PIN: %s", sensors[sensor_index].config.enable_nRDY ? "ENABLED" : "DISABLED");
                                    break;
                                }
                                case 21:
                                {
                                    snprintf(buf, 32, "nRDY DIR: %s", sensors[sensor_index].config.invert_nRDY ? "INVERTED" : "NORMAL");
                                    break;
                                }
                                default: break;
                            }
                            position.x = 0;
                            position.y = i + 1;
                            gfx_pack_write_text(&position, buf);
                            memset(buf, 0x00, 32);
                        }
                        gfx_pack_invert_row_color(row_select + 1);
                        break;
                    }
                    case SCD30:
                    {
                        for (int i = 0; i < 5; i++)
                        {
                            switch (page_offset + i)
                            {
                                case 0:
                                {
                                    snprintf(buf, 32, "SENSOR NO: %x%x", sensors[sensor_index].config.sensor_type, sensors[sensor_index].config.sensor_ord);
                                    break;
                                }
                                case 1:
                                {
                                    snprintf(buf, 32, "INPUT: %i", sensors[sensor_index].index);
                                    break;
                                }
                                case 2:
                                {
                                    snprintf(buf, 32, "INTERFACE: %s", sensors[sensor_index].config.sensor_IIC ? "IIC" : "UART");
                                    break;
                                }
                                case 3:
                                {
                                    snprintf(buf, 32, "POWER: %s", sensors[sensor_index].config.sensor_active ? "ENABLED" : "DISABLED");
                                    break;
                                }
                                case 4:
                                {
                                    snprintf(buf, 32, "VOLTAGE: %s", sensors[sensor_index].config.power_5V ? "5 V" : "3.3 V");
                                    break;
                                }
                                case 5:
                                {
                                    snprintf(buf, 32, "AUX 12 V: %s", sensors[sensor_index].config.power_12V ? "ENABLED" : "DISABLED");
                                    break;
                                }
                                case 6:
                                {
                                    snprintf(buf, 32, "PWR CONN: %s", sensors[sensor_index].config.power_continuous ? "CONT" : "ON MEAS");
                                    break;
                                }
                                case 7:
                                {
                                    snprintf(buf, 32, "PWR CTRL: %s", sensors[sensor_index].config.power_global_control ? "GLOBAL" : "SENSOR");
                                    break;
                                }
                                case 8:
                                {
                                    snprintf(buf, 32, "MANUAL P COMP: %s", sensors[sensor_index].config.ext_pressure_comp ? "TRUE" : "FALSE");
                                    break;
                                }
                                case 9:
                                {
                                    snprintf(buf, 32, "POWER UP TIME: %u s", sensors[sensor_index].config.sensor_power_up_time);
                                    break;
                                }
                                case 10:
                                {
                                    snprintf(buf, 32, "MEAS PERIOD: %u s", sensors[sensor_index].config.meas_period);
                                    break;
                                }
                                case 11:
                                {
                                    uint8_t buf2[16];
                                    if (sensors[sensor_index].config.pressure_en)
                                        snprintf(buf2, 16, "%i hPa", sensors[sensor_index].config.pressure);
                                    else snprintf(buf2, 16, "-");
                                    snprintf(buf, 32, "PRESS_COMP: %s", buf2);
                                    break;
                                }
                                case 12:
                                {
                                    uint8_t buf2[16];
                                    if (sensors[sensor_index].config.enable_altitude_comp)
                                        snprintf(buf2, 16, "%i m", sensors[sensor_index].config.altitude);
                                    else snprintf(buf2, 16, "-");
                                    snprintf(buf, 32, "ALTITUDE: %s", buf2);
                                    break;
                                }
                                case 13:
                                {
                                    snprintf(buf, 32, "ABC: %s", sensors[sensor_index].config.enable_abc ? "ENABLED" : "DISABLED");
                                    break;
                                }
                                default: break;
                            }
                            position.x = 0;
                            position.y = i + 1;
                            gfx_pack_write_text(&position, buf);
                            memset(buf, 0x00, 32);
                        }
                        gfx_pack_invert_row_color(row_select + 1);
                        break;
                    }
                    case SCD41:
                    {
                        for (int i = 0; i < 5; i++)
                        {
                            switch (page_offset + i)
                            {
                                case 0:
                                {
                                    snprintf(buf, 32, "SENSOR NO: %x%x", sensors[sensor_index].config.sensor_type, sensors[sensor_index].config.sensor_ord);
                                    break;
                                }
                                case 1:
                                {
                                    snprintf(buf, 32, "INPUT: %i", sensors[sensor_index].index);
                                    break;
                                }
                                case 2:
                                {
                                    snprintf(buf, 32, "INTERFACE: %s", sensors[sensor_index].config.sensor_IIC ? "IIC" : "UART");
                                    break;
                                }
                                case 3:
                                {
                                    snprintf(buf, 32, "POWER: %s", sensors[sensor_index].config.sensor_active ? "ENABLED" : "DISABLED");
                                    break;
                                }
                                case 4:
                                {
                                    snprintf(buf, 32, "VOLTAGE: %s", sensors[sensor_index].config.power_5V ? "5 V" : "3.3 V");
                                    break;
                                }
                                case 5:
                                {
                                    snprintf(buf, 32, "AUX 12 V: %s", sensors[sensor_index].config.power_12V ? "ENABLED" : "DISABLED");
                                    break;
                                }
                                case 6:
                                {
                                    snprintf(buf, 32, "PWR CONN: %s", sensors[sensor_index].config.power_continuous ? "CONT" : "ON MEAS");
                                    break;
                                }
                                case 7:
                                {
                                    snprintf(buf, 32, "PWR CTRL: %s", sensors[sensor_index].config.power_global_control ? "GLOBAL" : "SENSOR");
                                    break;
                                }
                                case 8:
                                {
                                    snprintf(buf, 32, "MANUAL P COMP: %s", sensors[sensor_index].config.ext_pressure_comp ? "TRUE" : "FALSE");
                                    break;
                                }
                                case 9:
                                {
                                    snprintf(buf, 32, "POWER UP TIME: %u s", sensors[sensor_index].config.sensor_power_up_time);
                                    break;
                                }
                                case 10:
                                {
                                    uint8_t buf2[16];
                                    if (sensors[sensor_index].config.pressure_en)
                                        snprintf(buf2, 16, "%i hPa", sensors[sensor_index].config.pressure);
                                    else snprintf(buf2, 16, "-");
                                    snprintf(buf, 32, "PRESS_COMP: %s", buf2);
                                    break;
                                }
                                case 11:
                                {
                                    uint8_t buf2[16];
                                    if (sensors[sensor_index].config.enable_altitude_comp)
                                        snprintf(buf2, 16, "%i m", sensors[sensor_index].config.altitude);
                                    else snprintf(buf2, 16, "-");
                                    snprintf(buf, 32, "ALTITUDE: %s", buf2);
                                    break;
                                }
                                case 12:
                                {
                                    snprintf(buf, 32, "ABC: %s", sensors[sensor_index].config.enable_abc ? "ENABLED" : "DISABLED");
                                    break;
                                }
                                case 13:
                                {
                                    snprintf(buf, 32, "ABC INIT PER: %u h", sensors[sensor_index].config.abc_init_period);
                                    break;
                                }
                                case 14:
                                {
                                    snprintf(buf, 32, "ABC PER: %u h", sensors[sensor_index].config.abc_period);
                                    break;
                                }
                                default: break;
                            }
                            position.x = 0;
                            position.y = i + 1;
                            gfx_pack_write_text(&position, buf);
                            memset(buf, 0x00, 32);
                        }
                        gfx_pack_invert_row_color(row_select + 1);
                        break;
                    }
                    case COZIR_LP3:
                    {
                        for (int i = 0; i < 5; i++)
                        {
                            switch (page_offset + i)
                            {
                                case 0:
                                {
                                    snprintf(buf, 32, "SENSOR NO: %x%x", sensors[sensor_index].config.sensor_type, sensors[sensor_index].config.sensor_ord);
                                    break;
                                }
                                case 1:
                                {
                                    snprintf(buf, 32, "INPUT: %i", sensors[sensor_index].index);
                                    break;
                                }
                                case 2:
                                {
                                    snprintf(buf, 32, "INTERFACE: %s", sensors[sensor_index].config.sensor_IIC ? "IIC" : "UART");
                                    break;
                                }
                                case 3:
                                {
                                    snprintf(buf, 32, "POWER: %s", sensors[sensor_index].config.sensor_active ? "ENABLED" : "DISABLED");
                                    break;
                                }
                                case 4:
                                {
                                    snprintf(buf, 32, "VOLTAGE: %s", sensors[sensor_index].config.power_5V ? "5 V" : "3.3 V");
                                    break;
                                }
                                case 5:
                                {
                                    snprintf(buf, 32, "AUX 12 V: %s", sensors[sensor_index].config.power_12V ? "ENABLED" : "DISABLED");
                                    break;
                                }
                                case 6:
                                {
                                    snprintf(buf, 32, "PWR CONN: %s", sensors[sensor_index].config.power_continuous ? "CONT" : "ON MEAS");
                                    break;
                                }
                                case 7:
                                {
                                    snprintf(buf, 32, "PWR CTRL: %s", sensors[sensor_index].config.power_global_control ? "GLOBAL" : "SENSOR");
                                    break;
                                }
                                case 8:
                                {
                                    snprintf(buf, 32, "MANUAL P COMP: %s", sensors[sensor_index].config.ext_pressure_comp ? "TRUE" : "FALSE");
                                    break;
                                }
                                case 9:
                                {
                                    snprintf(buf, 32, "POWER UP TIME: %u s", sensors[sensor_index].config.sensor_power_up_time);
                                    break;
                                }
                                case 10:
                                {
                                    snprintf(buf, 32, "FILTER COEFF: %u", sensors[sensor_index].config.filter_coeff);
                                    break;
                                }
                                case 11:
                                {
                                    uint8_t buf2[16];
                                    if (sensors[sensor_index].config.pressure_en)
                                        snprintf(buf2, 16, "%i hPa", sensors[sensor_index].config.pressure);
                                    else snprintf(buf2, 16, "-");
                                    snprintf(buf, 32, "PRESS_COMP: %s", buf2);
                                    break;
                                }
                                case 12:
                                {
                                    snprintf(buf, 32, "ABC: %s", sensors[sensor_index].config.enable_abc ? "ENABLED" : "DISABLED");
                                    break;
                                }
                                case 13:
                                {
                                    snprintf(buf, 32, "ABC INIT PER: %u h", sensors[sensor_index].config.abc_init_period);
                                    break;
                                }
                                case 14:
                                {
                                    snprintf(buf, 32, "ABC PER: %u h", sensors[sensor_index].config.abc_period);
                                    break;
                                }
                                case 15:
                                {
                                    snprintf(buf, 32, "ABC TARGET: %i ppm", sensors[sensor_index].config.abc_target_value);
                                    break;
                                }
                                case 16:
                                {
                                    uint8_t buf2[16];
                                    if (sensors[sensor_index].config.alarm_en)
                                        snprintf(buf2, 16, "%i hPa", sensors[sensor_index].config.alarm_treshold_co2_high);
                                    else snprintf(buf2, 16, "-");
                                    snprintf(buf, 32, "ALARM HI: %s ppm", buf2);
                                    break;
                                }
                                case 17:
                                {
                                    snprintf(buf, 32, "PWM: %s", sensors[sensor_index].config.enable_PWM_pin ? "ENABLED" : "DISABLED");
                                    break;
                                }
                                default: break;
                            }
                            position.x = 0;
                            position.y = i + 1;
                            gfx_pack_write_text(&position, buf);
                            memset(buf, 0x00, 32);
                        }
                        gfx_pack_invert_row_color(row_select + 1);
                        break;
                    }
                    case CM1107N:
                    {
                        for (int i = 0; i < 5; i++)
                        {
                            switch (page_offset + i)
                            {
                                case 0:
                                {
                                    snprintf(buf, 32, "SENSOR NO: %x%x", sensors[sensor_index].config.sensor_type, sensors[sensor_index].config.sensor_ord);
                                    break;
                                }
                                case 1:
                                {
                                    snprintf(buf, 32, "INPUT: %i", sensors[sensor_index].index);
                                    break;
                                }
                                case 2:
                                {
                                    snprintf(buf, 32, "INTERFACE: %s", sensors[sensor_index].config.sensor_IIC ? "IIC" : "UART");
                                    break;
                                }
                                case 3:
                                {
                                    snprintf(buf, 32, "POWER: %s", sensors[sensor_index].config.sensor_active ? "ENABLED" : "DISABLED");
                                    break;
                                }
                                case 4:
                                {
                                    snprintf(buf, 32, "VOLTAGE: %s", sensors[sensor_index].config.power_5V ? "5 V" : "3.3 V");
                                    break;
                                }
                                case 5:
                                {
                                    snprintf(buf, 32, "AUX 12 V: %s", sensors[sensor_index].config.power_12V ? "ENABLED" : "DISABLED");
                                    break;
                                }
                                case 6:
                                {
                                    snprintf(buf, 32, "PWR CONN: %s", sensors[sensor_index].config.power_continuous ? "CONT" : "ON MEAS");
                                    break;
                                }
                                case 7:
                                {
                                    snprintf(buf, 32, "PWR CTRL: %s", sensors[sensor_index].config.power_global_control ? "GLOBAL" : "SENSOR");
                                    break;
                                }
                                case 8:
                                {
                                    snprintf(buf, 32, "MANUAL P COMP: %s", sensors[sensor_index].config.ext_pressure_comp ? "TRUE" : "FALSE");
                                    break;
                                }
                                case 9:
                                {
                                    snprintf(buf, 32, "POWER UP TIME: %u s", sensors[sensor_index].config.sensor_power_up_time);
                                    break;
                                }
                                case 10:
                                {
                                    snprintf(buf, 32, "ABC: %s", sensors[sensor_index].config.enable_abc ? "ENABLED" : "DISABLED");
                                    break;
                                }
                                case 11:
                                {
                                    snprintf(buf, 32, "ABC PER: %u h", sensors[sensor_index].config.abc_period);
                                    break;
                                }
                                case 12:
                                {
                                    snprintf(buf, 32, "ABC TARGET: %i ppm", sensors[sensor_index].config.abc_target_value);
                                    break;
                                }
                                default: break;
                            }
                            position.x = 0;
                            position.y = i + 1;
                            gfx_pack_write_text(&position, buf);
                            memset(buf, 0x00, 32);
                        }
                        gfx_pack_invert_row_color(row_select + 1);
                        break;
                    }
                    default: break;
                }
            }
            break;
        }
        case DEBUG:
        {
            for (int i = 0; i < 5; i++)
            {
                uint8_t buf[32];
                switch (page_offset + i)
                {
                    case 0:
                    {
                        snprintf(buf, 32, "DEBUG MAIN INIT: %i", debug_main_init);
                        break;
                    }
                    case 1:
                    {
                        snprintf(buf, 32, "DEBUG MAIN LOOP: %i", debug_main_loop);
                        break;
                    }
                    case 2:
                    {
                        snprintf(buf, 32, "DEBUG SENSORS: %i", debug_sensors);
                        break;
                    }
                    case 3:
                    {
                        snprintf(buf, 32, "DEBUG SOAP: %i", debug_soap);
                        break;
                    }
                    case 4:
                    {
                        snprintf(buf, 32, "DEBUG RTC: %i", debug_rtc);
                        break;
                    }
                    case 5:
                    {
                        snprintf(buf, 32, "DEBUG DISPLAY: %i", debug_display);
                        break;
                    }
                    case 6:
                    {
                        snprintf(buf, 32, "DEBUG GFX: %i", debug_gfx);
                        break;
                    }
                    case 7:
                    {
                        snprintf(buf, 32, "DEBUG RAM: %i", debug_ram);
                        break;
                    }
                    case 8:
                    {
                        snprintf(buf, 32, "DEBUG SERVICE COMM: %i", debug_service_comm);
                        break;
                    }
                    case 9:
                    {
                        snprintf(buf, 32, "DEBUG EEPROM: %i", debug_eeprom);
                        break;
                    }
                    case 10:
                    {
                        snprintf(buf, 32, "DEBUG MUX: %i", debug_mux);
                        break;
                    }
                    case 11:
                    {
                        snprintf(buf, 32, "DEBUG POWER: %i", debug_power);
                        break;
                    }
                    case 12:
                    {
                        snprintf(buf, 32, "DEBUG MS5607: %i", debug_ms5607);
                        break;
                    }
                    case 13:
                    {
                        snprintf(buf, 32, "DEBUG HYT271: %i", debug_hyt271);
                        break;
                    }
                    case 14:
                    {
                        snprintf(buf, 32, "DEBUG EE895: %i", debug_ee895);
                        break;
                    }
                    case 15:
                    {
                        snprintf(buf, 32, "DEBUG CDM7162: %i", debug_cdm7162);
                        break;
                    }
                    case 16:
                    {
                        snprintf(buf, 32, "DEBUG SUNRISE: %i", debug_sunrise);
                        break;
                    }
                    case 17:
                    {
                        snprintf(buf, 32, "DEBUG SUNLIGHT: %i", debug_sunlight);
                        break;
                    }
                    case 18:
                    {
                        snprintf(buf, 32, "DEBUG SCD30: %i", debug_scd30);
                        break;
                    }
                    case 19:
                    {
                        snprintf(buf, 32, "DEBUG SCD41: %i", debug_scd41);
                        break;
                    }
                    case 20:
                    {
                        snprintf(buf, 32, "DEBUG COZIR-LP3: %i", debug_cozir_lp3);
                        break;
                    }
                    case 21:
                    {
                        snprintf(buf, 32, "DEBUG CM1107N: %i", debug_cm1107n);
                        break;
                    }
                    case 22:
                    {
                        snprintf(buf, 32, "DEBUG WIFI: %i", debug_wifi);
                        break;
                    }
                    case 23:
                    {
                        snprintf(buf, 32, "DEBUG TCP CLIENT: %i", debug_tcp_client);
                        break;
                    }
                    case 24:
                    {
                        snprintf(buf, 32, "DEBUG TCP DNS: %i", debug_tcp_dns);
                        break;
                    }
                    case 25:
                    {
                        snprintf(buf, 32, "DEBUG TCP SERVER: %i", debug_tcp_server);
                        break;
                    }
                    default:
                    {
                        break;
                    }
                }
                position.x = 0;
                position.y = i + 1;
                gfx_pack_write_text(&position, buf);
            }
            gfx_pack_invert_row_color(row_select + 1);
            break;
        }
        case GLOBAL_SETTINGS:
        {
            for (int i = 0; i < 5; i++)
            {
                uint8_t buf[48];
                switch (page_offset + i)
                {
                    case 0:
                    {
                        snprintf(buf, 32, "SN: %08x", global_configuration.ser_num);
                        break;
                    }
                    case 1:
                    {
                        snprintf(buf, 32, "AUX SN: %08x", global_configuration.ser_num_aux);
                        break;
                    }
                    case 2:
                    {
                        snprintf(buf, 48, "SSID: %s", global_configuration.sta_ssid);
                        break;
                    }
                    case 3:
                    {
                        snprintf(buf, 32, "IP: %s", global_configuration.sta_ip);
                        break;
                    }
                    case 4:
                    {
                        snprintf(buf, 32, "MEAS INT: %u s", global_configuration.meas_int / 1000);
                        break;
                    }
                    case 5:
                    {
                        snprintf(buf, 32, "SEND INT: %u s", global_configuration.soap_int / 1000);
                        break;
                    }
                    case 6:
                    {
                        switch (global_configuration.soap_mode)
                        {
                            case 0x00:
                                snprintf(buf, 32, "MSG TO: NONE"); break;
                            case 0x01:
                                snprintf(buf, 32, "MSG TO: DATABASE"); break;
                            case 0x02:
                                snprintf(buf, 32, "MSG TO: CLOUD"); break;
                        }
                        break;
                    }
                    case 7:
                    {
                        snprintf(buf, 32, "AUX MSG: %s", global_configuration.aux_msg ? "TRUE" : "FALSE");
                        break;
                    }
                    case 8:
                    {
                        snprintf(buf, 32, "SENSOR REINIT: %s", global_configuration.reinit_sensors_on_error ? "TRUE" : "FALSE");
                        break;
                    }
                    case 9:
                    {
                        snprintf(buf, 32, "DESC: %s", global_configuration.device_desc);
                        break;
                    }
                    case 10:
                    {
                        snprintf(buf, 32, "");
                        break;
                    }
                    default:
                    {
                        break;
                    }
                }
                position.x = 0;
                position.y = i + 1;
                if (i == row_select) gfx_pack_write_text(&position, &buf[item_offset]); // If selected write with offset
                else gfx_pack_write_text(&position, buf);
            }
            gfx_pack_invert_row_color(row_select + 1);
            break;
        }
        case STATUS:
        {
            for (int i = 0; i < 5; i++)
            {
                uint8_t buf[32];
                switch (page_offset + i)
                {
                    case 0:
                    {
                        snprintf(buf, 32, "WIFI: %s", wifi_is_running() ? "CONNECTED" : "NOT CONNECTED");
                        break;
                    }
                    case 1:
                    {
                        uint8_t server_state[12];
                        switch(tcp_server_is_running())
                        {
                            case 0:
                            {
                                snprintf(server_state, 12, "STOPPED");
                                break;
                            }
                            case 1:
                            {
                                snprintf(server_state, 12, "LISTENING");
                                break;
                            }
                            case 2:
                            {
                                snprintf(server_state, 12, "CONNECTED");
                                break;
                            }
                            default:
                            {
                                snprintf(server_state, 12, "UNKNOWN");
                                break;
                            }
                        }
                        snprintf(buf, 32, "TCP SERVER: %s", server_state);
                        break;
                    }
                    case 2:
                    {
                        snprintf(buf, 32, "TCP CLIENT: %s", tcp_client_is_running() ? "RUNNING" : "STOPPED");
                        break;
                    }
                    case 3:
                    {
                        snprintf(buf, 32, "LAST MSG: %s", last_message_time);
                        break;
                    }
                    case 4:
                    {
                        snprintf(buf, 32, "NEXT MSG IN: %lli s", MAX(absolute_time_diff_us(get_absolute_time(), send_data_time) / 1000000, 0));
                        break;
                    }
                    case 5:
                    {
                        snprintf(buf, 32, "LAST MSG ERR: %i", last_message_error);
                        break;
                    }
                    case 6:
                    {
                        snprintf(buf, 32, "NEXT MEAS IN: %lli s", MAX(absolute_time_diff_us(get_absolute_time(), sensor_start_measurement_time) / 1000000, 0));
                        break;
                    }
                    case 7:
                    {
                        snprintf(buf, 32, "FREE RAM: %llu B", getFreeHeap());
                        break;
                    }
                    case 8:
                    {
                        uint8_t svc_mode_str[12];
                        switch (service_mode)
                        {
                            case SERVICE_MODE_DISABLED:
                            {
                                snprintf(svc_mode_str, 12, "NONE");
                                break;
                            }
                            case SERVICE_MODE_ETHERNET:
                            {
                                snprintf(svc_mode_str, 12, "ETHERNET");
                                break;
                            }
                            case SERVICE_MODE_UART:
                            {
                                snprintf(svc_mode_str, 12, "UART");
                                break;
                            }
                            default: break;
                        }
                        snprintf(buf, 32, "SVC MODE: %s", svc_mode_str);
                        break;
                    }
                    case 9:
                    {
                        snprintf(buf, 32, "BOOT TIME: %s", boot_time_str);
                        break;
                    }
                    case 10:
                    {
                        snprintf(buf, 32, "SINCE BOOT: %i s", to_ms_since_boot(get_absolute_time()) / 1000);
                        break;
                    }
                    case 11:
                    {
                        snprintf(buf, 32, "FW VERSION: %s", fw_version_str);
                        break;
                    }
                    default:
                    {
                        break;
                    }
                }
                position.x = 0;
                position.y = i + 1;
                if (i == row_select) gfx_pack_write_text(&position, &buf[item_offset]); // If selected write with offset
                else gfx_pack_write_text(&position, buf);
            }
            gfx_pack_invert_row_color(row_select + 1);
            break;
        }
        case CHANNELS:
        {
            uint8_t buf[32];
            position.x = 0;
            position.y = 1;
            gfx_pack_write_text(&position, "NO|SENSOR|ACT|UNIT");
            for (int i = 0; i < 4; i++)
            {
                if (page_offset + i < 16)
                {
                    uint8_t unit[4];
                    switch (global_configuration.channel_quant[page_offset + i])
                    {
                        case MEASURED_VALUE_CO2:
                            snprintf(unit, 4, "ppm"); break;
                        case MEASURED_VALUE_P:
                            snprintf(unit, 4, "hPa"); break;
                        case MEASURED_VALUE_RH:
                            snprintf(unit, 4, "%%"); break;
                        case MEASURED_VALUE_T:
                            snprintf(unit, 4, "%cC", 0xB0); break;
                        default: break;
                    }
                    snprintf(buf, 32, "%02u|  %u%u  | %c |%s", page_offset + i, sensors[global_configuration.channel_idx[page_offset + i]].sensor_type, 
                        sensors[global_configuration.channel_idx[page_offset + i]].sensor_number, global_configuration.channel_act[page_offset + i] ? 'T' : 'F', unit);
                }
                else
                {
                    uint8_t unit[4] = {0};
                    switch (channels2[page_offset + i - 16]->measured_value_type)
                    {
                        case MEASURED_VALUE_CO2:
                            snprintf(unit, 4, "ppm"); break;
                        case MEASURED_VALUE_P:
                            snprintf(unit, 4, "hPa"); break;
                        case MEASURED_VALUE_RH:
                            snprintf(unit, 4, "%%"); break;
                        case MEASURED_VALUE_T:
                            snprintf(unit, 4, "%cC", 0xB0); break;
                        default: break;
                    }
                    snprintf(buf, 32, "%02u|      | %c |%s", page_offset + i, channels2[page_offset + i - 16]->channel_active ? 'T' : 'F', unit);
                }
                position.x = 0;
                position.y = i + 2;
                gfx_pack_write_text(&position, buf);
            }
            gfx_pack_invert_row_color(row_select + 2);
        }
    }
    return;
}

static void get_sensor_name_string(sensor_t* sensor, uint8_t* buf, uint8_t len)
{
    if (len < 24) return;
    switch(sensor->sensor_type) // On sensor type generate string
    {
        case EE895:
        {
            snprintf(buf, len, "E+E EE895");
            break;
        }
        case CDM7162:
        {
            snprintf(buf, len, "Figaro CDM7162");
            break;
        }
        case SUNRISE:
        {
            snprintf(buf, len, "Senseair SUNRISE");
            break;
        }
        case SUNLIGHT:
        {
            snprintf(buf, len, "Senseair SUNLIGHT");
            break;
        }
        case SCD30:
        {
            snprintf(buf, len, "Sensirion SCD30");
            break;
        }
        case SCD41:
        {
            snprintf(buf, len, "Sensirion SCD41");
            break;
        }
        case COZIR_LP3:
        {
            snprintf(buf, len, "GSS CozIR-LP3");
            break;
        }
        case CM1107N:
        {
            snprintf(buf, len, "Cubic CM1107N");
            break;
        }
        default:
        {
            memset(buf, 0x00, len);
            break;
        }
    }
    return;
}

static void write_display_sensor(uint8_t* sensor_name, int state, 
        bool co2, float co2_value, 
        bool temp, float temp_value, 
        bool pressure, float pressure_value,
        bool humidity, float humidity_value)
{
    uint8_t row = 1;
    point_t position;
    position.x = 0;
    position.y = row;
    gfx_pack_write_text(&position, sensor_name); // Write sensor name
    
    row++;
    position.x = 0;
    position.y = row;

    if (state != 0) // If sensor in invalid state
    {
        uint8_t buf[6];
        
        snprintf(buf, 6, "E%i", state);
        gfx_pack_write_text(&position, buf); // Write error code (sensor state)
        row++;
        position.x = 0;
        position.y = row;
    }
    else
    {
        uint8_t buf[16];
        memset(buf, 0x00, 16);

        if (co2)
        {
            snprintf(buf, 16, "CO2: %.0f ppm", co2_value);
            gfx_pack_write_text(&position, buf); // Write co2 concentration
            row++;
            position.x = 0;
            position.y = row;
            memset(buf, 0x00, 16);
        }
        if (temp)
        {
            snprintf(buf, 16, "T: %4.2f %cC", temp_value, 0xB0); // C char does not support ° symbol
            gfx_pack_write_text(&position, buf); // Write temperature
            row++;
            position.x = 0;
            position.y = row;
            memset(buf, 0x00, 16);

        }
        if (pressure)
        {
            snprintf(buf, 16, "p: %4.1f hPa", pressure_value);
            gfx_pack_write_text(&position, buf); // Write pressure
            row++;
            position.x = 0;
            position.y = row;
            memset(buf, 0x00, 16);
        }
        if (humidity)
        {
            snprintf(buf, 16, "RH: %4.1f %%", humidity_value);
            gfx_pack_write_text(&position, buf); // Write humidity
            row++;
            position.x = 0;
            position.y = row;
            memset(buf, 0x00, 16);
        }
    }
}

static void display_get_debug_str(uint8_t* buf, uint8_t len, uint8_t debug_severity)
{
    switch (debug_severity)
    {
        case 0:
            snprintf(buf, len, "NONE");
            break;
        case 1:
            snprintf(buf, len, "FATAL");
            break;
        case 2:
            snprintf(buf, len, "ERROR");
            break;
        case 3:
            snprintf(buf, len, "WARN");
            break;
        case 4:
            snprintf(buf, len, "INFO");
            break;
        case 5:
            snprintf(buf, len, "DEBUG");
            break;
        case 6:
            snprintf(buf, len, "TRACE");
            break;
        default:
            snprintf(buf, len, "???");
            break;
    }
}

static uint32_t getTotalHeap(void) {
   extern char __StackLimit, __bss_end__;
   
   return &__StackLimit  - &__bss_end__;
}

static uint64_t getFreeHeap(void) {
    struct mallinfo m = mallinfo();
    uint64_t memfree = getTotalHeap() - m.uordblks;
    print_ser_output(SEVERITY_DEBUG, SOURCE_RAM, SOURCE_NO_SOURCE, "Available memory: %d bytes", memfree);
    return memfree;
}
