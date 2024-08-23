#include "debug.h"
#include "stdarg.h"
#include "pico/stdlib.h"
#include "string.h"
#include "pico/printf.h"

#define COLORED_DEBUG true

#if COLORED_DEBUG
    #define RED_BOLD    "\033[1;31;40m"
    #define RED         "\033[0;31m"
    #define GRN         "\033[0;32m"
    #define YEL         "\033[0;33m"
    #define BLU         "\033[0;34m"
    #define MAG         "\033[0;35m"
    #define CYN         "\033[0;36m"
    #define WHT         "\033[0;37m"
    #define RESET       "\033[0;0m"
    #define COLOR_FATAL "\033[1;37;41m"
    #define COLOR_ERROR "\033[1;31m"
    #define COLOR_WARN  "\033[0;33m"
    #define COLOR_INFO  "\033[1m"
    #define COLOR_DEBUG "\033[1;90m"
    #define COLOR_TRACE "\033[0;90m"
#else
    #define RED_BOLD    ""
    #define RED         ""
    #define GRN         ""
    #define YEL         ""
    #define BLU         ""
    #define MAG         ""
    #define CYN         ""
    #define WHT         ""
    #define RESET       ""
    #define COLOR_FATAL ""
    #define COLOR_ERROR ""
    #define COLOR_WARN  ""
    #define COLOR_INFO  ""
    #define COLOR_DEBUG ""
    #define COLOR_TRACE ""
#endif

/**DEBUG LEVELS
 * 0 ... DEBUG messages disabled
 * 1 ... display FATAL errors
 * 2 ... display ERROR and higher
 * 3 ... display Warn and higher
 * 4 ... display info and higher
 * 5 ... display debug and higher
 * 6 ... display trace and higher
 */
uint8_t debug = 6; // Global max debug level

static uint8_t debug_main_init = 4;
static uint8_t debug_main_loop = 4;
static uint8_t debug_sensors = 4;
static uint8_t debug_soap = 3;
static uint8_t debug_rtc = 3;
static uint8_t debug_display = 0;
static uint8_t debug_gfx = 0;
static uint8_t debug_ram = 0;
static uint8_t debug_service_comm = 4;
static uint8_t debug_eeprom = 4;

static uint8_t debug_mux = 3;
static uint8_t debug_power = 3;
static uint8_t debug_ms5607 = 3;
static uint8_t debug_hyt271 = 3;
static uint8_t debug_ee895 = 3;
static uint8_t debug_cdm7162 = 3;
static uint8_t debug_sunrise = 3;
static uint8_t debug_sunlight = 3;
static uint8_t debug_scd30 = 3;
static uint8_t debug_scd41 = 3;
static uint8_t debug_cozir_lp3 = 3;
static uint8_t debug_cm1107n = 3;

uint8_t debug_wifi = 3;
uint8_t debug_tcp_client = 3;
uint8_t debug_tcp_server = 5;
uint8_t debug_tcp_dns = 3;

uint8_t debug_core0 = 6;
uint8_t debug_core1 = 4;

void print_ser_output(debug_severity_e severity, debug_source_e source, debug_source_e subsource, const uint8_t* message, ...)
{
    if (debug >= severity)
    {
        int32_t message_len = strlen(message);
        message_len += 64;
        uint8_t buf[message_len];
        uint8_t severity_str[8];
        uint8_t severity_color[12];
        uint8_t source_str[13] = "            ";
        uint8_t subsource_str[13] = "            ";

        switch(source)
        {
            case SOURCE_NO_SOURCE:
                snprintf(source_str, 13, "            ");
                break;
            case SOURCE_MAIN_INIT:
                if (severity > debug_main_init && subsource == SOURCE_NO_SOURCE) return;
                snprintf(source_str, 13, "[MAIN-INIT] ");
                break;
            case SOURCE_MAIN_LOOP:
                if (severity > debug_main_loop && subsource == SOURCE_NO_SOURCE) return;
                snprintf(source_str, 13, "[MAIN-LOOP] ");
                break;
            case SOURCE_SENSORS:
                if (severity > debug_sensors && subsource == SOURCE_NO_SOURCE) return;
                snprintf(source_str, 13, "[SENSORS]   ");
                break;
            case SOURCE_SOAP:
                if (severity > debug_soap && subsource == SOURCE_NO_SOURCE) return;
                snprintf(source_str, 13, "[SOAP]      ");
                break;
            case SOURCE_MUX:
                if (severity > debug_mux && subsource == SOURCE_NO_SOURCE) return;
                snprintf(source_str, 13, "[MUX]       ");
                break;
            case SOURCE_DISPLAY:
                if (severity > debug_display && subsource == SOURCE_NO_SOURCE) return;
                snprintf(source_str, 13, "[DISPLAY]   ");
                break;
            case SOURCE_RTC:
                if (severity > debug_rtc && subsource == SOURCE_NO_SOURCE) return;
                snprintf(source_str, 13, "[RTC]       ");
                break;
            case SOURCE_GFX:
                if (severity > debug_gfx && subsource == SOURCE_NO_SOURCE) return;
                snprintf(source_str, 13, "[GFX]       ");
                break;
            case SOURCE_RAM:
                if (severity > debug_ram && subsource == SOURCE_NO_SOURCE) return;
                snprintf(source_str, 13, "[RAM]       ");
                break;
            case SOURCE_SERVICE_COMM:
                if (severity > debug_service_comm && subsource == SOURCE_NO_SOURCE) return;
                snprintf(source_str, 13, "[SVC_COMM]  ");
                break;
            case SOURCE_EEPROM:
                if (severity > debug_eeprom && subsource == SOURCE_NO_SOURCE) return;
                snprintf(source_str, 13, "[EEPROM]    ");
                break;
            case SOURCE_EE895:
                if (severity > debug_ee895 && subsource == SOURCE_NO_SOURCE) return;
                snprintf(source_str, 13, "[EE895]     ");
                break;
            case SOURCE_CDM7162:
                if (severity > debug_cdm7162 && subsource == SOURCE_NO_SOURCE) return;
                snprintf(source_str, 13, "[CDM7162]   ");
                break;
            case SOURCE_SUNRISE:
                if (severity > debug_sunrise && subsource == SOURCE_NO_SOURCE) return;
                snprintf(source_str, 13, "[SUNRISE]   ");
                break;
            case SOURCE_SUNLIGHT:
                if (severity > debug_sunlight && subsource == SOURCE_NO_SOURCE) return;
                snprintf(source_str, 13, "[SUNLIGHT]  ");
                break;
            case SOURCE_SCD30:
                if (severity > debug_scd30 && subsource == SOURCE_NO_SOURCE) return;
                snprintf(source_str, 13, "[SCD30]     ");
                break;
            case SOURCE_SCD41:
                if (severity > debug_scd41 && subsource == SOURCE_NO_SOURCE) return;
                snprintf(source_str, 13, "[SCD41]     ");
                break;
            case SOURCE_COZIR_LP3:
                if (severity > debug_cozir_lp3 && subsource == SOURCE_NO_SOURCE) return;
                snprintf(source_str, 13, "[CozIR-LP3] ");
                break;
            case SOURCE_CM1107N:
                if (severity > debug_cm1107n && subsource == SOURCE_NO_SOURCE) return;
                snprintf(source_str, 13, "[CM1107N]   ");
                break;
            case SOURCE_POWER:
                if (severity > debug_power && subsource == SOURCE_NO_SOURCE) return;
                snprintf(source_str, 13, "[POWER]     ");
                break;
            case SOURCE_MS5607:
                if (severity > debug_ms5607 && subsource == SOURCE_NO_SOURCE) return;
                snprintf(source_str, 13, "[MS5607]    ");
                break;
            case SOURCE_HYT271:
                if (severity > debug_hyt271 && subsource == SOURCE_NO_SOURCE) return;
                snprintf(source_str, 13, "[HYT271]    ");
                break;
            case SOURCE_WIFI:
                if (severity > debug_wifi && subsource == SOURCE_NO_SOURCE) return;
                snprintf(source_str, 13, "[WiFi]      ");
                break;
            case SOURCE_TCP_CLIENT:
                if (severity > debug_tcp_client && subsource == SOURCE_NO_SOURCE) return;
                snprintf(source_str, 13, "[TCP-Client]");
                break;
            case SOURCE_TCP_SERVER:
                if (severity > debug_tcp_server && subsource == SOURCE_NO_SOURCE) return;
                snprintf(source_str, 13, "[TCP-Server]");
                break;
            case SOURCE_TCP_DNS:
                if (severity > debug_tcp_dns && subsource == SOURCE_NO_SOURCE) return;
                snprintf(source_str, 13, "[TCP-DNS]   ");
                break;
            default:
                snprintf(source_str, 13, "[Unknown]   ");
        }
        switch(subsource)
        {
            case SOURCE_NO_SOURCE:
                snprintf(subsource_str, 13, "            ");
                break;
            case SOURCE_MAIN_INIT:
                if (severity > debug_main_init) return;
                snprintf(subsource_str, 13, "[MAIN-INIT] ");
                break;
            case SOURCE_MAIN_LOOP:
                if (severity > debug_main_loop) return;
                snprintf(subsource_str, 13, "[MAIN-LOOP] ");
                break;
            case SOURCE_SENSORS:
                if (severity > debug_sensors) return;
                snprintf(subsource_str, 13, "[SENSORS]   ");
                break;
            case SOURCE_SOAP:
                if (severity > debug_soap) return;
                snprintf(subsource_str, 13, "[SOAP]      ");
                break;
            case SOURCE_MUX:
                if (severity > debug_mux) return;
                snprintf(subsource_str, 13, "[MUX]       ");
                break;
            case SOURCE_DISPLAY:
                if (severity > debug_display) return;
                snprintf(subsource_str, 13, "[DISPLAY]   ");
                break;
            case SOURCE_RTC:
                if (severity > debug_rtc) return;
                snprintf(subsource_str, 13, "[RTC]       ");
                break;
            case SOURCE_GFX:
                if (severity > debug_gfx) return;
                snprintf(subsource_str, 13, "[GFX]       ");
                break;
            case SOURCE_RAM:
                if (severity > debug_ram) return;
                snprintf(subsource_str, 13, "[RAM]       ");
                break;
            case SOURCE_SERVICE_COMM:
                if (severity > debug_service_comm) return;
                snprintf(subsource_str, 13, "[SVC_COMM]  ");
                break;
            case SOURCE_EEPROM:
                if (severity > debug_eeprom) return;
                snprintf(subsource_str, 13, "[EEPROM]    ");
                break;
            case SOURCE_EE895:
                if (severity > debug_ee895) return;
                snprintf(subsource_str, 13, "[EE895]     ");
                break;
            case SOURCE_CDM7162:
                if (severity > debug_cdm7162) return;
                snprintf(subsource_str, 13, "[CDM7162]   ");
                break;
            case SOURCE_SUNRISE:
                if (severity > debug_sunrise) return;
                snprintf(subsource_str, 13, "[SUNRISE]   ");
                break;
            case SOURCE_SUNLIGHT:
                if (severity > debug_sunlight) return;
                snprintf(subsource_str, 13, "[SUNLIGHT]  ");
                break;
            case SOURCE_SCD30:
                if (severity > debug_scd30) return;
                snprintf(subsource_str, 13, "[SCD30]     ");
                break;
            case SOURCE_SCD41:
                if (severity > debug_scd41) return;
                snprintf(subsource_str, 13, "[SCD41]     ");
                break;
            case SOURCE_COZIR_LP3:
                if (severity > debug_cozir_lp3) return;
                snprintf(subsource_str, 13, "[CozIR-LP3] ");
                break;
            case SOURCE_CM1107N:
                if (severity > debug_cm1107n) return;
                snprintf(subsource_str, 13, "[CM1107N]   ");
                break;
            case SOURCE_POWER:
                if (severity > debug_power) return;
                snprintf(subsource_str, 13, "[POWER]     ");
                break;
            case SOURCE_MS5607:
                if (severity > debug_ms5607) return;
                snprintf(subsource_str, 13, "[MS5607]    ");
                break;
            case SOURCE_HYT271:
                if (severity > debug_hyt271) return;
                snprintf(subsource_str, 13, "[HYT271]    ");
                break;
            case SOURCE_WIFI:
                if (severity > debug_wifi) return;
                snprintf(subsource_str, 13, "[WiFi]      ");
                break;
            case SOURCE_TCP_CLIENT:
                if (severity > debug_tcp_client) return;
                snprintf(subsource_str, 13, "[TCP-Client]");
                break;
            case SOURCE_TCP_SERVER:
                if (severity > debug_tcp_server) return;
                snprintf(subsource_str, 13, "[TCP-Server]");
                break;
            case SOURCE_TCP_DNS:
                if (severity > debug_tcp_dns) return;
                snprintf(subsource_str, 13, "[TCP-DNS]   ");
                break;
            default:
                snprintf(subsource_str, 13, "[Unknown]   ");
        }
        switch (severity)
        {
            case SEVERITY_TRACE:
                snprintf(severity_str, 8, "[trace]");
                snprintf(severity_color, 11, COLOR_TRACE);
                break;
            case SEVERITY_DEBUG:
                snprintf(severity_str, 8, "[debug]");
                snprintf(severity_color, 11, COLOR_DEBUG);
                break;
            case SEVERITY_INFO:
                snprintf(severity_str, 8, "[info] ");
                snprintf(severity_color, 11, COLOR_INFO);
                break;
            case SEVERITY_WARN:
                snprintf(severity_str, 8, "[Warn] ");
                snprintf(severity_color, 11, COLOR_WARN);
                break;
            case SEVERITY_ERROR:
                snprintf(severity_str, 8, "[ERROR]");
                snprintf(severity_color, 11, COLOR_ERROR);
                break;
            case SEVERITY_FATAL:
                snprintf(severity_str, 8, "[FATAL]");
                snprintf(severity_color, 11, COLOR_FATAL);
                break;
            default:
                break;
        }

        va_list va;
        va_start(va, message);
        vsnprintf(buf, message_len, message, va);
        va_end(va);

        float time_sec = (float)(to_us_since_boot(get_absolute_time()) / 1000) / 1000.0f;
        printf("%s[%12.3f] %s %s %s %s\n"RESET"", severity_color, time_sec, severity_str, source_str, subsource_str, buf);
    }
}