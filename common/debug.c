#include "debug.h"
#include "stdarg.h"
#include "pico/stdlib.h"
#include "string.h"
#include "pico/printf.h"

#define COLORED_DEBUG true


#define DEBUG_MAIN_INIT true
#define DEBUG_MAIN_LOOP true

#define DEBUG_SENSORS true
#define DEBUG_SOAP false
#define DEBUG_RTC false
#define DEBUG_DISPLAY false
#define DEBUG_GFX false
#define DEBUG_RAM false
#define DEBUG_SERVICE_COMM true

#define DEBUG_MUX true
#define DEBUG_POWER true
#define DEBUG_EE895 true
#define DEBUG_CDM7162 true
#define DEBUG_SUNRISE true
#define DEBUG_SUNLIGHT true
#define DEBUG_SCD30 true
#define DEBUG_SCD41 true
#define DEBUG_COZIR_LP3 true
#define DEBUG_CM1107N true

#define DEBUG_TCP_SERVER true
#define DEBUG_TCP_DNS true

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
        switch(source)
        {
            case SOURCE_NO_SOURCE:
                snprintf(source_str, 13, "            ");
                break;
            case SOURCE_MAIN_INIT:
            #if !(DEBUG_MAIN_INIT && DEBUG_CORE_0)
                return;
            #endif
                snprintf(source_str, 13, "[MAIN-INIT] ");
                break;
            case SOURCE_MAIN_LOOP:
            #if !(DEBUG_MAIN_LOOP && DEBUG_CORE_0)
                return;
            #endif
                snprintf(source_str, 13, "[MAIN-LOOP] ");
                break;
            case SOURCE_SENSORS:
            #if !(DEBUG_SENSORS && DEBUG_CORE_0)
                return;
            #endif
                snprintf(source_str, 13, "[SENSORS]   ");
                break;
            case SOURCE_SOAP:
            #if !(DEBUG_SOAP && DEBUG_CORE_0)
                return;
            #endif
                snprintf(source_str, 13, "[SOAP]      ");
                break;
            case SOURCE_MUX:
            #if !(DEBUG_SENSORS && DEBUG_MUX && DEBUG_CORE_0)
                return;
            #endif
                snprintf(source_str, 13, "[MUX]       ");
                break;
            case SOURCE_DISPLAY:
            #if !(DEBUG_DISPLAY && DEBUG_CORE_0)
                return;
            #endif
                snprintf(source_str, 13, "[DISPLAY]   ");
                break;
            case SOURCE_RTC:
            #if !(DEBUG_RTC && DEBUG_CORE_0)
                return;
            #endif
                snprintf(source_str, 13, "[RTC]       ");
                break;
            case SOURCE_GFX:
            #if !(DEBUG_DISPLAY && DEBUG_GFX && DEBUG_CORE_0)
                return;
            #endif
                snprintf(source_str, 13, "[GFX]       ");
                break;
            case SOURCE_RAM:
            #if !(DEBUG_RAM && DEBUG_MAIN_LOOP && DEBUG_CORE_0)
                return;
            #endif
                snprintf(source_str, 13, "[RAM]       ");
                break;
            case SOURCE_SERVICE_COMM:
             #if !(DEBUG_SERVICE_COMM && DEBUG_MAIN_LOOP && DEBUG_CORE_0)
                return;
            #endif
                snprintf(source_str, 13, "[SVC_COMM]  ");
                break;
            case SOURCE_EE895:
            #if !(DEBUG_SENSORS && DEBUG_EE895 && DEBUG_CORE_0)
                return;
            #endif
                snprintf(source_str, 13, "[EE895]     ");
                break;
            case SOURCE_CDM7162:
            #if !(DEBUG_SENSORS && DEBUG_CDM7162 && DEBUG_CORE_0)
                return;
            #endif
                snprintf(source_str, 13, "[CDM7162]   ");
                break;
            case SOURCE_SUNRISE:
            #if !(DEBUG_SENSORS && DEBUG_SUNRISE && DEBUG_CORE_0)
                return;
            #endif
                snprintf(source_str, 13, "[SUNRISE]   ");
                break;
            case SOURCE_SUNLIGHT:
            #if !(DEBUG_SENSORS && DEBUG_SUNLIGHT && DEBUG_CORE_0)
                return;
            #endif
                snprintf(source_str, 13, "[SUNLIGHT]  ");
                break;
            case SOURCE_SCD30:
            #if !(DEBUG_SENSORS && DEBUG_SCD30 && DEBUG_CORE_0)
                return;
            #endif
                snprintf(source_str, 13, "[SCD30]     ");
                break;
            case SOURCE_SCD41:
            #if !(DEBUG_SENSORS && DEBUG_SCD41 && DEBUG_CORE_0)
                return;
            #endif
                snprintf(source_str, 13, "[SCD41]     ");
                break;
            case SOURCE_COZIR_LP3:
            #if !(DEBUG_SENSORS && DEBUG_COZIR_LP3 && DEBUG_CORE_0)
                return;
            #endif
                snprintf(source_str, 13, "[CozIR-LP3] ");
                break;
            case SOURCE_CM1107N:
            #if !(DEBUG_SENSORS && DEBUG_CM1107N && DEBUG_CORE_0)
                return;
            #endif
                snprintf(source_str, 13, "[CM1107N]   ");
                break;
            case SOURCE_POWER:
            #if !(DEBUG_SENSORS && DEBUG_POWER && DEBUG_CORE_0)
                return;
            #endif
                snprintf(source_str, 13, "[POWER]     ");
                break;
            case SOURCE_WIFI:
            #if !(DEBUG_WIFI && DEBUG_CORE_1)
                return;
            #endif
                snprintf(source_str, 13, "[WiFi]      ");
                break;
            case SOURCE_TCP_CLIENT:
            #if !(DEBUG_WIFI && DEBUG_TCP_CLIENT && DEBUG_CORE_1)
                return;
            #endif
                snprintf(source_str, 13, "[TCP-Client]");
                break;
            case SOURCE_TCP_SERVER:
            #if !(DEBUG_WIFI && DEBUG_TCP_SERVER && DEBUG_CORE_1)
                return;
            #endif
                snprintf(source_str, 13, "[TCP-Server]");
                break;
            case SOURCE_TCP_DNS:
            #if !(DEBUG_WIFI && DEBUG_TCP_DNS && DEBUG_CORE_1)
                return;
            #endif
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
            #if !(DEBUG_MAIN_INIT && DEBUG_CORE_0)
                return;
            #endif
                snprintf(subsource_str, 13, "[MAIN-INIT] ");
                break;
            case SOURCE_MAIN_LOOP:
            #if !(DEBUG_MAIN_LOOP && DEBUG_CORE_0)
                return;
            #endif
                snprintf(subsource_str, 13, "[MAIN-LOOP] ");
                break;
            case SOURCE_SENSORS:
            #if !(DEBUG_SENSORS && DEBUG_CORE_0)
                return;
            #endif
                snprintf(subsource_str, 13, "[SENSORS]   ");
                break;
            case SOURCE_SOAP:
            #if !(DEBUG_SOAP && DEBUG_CORE_0)
                return;
            #endif
                snprintf(subsource_str, 13, "[SOAP]      ");
                break;
            case SOURCE_MUX:
            #if !(DEBUG_SENSORS && DEBUG_MUX && DEBUG_CORE_0)
                return;
            #endif
                snprintf(subsource_str, 13, "[MUX]       ");
                break;
            case SOURCE_DISPLAY:
            #if !(DEBUG_DISPLAY && DEBUG_CORE_0)
                return;
            #endif
                snprintf(subsource_str, 13, "[DISPLAY]   ");
                break;
            case SOURCE_RTC:
            #if !(DEBUG_RTC && DEBUG_CORE_0)
                return;
            #endif
                snprintf(subsource_str, 13, "[RTC]       ");
                break;
            case SOURCE_GFX:
            #if !(DEBUG_DISPLAY && DEBUG_GFX && DEBUG_CORE_0)
                return;
            #endif
                snprintf(subsource_str, 13, "[GFX]       ");
                break;
            case SOURCE_RAM:
            #if !(DEBUG_RAM && DEBUG_MAIN_LOOP && DEBUG_CORE_0)
                return;
            #endif
                snprintf(subsource_str, 13, "[RAM]       ");
                break;
            case SOURCE_SERVICE_COMM:
             #if !(DEBUG_SERVICE_COMM && DEBUG_MAIN_LOOP && DEBUG_CORE_0)
                return;
            #endif
                snprintf(subsource_str, 13, "[SVC_COMM]  ");
                break;
            case SOURCE_EE895:
            #if !(DEBUG_SENSORS && DEBUG_EE895 && DEBUG_CORE_0)
                return;
            #endif
                snprintf(subsource_str, 13, "[EE895]     ");
                break;
            case SOURCE_CDM7162:
            #if !(DEBUG_SENSORS && DEBUG_CDM7162 && DEBUG_CORE_0)
                return;
            #endif
                snprintf(subsource_str, 13, "[CDM7162]   ");
                break;
            case SOURCE_SUNRISE:
            #if !(DEBUG_SENSORS && DEBUG_SUNRISE && DEBUG_CORE_0)
                return;
            #endif
                snprintf(subsource_str, 13, "[SUNRISE]   ");
                break;
            case SOURCE_SUNLIGHT:
            #if !(DEBUG_SENSORS && DEBUG_SUNLIGHT && DEBUG_CORE_0)
                return;
            #endif
                snprintf(subsource_str, 13, "[SUNLIGHT]  ");
                break;
            case SOURCE_SCD30:
            #if !(DEBUG_SENSORS && DEBUG_SCD30 && DEBUG_CORE_0)
                return;
            #endif
                snprintf(subsource_str, 13, "[SCD30]     ");
                break;
            case SOURCE_SCD41:
            #if !(DEBUG_SENSORS && DEBUG_SCD41 && DEBUG_CORE_0)
                return;
            #endif
                snprintf(subsource_str, 13, "[SCD41]     ");
                break;
            case SOURCE_COZIR_LP3:
            #if !(DEBUG_SENSORS && DEBUG_COZIR_LP3 && DEBUG_CORE_0)
                return;
            #endif
                snprintf(subsource_str, 13, "[CozIR-LP3] ");
                break;
            case SOURCE_CM1107N:
            #if !(DEBUG_SENSORS && DEBUG_CM1107N && DEBUG_CORE_0)
                return;
            #endif
                snprintf(subsource_str, 13, "[CM1107N]   ");
                break;
            case SOURCE_POWER:
            #if !(DEBUG_SENSORS && DEBUG_POWER && DEBUG_CORE_0)
                return;
            #endif
                snprintf(subsource_str, 13, "[POWER]     ");
                break;
            case SOURCE_WIFI:
            #if !(DEBUG_WIFI && DEBUG_CORE_1)
                return;
            #endif
                snprintf(subsource_str, 13, "[WiFi]      ");
                break;
            case SOURCE_TCP_CLIENT:
            #if !(DEBUG_WIFI && DEBUG_TCP_CLIENT && DEBUG_CORE_1)
                return;
            #endif
                snprintf(subsource_str, 13, "[TCP-Client]");
                break;
            case SOURCE_TCP_SERVER:
            #if !(DEBUG_WIFI && DEBUG_TCP_SERVER && DEBUG_CORE_1)
                return;
            #endif
                snprintf(subsource_str, 13, "[TCP-Server]");
                break;
            case SOURCE_TCP_DNS:
            #if !(DEBUG_WIFI && DEBUG_TCP_DNS && DEBUG_CORE_1)
                return;
            #endif
                snprintf(subsource_str, 13, "[TCP-DNS]   ");
                break;
            default:
                snprintf(subsource_str, 13, "[Unknown]   ");
        }
        
        va_list va;
        va_start(va, message);
        vsnprintf(buf, message_len, message, va);
        va_end(va);

        float time_sec = (float)(to_us_since_boot(get_absolute_time()) / 1000) / 1000.0f;
        printf("%s[%12.3f] %s %s %s %s\n"RESET"", severity_color, time_sec, severity_str, source_str, subsource_str, buf);
    }
}