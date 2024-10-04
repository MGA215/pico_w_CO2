/**
 * @file main.c
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief PICO and peripherals control implementation for CO2 sensors measurement & data sending
 * @version 0.1
 * @date 2024-06-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "main.h"

#if FULL_BUILD
    #include "service_comm/service_comm.h"
    #include "soap/soap.h"
    #include "soap/soap_channels.h"
    #include "wifi/wifi.h"
#endif

#include "config/config.h"
#include "display/display.h"
#include "error_handler/error_handler.h"
#include "sensors/sensors.h"

#include "common/debug.h"

#include "common/constants.h"
#include "common/shared.h"
#include "error_codes.h"

#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include "pico/multicore.h"
#include "pico/mutex.h"


__attribute__((optimize("O0")))
void __attribute__((noreturn)) _exit(int status) {
    print_ser_output(SEVERITY_FATAL, SOURCE_NO_SOURCE, SOURCE_NO_SOURCE, "Runtime error has occured: %i, rebooting", status);
    sleep_ms(100);
    watchdog_enable(1, 1);
    while (true) tight_loop_contents();
}


int main()
{
    int32_t ret;
    if ((ret = init()) != 0) // init function
    {
        print_ser_output(SEVERITY_FATAL, SOURCE_MAIN_INIT, SOURCE_NO_SOURCE, "Initialization failure: %i; Aborting...", ret);
        return ret;
    } 

    while (true) {
        tight_loop_contents();
        if ((ret = loop()) != 0) // main loop
        {
            print_ser_output(SEVERITY_FATAL, SOURCE_MAIN_LOOP, SOURCE_NO_SOURCE, "Loop failure: %i; Aborting...", ret);
            return ret;
        }
    }
    return SUCCESS;
}

void core1_main(void)
{
    print_ser_output(SEVERITY_DEBUG, SOURCE_WIFI, SOURCE_NO_SOURCE, "Starting core 1...");
    error_handler_set_hardfault_core1();
#ifdef __WIFI_H__
    wifi_main();
#endif
}

int init(void)
{
    int32_t ret;

    extern char __flash_binary_start;  // defined in linker script
    extern char __flash_binary_end;    // defined in linker script
    uintptr_t start = (uintptr_t) &__flash_binary_start;
    uintptr_t end = (uintptr_t) &__flash_binary_end;
    print_ser_output(SEVERITY_FATAL, SOURCE_NO_SOURCE, SOURCE_NO_SOURCE, "Binary starts at %08x and ends at %08x, size is %08x", start, end, end-start);

    if (!stdio_init_all()) return ERROR_STDIO_INIT; // Initializing STDIO
    error_handler_set_hardfault_core0(); // Set core 0 hardfault exception handler
    
    if (watchdog_enable_caused_reboot()) print_ser_output(SEVERITY_WARN, SOURCE_MAIN_INIT, SOURCE_NO_SOURCE, "Reboot caused by watchdog");
    else print_ser_output(SEVERITY_INFO, SOURCE_MAIN_INIT, SOURCE_NO_SOURCE, "Booting clean");

    if (!mutex_is_initialized(&soap_data[0].data_mutex)) // Initializes mutexes for soap message buffer
        mutex_init(&soap_data[0].data_mutex);
    if (!mutex_is_initialized(&soap_data[1].data_mutex))
        mutex_init(&soap_data[1].data_mutex);

    svc_pin_init(); // Initialize service mode pin

    display_init(); // Initialize display

    check_svc_mode();

    multicore_launch_core1(core1_main); // Launch second core
    
    watchdog_enable(3000, true); // 3 sec watchdog

    while (service_mode == SERVICE_MODE_UART) // If in UART service mode do main loop without initialization
    {
        loop();
    }

    if (!config_read_all()) return ERROR_CONFIG_INIT; // Reading config from EEPROM
    sensors_init_all(); // initialize sensors

#if defined __SOAP_H__ && defined __SOAP_CHANNELS_H__
    soap_init(channels1); // Initialize SOAP channels
    soap_init_general(&channel00G, &hyt271.temperature, "Tamb", &hyt271.state, MEASURED_VALUE_T, 0, channels2);
    soap_init_general(&channel01G, &hyt271.humidity, "RHamb", &hyt271.state, MEASURED_VALUE_RH, 1, channels2);
    soap_init_general(&channel02G, &ms5607.pressure, "Pamb", &ms5607.state, MEASURED_VALUE_P, 2, channels2);
    soap_init_general(&channel03G, &cozir_unfiltered, "Sensor_60Raw", &sensors[6].state, MEASURED_VALUE_CO2, 3, channels2);
#endif

    print_ser_output(SEVERITY_INFO, SOURCE_MAIN_INIT, SOURCE_NO_SOURCE, "Boot time: %s", datetime_str);

    sleep_ms(10); // Init wait
    return SUCCESS;
}

int loop(void)
{
    if (!service_mode)
    {
        sensors_read_all(); // Read sensor values
        create_soap_messages(); // Create SOAP messages
    }
#ifdef __SERVICE_COMM_H__
    if (config_data.command_rdy) // Check if service command is ready
    {
        service_comm_eng_process_command(); // Process service command
    }
#endif
    update(); // Call update function as fast as possible
    check_svc_mode(); // Check for UART service mode state
    watchdog_update();
    return SUCCESS;
}

void update(void)
{
    display_update(false);
    return;
}

void create_soap_messages(void)
{
#ifdef __SOAP_H__
    if (!sensors_measurement_ready || sensors_was_measurement_read) return; // Generate new message if new data available
    if (!soap_build(global_configuration.ser_num, channels1)) // Create SOAP message
        print_ser_output(SEVERITY_ERROR, SOURCE_SOAP, SOURCE_NO_SOURCE, "Failed to generate SOAP message");
    else print_ser_output(SEVERITY_INFO, SOURCE_SOAP, SOURCE_NO_SOURCE, "Generated SOAP message");
    if (!soap_build_general(global_configuration.ser_num_aux, channels2)) // Create SOAP message
        print_ser_output(SEVERITY_ERROR, SOURCE_SOAP, SOURCE_NO_SOURCE, "Failed to generate SOAP message");
    else print_ser_output(SEVERITY_INFO, SOURCE_SOAP, SOURCE_NO_SOURCE, "Generated SOAP message");
    sensors_was_measurement_read = true;
#endif
    return;
}

void svc_pin_init(void)
{
    gpio_init(SVC_MODE); // Initialize service mode pin
    gpio_set_function(SVC_MODE, GPIO_FUNC_SIO); // Set service mode pin function to I/O
    gpio_pull_up(SVC_MODE); // Pull up
}

void check_svc_mode(void)
{
    bool svc_pin_state = gpio_get(SVC_MODE) ? true : false; // Get SVC mode pin state
    bool service_mode_active = service_mode ? true : false;
    if ((svc_pin_state == service_mode_active && service_mode != SERVICE_MODE_ETHERNET) || // If state is the same as service mode (change detected)
        (!svc_pin_state && service_mode == SERVICE_MODE_ETHERNET)) // or if in ethernet svc mode and pin is low
    {
        static uint8_t debug_severity_state = 0;
        if (!svc_pin_state) // Service mode enter
        {
            debug_severity_state = debug; // Save debug severity level
            print_ser_output(SEVERITY_WARN, SOURCE_MAIN_LOOP, SOURCE_NO_SOURCE, "Entering service mode via UART");
            debug = 0; // Disable debug
            service_mode = SERVICE_MODE_UART; // Enter service mode
        }
        else // Service mode exit
        {
            service_mode = SERVICE_MODE_DISABLED; // Exit service mode
            debug = debug_severity_state; // Restore debug level
        }
    }
    return;
}
