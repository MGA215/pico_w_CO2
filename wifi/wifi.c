/**
 * @file wifi.c
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief WiFi handling implementation
 * @version 0.1
 * @date 2024-07-11
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __WIFI_C__
#define __WIFI_C__

#include "wifi.h"
#include "credentials.h"
#include "tcp_client.h"
#include "tcp_server.h"
#include "common/shared.h"
#include "error_codes.h"
#include "common/debug.h"
#include "uart/uart.h"

#include "pico/cyw43_arch.h"
#include "lwipopts.h"



static bool wifi = false;
absolute_time_t send_data_time;
static absolute_time_t wait_dns;
static bool data_client_sending = false;
static uint8_t message_index;
static uint8_t message_sent_index;
static bool sending = false;
static bool retry_send_message = false;

static bool enable_tcp_closing;

extern bool ip_found;


/**
 * @brief Attempts to connect to WiFi
 * 
 * @param timeout_ms Total timeout the wifi should be tried to be joined
 * @param ssid WiFi SSID
 * @param password WiFi password
 * @param auth_mode WiFi auth mode
 * @return int Return code
 */
static int wifi_connect(int32_t timeout_ms, uint8_t* ssid, uint8_t* password, uint32_t auth_mode);

/**
 * @brief Loop
 * 
 */
static void wifi_loop(void);

/**
 * @brief Initializes wifi stuff
 * 
 */
static void wifi_init(void);


 // Callback function for the wifi scan
static int scan_result(void *env, const cyw43_ev_scan_result_t *result) {
    if (result && !strcmp(result->ssid, global_configuration.sta_ssid)) 
    {
        wifi = true;
    }
    return 0;
}

static int wifi_connect(int32_t timeout_ms, uint8_t* ssid, uint8_t* password, uint32_t auth_mode)
{
    absolute_time_t scan_time = nil_time; // Scan time
    absolute_time_t wifi_timeout = make_timeout_time_ms(timeout_ms); // Wifi timeout time
    bool scan_in_progress = false;
    print_ser_output(SEVERITY_INFO, SOURCE_WIFI, SOURCE_NO_SOURCE, "Starting WiFi scan");

    while(true) {
        if (absolute_time_diff_us(get_absolute_time(), scan_time) < 0) { // time to scan
            if (!scan_in_progress) { // Scan not running
                cyw43_wifi_scan_options_t scan_options = {0};
                int err = cyw43_wifi_scan(&cyw43_state, &scan_options, NULL, scan_result); // Start scanning
                if (err == 0) { // Scan initiated
                    print_ser_output(SEVERITY_DEBUG, SOURCE_WIFI, SOURCE_NO_SOURCE, "Performing WiFi scan");
                    scan_in_progress = true;
                } else { // Scan failed to initialize
                    print_ser_output(SEVERITY_ERROR, SOURCE_WIFI, SOURCE_NO_SOURCE, "Failed to start scan: %d\n", err);
                    scan_time = make_timeout_time_ms(10000); // wait 10s and scan again
                }
            } else if (!cyw43_wifi_scan_active(&cyw43_state) && !wifi) { // Scan inactive and wifi not found
                scan_time = make_timeout_time_ms(10000); // wait 10s and scan again
                scan_in_progress = false; 
                print_ser_output(SEVERITY_WARN, SOURCE_WIFI, SOURCE_NO_SOURCE, "WiFi network not found");
            }
            else if (wifi)
            {
                print_ser_output(SEVERITY_DEBUG, SOURCE_WIFI, SOURCE_NO_SOURCE, "Specified WiFi network found, connecting...");
                switch(auth_mode) // Ensure auth mode
                {
                    case CYW43_AUTH_OPEN:
                    case CYW43_AUTH_WPA2_AES_PSK:
                    case CYW43_AUTH_WPA2_MIXED_PSK:
                    case CYW43_AUTH_WPA_TKIP_PSK:
                        break;
                    default: auth_mode = CYW43_AUTH_OPEN;
                }
                int32_t ret;
                if ((ret = cyw43_arch_wifi_connect_timeout_ms(ssid, password, auth_mode, 30000)) != 0) // Connect to wifi
                {
                    print_ser_output(SEVERITY_ERROR, SOURCE_WIFI, SOURCE_NO_SOURCE, "Failed to connect to WiFi: %i", ret);
                    wifi = false; // Wifi disconnected
                }
                else
                {
                    print_ser_output(SEVERITY_INFO, SOURCE_WIFI, SOURCE_NO_SOURCE, "Connected to WiFi");
                    print_ser_output(SEVERITY_INFO, SOURCE_WIFI, SOURCE_NO_SOURCE, "IP addr: %s", ip4addr_ntoa(&cyw43_state.netif[0].ip_addr));
                    print_ser_output(SEVERITY_INFO, SOURCE_WIFI, SOURCE_NO_SOURCE, "GW addr: %s", ip4addr_ntoa(&cyw43_state.netif[0].gw));
                    print_ser_output(SEVERITY_INFO, SOURCE_WIFI, SOURCE_NO_SOURCE, "Mask: %s", ip4addr_ntoa(&cyw43_state.netif[0].netmask));
                    print_ser_output(SEVERITY_INFO, SOURCE_WIFI, SOURCE_NO_SOURCE, "Hostname: %s", cyw43_state.netif->hostname);

                    return SUCCESS;
                }

            }
        }
        if (time_reached(wifi_timeout)) return PICO_ERROR_CONNECT_FAILED; // Check for wifi timeout
        sleep_ms(1000);
    }
    return SUCCESS;
}

void wifi_main() 
{
    wifi_init();

    absolute_time_t wifi_wait_next_connect_time = nil_time; // Time to connect to wifi time

    uart_service_init(); // Initialize service comm UART

    while (true) // Connection attempt
    {
        if (service_mode == SERVICE_MODE_UART)
        {
            while (service_mode == SERVICE_MODE_UART)
            {
                uart_service_read_command();
                if (config_data.response_rdy) uart_service_send_response();
                tight_loop_contents();
            }
            cyw43_arch_deinit();
            wifi_init();
        }
            
        
        if (!time_reached(wifi_wait_next_connect_time)) // If time to connect not reached
        {
            sleep_ms(100);
            continue;
        }
        enable_tcp_closing = global_configuration.soap_int > 60000; // Enable TCP socket closing if sending interval > 1 min

        uint32_t auth_mode;
        switch(global_configuration.sta_security)
        {
            case 0x00: auth_mode = CYW43_AUTH_OPEN; break;
            case 0x02: auth_mode = CYW43_AUTH_WPA2_AES_PSK; break;
            default: auth_mode = CYW43_AUTH_OPEN; break;
        }

        if (wifi_connect(100000, global_configuration.sta_ssid, global_configuration.sta_password, auth_mode))// Try connecting to the network
        {
            print_ser_output(SEVERITY_ERROR, SOURCE_WIFI, SOURCE_NO_SOURCE, "Failed to connect to the network, next try in %d seconds", wifi_wait_next_connect_ms);
            wifi_wait_next_connect_time = make_timeout_time_ms(wifi_wait_next_connect_ms); // Try again in wifi_wait_next_connect_ms
            continue;
        }
        wifi_wait_next_connect_time = nil_time;
        tcp_client_init(&retry_send_message); // Initialize TCP client
        tcp_server_init(); // Initialize TCP server structs

        send_data_time = make_timeout_time_ms(global_configuration.soap_int * 4 / 3); // Send data after wifi_send_data_time_ms + initial offset
        wait_dns = make_timeout_time_ms(wifi_wait_for_dns);

        while (wifi)
        {
            if (service_mode == SERVICE_MODE_UART)
            {
                tcp_server_stop();
                tcp_client_stop();
                break;
            }
            wifi_loop(); // Main WiFi loop
        }
        sleep_ms(10);
    }
    
    cyw43_arch_deinit(); // Deinit CYW43 driver
    return;
}

static void wifi_init(void)
{
    if (cyw43_arch_init()) { // Initialize CYW43 WiFi driver
        print_ser_output(SEVERITY_FATAL, SOURCE_WIFI, SOURCE_NO_SOURCE, "Failed to initialize WiFi on core 1");
        return;
    }    
    print_ser_output(SEVERITY_INFO, SOURCE_WIFI, SOURCE_NO_SOURCE, "Initialized WiFi on core 1");
    cyw43_arch_enable_sta_mode(); // enable station mode
    if (!global_configuration.wlan_mode)
    {
        cyw43_arch_lwip_begin();
        dhcp_release_and_stop(cyw43_state.netif);
        ip4_addr_t ip_addr;
        ip4_addr_t gw_addr;
        ip4_addr_t netmask;
        ip4addr_aton(global_configuration.sta_ip, &ip_addr);
        ip4addr_aton(global_configuration.sta_gw, &gw_addr);
        ip4addr_aton(global_configuration.sta_mask, &netmask);

        netif_set_addr(cyw43_state.netif, &ip_addr, &netmask, &gw_addr);
        cyw43_arch_lwip_end();
    }
    netif_set_hostname(cyw43_state.netif, global_configuration.host_name);
    cyw43_wifi_pm(&cyw43_state, CYW43_NO_POWERSAVE_MODE);
}

static void wifi_loop(void)
{
    if (cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) != CYW43_LINK_UP) // Check wifi connection status
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_WIFI, SOURCE_NO_SOURCE, "Connection down");
        tcp_client_stop(); // Stop TCP client
        tcp_server_stop(); // Stop TCP server
        wifi = false; // WiFi failed
        return; 
    }
            
    if (!ip_found && time_reached(wait_dns)) // Check for dns timeout
    {
        tcp_client_init(&retry_send_message);
        wait_dns = make_timeout_time_ms(wifi_wait_for_dns); // Reset timeout
    }

    if (!sending) sending = time_reached(send_data_time); // Check if message should be sent
    
    if ((!service_mode && sending) && ip_found && global_configuration.soap_mode) // If should send data
    {
        sleep_ms(5);
        print_ser_output(SEVERITY_DEBUG, SOURCE_WIFI, SOURCE_NO_SOURCE, "Running TCP client...");
        if (time_reached(send_data_time))
        {
            message_index = 0;
            message_sent_index = 255;
            send_data_time = make_timeout_time_ms(global_configuration.soap_int); // Next message timeout
        }

        if ((data_client_sending || !tcp_client_is_running()) && message_sent_index != message_index) // If data should be being sent or client is not running (for initial condition) AND message with the same index was not sent
        {
            data_client_sending = run_tcp_client(message_index); // Run TCP client FSM
        }
        if (!data_client_sending) message_sent_index = message_index; // Save last message index that has been sent if already sent
        if (!data_client_sending && !tcp_client_is_running() && !retry_send_message) message_index++; // If no data being sent and client is not running and shouldn't retry message sending
        if ((global_configuration.aux_msg == 0x01 && message_index == 2) || 
            (global_configuration.aux_msg != 0x01 && message_index == 1)) sending = false; // Chech messages sent
    }
    tcp_server_run(); // Run TCP server
}

bool wifi_is_running(void)
{
    return wifi && cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_UP;
}


#endif