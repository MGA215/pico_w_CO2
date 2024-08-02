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

#include "pico/cyw43_arch.h"
#include "lwipopts.h"



static bool wifi = false;
static absolute_time_t send_data_time;
static absolute_time_t wait_dns;
static bool data_client_sending = false;

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


 // Callback function for the wifi scan
static int scan_result(void *env, const cyw43_ev_scan_result_t *result) {
    if (result && !strcmp(result->ssid, WIFI_SSID)) 
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
    if (cyw43_arch_init()) { // Initialize CYW43 WiFi driver
        print_ser_output(SEVERITY_FATAL, SOURCE_WIFI, SOURCE_NO_SOURCE, "Failed to initialize WiFi on core 1");
        return;
    }    
    print_ser_output(SEVERITY_INFO, SOURCE_WIFI, SOURCE_NO_SOURCE, "Initialized WiFi on core 1");
    cyw43_arch_enable_sta_mode(); // enable station mode

    cyw43_arch_lwip_begin();
    dhcp_release_and_stop(cyw43_state.netif);
    ip4_addr_t ip_addr;
    ip4_addr_t gw_addr;
    ip4_addr_t netmask;
    ip4addr_aton(IP_ADDR, &ip_addr);
    ip4addr_aton(GW_ADDR, &gw_addr);
    ip4addr_aton(NETMASK, &netmask);

    netif_set_addr(cyw43_state.netif, &ip_addr, &netmask, &gw_addr);
    cyw43_arch_lwip_end();
    netif_set_hostname(cyw43_state.netif, HOSTNAME);
    cyw43_wifi_pm(&cyw43_state, CYW43_NO_POWERSAVE_MODE);

    absolute_time_t wifi_wait_next_connect_time = nil_time; // Time to connect to wifi time

    while (true) // Connection attempt
    {
        
        if (!time_reached(wifi_wait_next_connect_time)) // If time to connect not reached
        {
            sleep_ms(100);
            continue;
        }
        enable_tcp_closing = wifi_send_soap_ms > 60000; // Enable TCP socket closing if sending interval > 1 min

        if (wifi_connect(100000, WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK))// Try connecting to the network
        {
            print_ser_output(SEVERITY_ERROR, SOURCE_WIFI, SOURCE_NO_SOURCE, "Failed to connect to the network, next try in %d seconds", wifi_wait_next_connect_ms);
            wifi_wait_next_connect_time = make_timeout_time_ms(wifi_wait_next_connect_ms); // Try again in wifi_wait_next_connect_ms
            continue;
        }
        wifi_wait_next_connect_time = nil_time;
        tcp_client_init(); // Initialize TCP client
        tcp_server_init(); // Initialize TCP server

        send_data_time = make_timeout_time_ms(wifi_send_soap_ms * 2); // Send data after wifi_send_data_time_ms + initial offset
        wait_dns = make_timeout_time_ms(wifi_wait_for_dns);

        while (wifi)
        {
            wifi_loop(); // Main WiFi loop
        }
        sleep_ms(10);
    }
    
    cyw43_arch_deinit(); // Deinit CYW43 driver
    return;
}

static void wifi_loop(void)
{
    if (cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) != CYW43_LINK_UP) // Check wifi connection status
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_WIFI, SOURCE_NO_SOURCE, "Connection down");
        wifi = false; // WiFi failed
        return; 
    }
            
    if (!ip_found && time_reached(wait_dns)) // Check for dns timeout
    {
        tcp_client_init();
        wait_dns = make_timeout_time_ms(wifi_wait_for_dns); // Reset timeout
    }
    if (((!service_mode && time_reached(send_data_time)) || data_client_sending) && ip_found) // If should send data
    {
        sleep_ms(5);
        print_ser_output(SEVERITY_INFO, SOURCE_WIFI, SOURCE_NO_SOURCE, "Sending data to the server...");
        send_data_time = make_timeout_time_ms(wifi_send_soap_ms); // Next message in soap_write_message_s
        data_client_sending = run_tcp_client(false); // Run TCP client FSM
    }
    tcp_server_run(); // Run TCP server
}


#endif