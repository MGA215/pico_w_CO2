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
#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/vreg.h"
#include "hardware/clocks.h"
#include "http.h"


static bool wifi = false;
static absolute_time_t send_data_time;
static bool enable_tcp_closing;

static soap_data_t* soap_message1;
static soap_data_t* soap_message2;


/**
 * @brief Attempts to connect to WiFi
 * 
 * @param timeout_ms Total timeout the wifi should be tried to be joined
 * @param ssid WiFi SSID
 * @param password WiFi password
 * @param auth_mode WiFi auth mode
 * @return int Return code
 */
int wifi_connect(int32_t timeout_ms, uint8_t* ssid, uint8_t* password, uint32_t auth_mode);

/**
 * @brief Sends data to the server
 * 
 */
void wifi_send_data(void);

/**
 * @brief Loop
 * 
 */
void wifi_loop(void);


 // Callback function for the wifi scan
static int scan_result(void *env, const cyw43_ev_scan_result_t *result) {
    if (result && !memcmp(result->ssid, WIFI_SSID, result->ssid_len)) wifi = true;
    return 0;
}

int wifi_connect(int32_t timeout_ms, uint8_t* ssid, uint8_t* password, uint32_t auth_mode)
{
    absolute_time_t scan_time = nil_time; // Scan time
    absolute_time_t wifi_timeout = make_timeout_time_ms(timeout_ms); // Wifi timeout time
    bool scan_in_progress = false;
    print_ser_output(SEVERITY_INFO, "WiFi", "Starting WiFi scan");

    while(true) {
        if (absolute_time_diff_us(get_absolute_time(), scan_time) < 0) { // time to scan
            if (!scan_in_progress) { // Scan not running
                cyw43_wifi_scan_options_t scan_options = {0};
                int err = cyw43_wifi_scan(&cyw43_state, &scan_options, NULL, scan_result); // Start scanning
                if (err == 0) { // Scan initiated
                    print_ser_output(SEVERITY_DEBUG, "WiFi", "Performing WiFi scan");
                    scan_in_progress = true;
                } else { // Scan failed to initialize
                    print_ser_output(SEVERITY_ERROR, "WiFi", "Failed to start scan: %d\n", err);
                    scan_time = make_timeout_time_ms(10000); // wait 10s and scan again
                }
            } else if (!cyw43_wifi_scan_active(&cyw43_state) && !wifi) { // Scan inactive and wifi not found
                scan_time = make_timeout_time_ms(10000); // wait 10s and scan again
                scan_in_progress = false; 
                print_ser_output(SEVERITY_WARN, "WiFi", "WiFi network not found");
            }
            else if (wifi)
            {
                print_ser_output(SEVERITY_DEBUG, "WiFi", "Specified WiFi network found, connecting...");
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
                    print_ser_output(SEVERITY_ERROR, "WiFi", "Failed to connect to WiFi: %i", ret);
                    wifi = false; // Wifi disconnected
                }
                else
                {
                    print_ser_output(SEVERITY_INFO, "WiFi", "Connected to WiFi");
                    return SUCCESS;
                }

            }
        }
        if (time_reached(wifi_timeout)) return PICO_ERROR_CONNECT_FAILED; // Check for wifi timeout
        sleep_ms(1000);
    }
    return SUCCESS;
}

void wifi_main(soap_data_t* soap_1, soap_data_t* soap_2) 
{
    soap_message1 = soap_1; // Shared SOAP message 1 buffer
    soap_message2 = soap_2; // Shared SOAP message 2 buffer
    if (cyw43_arch_init()) { // Initialize CYW43 WiFi driver
        print_ser_output(SEVERITY_FATAL, "WiFi", "Failed to initialize WiFi on core 1");
        return;
    }
    print_ser_output(SEVERITY_INFO, "WiFi", "Initialized WiFi on core 1");
    cyw43_arch_enable_sta_mode(); // enable station mode

    tcp_client_init(); // Initialize TCP client

    while (true) // Connection attempt
    {
        absolute_time_t wifi_wait_next_connect_time = nil_time; // Time to connect to wifi time
        
        if (!time_reached(wifi_wait_next_connect_time)) // If time to connect not reached
        {
            sleep_ms(100);
            continue;
        }
        enable_tcp_closing = wifi_send_data_ms > 60000; // Enable TCP socket closing if sending interval > 1 min

        if (wifi_connect(100000, WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK))// Try connecting to the network
        {
            print_ser_output(SEVERITY_ERROR, "WiFi", "Failed to connect to the network, next try in %d seconds", wifi_wait_next_connect_ms);
            wifi_wait_next_connect_time = make_timeout_time_ms(wifi_wait_next_connect_ms); // Try again in wifi_wait_next_connect_ms
            continue;
        }

        send_data_time = make_timeout_time_ms(wifi_send_data_ms + 20000); // Send data after wifi_send_data_time_ms + initial offset
        
        while (wifi)
        {
            wifi_loop(); // Main WiFi loop
        }
    }
    
    cyw43_arch_deinit(); // Deinit CYW43 driver
    return;
}

void wifi_loop(void)
{
    if (cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA) != CYW43_LINK_JOIN) // Check wifi connection status
    {
        print_ser_output(SEVERITY_ERROR, "WiFi", "Connection down");
        wifi = false; // WiFi failed
        return; 
    }
    if (time_reached(send_data_time)) // If should send data
    {
        print_ser_output(SEVERITY_INFO, "WiFi", "Send data start");
        wifi_send_data();
        send_data_time = make_timeout_time_ms(wifi_send_data_ms); // Next message in wifi_send_data_ms
    }
}

void wifi_send_data(void)
{
    sleep_ms(100);
    uint8_t* message = create_http_header(TCP_CLIENT_SERVER_IP, false, TCP_CLIENT_SERVER_PATH, TCP_CLIENT_SERVER_PORT, 
        "http://tempuri.org/InsertMSxSample", soap_message1->data, soap_message1->data_len, &soap_message1->data_mutex); // Add safely HTTP header to SOAP message
    while (run_tcp_client(message, strlen(message), false, &soap_message1->data_mutex)) // Run TCP client FSM
    {
        tight_loop_contents();
    }
    free(message); // Dekete message
}


#endif