#include "wifi.h"
#include "credentials.h"

/**
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/vreg.h"
#include "hardware/clocks.h"

static bool wifi = false;
static absolute_time_t send_data_time;
static bool enable_tcp_closing;

static int scan_result(void *env, const cyw43_ev_scan_result_t *result) {
    // if (result) {
    //     printf("ssid: %-32s rssi: %4d chan: %3d mac: %02x:%02x:%02x:%02x:%02x:%02x sec: %u\n",
    //         result->ssid, result->rssi, result->channel,
    //         result->bssid[0], result->bssid[1], result->bssid[2], result->bssid[3], result->bssid[4], result->bssid[5],
    //         result->auth_mode);
    // }
    if (result && !memcmp(result->ssid, WIFI_SSID, result->ssid_len)) wifi = true;
    return 0;
}

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

void wifi_send_data(void);

void wifi_loop(void);





int wifi_connect(int32_t timeout_ms, uint8_t* ssid, uint8_t* password, uint32_t auth_mode)
{
    absolute_time_t scan_time = nil_time;
    absolute_time_t wifi_timeout = make_timeout_time_ms(timeout_ms);
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
                switch(auth_mode)
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
                    wifi = false;
                }
                else
                {
                    print_ser_output(SEVERITY_INFO, "WiFi", "Connected to WiFi");
                    return SUCCESS;
                }

            }
        }
        // the following #ifdef is only here so this same example can be used in multiple modes;
        // you do not need it in your code
#if PICO_CYW43_ARCH_POLL
        // if you are using pico_cyw43_arch_poll, then you must poll periodically from your
        // main loop (not from a timer) to check for Wi-Fi driver or lwIP work that needs to be done.
        cyw43_arch_poll();
        // you can poll as often as you like, however if you have nothing else to do you can
        // choose to sleep until either a specified time, or cyw43_arch_poll() has work to do:
        cyw43_arch_wait_for_work_until(scan_time);
#else
        // if you are not using pico_cyw43_arch_poll, then WiFI driver and lwIP work
        // is done via interrupt in the background. This sleep is just an example of some (blocking)
        // work you might be doing.
        if (time_reached(wifi_timeout)) return PICO_ERROR_CONNECT_FAILED;
        sleep_ms(1000);
#endif
    }
    return SUCCESS;
}

void wifi_main(void) {

    if (cyw43_arch_init()) {
        print_ser_output(SEVERITY_FATAL, "WiFi", "Failed to initialize WiFi on core 1");
        return;
    }
    print_ser_output(SEVERITY_INFO, "WiFi", "Initialized WiFi on core 1");
    cyw43_arch_enable_sta_mode(); // enable station mode

    while (true) // Connection attempt
    {
        absolute_time_t wifi_wait_next_connect_time = nil_time;
        
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
    
    cyw43_arch_deinit();
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
    if (time_reached(send_data_time))
    {
        print_ser_output(SEVERITY_INFO, "WiFi", "Send data start");
        wifi_send_data();
        send_data_time = make_timeout_time_ms(wifi_send_data_ms); // Next message in wifi_send_data_ms
    }
}

void wifi_send_data(void)
{
    // init
    // open TCP
    // free
    sleep_ms(100);
}
