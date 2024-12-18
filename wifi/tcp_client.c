/**
 * @file tcp_client.c
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Implementation of the TCP client
 * @version 0.1
 * @date 2024-07-11
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __TCP_CLIENT_C__
#define __TCP_CLIENT_C__

#include "credentials.h"
#include "tcp_client.h"
#include "http.h"
#include "common/shared.h"
#include "common/debug.h"

#include "lwip/pbuf.h"
#include "lwip/dns.h"
#include "lwip/tcp.h"
#include "pico/cyw43_arch.h"

#define BUF_SIZE 4096 + 1024
#define POLL_TIME_S 3

#define DUMP_BYTES(A,B)

// TCP client structure
typedef struct TCP_CLIENT_T_ {
    struct tcp_pcb *tcp_pcb;
    ip_addr_t remote_addr;
    uint8_t buffer[BUF_SIZE];
    int buffer_len;
    int sent_len;
    bool complete;
    int run_count;
    bool connected;
} TCP_CLIENT_T;

// TCP client FSM
typedef enum tcp_client_fsm {
    CONNECTION_CLOSED = 0,
    CONNECTING = 1,
    CONNECTED = 2
} tcp_client_fsm_e;

// Client state
static tcp_client_fsm_e client_state;

// TCP client
static TCP_CLIENT_T state;

// Has data been sent
static bool data_sent;

// Pointer to bool if should try to send the same message again, is set by the error handler
static bool* retry_send_message;

// Length of sent data
static uint16_t data_sent_len;

// If an error has occured - to prevent infinite loop
static bool error_flag;

// Is data being sent
static bool data_sending;

// Last message timestamp
uint8_t last_message_time[32] = {0};

uint8_t last_message_error = ERR_OK;


/**
 * @brief Opens a TCP socket
 * 
 * @param arg TCP client structure
 * @return true if socket successfully opened
 * @return false otherwise
 */
static bool tcp_client_open(void* arg);

/**
 * @brief Closes the TCP socket
 * 
 * @param arg TCP client structure
 * @return err_t Error the socked was closed with
 */
static err_t tcp_client_close(void* arg);

/**
 * @brief Callback for fatal error on TCP socket
 * 
 * @param arg TCP client structure
 * @param err Return error
 */
static void tcp_client_err(void* arg, err_t err);

/**
 * @brief Prints any TCP status and closes the socket
 * 
 * @param arg TCP client structure
 * @param status TCP socket status
 * @return err_t Closing error
 */
static err_t tcp_client_result(void* arg, int status);

/**
 * @brief Callback function when TCP socket opens and connects
 * 
 * @param arg TCP client structure
 * @param tpcb pointer to the PCB structure
 * @param err Error of the connection
 * @return err_t Return error
 */
static err_t tcp_client_connected(void* arg, struct tcp_pcb* tpcb, err_t err);

/**
 * @brief Callback for polling the application
 * 
 * @param arg TCP client structure
 * @param tpcb pointer to the PCB structure
 * @return err_t Return error
 */
static err_t tcp_client_poll(void* arg, struct tcp_pcb* tpcb);

/**
 * @brief Callback when all data has been sent by the client to the server
 * 
 * @param arg TCP client structure
 * @param tpcb pointer to the PCB structure
 * @param len Length of the sent data
 * @return err_t Return error
 */
static err_t tcp_client_sent(void* arg, struct tcp_pcb* tpcb, u16_t len);

/**
 * @brief Sends data to the server
 * 
 * @param arg TCP client structure
 * @return err_t Return error
 */
static err_t tcp_client_send(void* arg, uint8_t* data);

/**
 * @brief Callback when client receives any data
 * 
 * @param arg TCP client structure
 * @param tpcb pointer to the PCB structure
 * @param p packet buffer
 * @param err Error when receiving
 * @return err_t Return error
 */
static err_t tcp_client_recv(void* arg, struct tcp_pcb* tpcb, struct pbuf* p, err_t err);

/**
 * @brief Callback when DNS found specified host
 * 
 * @param name host name
 * @param ipaddr host IP
 * @param callback_arg NULL
 */
void tcp_client_ip_found(const char *name, const ip_addr_t *ipaddr, void *callback_arg);

bool ip_found = false;

bool run_tcp_client(uint8_t soap_index)
{
    if (!&state) // No state exists
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_WIFI, SOURCE_TCP_CLIENT, "TCP state not initialized");
        return false;
    }
    switch (client_state) // On connection state
    {
        case CONNECTION_CLOSED: // Connection closed
        {
            data_sent = false; // No data has been sent
            error_flag = false; // No error present
            if (!tcp_client_open(&state)) // try to open the connection
            {
                tcp_client_result(&state, -1); // Print error and try to close connection
                return false;
            }
            client_state = CONNECTING; // Switch state to connectiong
            data_sending = false;
            return true;
        }
        case CONNECTING: // On connecting state
            return true; // Wait for connected state
        case CONNECTED: // On connected state
        {
            err_t err;
            if (!data_sending && !data_sent)
            {
                uint8_t* message;
                switch (global_configuration.soap_mode)
                {
                    case 0x01: // soap
                    {
                        message = create_http_header(global_configuration.soap_ip, false, global_configuration.soap_path, global_configuration.soap_port, 
                            "http://tempuri.org/InsertMSxSample", soap_data[soap_index].data, soap_data[soap_index].data_len, &soap_data[soap_index].data_mutex); // Safe add HTTP header to message
                        break;
                    }
                    case 0x02: // cloud
                    {
                        message = create_http_header(global_configuration.cloud_ip, false, global_configuration.cloud_path, global_configuration.cloud_port, 
                            "http://tempuri.org/InsertMSxSample", soap_data[soap_index].data, soap_data[soap_index].data_len, &soap_data[soap_index].data_mutex); // Safe add HTTP header to message
                        break;
                    }
                    default:
                    {
                        tcp_client_result(&state, ERR_ARG); // Cannot send message -> close client
                        return false;
                    }
                }

                cyw43_arch_lwip_begin();
                data_sent = false;
                err = tcp_client_send(&state, message);
                last_message_error = err;
                if (err != ERR_OK) // Send data
                {
                    cyw43_arch_lwip_end();
                    free(message);
                    tcp_client_result(&state, err); // Print error code and close socket
                    return false;
                }
                cyw43_arch_lwip_end();
                free(message);
            }
            if (data_sent) {
                if (error_flag) // If error flag raised close socket
                {
                    tcp_client_result(&state, 2); // Print error code 2 and close socket
                    break;
                }
                tcp_client_result(&state, 0); // Close connection
            }
            else if (data_sent) data_sent = false;
            return false;
        }
    }
    return false;
}

bool tcp_client_is_running(void)
{
    return client_state ? true : false;
}

err_t tcp_client_init(bool* retry_send)
{
    retry_send_message = retry_send;
    memset(&state, 0x00, sizeof(TCP_CLIENT_T)); // Create clear structure
    if (global_configuration.soap_mode == 0x02) // data to cloud
    {
        ip_found = false;
        ip_addr_t ip_dns;
        ip4addr_aton(global_configuration.sta_dns, &ip_dns); // Convert DNS IP string to IP
        dns_init();
        dns_setserver(0, &ip_dns); // Set DNS server
        cyw43_arch_lwip_begin();
        err_t err = dns_gethostbyname(global_configuration.cloud_ip, &state.remote_addr, tcp_client_ip_found, NULL); // Get cloud IP
        if (err && err != ERR_INPROGRESS)
        {
            print_ser_output(SEVERITY_ERROR, SOURCE_WIFI, SOURCE_TCP_DNS, "Failed to retrieve IP: %i", err);
            cyw43_arch_lwip_end();
            return err;
        }
        cyw43_arch_lwip_end();
    }
    else // data to DB
    {
        ip4addr_aton(global_configuration.soap_ip, &state.remote_addr); // Assign DB IP addr
        ip_found = true;
    }
    client_state = CONNECTION_CLOSED; // Set connection state to closed
    return ERR_OK;
}

void tcp_client_ip_found(const char *name, const ip_addr_t *ipaddr, void *callback_arg)
{
    state.remote_addr = *ipaddr; // Assign IP address
    ip_found = true; // IP was found
    print_ser_output(SEVERITY_INFO, SOURCE_WIFI, SOURCE_TCP_DNS, "IP address found");
    return;
}

static bool tcp_client_open(void* arg)
{
    TCP_CLIENT_T* state = (TCP_CLIENT_T*)arg; // Client state structure
    print_ser_output(SEVERITY_DEBUG, SOURCE_WIFI, SOURCE_TCP_CLIENT, "Connecting to %s port %u...", ip4addr_ntoa(&state->remote_addr), 
        global_configuration.soap_mode == 0x02 ? global_configuration.cloud_port : global_configuration.soap_port);
    state->tcp_pcb = tcp_new_ip_type(IP_GET_TYPE(&state->remote_addr)); // Create PCB with IP address
    if (!state->tcp_pcb)
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_WIFI, SOURCE_TCP_CLIENT, "Failed to create PCB");
        return false;
    }

    tcp_arg(state->tcp_pcb, state); // Assign callback arg to TCP client structure
    tcp_poll(state->tcp_pcb, tcp_client_poll, POLL_TIME_S * 2); // Assign polling callback
    tcp_recv(state->tcp_pcb, tcp_client_recv); // Assign receive callback
    tcp_sent(state->tcp_pcb, tcp_client_sent); // Assign send callback
    tcp_err(state->tcp_pcb, tcp_client_err); // Assign error callback

    state->buffer_len = 0;

    cyw43_arch_lwip_begin();
    err_t err = tcp_connect(state->tcp_pcb, &state->remote_addr, 
        global_configuration.soap_mode == 0x02 ? global_configuration.cloud_port : global_configuration.soap_port, tcp_client_connected); // Connect to the TCP socket
    cyw43_arch_lwip_end();
    if (err) // If connection errored
    {
        tcp_client_result(arg, err);
        return false;
    }

    return true;
}

static err_t tcp_client_poll(void* arg, struct tcp_pcb* tpcb)
{
    // print_ser_output(SEVERITY_DEBUG, SOURCE_WIFI, SOURCE_TCP_CLIENT, "Client poll, closing connection");
    return ERR_OK;
    // return tcp_client_result(arg, 0);
}

static err_t tcp_client_recv(void* arg, struct tcp_pcb* tpcb, struct pbuf* p, err_t err)
{
    TCP_CLIENT_T* state = (TCP_CLIENT_T*) arg;
    if (!p) return tcp_client_result(arg, ERR_CLSD); // Check if server closed connection
    
    cyw43_arch_lwip_check();
    if (p->tot_len > 0) {
        print_ser_output(err ? SEVERITY_ERROR : SEVERITY_DEBUG, SOURCE_WIFI, SOURCE_TCP_CLIENT, "recved %d err %d", p->tot_len, err); // Check num bytes received & error
        for (struct pbuf *q = p; q != NULL; q = q->next) {
            DUMP_BYTES(q->payload, q->len);
        }
        // Receive the buffer
        memset(state->buffer, 0x00, BUF_SIZE);
        state->buffer_len = 0;
        const uint16_t buffer_left = BUF_SIZE - state->buffer_len;
        state->buffer_len += pbuf_copy_partial(p, state->buffer + state->buffer_len,
                                               p->tot_len > buffer_left ? buffer_left : p->tot_len, 0);
        tcp_recved(tpcb, p->tot_len); // Buffer received
        print_ser_output(SEVERITY_TRACE, SOURCE_WIFI, SOURCE_TCP_CLIENT, "Received:");
        if (debug >= SEVERITY_TRACE && debug_tcp_client >= SEVERITY_TRACE)
            printf("%s\n", state->buffer);

    }
    busy_wait_ms(10);
    if (p->len == p->tot_len || p->ref <= 1) // Free if references < 1 and in last part of the pbuf
    {
        pbuf_free(p);
        print_ser_output(SEVERITY_TRACE, SOURCE_WIFI, SOURCE_TCP_CLIENT, "Freed pbuf");
    }
    return ERR_OK;    
}

static err_t tcp_client_sent(void* arg, struct tcp_pcb* tpcb, u16_t len)
{
    TCP_CLIENT_T* state = (TCP_CLIENT_T*)arg;
    print_ser_output(SEVERITY_DEBUG, SOURCE_WIFI, SOURCE_TCP_CLIENT, "Message sending finished, sent %u bytes", len);
    data_sent = true; // Data sent
    data_sent_len = len; // Copy data sent length
    data_sending = false;
    *retry_send_message = false;
    return ERR_OK;
}

static err_t tcp_client_send(void* arg, uint8_t* data)
{
    TCP_CLIENT_T* state = (TCP_CLIENT_T*)arg;
    print_ser_output(SEVERITY_TRACE, SOURCE_WIFI, SOURCE_TCP_CLIENT, "Message:\n");
    if (debug >= SEVERITY_TRACE && debug_tcp_client >= SEVERITY_TRACE)
        printf("%s\n", data);
    sleep_ms(10);
    if (strlen(data) >= BUF_SIZE) // Check buffer size
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_WIFI, SOURCE_TCP_CLIENT, "Trying to send too much data");
        return ERR_ARG;
    }
    err_t err = tcp_write(state->tcp_pcb, data, strlen(data), TCP_WRITE_FLAG_COPY); // Prepare data to send
    if (err != ERR_OK)
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_WIFI, SOURCE_TCP_CLIENT, "Failed to write data to be sent: %i", err);
        return err;
    }
    err = tcp_output(state->tcp_pcb); // Send data
    if (err != ERR_OK)
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_WIFI, SOURCE_TCP_CLIENT, "Failed to send data: %i", err);
        return err;
    }
    data_sending = true;
    print_ser_output(SEVERITY_INFO, SOURCE_WIFI, SOURCE_TCP_CLIENT, "Message sent");
    memcpy(last_message_time, datetime_str, 30);
    return ERR_OK;
}

static err_t tcp_client_connected(void* arg, struct tcp_pcb* tpcb, err_t err)
{
    TCP_CLIENT_T* state = (TCP_CLIENT_T*)arg;
    if (err != ERR_OK) // Check for errors
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_WIFI, SOURCE_TCP_CLIENT, "TCP connection failed");
        return tcp_client_result(arg, err); // Close connection
    }
    client_state = CONNECTED; // state connected
    state->connected = true; 
    print_ser_output(SEVERITY_INFO, SOURCE_WIFI, SOURCE_TCP_CLIENT, "TCP connection established");
    return ERR_OK;
}

static err_t tcp_client_result(void* arg, int status)
{
    TCP_CLIENT_T* state = (TCP_CLIENT_T*)arg;
    if (status == ERR_CLSD) // Error server closed
    {
        print_ser_output(SEVERITY_WARN, SOURCE_WIFI, SOURCE_TCP_CLIENT, "Server closed connection");
    }
    else if (status != ERR_OK) // Other error
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_WIFI, SOURCE_TCP_CLIENT, "ERROR: %i", status);
    }
    return tcp_client_close(arg); // Close socket
}

static void tcp_client_err(void* arg, err_t err)
{
    TCP_CLIENT_T* state = (TCP_CLIENT_T*)arg;
    state->connected = false; // Disconnected
    data_sending = false;
    data_sent = false;
    client_state = CONNECTION_CLOSED;
    error_flag = true; // Error has occured
    *retry_send_message = true; // Should retry to send the message
    if (err != ERR_ABRT) // Connection not aborted, still fatal error
    {
        print_ser_output(SEVERITY_FATAL, SOURCE_WIFI, SOURCE_TCP_CLIENT, "TCP error: %i", err);
        tcp_client_result(arg, err); // Close connection
    }
    else print_ser_output(SEVERITY_FATAL, SOURCE_WIFI, SOURCE_TCP_CLIENT, "Connection aborted");
}

static err_t tcp_client_close(void* arg)
{
    TCP_CLIENT_T* state = (TCP_CLIENT_T*)arg;
    err_t err = ERR_OK;
    state->connected = false; // Disconnected
    if (state->tcp_pcb != NULL)
    {
        tcp_arg(state->tcp_pcb, NULL); // Set function pointers to NULL
        tcp_poll(state->tcp_pcb, NULL, 0);
        tcp_sent(state->tcp_pcb, NULL);
        tcp_recv(state->tcp_pcb, NULL);
        tcp_err(state->tcp_pcb, NULL);
        err = tcp_close(state->tcp_pcb); // Close socket
        if (err != ERR_OK) // Check for errors
        {
            print_ser_output(SEVERITY_ERROR, SOURCE_WIFI, SOURCE_TCP_CLIENT, "TCP connection close failed: %i, aborting connection", err);
            tcp_abort(state->tcp_pcb); // Abort connection
            err = ERR_ABRT;
        }
        state->tcp_pcb = NULL; // Destroy PCB
    }
    client_state = CONNECTION_CLOSED;
    return err;
}

void tcp_client_stop(void)
{
    tcp_client_close(&state);
}















#endif