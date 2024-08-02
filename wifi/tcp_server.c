#include "tcp_server.h"
#include "credentials.h"
#include "lwip/pbuf.h"
#include "lwip/dns.h"
#include "lwip/tcp.h"
#include "pico/cyw43_arch.h"
#include "../common/common_include.h"
#include "../cobs/cobslib.h"
#include "../common/shared.h"
#include "../service_comm/service_comm.h"

#define POLL_TIME_S 5

typedef struct TCP_SERVER_T_ {
    struct tcp_pcb *server_pcb;
    struct tcp_pcb *client_pcb;
    bool connected;
    uint8_t* buffer_sent;
    uint8_t* buffer_recv;
    int sent_len;
    int recv_len;
    int frame_index;
} TCP_SERVER_T;

typedef enum tcp_server_fsm {
    CONNECTION_CLOSED = 0,
    CONNECTING = 1,
    CONNECTED = 2
} tcp_server_fsm_e;

TCP_SERVER_T state;

uint8_t buffer_sent[BUF_SIZE];
uint32_t buffer_sent_len = 0;
uint8_t buffer_recv[BUF_SIZE];

uint8_t buffer_recv_frame[CONFIG_RECVD_BUFFER_SIZE];

tcp_server_fsm_e server_state;

/**
 * @brief Opens server socket on IPv4 any addr, port 10001
 * 
 * @param arg server state structure
 * @return true if successfully opened
 * @return false if error has been detected
 */
static bool tcp_server_open(void* arg);

/**
 * @brief Callback to accept client connecting to the server
 * 
 * @param arg server state structure
 * @param client_pcb pcb of the incoming connection
 * @param err error value
 * @return err_t return error code
 */
static err_t tcp_server_accept(void *arg, struct tcp_pcb *client_pcb, err_t err);

/**
 * @brief Callback after data has been sent to the client
 * 
 * @param arg server state structure
 * @param tpcb pcb structure
 * @param len length of the sent data
 * @return err_t return error code
 */
static err_t tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);

/**
 * @brief Callback when data received
 * 
 * @param arg server state structure
 * @param tpcb pcb structure
 * @param p pointer to the packet buffer
 * @param err received error code
 * @return err_t return error code
 */
err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);

/**
 * @brief Callback to the server poll
 * 
 * @param arg server state structure
 * @param tpcb pcb structure
 * @return err_t return error code
 */
static err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb);

/**
 * @brief Callback for fatal server error
 * 
 * @param arg server state structure
 * @param err error code
 */
static void tcp_server_err(void *arg, err_t err);

/**
 * @brief Result function to close server connection
 * 
 * @param arg server state structure
 * @param status exit status code
 * @return err_t return error code
 */
static err_t tcp_server_result(void* arg, int status);

/**
 * @brief Function to close the server
 * 
 * @param arg server state structure
 * @return err_t return error code
 */
static err_t tcp_server_close(void* arg);

/**
 * @brief Sends data to the client
 * 
 * @param arg server state structure
 * @return err_t return error code
 */
static err_t tcp_server_send_data(void* arg);

/**
 * @brief Asserts client disconnected and removes client pcb
 * 
 * @param arg server state structure
 * @param status Client disconnected status code
 * @return err_t return error code
 */
static err_t tcp_server_client_disconnected(void* arg, int status);

/**
 * @brief Removes client pcb & callbacks
 * 
 * @param arg Server state structure
 * @return err_t Return error code
 */
static err_t tcp_server_remove_client(void* arg);



err_t tcp_server_init()
{
    memset(&state, 0x00, sizeof(TCP_SERVER_T)); // Clear tcp server structure
    state.buffer_recv = buffer_recv; // Set buffer pointers
    state.buffer_sent = buffer_sent;
    server_state = CONNECTION_CLOSED; // Set initial connection to closed

    if (!mutex_is_initialized(&config_data.command_mutex)) // Initialize server command & response mutexes
        mutex_init(&config_data.command_mutex);
    if (!mutex_is_initialized(&config_data.response_mutex))
        mutex_init(&config_data.response_mutex);

    return ERR_OK;
}

void tcp_server_run(void)
{
    if (!&state) // No state exists
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_WIFI, SOURCE_TCP_SERVER, "TCP state not initialized");
        return;
    }
    switch (server_state) // Check server state
    {
        case CONNECTION_CLOSED: // If connection closed
            if (!tcp_server_open(&state)) // Spin up server
            {
                print_ser_output(SEVERITY_ERROR, SOURCE_WIFI, SOURCE_TCP_SERVER, "Failed to spin up TCP server.");
                server_state = CONNECTION_CLOSED; // On error set state to closed
                return;
            }
            print_ser_output(SEVERITY_INFO, SOURCE_WIFI, SOURCE_TCP_SERVER, "Listening on port %i...", TCP_SERVER_PORT);
            break;
        case CONNECTED:
        {
            if (config_data.response_rdy)
            {
                mutex_enter_timeout_ms(&config_data.response_mutex, 1000); // Safe encode copy response
                encodeCOBS(config_data.response, buffer_sent, &config_data.response_len);
                buffer_sent_len = config_data.response_len;
                mutex_exit(&config_data.response_mutex);
                tcp_server_send_data(&state); // ToDo send data
                config_data.response_rdy = false;
            }
        }
        
    }
    
}

static bool tcp_server_open(void* arg)
{
    TCP_SERVER_T* state = (TCP_SERVER_T*)arg; // Get server state
    server_state = CONNECTING; // Set state to connecting
    struct tcp_pcb* pcb = tcp_new_ip_type(IPADDR_TYPE_V4); // Set PCB to accept only IPv4
    if (!pcb) // Check pcb created
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_WIFI, SOURCE_TCP_SERVER, "Failed to create pcb");
        return false;
    }

    err_t err = tcp_bind(pcb, NULL, TCP_SERVER_PORT); // Bind pcb to port
    if (err) // Binding failed
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_WIFI, SOURCE_TCP_SERVER, "Failed to bind server to port %i: error %i", TCP_SERVER_PORT, err);
        tcp_close(pcb);
        return false;
    }

    state->server_pcb = tcp_listen_with_backlog(pcb, 1); // Listen on pcb, allow only 1 device to connect
    if (!state->server_pcb) // Check listening started
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_WIFI, SOURCE_TCP_SERVER, "Failed to listen");
        if (pcb) // If pcb exists
        {
            tcp_close(pcb); // close pcb
            sleep_ms(10);
        }
        return false;
    }

    tcp_arg(state->server_pcb, state); // set tcp arg variable
    tcp_accept(state->server_pcb, tcp_server_accept); // set tcp server accept callback fn
    return true;
}

static err_t tcp_server_accept(void *arg, struct tcp_pcb *client_pcb, err_t err)
{
    TCP_SERVER_T* state = (TCP_SERVER_T*)arg; // get server state
    if (err != ERR_OK || client_pcb == NULL) // if error or no client pcb
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_WIFI, SOURCE_TCP_SERVER, "Failed to accept client: %i", err);
        tcp_server_result(arg, err != 0 ? err : ERR_CONN); // close connection
        return ERR_VAL;
    }
    print_ser_output(SEVERITY_INFO, SOURCE_WIFI, SOURCE_TCP_SERVER, "Client connected");
    state->client_pcb = client_pcb; // set client pcb
    tcp_arg(client_pcb, state); // set client args
    tcp_sent(client_pcb, tcp_server_sent); // set server sent to client callback
    tcp_recv(client_pcb, tcp_server_recv); // set server received from client callback
    tcp_poll(client_pcb, tcp_server_poll, POLL_TIME_S * 2); // set server polling pcb callback
    tcp_err(client_pcb, tcp_server_err); // set client errored callback
    server_state = CONNECTED; // client is connected to the server
    state->connected = true;
    state->frame_index = 0;

    return ERR_OK;
}

static err_t tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len) 
{
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg; // get server state
    print_ser_output(SEVERITY_DEBUG, SOURCE_WIFI, SOURCE_TCP_SERVER, "Sent %i bytes of data", len);
    return ERR_OK;
}

err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) 
{
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg; // get server state
    if (!p) { // no pbuf available
        return tcp_server_client_disconnected(arg, 0); // close with 0
        // return ERR_OK;
    }
    // print_ser_output(SEVERITY_WARN, SOURCE_WIFI, SOURCE_TCP_SERVER, "Message incoming");
    // this method is callback from lwIP, so cyw43_arch_lwip_begin is not required, however you
    // can use this method to cause an assertion in debug mode, if this method is called when
    // cyw43_arch_lwip_begin IS needed
    cyw43_arch_lwip_check();
    state->recv_len = 0; // set received data length to 0
    config_data.response_sent = false; // Reset sent response flag
    if (p->tot_len > 0) { // check if pbuf contains data
        memset(state->buffer_recv, 0x00, BUF_SIZE); // clear received data buffer

        // Receive the buffer
        state->recv_len += pbuf_copy_partial(p, state->buffer_recv + state->recv_len,
                                            p->tot_len, 0); // copy data from pbuf

        tcp_recved(tpcb, p->tot_len); // inform client data has been received
        print_ser_output(err ? SEVERITY_ERROR : SEVERITY_DEBUG, SOURCE_WIFI, SOURCE_TCP_SERVER, "recved %d err %d", p->tot_len, err);
    }
    pbuf_free(p); // free pbuf

    for (int i = 0; i < state->recv_len; i++)
    {
        if (state->buffer_recv[i] == FRAME_DETECT) state->frame_index = 0;
        if (state->frame_index < CONFIG_RECVD_BUFFER_SIZE) buffer_recv_frame[state->frame_index++] = state->buffer_recv[i];
    }

    if (state->frame_index < CMD_RAW_MIN_LEN) // decode header
    {
        return ERR_OK;
    }
    uint32_t frame_len = state->frame_index; // Copy frame length

    if (mutex_enter_timeout_ms(&config_data.command_mutex, 1000)) // Safe decode copy command
    {
        // encodeCOBS(buffer_recv_frame, config_data.command, &frame_len);
        if (decodeCOBS(buffer_recv_frame, config_data.command, &frame_len) != 0) // Decode message
        {
            print_ser_output(SEVERITY_ERROR, SOURCE_WIFI, SOURCE_TCP_SERVER, "Failed to decode message");
            mutex_exit(&config_data.command_mutex);
            state->frame_index = 0;

            return ERR_OK;
        }
        config_data.command_len = frame_len; // Save frame length
        mutex_exit(&config_data.command_mutex);
        
        uint16_t data_len = service_comm_parse_message(); // Parse message

        if (frame_len > CMD_PADDING + CMD_MAX_LEN || data_len > CMD_MAX_LEN) // Check message too long
        {
            print_ser_output(SEVERITY_ERROR, SOURCE_WIFI, SOURCE_TCP_SERVER, "Message too long");
            state->frame_index = 0;
            return tcp_server_result(arg, ERR_ARG);
        }
        if (frame_len < CMD_PADDING + data_len) // Check for all data
        {
            print_ser_output(SEVERITY_INFO, SOURCE_WIFI, SOURCE_TCP_SERVER, "Waiting for the rest of the data");
            return ERR_OK;
        }
        config_data.err = SUCCESS; // Successfully retrieved data
        config_data.command_rdy = true; // Data has been received
        state->frame_index = 0;
        return ERR_OK;
    } 

    return ERR_OK;
}

static err_t tcp_server_client_disconnected(void* arg, int status)
{
    TCP_SERVER_T* state = (TCP_SERVER_T*)arg;
    print_ser_output(status == 0 ? SEVERITY_INFO : SEVERITY_ERROR, SOURCE_WIFI, SOURCE_TCP_SERVER, "Client disconnected: error status: %i", status);
    return tcp_server_remove_client(arg);
}

static err_t tcp_server_remove_client(void* arg)
{
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    err_t err = ERR_OK;
    if (state->client_pcb != NULL) {
        tcp_arg(state->client_pcb, NULL);
        tcp_poll(state->client_pcb, NULL, 0);
        tcp_sent(state->client_pcb, NULL);
        tcp_recv(state->client_pcb, NULL);
        tcp_err(state->client_pcb, NULL);
        err = tcp_close(state->client_pcb);
        if (err != ERR_OK) {

            print_ser_output(SEVERITY_ERROR, SOURCE_WIFI, SOURCE_TCP_SERVER, "TCP connection close failed: %i, aborting connection", err);
            tcp_abort(state->client_pcb);
            err = ERR_ABRT;
        }
        state->client_pcb = NULL;
    }
    server_state = CONNECTING;
    state->connected = false;
    return err;
}

static err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb) 
{
    print_ser_output(SEVERITY_DEBUG, SOURCE_WIFI, SOURCE_TCP_SERVER, "TCP server poll");
    return ERR_OK;
}

static void tcp_server_err(void *arg, err_t err) 
{
    if (err != ERR_ABRT) {
        print_ser_output(SEVERITY_ERROR, SOURCE_WIFI, SOURCE_TCP_SERVER, "TCP client error: %i", err);
        tcp_server_result(arg, err);
    }
}

static err_t tcp_server_result(void* arg, int status)
{
    TCP_SERVER_T* state = (TCP_SERVER_T*)arg;
    if (status != ERR_OK)
    {
        print_ser_output(status == 0 ? SEVERITY_INFO : SEVERITY_ERROR, SOURCE_WIFI, SOURCE_TCP_SERVER, "Server stopping: error status: %i", status);
    }
    return tcp_server_close(arg);
}

static err_t tcp_server_close(void* arg)
{
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    err_t err = ERR_OK;
    if (state->client_pcb != NULL) {
        tcp_arg(state->client_pcb, NULL);
        tcp_poll(state->client_pcb, NULL, 0);
        tcp_sent(state->client_pcb, NULL);
        tcp_recv(state->client_pcb, NULL);
        tcp_err(state->client_pcb, NULL);
        err = tcp_close(state->client_pcb);
        if (err != ERR_OK) {

            print_ser_output(SEVERITY_ERROR, SOURCE_WIFI, SOURCE_TCP_SERVER, "TCP connection close failed: %i, aborting connection", err);
            tcp_abort(state->client_pcb);
            err = ERR_ABRT;
        }
        state->client_pcb = NULL;
    }
    if (state->server_pcb != NULL) {
        tcp_arg(state->server_pcb, NULL);
        err = tcp_close(state->server_pcb);
        if (err != ERR_OK)
        {
            print_ser_output(SEVERITY_ERROR, SOURCE_WIFI, SOURCE_TCP_SERVER, "Server close failed: %i, aborting server", err);
            tcp_abort(state->server_pcb);
            err = ERR_ABRT;
        }
        state->server_pcb = NULL;
    }
    server_state = CONNECTION_CLOSED;
    state->connected = false;
    return err;
}

static err_t tcp_server_send_data(void* arg)
{
    TCP_SERVER_T* state = (TCP_SERVER_T*)arg;
    state->sent_len = 0;
    print_ser_output(SEVERITY_DEBUG, SOURCE_WIFI, SOURCE_TCP_SERVER, "Writing %i bytes of data", buffer_sent_len);
    err_t err = tcp_write(state->client_pcb, state->buffer_sent, buffer_sent_len, 0);
    if (err)
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_WIFI, SOURCE_TCP_SERVER, "Failed to write data over tcp: err %i", err);
        return tcp_server_client_disconnected(arg, err);
    }
    err = tcp_output(state->client_pcb); // Send data
    if (err != ERR_OK)
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_WIFI, SOURCE_TCP_CLIENT, "Failed to send data: %i", err);
        return tcp_server_client_disconnected(arg, err);
    }
    config_data.response_sent = true;
    return ERR_OK;
}






