#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "pico/cyw43_arch.h"
#include "../common/functions.h"
#include "credentials.h"
#include "tcp_client.h"

#define BUF_SIZE 4096 + 1024
#define POLL_TIME_S 5

#define DUMP_BYTES(A,B)

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

typedef enum tcp_client_fsm {
    CONNECTION_CLOSED = 0,
    CONNECTING = 1,
    CONNECTED = 2
} tcp_client_fsm_e;

static tcp_client_fsm_e client_state;
static TCP_CLIENT_T state;
static absolute_time_t tcp_connect_time;
static uint32_t tcp_connect_timeout_ms = 10000;
static bool data_sent;
static uint16_t data_sent_len;


static bool tcp_client_open(void* arg);

static err_t tcp_client_close(void* arg);

static void tcp_client_err(void* arg, err_t err);

static err_t tcp_client_result(void* arg, int status);

static err_t tcp_client_connected(void* arg, struct tcp_pcb* tpcb, err_t err);

static err_t tcp_client_poll(void* arg, struct tcp_pcb* tpcb);

static err_t tcp_client_sent(void* arg, struct tcp_pcb* tpcb, u16_t len);

static err_t tcp_client_send(void* arg, uint8_t* data, uint16_t data_len);

static err_t tcp_client_recv(void* arg, struct tcp_pcb* tpcb, struct pbuf* p, err_t err);


void run_tcp_client(uint8_t* data, uint16_t data_len, bool close_tcp)
{
    if (!&state)
    {
        print_ser_output(SEVERITY_ERROR, "TCP client", "Failed to create TCP state");
        return;
    }
    if (client_state == CONNECTION_CLOSED) // If connection closed
    {
        if (!tcp_client_open(&state)) // try to open the connection
        {
            tcp_client_result(&state, -1); // Print error and try to close connection
            return;
        }
        client_state = CONNECTING;
        return;
    }
    if (client_state == CONNECTED)
    {
        int32_t ret;
        if ((ret = tcp_client_send(&state, data, data_len)) != ERR_OK) // Send data
        {
            if (!state.connected) client_state = CONNECTION_CLOSED;
            return;
        }
        if (close_tcp)
        {
            tcp_client_result(&state, 0);
        }
    }
}

void tcp_client_init(void)
{
    memset(&state, 0x00, sizeof(TCP_CLIENT_T)); // Create clear structure
    ip4addr_aton(TCP_SERVER_IP, &state.remote_addr); // Assign IP addr
    client_state = CLOSED; // Set connection state to closed
}

static bool tcp_client_open(void* arg)
{
    TCP_CLIENT_T* state = (TCP_CLIENT_T*)arg;
    print_ser_output(SEVERITY_DEBUG, "TCP client", "Connecting to %s port %u...", ip4addr_ntoa(&state->remote_addr), TCP_PORT);
    state->tcp_pcb = tcp_new_ip_type(IP_GET_TYPE(&state->remote_addr));
    if (!state->tcp_pcb)
    {
        print_ser_output(SEVERITY_ERROR, "TCP client", "Failed to create PCB");
        return false;
    }

    tcp_arg(state->tcp_pcb, state);
    tcp_poll(state->tcp_pcb, tcp_client_poll, POLL_TIME_S * 2);
    tcp_sent(state->tcp_pcb, tcp_client_sent);
    tcp_err(state->tcp_pcb, tcp_client_err);

    state->buffer_len = 0;

    cyw43_arch_lwip_begin();
    err_t err = tcp_connect(state->tcp_pcb, &state->remote_addr, TCP_PORT, tcp_client_connected);
    cyw43_arch_lwip_end();

    return err;
}

static err_t tcp_client_poll(void* arg, struct tcp_pcb* tpcb)
{
    print_ser_output(SEVERITY_DEBUG, "TCP client", "client poll");
    return tcp_client_result(arg, -1);
}

static err_t tcp_client_recv(void* arg, struct tcp_pcb* tpcb, struct pbuf* p, err_t err)
{
    TCP_CLIENT_T* state = (TCP_CLIENT_T*) arg;
    if (!p) return tcp_client_result(arg, -1);
    
    cyw43_arch_lwip_check();
    if (p->tot_len > 0) {
        print_ser_output(err ? SEVERITY_ERROR : SEVERITY_INFO, "TCP client", "recv %d err %d\n", p->tot_len, err);
        for (struct pbuf *q = p; q != NULL; q = q->next) {
            DUMP_BYTES(q->payload, q->len);
        }
        // Receive the buffer
        const uint16_t buffer_left = BUF_SIZE - state->buffer_len;
        state->buffer_len += pbuf_copy_partial(p, state->buffer + state->buffer_len,
                                               p->tot_len > buffer_left ? buffer_left : p->tot_len, 0);
        tcp_recved(tpcb, p->tot_len);
    }
    pbuf_free(p);
}

static err_t tcp_client_sent(void* arg, struct tcp_pcb* tpcb, u16_t len)
{
    TCP_CLIENT_T* state = (TCP_CLIENT_T*)arg;
    print_ser_output(SEVERITY_DEBUG, "TCP client", "Message sending finished");
    tcp_client_result(arg, 0);
    return ERR_OK;
}

static err_t tcp_client_send(void* arg, uint8_t* data, uint16_t data_len)
{
    TCP_CLIENT_T* state = (TCP_CLIENT_T*)arg;
    if (data_len >= BUF_SIZE) // Check buffer size
    {
        print_ser_output(SEVERITY_ERROR, "TCP client", "Trying to send too much data");
        return ERR_ARG;
    }
    err_t err = tcp_write(state->tcp_pcb, data, data_len, 0); // Prepare data to send
    if (err != ERR_OK)
    {
        print_ser_output(SEVERITY_ERROR, "TCP client", "Failed to prepare data to be sent: %i", err);
        return err;
    }
    err = tcp_output(state->tcp_pcb); // Send data
    if (err != ERR_OK)
    {
        print_ser_output(SEVERITY_ERROR, "TCP client", "Failed to send data: %i", err);
        return err;
    }
    print_ser_output(SEVERITY_INFO, "TCP client", "Message sent");
    return ERR_OK;
}

static err_t tcp_client_connected(void* arg, struct tcp_pcb* tpcb, err_t err)
{
    TCP_CLIENT_T* state = (TCP_CLIENT_T*)arg;
    if (err != ERR_OK)
    {
        print_ser_output(SEVERITY_ERROR, "TCP client", "TCP connection failed");
        return tcp_client_result(arg, err);
    }
    client_state = CONNECTED;
    state->connected = true;
    print_ser_output(SEVERITY_INFO, "TCP client", "TCP connection established");
    return ERR_OK;
}

static err_t tcp_client_result(void* arg, int status)
{
    TCP_CLIENT_T* state = (TCP_CLIENT_T*)arg;
    if (status != ERR_OK)
    {
        print_ser_output(SEVERITY_ERROR, "TCP client", "ERROR: %i", status);
    }
    return tcp_client_close(arg);
}

static void tcp_client_err(void* arg, err_t err)
{
    if (err != ERR_ABRT)
    {
        print_ser_output(SEVERITY_ERROR, "TCP client", "TCP error: %i", err);
        tcp_client_result(arg, err);
    }
    else print_ser_output(SEVERITY_ERROR, "TCP client", "Connection aborted");
}

static err_t tcp_client_close(void* arg)
{
    TCP_CLIENT_T* state = (TCP_CLIENT_T*)arg;
    err_t err = ERR_OK;
    if (state->tcp_pcb != NULL)
    {
        tcp_arg(state->tcp_pcb, NULL);
        tcp_poll(state->tcp_pcb, NULL, 0);
        tcp_sent(state->tcp_pcb, NULL);
        //tcp_recv(state->tcp_pcb, NULL);
        tcp_err(state->tcp_pcb, NULL);
        err = tcp_close(state->tcp_pcb);
        if (err != ERR_OK)
        {
            print_ser_output(SEVERITY_ERROR, "TCP client", "TCP connection close failed: %i, aborting connection", err);
            tcp_abort(state->tcp_pcb);
            err = ERR_ABRT;
        }
        state->tcp_pcb = NULL;
    }
    client_state = CONNECTION_CLOSED;
    return err;
}

















