#include "tcp_server.h"
#include "credentials.h"
#include "lwip/pbuf.h"
#include "lwip/dns.h"
#include "lwip/tcp.h"
#include "pico/cyw43_arch.h"
#include "../common/common_include.h"

#define POLL_TIME_S 5

typedef struct TCP_SERVER_T_ {
    struct tcp_pcb *server_pcb;
    struct tcp_pcb *client_pcb;
    bool complete;
    uint8_t* buffer_sent;
    uint8_t* buffer_recv;
    int sent_len;
    int recv_len;
    int run_count;
} TCP_SERVER_T;

typedef enum tcp_server_fsm {
    CONNECTION_CLOSED = 0,
    CONNECTING = 1,
    CONNECTED = 2
} tcp_server_fsm_e;

TCP_SERVER_T state;

tcp_server_fsm_e server_state;


err_t tcp_server_init()
{
    memset(&state, 0x00, sizeof(TCP_SERVER_T)); // Clear tcp server structure
    state.buffer_recv = buffer_recv; // Set buffer pointers
    state.buffer_sent = buffer_sent;
    server_state = CONNECTION_CLOSED; // Set initial connection to closed
    return ERR_OK;
}

void tcp_server_run(void)
{
    if (!&state) // No state exists
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_WIFI, SOURCE_TCP_SERVER, "TCP state not initialized");
        return false;
    }
    switch (server_state)
    {
        case CONNECTION_CLOSED:
            print_ser_output(SEVERITY_INFO, SOURCE_WIFI, SOURCE_TCP_SERVER, "Opening TCP socket on port %i...", TCP_SERVER_PORT);
            if (!tcp_server_open(&state))
            {
                print_ser_output(SEVERITY_ERROR, SOURCE_WIFI, SOURCE_TCP_SERVER, "Failed to spin up TCP server.");
                return;
            }
            break;
        
    }
}

bool tcp_server_open(void* arg)
{
    TCP_SERVER_T* state = (TCP_SERVER_T*)arg;
    server_state = CONNECTING;
    struct tcp_pcb* pcb = tcp_new_ip_type(IPADDR_TYPE_V4);
    if (!pcb)
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_WIFI, SOURCE_TCP_SERVER, "Failed to create pcb");
        return false;
    }

    err_t err = tcp_bind(pcb, NULL, TCP_SERVER_PORT);
    if (err)
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_WIFI, SOURCE_TCP_SERVER, "Failed to bind server to port %i: error %i", TCP_SERVER_PORT, err);
        return false;
    }

    state->server_pcb = tcp_listen_with_backlog(pcb, 1);
    if (!state->server_pcb)
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_WIFI, SOURCE_TCP_SERVER, "Failed to listen");
        if (pcb)
        {
            tcp_close(pcb);
        }
        return false;
    }

    tcp_arg(state->server_pcb, state);
    tcp_accept(state->server_pcb, tcp_server_accept);
    return true;
}

err_t tcp_server_accept(void *arg, struct tcp_pcb *client_pcb, err_t err)
{
    TCP_SERVER_T* state = (TCP_SERVER_T*)arg;
    if (err != ERR_OK || client_pcb == NULL)
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_WIFI, SOURCE_TCP_SERVER, "Failed to accept client: %i", err);
        tcp_server_result(arg, err != 0 ? err : ERR_CONN);
        return ERR_VAL;
    }
    print_ser_output(SEVERITY_INFO, SOURCE_WIFI, SOURCE_TCP_SERVER, "Client connected");

    state->client_pcb = client_pcb;
    tcp_arg(client_pcb, state);
    tcp_sent(client_pcb, tcp_server_sent);
    tcp_recv(client_pcb, tcp_server_recv);
    tcp_poll(client_pcb, tcp_server_poll, POLL_TIME_S * 2);
    tcp_err(client_pcb, tcp_server_err);

    return ERR_OK;
}

static err_t tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len) 
{
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    print_ser_output(SEVERITY_DEBUG, SOURCE_WIFI, SOURCE_TCP_SERVER, "Sent %i bytes of data", len);
    return ERR_OK;
}

err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) 
{
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    if (!p) {
        return tcp_server_result(arg, -1);
    }
    // this method is callback from lwIP, so cyw43_arch_lwip_begin is not required, however you
    // can use this method to cause an assertion in debug mode, if this method is called when
    // cyw43_arch_lwip_begin IS needed
    cyw43_arch_lwip_check();
    if (p->tot_len > 0) {
        state->recv_len = 0;
        memset(state->buffer_recv, 0x00, BUF_SIZE);
        print_ser_output(err ? SEVERITY_ERROR : SEVERITY_DEBUG, SOURCE_WIFI, SOURCE_TCP_SERVER, "recved %d err %d", p->tot_len, err);

        // Receive the buffer
        const uint16_t buffer_left = BUF_SIZE - state->recv_len;
        state->recv_len += pbuf_copy_partial(p, state->buffer_recv + state->recv_len,
                                             p->tot_len, 0);
        tcp_recved(tpcb, p->tot_len);
    }
    pbuf_free(p);

    return ERR_OK;
}

static err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb) 
{
    DEBUG_printf("tcp_server_poll_fn\n");
    return ERR_OK;
}

static void tcp_server_err(void *arg, err_t err) 
{
    if (err != ERR_ABRT) {
        DEBUG_printf("tcp_client_err_fn %d\n", err);
        print_ser_output(SEVERITY_ERROR, SOURCE_WIFI, SOURCE_TCP_SERVER, "TCP client error: %i", err);
        tcp_server_result(arg, err);
    }
}

static err_t tcp_server_result(void* arg, int status)
{
    TCP_SERVER_T* state = (TCP_SERVER_T*)arg;
    if (status != ERR_OK)
    {
        print_ser_output(SEVERITY_ERROR, SOURCE_WIFI, SOURCE_TCP_SERVER, "Server error status: %i", status);
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
    if (state->server_pcb) {
        tcp_arg(state->server_pcb, NULL);
        tcp_close(state->server_pcb);
        state->server_pcb = NULL;
    }
    return err;
}








