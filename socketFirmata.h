#ifndef SOCKETFIRMATA_H
#define SOCKETFIRMATA_H


#include <vector>
#include <csignal>

#ifdef RTE_APP

#include "smodule.h"
#include "logger_rte.h"
#include "plc_rte.h"
#include "Rtos.h"

#endif

#include "mFirmata.h"


#ifdef USE_LWIP

#include "lwip/tcpip.h"
#include "lwip/sockets.h"

#endif


#ifdef SYLIXOS

#include "lwip/tcpip.h"
#include "lwip/sockets.h"

#define closesocket close
#endif

#ifdef ARDUINO_ARCH_STM32

#include <stm32_def.h>

#endif

#ifndef ETH_MAX_PAYLOAD
#define ETH_MAX_PAYLOAD FIRMATA_BUFFER_SZ
#endif
#ifndef INET_ADDRSTRLEN
#define INET_ADDRSTRLEN 16
#endif

#undef write
#undef read
#undef close
#undef bind
using peer_t = struct {
    int socket{};
    struct sockaddr_in addres{};

    /* The same for the receiving message. */
    char receiving_buffer[ETH_MAX_PAYLOAD]{};
    int current_receiving_byte{};
    u32 last_tick;
};

class socketFirmata : public nStream, public smodule {

public:

    const char *name() final { return "socketFirmata"; }

    int encode(pb_msg *msg, pb_ostream_t *stream) override {
        msg->msg.skfm = data;
        msg->which_msg = pb_msg_skfm_tag;
        return pb_encode(stream, pb_msg_fields, msg);
    }

    int iter(pb_field_iter_t *pS) override {
        return pb_field_iter_begin(pS, pb_skfm_info_fields, &data);
    }

    int type() override { return pb_msg_skfm_tag; }

    pb_skfm_info data;

    int begin(u32 tick) override;

    int run(u32 tick) override;

#ifdef __PLATFORMIO_BUILD_DEBUG__
    int dev_test(u32 tick) override;
#endif

    int diag(u32 tick) override;


    int write(u8 c) override;

    int available() override;

    int available_wait(int Delay) override;

    int read() override;

    int read_wait(int timeout = -1) override;

    int peek() override;

    /* Start listening socket sock. */
    int start_listen_socket(int *sock);

    void shutdown_properly(int code);

    int handle_new_connection();

    int receive_from_peer(void *peer);

    int handle_received_message();

    int loop();

    [[noreturn]] static void thread(const void *arg) {
        auto *debug = (socketFirmata *) arg;

        debug->loop();
    }

    void flush() override;

    void report();


    int tx_max_size() override {
        return ETH_MAX_PAYLOAD;
    }

private:
    std::vector<u8> txbuf;
    mFirmata firm;

    void check_socket();

    int close_client_connection(peer_t *client);

};

#endif
