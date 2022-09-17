#ifndef SOCKETFIRMATA_H
#define SOCKETFIRMATA_H
#if defined(USE_LWIP) || defined(windows_x86) || defined(SYLIXOS)

#include <vector>

#ifdef RTE_APP

#include "smodule.h"
#include "logger_rte.h"
#include "plc_rte.h"
#include "rtos.h"

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

#include <csignal>


#define MAX_CLIENTS 3

#define DATA_MAXSIZE FIRMATA_BUFFER_SZ
#ifndef INET_ADDRSTRLEN
#define INET_ADDRSTRLEN 16
#endif
#ifndef windows_x86
extern "C" const char *inet_ntop(int af, const void *src, char *dst, socklen_t cnt);
#endif
#undef write
#undef read
#undef close


class socketFirmata : public nStream, public smodule {

public:
    virtual ~socketFirmata() = default;

    const char *name() final { return "socketFirmata"; }

    int begin(mFirmata *);


    int begin(u32 tick) override;

    int run(u32 tick) override;

    int dev_test(u32 tick) override;

    int diag(u32 tick) override;


    size_t write(u8 c) override;

    int available() override;

    int read() override;

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

    int connect_server(const char *host, int port);

    size_t tx_max_size() override {
        return 536;
    }

private:
    std::vector<u8> txbuf;
    mFirmata *firm;
};

#endif
#endif