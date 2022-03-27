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

#define DATA_MAXSIZE 512
#ifndef INET_ADDRSTRLEN
#define INET_ADDRSTRLEN 16
#endif
#ifndef windows_x86
extern "C" const char *inet_ntop(int af, const void *src, char *dst, socklen_t cnt);
#endif
#undef write
#undef read
#undef close


class socketFirmata : public Stream
#ifdef RTE_APP
        , public smodule
#endif
{

public:
    virtual ~socketFirmata() = default;

    int begin(mFirmata *);

#ifdef RTE_APP

    int begin(u32 tick) override;

    int run(u32 tick) override;

    int diag(u32 tick) override;

#endif

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
private:
    std::vector<u8> txbuf;
    mFirmata *firm;
};

#endif
#endif