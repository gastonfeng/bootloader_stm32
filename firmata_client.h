#ifndef RTE_KB1277_REMOTE
#define RTE_KB1277_REMOTE


#include "mFirmata.h"
#include "rtos.h"

using u8 = unsigned char;

class firmata_client : public mFirmata
{

public:
    int begin(Stream *s) {
        stream = s;
        rtos::create_thread_run("fc", 512, PriorityNormal, (void *) thd_loop, s);
        return 0;
    }

    static int get_var_bool(int index, u8 *value);

    static int get_var_int(int index, short *value);

    static int get_var_float(int index, float *value);

    static int set_var_bool(int index, u8 value);

    static int set_var_int(int index, int value);

    static int set_var_float(int index, float value);

    void *thd{};
    static Stream *stream;

    [[noreturn]] static void thd_loop(void *arg);

};

extern firmata_client fm_client;
#endif //RTE_KB1277_REMOTE
