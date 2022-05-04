#ifndef RTE_KB1277_REMOTE
#define RTE_KB1277_REMOTE


#include "mFirmata.h"
#include "rtos.h"

using u8 = unsigned char;
using namespace firmata;

class firmata_client : public mFirmata {

public:
    int begin(Stream *s) {
        stream = s;
        rtos::create_thread_run("fc", 512, PriorityNormal, (void *) thd_loop, s);
        return 0;
    }

    static int get_var_bool(int index, u8 *value, u16 len);

    static int get_var_int(int index, short *value, u16 len);

    static int get_var_float(int index, float *value, u16 len);

    static int set_var_bool(int index, u8 value);

    static int set_var_int(int index, int value);

    static int set_var_float(int index, float value);

    void *thd{};
    static Stream *stream;

    [[noreturn]] static void thd_loop(void *arg);

    static int get_plc_var(int index);

    void sendSysex(byte command, uint16_t bytec, byte *bytev) {
        FirmataClass::sendSysex(stream, command, bytec, bytev);
    }

    static int set_plc_var(int index, byte *value, int);
};

extern firmata_client fm_client;
#endif //RTE_KB1277_REMOTE
