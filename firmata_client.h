#ifndef RTE_KB1277_REMOTE
#define RTE_KB1277_REMOTE


#include "mFirmata.h"
#include "Rtos.h"

using u8 = unsigned char;

class firmata_client : public mFirmata {

public:
    firmata_client() = default;

    ~firmata_client() = default;

    int begin(nStream *s);

    static int get_var_bool(int index, u8 len);

    static int get_var_int(int index, short *value, u16 len);

    static int get_var_float(int index, float *value, u16 len);

    static int set_var_bool(int index, u8 value);

    static int set_var_int(int index, int value);

    static int set_var_float(int index, float value);

    void *thd{};
    static nStream *stream;

    [[noreturn]] static void thd_loop(void *arg);

    static int get_plc_var(int index);

    static int set_plc_var(int index, byte *value, int v);
};

extern firmata_client fm_client;
#endif //RTE_KB1277_REMOTE
