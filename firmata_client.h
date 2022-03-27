#ifndef RTE_KB1277_REMOTE
#define RTE_KB1277_REMOTE


#include "mFirmata.h"

class firmata_client {

public:
    int begin(mFirmata *f, Stream *s) {
        firmata = f;
        stream = s;
        return 0;
    }

    static int get_var_bool(int index, u8 *value);

    static int get_var_int(int index, short *value);

    static int get_var_float(int index, float *value);

    static int set_var_bool(int index, u8 value);

    static int set_var_int(int index, int value);

    static int set_var_float(int index, float value);

    static mFirmata *firmata;
    static Stream *stream;
};


#endif //RTE_KB1277_REMOTE
