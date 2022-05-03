#include "firmata_client.h"
#ifdef USE_REMOTE_WITH_FIRMATA

#if (defined(RTE_APP) || defined(PLC))

#include <remote.h>

const remote_abi remote = {
        firmata_client::get_var_bool, firmata_client::get_var_int, firmata_client::get_var_float,
        firmata_client::set_var_bool, firmata_client::set_var_int,
        firmata_client::set_var_float,
};
#else
#define NO_OBJECT (-1)
#endif

int firmata_client::get_var_bool(int index, u8 *value,u16 len) {
    int res = fm_client.getBit(stream, index, value,len);
    return res;
}

int firmata_client::get_var_int(int index, short *value,u16 len) {
    int res = fm_client.getValue(stream, index, (u8 *) value,len*2);
    return res;
}

int firmata_client::get_var_float(int index, float *value,u16 len) {
    int res = fm_client.getValue(stream, index, (u8 *) value,len);
    return res;
}


Stream *firmata_client::stream = nullptr;

int firmata_client::set_var_bool(int index, u8 value) {
    int res = fm_client.setValue(stream, index, (u8 *) &value, 1);
    return res;
}

int firmata_client::set_var_int(int index, int value) {
    int res = fm_client.setValue(stream, index, (u8 *) &value, 2);
    return res;
}

int firmata_client::set_var_float(int index, float value) {
    int res = fm_client.setValue(stream, index, (u8 *) &value, 4);
    return res;
}

void firmata_client::thd_loop(void *arg) {
    auto *fm = (Stream *) arg;
    while (true) {
        fm_client.loop(fm);
        rtos::Delay(1);
    }
}

firmata_client fm_client;
#endif
