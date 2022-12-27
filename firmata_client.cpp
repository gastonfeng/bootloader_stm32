#include "firmata_client.h"

#ifdef USE_REMOTE_WITH_FIRMATA

#if (defined(RTE_APP) || defined(PLC))

#include <remote.h>

const remote_abi remote =
        {
                firmata_client::get_var_bool, firmata_client::get_var_int, firmata_client::get_var_float,
                firmata_client::set_var_bool, firmata_client::set_var_int,
                firmata_client::set_var_float,
        };
#else
#define NO_OBJECT (-1)
#endif

int firmata_client::get_var_bool(int index, u8 len) {
    byte buf[6];
    *(int *) buf = index;
    *(u16 *) &buf[4] = len;
    fm_client.sendSysex(stream, FM_READ_BIT, 6, (byte *) buf);
    return 0;
}

int firmata_client::get_var_int(int index, short *value, u16 len) {
    int res = fm_client.getValue(stream, index, (u8 *) value, len * 2);
    return res;
}

int firmata_client::get_var_float(int index, float *value, u16 len) {
    int res = fm_client.getValue(stream, index, (u8 *) value, len);
    return res;
}


nStream *firmata_client::stream = nullptr;

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
    auto *fm = (nStream *) arg;
    while (true) {
        fm_client.loop(fm);
        rtos::Delay(1);
    }
}

firmata_client fm_client;

int firmata_client::get_plc_var(int index) {
    fm_client.sendSysex(stream, FM_GET_DBG, 4, (byte *) &index);
    return 0;
}

int firmata_client::set_plc_var(int index, byte *varp, int len) {
    auto *buf = (byte *) malloc(len + 4);
    *(int *) buf = index;
    memcpy(&buf[4], varp, len);
    fm_client.sendSysex(stream, FM_SET_DBG, len + 4, buf);
    free(buf);
    return 0;
}

int firmata_client::begin(nStream *s) {
    stream = s;
#ifdef USE_FREERTOS
    rtos::create_thread_run("fc", 512, PriorityNormal, (void *) &thd_loop, s);
#endif
    return 0;
}

#endif


