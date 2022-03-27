#include "firmata_client.h"

#ifdef RTE_APP

#include <remote.h>

const remote_abi remote = {
        firmata_client::get_var_bool, firmata_client::get_var_int, firmata_client::get_var_float,
        firmata_client::set_var_bool, firmata_client::set_var_int,
        firmata_client::set_var_float,
};
#else
#define NO_OBJECT -1
#endif

int firmata_client::get_var_bool(int index, u8 *value) {
    if (!firmata)return NO_OBJECT;
    int res = firmata->getValue(stream, index, value);
    return res;
}

int firmata_client::get_var_int(int index, short *value) {
    if (!firmata)return NO_OBJECT;
    int res = firmata->getValue(stream, index, (u8 *) value);
    return res;
}

int firmata_client::get_var_float(int index, float *value) {
    if (!firmata)return NO_OBJECT;
    int res = firmata->getValue(stream, index, (u8 *) value);
    return res;
}

mFirmata *firmata_client::firmata = nullptr;
Stream *firmata_client::stream = nullptr;

int firmata_client::set_var_bool(int index, u8 value) {
    if (!firmata)return NO_OBJECT;
    int res = firmata->setValue(stream, index, (u8 *) &value, 1);
    return res;
}

int firmata_client::set_var_int(int index, int value) {
    if (!firmata)return NO_OBJECT;
    int res = firmata->setValue(stream, index, (u8 *) &value, 2);
    return res;
}

int firmata_client::set_var_float(int index, float value) {
    if (!firmata)return NO_OBJECT;
    int res = firmata->setValue(stream, index, (u8 *) &value, 4);
    return res;
}
