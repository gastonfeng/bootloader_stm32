#ifndef RTE_KT1269_MFIRMATA_H
#define RTE_KT1269_MFIRMATA_H
#undef write
#undef read

#include <smodule.h>
#include "Firmata.h"
#include "../firmata/Firmata.h"

#if defined(RTE_APP) || defined(PLC)
#include "plc_rte.h"
#include <smodule.h>
#endif

using u8 = unsigned char;
using u16 = unsigned short;
using u32 = unsigned int;
struct i2c_device_info {
    byte addr;
    int reg;
    byte bytes;
    byte stopTX;
};
enum {
    CB_GET_LOG_NUMBER = 0x0,
    CB_GET_LOG,
    CB_GET_REMAIN_MEM,
    CB_GET_RTE_VERSION,
    CB_GET_BOOT_VERSION,
    CB_PLC_START,
    CB_PLC_STOP,
    CB_GET_RTC,
    CB_SET_RTC,
    CB_GET_IP,

    CB_SET_IP,
    CB_RESET,
    CB_YMODEM,
    CB_THREAD_INFO,
    CB_SET_FORCE,
    CB_SET_V,
    CB_GET_V,
    CB_SET_SERIAL_RX,
    CB_SET_SERIAL_TX_HIGH,
    CB_SET_SERIAL_TX_LOW,

    FM_GET_TASK_NRS,
    FM_GET_TASK_NAME,
    FM_GET_TASK_DETAIL,
    FM_GET_PLC_STATE,
    REPORT_PLC_MD5,
    CB_PLC_LOAD,
    CB_PLC_REPAIR,
    CB_CLEAR_V,
    CB_READ_KEY,
    CB_WRITE_KEY,

    CB_RM_KEY,
    CB_SET_TSL_RANGE,
    CB_SET_TSL_START,
    CB_SET_TSL_STATUS,
    CB_GET_TSL,
    CB_GET_TSL_COUNT,
    CB_TSL_CLEAR,
    FM_SOEM_SCAN,
    CB_SET_PLC_FILE,
    CB_CPU_USAGE,

    CB_REMOVE_FILE,
    CB_WIFI_LIST,
    CB_WIFI_SET_PASS,
    CB_GOTO_IAP,
    FM_PUT_DATA_BLOCK,
    FM_GET_LOC_SIZE,
    FM_GET_LOC,
    FM_SET_LOC,
    FM_GET_DBG_SIZE,
    FM_GET_DBG,
    FM_SET_DBG,
    FM_GET_CPU_SN,
    FM_GET_PLC_INFO,
    FM_FLASH_CLEAR,
    FM_READ_LOC,
    FM_WRITE_LOC,
    FM_LOG_SET_LEVEL,
    FM_READ_MEM,
    FM_WRITE_MEM,
    FM_READ_VALUE,
    FM_READ_VALUE_REP,
    FM_WRITE_VALUE,
    FM_WRITE_VALUE_REP,
    FM_READ_BIT,
    FM_READ_BIT_REP,
    FM_WRITE_BIT,
    FM_WRITE_BIT_REP,
    FM_GET_NET_BUF_STAT,
    FM_GET_LOCATION,
    FM_SET_LOCATION,
    FM_LIST_KEY,
    FM_READ_KEY_BYTES,
    FM_WRITE_KEY_BYTES,
    FM_FLASH_BOOT,
    FM_LAST
};

class mFirmata : public firmata::FirmataClass, public smodule {
public:
    mFirmata();

    ~mFirmata() override = default;

    int loop(Stream *FirmataStream);

    int run(u32 tick) override;

    int begin(u32 tick) override { return 0; }

    int diag(u32 tick) override {
        return 0;
    }

    int dev_test(u32 tick) override {
        return 0;
    }

    void begin(Stream *FirmataStream) {
        //
    }


    void report(Stream *FirmataStream);

    int getPinState(byte pin) override {
        return pinState[pin];
    }

    /**
     * Set the pin state. The pin state of an output pin is the pin value. The state of an
     * input pin is 0, unless the pin has it's internal pull up resistor enabled, then the value is 1.
     * @param pin The pin to set the state of
     * @param state Set the state of the specified pin
     */
    void setPinState(byte pin, int state) override {
        pinState[pin] = state;
    }

    u32 previousMillis = 0;
    // u32 analogInputsToReport = 0;
    int queryIndex = -1;

    void outputPort(Stream *FirmataStream, byte portNumber, byte portValue, byte forceSend);

#ifdef USE_MEMBLOCK
    mem_block *dev = nullptr;
#endif
    //时序数据库操作
#ifdef USE_KVDB
    struct tsdb_sec_info sector{};
    uint32_t traversed_len{};
#endif
    systemCallbackFunction i_am_here_cb;

    int setValue(Stream *FirmataStream, int index, void *valBuf, u8 size);

    int get_flag(u16 cmd);

    int clr_flag(u16 cmd);

    int set_flag(u16 cmd);

    u8 respose[FM_LAST / 8 + 1]{};

    int getValue(Stream *pStream, int index, u8 *value_buf, u16 len);

    int getBit(Stream *pStream, int index, u8 *value_buf, u16 len);

    u8 valueBuf[8]{};
    char valueLen{};
    u32 last_tick{};
};

extern mFirmata ifirmata;
#endif