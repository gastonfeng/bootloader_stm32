#ifndef RTE_KT1269_MFIRMATA_H
#define RTE_KT1269_MFIRMATA_H
#undef write
#undef read

#include "Firmata.h"
#include "../firmata/Firmata.h"

#ifdef RTE_APP
#include "plc_rte.h"
#include <smodule.h>
#endif

#define I2C_WRITE B00000000
#define I2C_READ B00001000
#define I2C_READ_CONTINUOUSLY B00010000
#define I2C_STOP_READING B00011000
#define I2C_READ_WRITE_MODE_MASK B00011000
#define I2C_10BIT_ADDRESS_MODE_MASK B00100000
#define I2C_END_TX_MASK B01000000
#define I2C_STOP_TX 1
#define I2C_RESTART_TX 0
#define I2C_MAX_QUERIES 3
#define I2C_REGISTER_NOT_SPECIFIED -1
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
    CB_SET_TSL_END,
    CB_GET_TSL,
    CB_GET_TSL_COUNT,
    CB_TSL_REMOVE,
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
    FM_GET_NET_BUF_STAT,
    FM_LAST
};

class mFirmata : public firmata::FirmataClass
#ifdef RTE_APP
    , public smodule
#endif
{
public:
    mFirmata();

    ~mFirmata() override = default;

    int loop(Stream *FirmataStream);

#ifdef RTE_APP
    int run(u32 tick) override { return 0; }

    int begin(u32 tick) override { return 0; }
    int diag(u32 tick) override {
        return 0;
    }
#endif

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
    /* for i2c read continuous more */
    i2c_device_info query[I2C_MAX_QUERIES]{};

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

    int getValue(Stream *pStream, int index, u8 *value_buf);

    u8 valueBuf[8]{};
    char valueLen{};
};

extern mFirmata ifirmata;
#endif