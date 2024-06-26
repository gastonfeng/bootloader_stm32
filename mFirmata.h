#ifndef RTE_KT1269_MFIRMATA_H
#define RTE_KT1269_MFIRMATA_H

#include <nStream.h>

#ifdef USE_KVDB

#include <flashdb.h>

#endif

#include <mem_block.h>
#include "firmata.pb.h"
#include "../../nanopb-src/firmata.pb.h"

#define firmwareVersionString "Firmata"
static const int FIRMWARE_MAJOR_VERSION = 3;
static const int FIRMWARE_MINOR_VERSION = 0;
static const int FIRMWARE_BUGFIX_VERSION = 0;

/* Version numbers for the protocol.  The protocol is still changing, so these
 * version numbers are important.
 * Query using the REPORT_VERSION message.
 */
static const int PROTOCOL_MAJOR_VERSION = 3; // for non-compatible changes
static const int PROTOCOL_MINOR_VERSION = 1; // for backwards compatible changes
static const int PROTOCOL_BUGFIX_VERSION = 1; // for bugfix releases

//    static const int MAX_DATA_BYTES = 1024; // max number of data bytes in incoming messages

// message command bytes (128-255/0x80-0xFF)
#define TOTAL_PORTS 0
#define SERIAL_MESSAGE 0x60
static const int DIGITAL_MESSAGE = 0x90; // send data for a digital port (collection of 8 pins)
static const int ANALOG_MESSAGE = 0xE0; // send data for an analog pin (or PWM)
static const int REPORT_ANALOG = 0xC0; // enable analog input by pin #
static const int REPORT_DIGITAL = 0xD0; // enable digital input by port pair
//
static const int SET_PIN_MODE = 0xF4; // set a pin to INPUT/OUTPUT/PWM/etc
static const int SET_DIGITAL_PIN_VALUE = 0xF5; // set value of an individual digital pin
//
static const int REPORT_VERSION = 0xF9; // report protocol version
static const int SYSTEM_RESET = 0xFF; // reset from MIDI
//
static const int START_SYSEX = 0xF0; // start a MIDI Sysex message
static const int END_SYSEX = 0xF7; // end a MIDI Sysex message

// extended command set using sysex (0-127/0x00-0x7F)
/* 0x00-0x0F reserved for user-defined commands */
static const int ARE_YOU_THERE = 0x51;
static const int I_AM_HERE = 0x52;
static const int SERIAL_DATA = 0x60; // communicate with serial devices, including other boards
static const int ENCODER_DATA = 0x61; // reply with encoders current positions
static const int SERVO_CONFIG = 0x70; // set max angle, minPulse, maxPulse, freq
static const int STRING_DATA = 0x71; // a string message with 14-bits per char
static const int STEPPER_DATA = 0x72; // control a stepper motor
static const int ONEWIRE_DATA = 0x73; // send an OneWire read/write/reset/select/skip/search request
static const int SHIFT_DATA = 0x75; // a bitstream to/from a shift register
static const int I2C_REQUEST = 0x76; // send an I2C read/write request
static const int I2C_REPLY = 0x77; // a reply to an I2C read request
static const int I2C_CONFIG = 0x78; // config I2C settings such as delay times and power pins
static const int REPORT_FIRMWARE = 0x79; // report name and version of the firmware
static const int EXTENDED_ANALOG = 0x6F; // analog write (PWM, Servo, etc) to any pin
static const int PIN_STATE_QUERY = 0x6D; // ask for a pin's current mode and value
static const int PIN_STATE_RESPONSE = 0x6E; // reply with pin's current mode and value
static const int CAPABILITY_QUERY = 0x6B; // ask for supported modes and resolution of all pins
static const int CAPABILITY_RESPONSE = 0x6C; // reply with supported modes and resolution
static const int ANALOG_MAPPING_QUERY = 0x69; // ask for mapping of analog to pin numbers
static const int ANALOG_MAPPING_RESPONSE = 0x6A; // reply with mapping info
static const int SAMPLING_INTERVAL = 0x7A; // set the poll rate of the main loop
static const int SCHEDULER_DATA = 0x7B;
// send a createtask/deletetask/addtotask/schedule/querytasks/querytask request to the scheduler
static const int SYSEX_NON_REALTIME = 0x7E; // MIDI Reserved for non-realtime messages
static const int SYSEX_REALTIME = 0x7F; // MIDI Reserved for realtime messages

// pin modes
static const int PIN_MODE_INPUT = 0x00; // same as INPUT defined in Arduino.h
static const int PIN_MODE_OUTPUT = 0x01; // same as OUTPUT defined in Arduino.h
static const int PIN_MODE_ANALOG = 0x02; // analog pin in analogInput mode
static const int PIN_MODE_PWM = 0x03; // digital pin in PWM output mode
static const int PIN_MODE_SERVO = 0x04; // digital pin in Servo output mode
static const int PIN_MODE_SHIFT = 0x05; // shiftIn/shiftOut mode
static const int PIN_MODE_I2C = 0x06; // pin included in I2C setup
static const int PIN_MODE_ONEWIRE = 0x07; // pin configured for 1-wire
static const int PIN_MODE_STEPPER = 0x08; // pin configured for stepper motor
static const int PIN_MODE_ENCODER = 0x09; // pin configured for rotary encoders
static const int PIN_MODE_SERIAL = 0x0A; // pin configured for serial communication
static const int PIN_MODE_PULLUP = 0x0B; // enable internal pull-up resistor for pin
static const int PIN_MODE_IGNORE = 0x7F; // pin configured to be ignored by digitalWrite and capabilityResponse

static const int TOTAL_PIN_MODES = 13;

using u16 = unsigned short;
using uint32_t = uint32_t;

struct i2c_device_info {
    byte addr;
    int reg;
    byte bytes;
    byte stopTX;
};

enum {
    IOT_LOGIN_OK = 0x55
} iot_enum;

class mFirmata {
public:
    mFirmata();

    int loop(nStream *FirmataStream);

    void sendSysex(nStream *FirmataStream, byte command, uint16_t bytec, byte *bytev, bool crc_en = true);

    void replySysex(nStream *FirmataStream, byte command, uint16_t bytec, byte *bytev);

    void sendAnalog(nStream *pStream, byte i, int i1);

    void setPinMode(byte i, int i1);

#ifndef FIRMATA_DISABLE_REPORT

    void report(nStream *FirmataStream);

#endif

    int decodeByteStream(size_t bytec, const byte *bytev, byte *buf);

    pb_object msg;

    byte sendBuffer[FIRMATA_BUFFER_SZ];

private:
    int getPinState(byte pin);

    void reportAnalogCallback(nStream *stream, byte analogPin, int value);

    /**
     * Set the pin state. The pin state of an output pin is the pin value. The state of an
     * input pin is 0, unless the pin has it's internal pull up resistor enabled, then the value is 1.
     * @param pin The pin to set the state of
     * @param state Set the state of the specified pin
     */
    void setPinState(byte pin, int state);

    uint32_t previousMillis = 0;
    // uint32_t analogInputsToReport = 0;
    int queryIndex = -1;

    void outputPort(nStream *FirmataStream, byte portNumber, byte portValue, byte forceSend);

#ifdef USE_MEMBLOCK
    mem_block *dev = nullptr;
#endif
    // 时序数据库操作
#ifdef USE_KVDB
    struct tsdb_sec_info sector
    {
    };
    uint32_t traversed_len{};
#endif

    int get_flag(u16 cmd);

    int clr_flag(u16 cmd);

    int set_flag(u16 cmd);

    u8 respose[pb_firmata_cmd_FM_LAST / 8 + 1]{};

    int getBit(nStream *pStream, int index, u8 *value_buf, u16 len);

    u8 valueBuf[8]{};
    char valueLen{};
    uint32_t last_tick{};
    bool use_sn;
    uint32_t sn;
    int blocksize;

    void marshaller_sendSysex(nStream *FirmataStream, uint8_t command, size_t bytec, uint8_t *bytev);

    void encodeByteStream(nStream *FirmataStream, size_t bytec, uint8_t *bytev, size_t max_bytes);

    int dataBufferSize = FIRMATA_BUFFER_SZ;

    void processInput(nStream *FirmataStream);

    void sendDigitalPort(nStream *FirmataStream, uint8_t portNumber, uint16_t portData);

    void parse(nStream *stream, uint8_t inputData);

    bool forward;
    bool parsingSysex;

    void processSysexMessage(nStream *stream);

    int sysexBytesRead;
    byte dataBuffer[FIRMATA_BUFFER_SZ];
    byte dataBufferDecode[FIRMATA_BUFFER_SZ];

    bool bufferDataAtPosition(nStream *stream, const uint8_t data, const size_t pos);

    int waitForData;
    uint8_t executeMultiByteCommand; // execute this after getting multi-byte data
    uint8_t multiByteChannel; // channel data for multiByteCommands

    void currentReportFirmwareCallback(nStream *pStream);

    void currentStringCallback(nStream *pStream, const char *string);

    bool allowBufferUpdate;

    void currentDataBufferOverflowCallback(nStream *pStream);

    bool getPinMode(byte i);

    void sendString(nStream *pStream, const char *string);

    virtual void sysexCallback(nStream *FirmataStream, byte command, uint16_t argc, byte *argv);

    void stringCallback(nStream *Fs, char *myString);

    void analogWriteCallback(nStream *, byte i, int val);

    bool crc_en;

protected:
    int getValue(nStream *pStream, int index, u8 *value_buf, u16 len);

    int setValue(nStream *FirmataStream, int index, void *valBuf, u8 size);

    void *lock;

public:
    static int read_rte_const(mFirmata *mf, nStream *pStream, pb_cmd cmd);

    static int read_rte_info(mFirmata *mf, nStream *pStream, pb_cmd cmd);

    static int write_rte_info(mFirmata *mf, nStream *pStream, pb_cmd cmd);

    static int read_module(mFirmata *mf, nStream *pStream, pb_cmd cmd);


    static int read_rte_ctrl(mFirmata *mf, nStream *pStream, pb_cmd cmd);

    static int write_rte_ctrl(mFirmata *mf, nStream *pStream, pb_cmd cmd);

    static int get_thread_info(mFirmata *mf, nStream *pStream, pb_cmd cmd);

    static int write_module(mFirmata *mf, nStream *pStream, pb_cmd cmd);

    static int read_rte_data(mFirmata *mf, nStream *pStream, pb_cmd cmd);


    static int read_rte_holder(mFirmata *mf, nStream *pStream, pb_cmd cmd);

    static int write_rte_data(mFirmata *mf, nStream *pStream, pb_cmd cmd);


    static int write_rte_holder(mFirmata *mf, nStream *pStream, pb_cmd cmd);

    static int goto_iap(mFirmata *mf, nStream *pStream, pb_cmd cmd);

    static int reboot(mFirmata *mf, nStream *pStream, pb_cmd cmd);

    static int goto_boot(mFirmata *mf, nStream *pStream, pb_cmd cmd);

    static int get_tsdb_info(mFirmata *mf, nStream *pStream, pb_cmd cmd);

    pb_ostream_t *SysexStream(nStream *stream, pb_firmata_cmd cmd);

    pb_ostream_t stream;
    nStream *pb_Stream;

    static bool buf_write(pb_ostream_t *stream, const pb_byte_t *buf, size_t count);

    static int get_module_info(mFirmata *mf, nStream *pStream, pb_cmd cmd);
};

#endif
