#include "mFirmata.h"

#include <plc_rte.h>
#include "hwboard.h"
#include "SerialFirmata.h"
#include <pb_encode.h>
#include <cassert>
#include "firmata.pb.h"
#include "lib/nanopb/pb.h"
#include "lib/nanopb/pb_decode.h"


#ifdef USE_FILESYSTEM

#include "kFs.h"

#endif

#if defined(RTE_APP) || defined(PLC)

#include <iec_types.h>

#endif

#if defined(USE_RTC) || defined(USE_PCF8563)

#include "rte_rtc.h"

#endif

#include "inline_ctrl.h"
#include "rte_soem.h"

#ifdef USE_TSDB

#include <flashdb.h>

#endif
#if defined(USE_KVDB) || defined(USE_KVDB_LFS)

#include <kvdb.h>
#include <pb_common.h>

#endif
#ifdef USE_WIFI

#include <iWiFi.h>

#endif
#ifdef USE_SERVO
#include <Servo.h>
#endif

#ifdef USE_FIRMATA_WIRE
#include <Wire.h>
#endif
#ifdef FIRMATA_SERIAL_FEATURE

SerialFirmata *serialFeature;
#endif

#ifdef USE_FIRMATA_WIRE
bool isI2CEnabled;
int i2cReadDelayTime;
byte i2cRxData[8];
#endif
int queryIndex;

#ifdef USE_SERVO
Servo servos[MAX_SERVOS];
byte servoPinMap[IO_YO_NRS + IO_XI_NRS + IO_XA_NRS + IO_YA_NRS];
byte detachedServos[MAX_SERVOS];
byte detachedServoCount = 0;
byte servoCount = 0;
#endif

#ifndef ARDUINO
// from arduino
#define INPUT 0
#define OUTPUT 1
#endif

using cb_fm = int (*)(mFirmata *mf, nStream *, pb_cmd);
cb_fm fm_cmd[] = {
        &mFirmata::read_rte_const,
        &mFirmata::read_rte_info,
        &mFirmata::write_rte_info,
        &mFirmata::read_rte_data,
        &mFirmata::write_rte_data,
        &mFirmata::read_rte_holder,
        &mFirmata::write_rte_holder,
        &mFirmata::read_rte_ctrl,
        &mFirmata::write_rte_ctrl,
        mFirmata::read_module,
        mFirmata::write_module,
        mFirmata::goto_iap,
        mFirmata::reboot,
        mFirmata::goto_boot,
        mFirmata::get_tsdb_info,
        mFirmata::get_module_info,
};

#ifdef USE_FIRMATA_WIRE
void wireWrite(byte data)
{
    Wire.write((byte)data);
}

byte wireRead()
{
    return Wire.read();
}

void enableI2CPins()
{
    isI2CEnabled = true;

    Wire.begin();
}
void enableI2CPins(mFirmata *fm)
{
    // is there a faster way to do this? would probaby require importing
    // Arduino.h to get SCL and SDA pins
    for (int i = 0; i < TOTAL_PINS; i++)
    {
        if (IS_PIN_I2C(i))
        {
            // mark pins as i2c so they are ignore in non i2c data requests
            setPinModeCallback(fm, i, PIN_MODE_I2C);
        }
    }

    isI2CEnabled = true;

    Wire.begin();
}

/* disable the i2c pins so they can be used for other functions */
void disableI2CPins()
{
    isI2CEnabled = false;
    // disable read continuous mode for all devices
    queryIndex = -1;
}
#endif

void mFirmata::reportAnalogCallback(nStream *stream, byte analogPin, int value) {
#if defined(RTE_APP) || defined(PLC)
    if (analogPin < (ANALOGVALUE_LENGTH)) {
        if (0 == value) {
            bitClear(rte_data.firmata.analogInputsToReport, analogPin);
        } else {
            bitSet(rte_data.firmata.analogInputsToReport, analogPin);
            // prevent during system reset or all analog pin values will be reported
            // which may report noise for unconnected analog pins
            if (rte.data.state < pb_state_FLASH_FORMAT) {
                // Send pin value immediately. This is helpful when connected via
                // ethernet, wi-fi or bluetooth so pin states can be known upon
                // reconnecting.
                sendAnalog(stream, analogPin, rte_data.XA[analogPin]);
            }
        }
    }
#endif
}

#ifdef USE_SERVO

void attachServo(mFirmata *fm, Stream *fs, byte pin, int minPulse, int maxPulse)
{
    if (servoCount < MAX_SERVOS)
    {
        // reuse indexes of detached servos until all have been reallocated
        if (detachedServoCount > 0)
        {
            servoPinMap[pin] = detachedServos[detachedServoCount - 1];
            if (detachedServoCount > 0)
                detachedServoCount--;
        }
        else
        {
            servoPinMap[pin] = servoCount;
            servoCount++;
        }
        if (minPulse > 0 && maxPulse > 0)
        {
            servos[servoPinMap[pin]].attach(PIN_TO_DIGITAL(pin), minPulse, maxPulse);
        }
        else
        {
            servos[servoPinMap[pin]].attach(PIN_TO_DIGITAL(pin));
        }
    }
    else
    {
        sendString(fs, "Max servos attached");
    }
}

void detachServo(byte pin)
{
    servos[servoPinMap[pin]].detach();
    // if we're detaching the last servo, decrement the count
    // otherwise store the index of the detached servo
    if (servoPinMap[pin] == servoCount && servoCount > 0)
    {
        servoCount--;
    }
    else if (servoCount > 0)
    {
        // keep track of detached servos because we want to reuse their indexes
        // before incrementing the count of attached servos
        detachedServoCount++;
        detachedServos[detachedServoCount - 1] = servoPinMap[pin];
    }

    servoPinMap[pin] = 255;
}
#endif

void mFirmata::analogWriteCallback(nStream *, byte i, int val) {
#if defined(RTE_APP) || defined(PLC)
    auto v = (u16) val;
    rte_data.XA[i] = v;
#endif
}

void mFirmata::stringCallback(nStream *Fs, char *myString) {
#ifdef USE_LFS
    if (strncmp(myString, "rm ", 3) == 0) {
        if (FlashFs::unlink(&myString[3]) == 0)
            sendString(Fs, "rm ok");
        else
            sendString(Fs, "rm fail");
    }
    else
#endif
        sendString(Fs, "unknown input");
}

int fill_dbg(const void *, int index, u8 *buf);

int set_dbg(const void *, unsigned int index, byte *varp, int len);

int mFirmata::loop(nStream *FirmataStream) {
    processInput(FirmataStream);
    return 0;
}

#ifndef FIRMATA_DISABLE_REPORT

void mFirmata::report(nStream *FirmataStream) {
    uint32_t currentMillis = Rtos::ticks();

    if (currentMillis - previousMillis > rte_config.reportInterval) {
        previousMillis += rte_config.reportInterval;
        /* ANALOGREAD - do all analogReads() at the configured sampling interval */
        board.readAnalogValue(this, FirmataStream, rte_data.firmata.analogInputsToReport,
                              sizeof(rte_data.firmata.analogInputsToReport));
        for (byte pin = 0; pin < IO_XI_NRS + IO_YO_NRS; pin++) {
            if (bitRead(rte_data.firmata.reportPINs, pin)) {
                outputPort(FirmataStream, pin, getPinState(pin), true);
            }
        }
    }
#ifdef FIRMATA_SERIAL_FEATURE
    serialFeature->update(this, FirmataStream);
#endif
    // if (queryIndex > -1) {
    //     for (byte i = 0; i < queryIndex + 1; i++) {
    //         readAndReportData(Firmata, query[i].addr, query[i].reg, query[i].bytes, query[i].stopTX);
    //     }
    // }
}

#endif

void mFirmata::outputPort(nStream *FirmataStream, byte portNumber, byte portValue, byte forceSend) {
    // pins not configured as INPUT are cleared to zeros
    //    portValue = portValue & portConfigInputs[portNumber];
    // only send if the value is different than previously sent
    if (portNumber < (IO_XI_NRS + IO_YO_NRS) &&
        (forceSend || bitRead(rte_data.firmata.previousPINs, portNumber) != portValue)) {
        sendDigitalPort(FirmataStream, portNumber, portValue);
        FirmataStream->flush();
        if (portValue == 0) {
            bitClear(rte_data.firmata.previousPINs, portNumber);
        } else {
            bitSet(rte_data.firmata.previousPINs, portNumber);
        }
    }
}

#ifdef windows_x86
#include <pthread.h>
#include <minwindef.h>
#include <profileapi.h>
#include <sysinfoapi.h>
uint32_t ticks()
{
    static LARGE_INTEGER s_frequency;
    static BOOL s_use_qpc = QueryPerformanceFrequency(&s_frequency);
    if (s_use_qpc)
    {
        LARGE_INTEGER now;
        QueryPerformanceCounter(&now);
        return (1000LL * now.QuadPart) / s_frequency.QuadPart;
    }
    //    else {
    //        return GetTickCount();
    //    }
}
void Delay(uint32_t ms)
{
    struct timespec interval
    {
    };
    interval.tv_nsec = ms * 1000000;
    pthread_delay_np(&interval);
}
#endif

int mFirmata::setValue(nStream *FirmataStream, int index, void *valBuf, u8 size) {
    byte *buf;
    buf = (byte *) malloc(size + 4);
    *(int *) buf = index;
    memcpy(&buf[4], valBuf, size);
    clr_flag(pb_firmata_cmd_FM_WRITE_VALUE_REP);
    sendSysex(FirmataStream, pb_firmata_cmd_FM_WRITE_VALUE, size + 4, (byte *) buf);
    free(buf);
    uint32_t tick = Rtos::ticks() + 1000;
    while (get_flag(pb_firmata_cmd_FM_WRITE_VALUE_REP) == 0) {
        Rtos::Delay(1);
        if (Rtos::ticks() > tick)
            return pb_event_TIMEOUT;
    }
    return 0;
}

int mFirmata::get_flag(u16 cmd) {
    return respose[cmd / 8] & (1 << (cmd % 8));
}

int mFirmata::clr_flag(u16 cmd) {
    respose[cmd / 8] &= ~(1 << (cmd % 8));
    return 0;
}

int mFirmata::set_flag(u16 cmd) {
    if (cmd < pb_firmata_cmd_FM_LAST) {
        respose[cmd / 8] |= (1 << (cmd % 8));
        return 0;
    }
    return -1;
}

int mFirmata::getValue(nStream *pStream, int index, u8 *value_buf, u16 len) {
    u8 buf[6];
    *(int *) buf = index;
    *(u16 *) &buf[4] = len;
    clr_flag(pb_firmata_cmd_FM_READ_VALUE_REP);
    valueLen = 0;
    sendSysex(pStream, pb_firmata_cmd_FM_READ_VALUE, 6, (byte *) buf);
    uint32_t tick = Rtos::ticks() + 1000;
    while (get_flag(pb_firmata_cmd_FM_READ_VALUE_REP) == 0) {
        Rtos::Delay(1);
        if (Rtos::ticks() > tick)
            return pb_event_TIMEOUT;
    }
    if (valueLen == len) {
        memcpy(value_buf, valueBuf, valueLen);
        return valueLen;
    }
    return -1;
}

int mFirmata::getBit(nStream *pStream, int index, u8 *value_buf, u16 len) {
    u8 buf[6];
    *(int *) buf = index;
    *(u16 *) &buf[4] = len;
    clr_flag(pb_firmata_cmd_FM_READ_BIT_REP);
    valueLen = 0;
    sendSysex(pStream, pb_firmata_cmd_FM_READ_BIT, 6, (byte *) buf);
    uint32_t tick = Rtos::ticks() + 1000;
    while (get_flag(pb_firmata_cmd_FM_READ_BIT_REP) == 0) {
        Rtos::Delay(1);
        if (Rtos::ticks() > tick)
            return pb_event_TIMEOUT;
    }
    if (valueLen == len) {
        for (int i = 0; i < len; i++) {
            value_buf[i] = (valueBuf[i / 8] & (1 << (i % 8))) ? 1 : 0;
        }
        return valueLen;
    }
    return -1;
}

int mFirmata::getPinState(byte pin) {
    return rte_data.X[pin];
}

void mFirmata::setPinState(byte pin, int state) {
    rte_data.X[pin] = state;
}

void mFirmata::encodeByteStream(nStream *FirmataStream, size_t bytec, uint8_t *bytev, size_t max_bytes) {
    if (FirmataStream == nullptr) {
        return;
    }
    // logger.debug("encodeByteStream:%p bytec=%d,bytev=%p, max_bytes=%d", FirmataStream, bytec, bytev, max_bytes);
    static const size_t transmit_bits = 7;
    static const uint8_t transmit_mask = ((1 << transmit_bits) - 1);

    size_t bytes_sent = 0;
    size_t outstanding_bits = 0;
    uint8_t outstanding_bit_cache = *bytev;
    u16 crc = 0xFFFF;
    u8 v;
    // if ((int) bytev < 0x30000000) {
    // logger.debug("encodeByteStream:%p bytec=%d,bytev=%p, max_bytes=%d", FirmataStream, bytec, bytev, max_bytes);
    //     return;
    // }

    if (max_bytes == 0) {
        max_bytes = FIRMATA_BUFFER_SZ;
    }
    if (bytec > max_bytes) {
        // logger.error("encodeByteStream: bytec %d > max_bytes %d", bytec, max_bytes);
        return;
    }
    for (size_t i = 0; (i < bytec) && (bytes_sent < max_bytes); ++i) {
        auto transmit_byte = (uint8_t) (outstanding_bit_cache | (bytev[i] << outstanding_bits));
        v = transmit_mask & transmit_byte;
        FirmataStream->write(v);
        crc = crc_16(&v, 1, crc);
        ++bytes_sent;
        outstanding_bit_cache = (bytev[i] >> (transmit_bits - outstanding_bits));
        outstanding_bits = (outstanding_bits + (8 - transmit_bits));
        while ((outstanding_bits >= transmit_bits) && (bytes_sent < max_bytes)) {
            transmit_byte = outstanding_bit_cache;
            v = transmit_mask & transmit_byte;
            FirmataStream->write(v);
            crc = crc_16(&v, 1, crc);
            ++bytes_sent;
            outstanding_bit_cache >>= transmit_bits;
            outstanding_bits -= transmit_bits;
        }
    }
    if (outstanding_bits && (bytes_sent < max_bytes)) {
        v = (uint8_t) ((1 << outstanding_bits) - 1) & outstanding_bit_cache;
        FirmataStream->write(v);
        crc = crc_16(&v, 1, crc);
    }
    if (crc_en) {
        FirmataStream->write(crc & 0x7f);
        FirmataStream->write((crc >> 7) & 0x7f);
        FirmataStream->write((crc >> 14) & 0x7f);
        // logger.debug("%d,0x%x", bytec, crc);
    } else {
        // logger.debug("%d", bytec);
    }
}

void mFirmata::marshaller_sendSysex(nStream *FirmataStream, uint8_t command, size_t bytec, uint8_t *bytev) {
    assert(FirmataStream);
    if (bytec > FIRMATA_BUFFER_SZ) {
        //        core_debug("FirmataMarshaller byte limit");
        return;
    }
    if (use_sn)
        FirmataStream->write(0xE0);
    else
        FirmataStream->write(START_SYSEX);
    FirmataStream->write(command);
    if (bytec > 0)
        encodeByteStream(FirmataStream, bytec, bytev, FIRMATA_BUFFER_SZ);
    else {
        crc_en = false;
    }
    if (crc_en)
        FirmataStream->write(0xFA);
    else
        FirmataStream->write(END_SYSEX);
    FirmataStream->flush();
}

// uint32_t s = 0;

void mFirmata::sendSysex(nStream *FirmataStream, byte command, uint16_t bytec, byte *bytev, bool _crc_en) {
    crc_en = _crc_en || crc_en;
    if (lock)
        Rtos::mutex_lock(lock);
    if (FirmataStream->lock)
        Rtos::mutex_lock(FirmataStream->lock);
    //    if (s != 0)
    //        logger.error("sendSysex:%p c=%p", s, Rtos::pthread_self());
    //    s = Rtos::pthread_self();
    if (use_sn) {
        auto *c = (byte *) malloc(bytec + 4);
        *(uint32_t *) c = sn;
        memcpy(c + 4, bytev, bytec);
        marshaller_sendSysex(FirmataStream, command, bytec + 4, c);
        free(c);
    } else {
        marshaller_sendSysex(FirmataStream, command, bytec, bytev);
    }
    crc_en = false;
    //    s = 0;
    if (FirmataStream->lock)
        Rtos::mutex_unlock(FirmataStream->lock);
    if (lock)
        Rtos::mutex_unlock(lock);
}

void mFirmata::processInput(nStream *FirmataStream) {
#ifdef USE_FREERTOS
    int inputData = FirmataStream->read_wait(-1); // this is 'int' to handle -1 when no data
#else
    int inputData = FirmataStream->read(); // this is 'int' to handle -1 when no data
#endif
    if (inputData != -1) {
        parse(FirmataStream, (uint8_t) inputData);
    }
}

void mFirmata::sendDigitalPort(nStream *FirmataStream, uint8_t portNumber, uint16_t portData) {
    FirmataStream->write(DIGITAL_MESSAGE | (portNumber & 0xF));
    // Tx bits  0-6 (protocol v1 and higher)
    // Tx bits 7-13 (bit 7 only for protocol v2 and higher)
    encodeByteStream(FirmataStream, sizeof(portData), (uint8_t *) (&portData), sizeof(portData));
}

void mFirmata::parse(nStream *stream, uint8_t inputData) {
    uint8_t command;
#ifdef FORWARD_NRS
    if (stream->flag & FLAG_FORWARD)
    {
        stream->forward(inputData);
        if (inputData == END_SYSEX || Rtos::ticks() > stream->tick_forward)
        {
            stream->closeForward();
        }
        return;
    }
    if (forward > 0)
    {
        board.forwardMessage(forward, inputData);
        if (inputData == END_SYSEX + forward)
        {
            forward = 0;
        }
        return;
    }
#endif
    if (parsingSysex) {
        if (inputData == END_SYSEX) {
            // stop sysex byte
            parsingSysex = false;
            // fire off handler function
            processSysexMessage(stream);
            stream->flag &= ~FLAG_SYSEX;
        } else if (inputData == 0xFA) {
            // stop sysex byte
            crc_en = true;
            parsingSysex = false;
            u16 crc1 = 0, crc = 0;
            if (sysexBytesRead > 3) {
                decodeByteStream(3, &dataBuffer[sysexBytesRead - 3], (byte *) &crc1);
                crc = crc_16(dataBuffer + 1, sysexBytesRead - 4, 0xFFFF);
                sysexBytesRead -= 3;
            }
            if (crc == crc1) {
                // fire off handler function
                processSysexMessage(stream);
            } else {
                logger.error("crc error 0x%x 0x%x len=%d", crc, crc1, sysexBytesRead);
            }
            stream->flag &= ~FLAG_SYSEX;
        } else {
            // normal data byte - add to buffer
            bufferDataAtPosition(stream, inputData, sysexBytesRead);
            ++sysexBytesRead;
        }
    } else if ((waitForData > 0) && (inputData < 128)) {
        --waitForData;
        bufferDataAtPosition(stream, inputData, waitForData);
        if ((waitForData == 0) && executeMultiByteCommand) {
            // got the whole message
            switch (executeMultiByteCommand) {
#ifdef FIRMATA_FAST
                case ANALOG_MESSAGE:
                    if (currentAnalogCallback)
                    {
                        (*currentAnalogCallback)(currentAnalogCallbackContext, stream,
                                                 multiByteChannel,
                                                 (uint16_t)(dataBuffer[0] << 7) + dataBuffer[1]);
                    }
                    break;
                case DIGITAL_MESSAGE:
                    if (currentDigitalCallback)
                    {
                        (*currentDigitalCallback)(currentDigitalCallbackContext, stream,
                                                  multiByteChannel,
                                                  (uint16_t)(dataBuffer[0] << 7) + dataBuffer[1]);
                    }
                    break;
                case SET_PIN_MODE:
                    if (currentPinModeCallback)
                        (*currentPinModeCallback)(currentPinModeCallbackContext, stream, dataBuffer[1], dataBuffer[0]);
                    break;
                case SET_DIGITAL_PIN_VALUE:
                    if (currentPinValueCallback)
                        (*currentPinValueCallback)(currentPinValueCallbackContext, stream, dataBuffer[1],
                                                   dataBuffer[0]);
                    break;
                case REPORT_ANALOG:
                    if (currentReportAnalogCallback)
                        (*currentReportAnalogCallback)(currentReportAnalogCallbackContext, stream, multiByteChannel,
                                                       dataBuffer[0]);
                    break;
                case REPORT_DIGITAL:
                    if (currentReportDigitalCallback)
                        (*currentReportDigitalCallback)(currentReportDigitalCallbackContext, stream, multiByteChannel,
                                                        dataBuffer[0]);
                    break;
#endif
                default:
                    break;
            }
            executeMultiByteCommand = 0;
        }
    } else {
        // remove channel info from command byte if less than 0xF0
        if (inputData < 0xF0) {
            command = inputData & 0xF0;
            multiByteChannel = inputData & 0x0F;
        } else {
            command = inputData;
            // commands in the 0xF* range don't use channel data
        }
        switch (command) {
#ifdef FIRMATA_FAST
            case ANALOG_MESSAGE:
            case DIGITAL_MESSAGE:
            case SET_PIN_MODE:
            case SET_DIGITAL_PIN_VALUE:
                waitForData = 2; // two data bytes needed
                executeMultiByteCommand = command;
                break;
            case REPORT_ANALOG:
            case REPORT_DIGITAL:
                waitForData = 1; // one data byte needed
                executeMultiByteCommand = command;
                break;
            case SYSTEM_RESET:
                systemReset(stream);
                break;
            case REPORT_VERSION:
                if (currentReportVersionCallback)
                    (*currentReportVersionCallback)(currentReportVersionCallbackContext, stream);
                break;
#endif
            case START_SYSEX:
                use_sn = false;
                parsingSysex = true;
                sysexBytesRead = 0;
                stream->flag |= FLAG_SYSEX;
                memset(dataBuffer, 0, sizeof(dataBuffer));
                break;
            case 0xE0:
                use_sn = true;
                parsingSysex = true;
                sysexBytesRead = 0;
                stream->flag |= FLAG_SYSEX;
                memset(dataBuffer, 0, sizeof(dataBuffer));
                break;
            default:
#ifdef FORWARD_NRS
                if (command > START_SYSEX && command < START_SYSEX + FORWARD_NRS)
                {
                    forward = command - START_SYSEX;
                    board.forwardMessage(forward, 0xf0);
                }
#endif
                break;
        }
    }
}

void mFirmata::processSysexMessage(nStream *stream) {
    size_t end_of_string;
    switch (dataBuffer[0]) {
        // first byte in buffer is command
        case REPORT_FIRMWARE:
            currentReportFirmwareCallback(stream);
            break;

        case STRING_DATA:
            // const size_t end_of_string = (string_offset + decodeByteStream((sysexBytesRead - string_offset),
            //                                                                &dataBuffer[string_offset]));
            // bufferDataAtPosition('\0', end_of_string); // nullptr terminate the string
            currentStringCallback(stream, (const char *) &dataBuffer[1]);
            break;
        default:
            if (sysexBytesRead > 1) {
                memset(dataBufferDecode, 0, sizeof(dataBufferDecode));
                int len = decodeByteStream(sysexBytesRead - 1, &dataBuffer[1], dataBufferDecode);
                byte *data = dataBufferDecode;
                if (use_sn) {
                    sn = *(uint32_t *) dataBufferDecode;
                    data = dataBufferDecode + 4;
                    len -= 4;
                }
                sysexCallback(stream, dataBuffer[0], len, data);
            } else {
                if (sysexBytesRead > 0)
                    sysexBytesRead -= 1;
                sysexCallback(stream, dataBuffer[0], sysexBytesRead, nullptr);
            }
    }
}

bool mFirmata::bufferDataAtPosition(nStream *stream, const uint8_t data, const size_t pos) {
    bool bufferOverflow = (pos >= dataBufferSize);

    // Notify of overflow condition
    if (bufferOverflow) {
        allowBufferUpdate = true;
        currentDataBufferOverflowCallback(stream);
        // Check if overflow was resolved during callback
        bufferOverflow = (pos >= dataBufferSize);
    }

    // Write data to buffer if no overflow condition persist
    if (!bufferOverflow) {
        dataBuffer[pos] = data;
    }

    return bufferOverflow;
}

void mFirmata::currentReportFirmwareCallback(nStream *FirmataStream) {
    byte *buffer = (byte *) malloc(sizeof(HWMODEL) + 3);
    buffer[0] = FIRMWARE_MAJOR_VERSION;
    buffer[1] = FIRMWARE_MINOR_VERSION;
    memcpy(buffer + 2, HWMODEL, sizeof(HWMODEL));
    sendSysex(FirmataStream, REPORT_FIRMWARE, sizeof(HWMODEL) + 2, buffer);
    free(buffer);
}

int mFirmata::decodeByteStream(size_t bytec, const byte *bytev, byte *buf) {
    u8 bits = 7;
    int i = 0, j = 0;
    u8 a;
    while (i < bytec - 1) {
        a = bytev[i] >> (7 - bits);
        a |= (bytev[i + 1] << bits) & 0xff;
        buf[j++] = a;
        i += 1;
        bits -= 1;
        if (bits == 0) {
            bits = 7;
            i += 1;
        }
    }
    return j;
}

void mFirmata::currentStringCallback(nStream *pStream, const char *string) {
}

void mFirmata::currentDataBufferOverflowCallback(nStream *pStream) {
}

void mFirmata::sendAnalog(nStream *pStream, byte i, int i1) {
}

bool mFirmata::getPinMode(byte i) {
    return false;
}

void mFirmata::setPinMode(byte i, int i1) {
}

void mFirmata::sendString(nStream *pStream, const char *string) {
}

mFirmata::mFirmata() {
#ifdef FIRMATA_SERIAL_FEATURE
    serialFeature = new SerialFirmata();
#endif
    lock = Rtos::Create_Mutex();
}

void mFirmata::sysexCallback(nStream *FirmataStream, byte command, uint16_t argc, byte *argv) {
    int index;
    int state;
    uint32_t start, end;
    uint32_t indexv;
    short len;
    u8 *data;
    size_t key_len;
    byte *buffer;
    struct {
        uint32_t build;
        char name[8];
    } info{};
    // logger.debug("sysexCallback: %d argc=%d,argv=%p", command, argc, argv);
    switch (command) {
        case pb_firmata_cmd_FM_PROTOBUF: {
            pb_cmd cmd;
            pb_istream_t stream = pb_istream_from_buffer(argv, argc);
            if (pb_decode(&stream, pb_cmd_fields, &cmd)) {
                if ((cmd.cmd < (sizeof(fm_cmd) / 4)) && fm_cmd[cmd.cmd]) {
                    fm_cmd[cmd.cmd](this, FirmataStream, cmd);
                } else {
                    pb_response response;
                    response.cmd = cmd.cmd;
                    response.result = -1;
                    response.msg = "command not support";
                    logger.error("command not support");
                }
            } else {
                pb_response response;
                response.cmd = cmd.cmd;
                response.result = -1;
                response.msg = "parse protobuf error";
                const char *error = PB_GET_ERROR(&stream);
                logger.error("parse protobuf error:%s", error);
            }
        }
            break;
#ifdef USE_SOEM
        case pb_firmata_cmd_FM_SOEM:
            soem.process(this, FirmataStream, argc, argv);
            break;
#endif
        case ARE_YOU_THERE:
#if defined(RTE_APP) || defined(PLC)
            // logger.disable(logger_t::LOGGER_SERIAL);
#endif
            FirmataStream->write(START_SYSEX);
            FirmataStream->write(I_AM_HERE);
            FirmataStream->write(1); // arduino_instance_id
            FirmataStream->write(END_SYSEX);
            FirmataStream->flush();
            break;
        case pb_firmata_cmd_CB_GET_RTE_VERSION:
            sendSysex(FirmataStream, pb_firmata_cmd_CB_GET_RTE_VERSION, sizeof(pb_rte_const), (uint8_t *) &rteConst);
            break;

#ifndef THIS_IS_BOOTLOADER
        case SAMPLING_INTERVAL:
            if (argc > 1) {
                rte.data.samplingInterval = (byte) (argv[0] + (argv[1] << 7));
                if (rte.data.samplingInterval < rte_config.MINIMUM_SAMPLING_INTERVAL) {
                    rte.data.samplingInterval = rte_config.MINIMUM_SAMPLING_INTERVAL;
                }
            } else {
                // sendString("Not enough data");
            }
            break;
        case REPORT_ANALOG:
            if (argc == 2) {
                reportAnalogCallback(FirmataStream, argv[0], argv[1]);
            }
            break;
        case EXTENDED_ANALOG:
            if (argc > 1) {
                int al = argv[1];
                if (argc > 2)
                    al |= (argv[2] << 7);
                if (argc > 3)
                    al |= (argv[3] << 14);
                analogWriteCallback(FirmataStream, argv[0], al);
            }
            break;
        case CAPABILITY_QUERY:
            FirmataStream->write(START_SYSEX);
            FirmataStream->write(CAPABILITY_RESPONSE);
            board.capability_query(this, FirmataStream);
            FirmataStream->write(END_SYSEX);
            FirmataStream->flush();
            break;
        case PIN_STATE_QUERY:
            if (argc > 0) {
                byte pin = argv[0];
                FirmataStream->write(START_SYSEX);
                FirmataStream->write(PIN_STATE_RESPONSE);
                FirmataStream->write(pin);
                if (pin < IO_YO_NRS + IO_XI_NRS) {
                    FirmataStream->write(getPinMode(pin));
                    FirmataStream->write((byte) getPinState(pin) & 0x7F);
                    if (getPinState(pin) & 0xFF80)
                        FirmataStream->write((byte) (getPinState(pin) >> 7) & 0x7F);
                    if (getPinState(pin) & 0xC000)
                        FirmataStream->write((byte) (getPinState(pin) >> 14) & 0x7F);
                }
                FirmataStream->write(END_SYSEX);
                FirmataStream->flush();
            }
            break;
        case ANALOG_MAPPING_QUERY:
            FirmataStream->write(START_SYSEX);
            FirmataStream->write(ANALOG_MAPPING_RESPONSE);
            board.analog_mapping_query(this, FirmataStream);

            FirmataStream->write(END_SYSEX);
            FirmataStream->flush();
            break;
#ifdef FIRMATA_SERIAL_FEATURE
            case SERIAL_MESSAGE:
                serialFeature->handleSysex(this, FirmataStream, command, argc, argv);
                break;
#endif
        case pb_firmata_cmd_CB_GET_REMAIN_MEM:
            sendSysex(FirmataStream, pb_firmata_cmd_CB_GET_REMAIN_MEM, 2, (byte *) &rte.data.remain_mem);
            break;
#if defined(RTE_APP) || defined(PLC)
        case pb_firmata_cmd_CB_PLC_START:
            rte.app_resume();
            len = 0;
            sendSysex(FirmataStream, pb_firmata_cmd_CB_PLC_START, 2, (byte *) &len);
            break;
        case pb_firmata_cmd_CB_PLC_STOP:
            rte.app_supsund();
            len = 0;
            sendSysex(FirmataStream, pb_firmata_cmd_CB_PLC_STOP, 2, (byte *) &len);
            break;
        case pb_firmata_cmd_REPORT_PLC_MD5:
            if (app.data.plc_curr_app)
                sendSysex(FirmataStream, pb_firmata_cmd_REPORT_PLC_MD5, 32,
                          (byte *) ((plc_app_abi_t *) app.data.plc_curr_app)->id);
            else
                sendSysex(FirmataStream, pb_firmata_cmd_REPORT_PLC_MD5, 0, (byte *) "");
            break;
        case pb_firmata_cmd_CB_PLC_LOAD:
            len = 0;
            sendSysex(FirmataStream, pb_firmata_cmd_CB_PLC_LOAD, 2, (byte *) &len);
            // rte.app_supsund();
            // app.unload();
            rte.load_app();
            break;
        case pb_firmata_cmd_CB_PLC_REPAIR:
            rte.app_supsund();
            app.unload();
            len = 0;
            sendSysex(FirmataStream, pb_firmata_cmd_CB_PLC_REPAIR, 2, (byte *) &len);
            break;
#endif

        case pb_firmata_cmd_FM_FLASH_CLEAR:
            len = 0;
            sendSysex(FirmataStream, pb_firmata_cmd_FM_FLASH_CLEAR, 2, (byte *) &len);
            board.flashClear();
            hwboard::reset();
            break;
#endif
#if defined(USE_RTC) || defined(USE_PCF8563)
        case pb_firmata_cmd_CB_GET_RTC:
            sendSysex(FirmataStream, pb_firmata_cmd_CB_GET_RTC, sizeof(pb_rtc_info), (byte *) &rtc.data);
                break;
        case pb_firmata_cmd_CB_SET_RTC:
            {
                tm new_time{};

                len = 0;
                new_time.tm_year = *(u16 *)&argv[0];
                new_time.tm_mon = argv[2];
                new_time.tm_mday = argv[3];
                new_time.tm_hour = argv[4];
                new_time.tm_min = argv[5];
                new_time.tm_sec = argv[6];
                new_time.tm_wday = argv[7];
                rtc.set_time(&new_time);
                sendSysex(FirmataStream, pb_firmata_cmd_CB_SET_RTC, 2, (byte *) &len);
            }
            break;
#endif
#ifdef ARDUINO
#ifdef USE_LWIP
#ifdef USE_IP_MODIFY
        case pb_firmata_cmd_CB_SET_IP:
            rte_config.lwip.ip = *(uint32_t *) argv;
            ETH_LWIP::set_ip();
            sendSysex(FirmataStream, pb_firmata_cmd_CB_SET_IP, 4, (byte *) (&rte_config.lwip.ip));
            break;
#endif
        case pb_firmata_cmd_CB_GET_IP:
            sendSysex(FirmataStream, pb_firmata_cmd_CB_GET_IP, 4, (byte *) (&rte_config.lwip.ip));
            break;
        case pb_firmata_cmd_FM_GET_NET_BUF_STAT: {
            buffer = (byte *) malloc(13 * MEMP_MAX);
            for (int i = 0; i < MEMP_MAX; i++) {
                *(u8 *) &buffer[0 + 13 * i] = memp_pools[i]->stats->avail;
                *(u8 *) &buffer[1 + 13 * i] = memp_pools[i]->stats->err;
                *(u8 *) &buffer[2 + 13 * i] = memp_pools[i]->stats->illegal;
                *(u8 *) &buffer[3 + 13 * i] = memp_pools[i]->stats->max;
                *(u8 *) &buffer[4 + 13 * i] = memp_pools[i]->stats->used;
                memcpy(&buffer[5 + 13 * i], memp_pools[i]->stats->name, 8);
            }
            sendSysex(FirmataStream, pb_firmata_cmd_FM_GET_NET_BUF_STAT, 13 * MEMP_MAX, (byte *) buffer);
            free(buffer);
        }
            break;
#endif
#endif
#if defined(RTE_APP) || defined(PLC)
        case pb_firmata_cmd_CB_SET_FORCE:
            for (int i = 0; i < argc;) {
                const u16 *byte = (u16 *) &argv[i];
                len = argv[i + 2];
                index = *byte;
                if (rte.data.state == pb_state_Started) {
                    ((plc_app_abi_t *) app.data.plc_curr_app)->dbg_set_force(index, len ? &argv[i + 3] : nullptr);
                }
                i += len + 3;
            }
            sendSysex(FirmataStream, pb_firmata_cmd_CB_SET_FORCE, 2, (byte *) &len);
            break;

        case pb_firmata_cmd_CB_CLEAR_V:
            len = -1;
            if (rte.data.state == pb_state_Started) {
                ((plc_app_abi_t *) app.data.plc_curr_app)->dbg_vars_reset(__IEC_DEBUG_FLAG);
                logger.debug("monitor var reset.");
                len = 0;
            } else {
                logger.debug("monitor var not reset.plc_state=0x%x ", rte.data.state);
            }
            sendSysex(FirmataStream, pb_firmata_cmd_CB_CLEAR_V, 2, (byte *) &len);
            break;
        case pb_firmata_cmd_CB_SET_V:
            len = -1;
            if (argc >= 2) {
                for (int i = 0; i < argc; i += 2) {
                    const u16 *byte = (u16 *) &argv[i];
                    indexv = *byte;
                    if (rte.data.state == pb_state_Started) {
                        ((plc_app_abi_t *) app.data.plc_curr_app)->dbg_var_register(indexv);
                    }
                }
                len = argc / 2;
                logger.debug("CB_SET_V %d", len);
            }
            sendSysex(FirmataStream, pb_firmata_cmd_CB_SET_V, 2, (byte *) &len);
            break;
        case pb_firmata_cmd_CB_GET_V: {
            int len = 0;
            data = (u8 *) malloc(FirmataStream->tx_max_size());
            if (rte.data.state == pb_state_Started) {
                void *b = nullptr;
                ((plc_app_abi_t *) app.data.plc_curr_app)->dbg_data_get((uint32_t *) &data[0], (uint32_t *) &len,
                                                                        (void **) &b);
                if (len < FirmataStream->tx_max_size())
                    memcpy(&data[4], b, len);
                else
                    logger.error("CB_GET_V len=%d", len);
                ((plc_app_abi_t *) app.data.plc_curr_app)->dbg_data_free();
            }
            sendSysex(FirmataStream, pb_firmata_cmd_CB_GET_V, len + 4, data);

            free(data);
        }
            break;
#endif
#ifdef ARDUINO
        case pb_firmata_cmd_CB_SET_SERIAL_RX:

            // port = *(uint16_t *) argv;
            //            kSerial::get_serial(port)->set_rx();
            break;
        case pb_firmata_cmd_CB_SET_SERIAL_TX_HIGH:

            // port1 = *(uint16_t *) argv;
            //            kSerial::get_serial(port1)->set_high();
            break;
        case pb_firmata_cmd_CB_SET_SERIAL_TX_LOW:

            // port2 = *(uint16_t *) argv;
            //            kSerial::get_serial(port2)->set_low();
            break;
#endif
#if defined(RTE_APP) || defined(PLC)
        case pb_firmata_cmd_FM_GET_TASK_NAME:
            if (tasks[argv[0]]) {
                sendSysex(FirmataStream, pb_firmata_cmd_FM_GET_TASK_NAME, (byte) strlen(tasks[argv[0]]->task_name),
                          (byte *) tasks[argv[0]]->task_name);
            }
            break;
        case pb_firmata_cmd_FM_GET_TASK_DETAIL:
            if (argv[0] < PLC_TASK_NRS && tasks[argv[0]]) {
                sendSysex(FirmataStream, pb_firmata_cmd_FM_GET_TASK_DETAIL, 24, tasks[argv[0]]->mata());
            }
            break;
        case pb_firmata_cmd_FM_GET_PLC_INFO:
            if (app.data.plc_curr_app) {
                info.build = ((plc_app_abi_t *) app.data.plc_curr_app)->buildnumber;
                strcpy(info.name, ((plc_app_abi_t *) app.data.plc_curr_app)->app_name);
                sendSysex(FirmataStream, pb_firmata_cmd_FM_GET_PLC_INFO, sizeof(info), (byte *) &info);
            } else
                sendSysex(FirmataStream, pb_firmata_cmd_FM_GET_PLC_INFO, 0, (byte *) &info);
            break;
#endif
        case pb_firmata_cmd_FM_GET_PLC_STATE:
            sendSysex(FirmataStream, pb_firmata_cmd_FM_GET_PLC_STATE, 1, (byte *) (&rte.data.state));
            break;
        case pb_firmata_cmd_CB_GET_LOG_NUMBER:
            sendSysex(FirmataStream, pb_firmata_cmd_CB_GET_LOG_NUMBER, 5, (byte *) (&rte.data.state));
            break;
        case pb_firmata_cmd_CB_GET_LOG:
            sendSysex(FirmataStream, pb_firmata_cmd_CB_GET_LOG, 0, (byte *) argv);
            break;
#ifdef USE_BOOTLOADER
            case CB_GET_BOOT_VERSION:
#ifdef BOOTINFO
                boot_t *b;
                b = (boot_t *)BOOTINFO; // platformio.ini中定义
                if (b)
                    sendSysex(FirmataStream, CB_GET_BOOT_VERSION, sizeof(boot_t), (byte *)b);
                else
#endif
                {
                    len = -1;
                    sendSysex(FirmataStream, CB_GET_BOOT_VERSION, 2, (byte *)&len);
                }
                break;
            case FM_FLASH_BOOT:
                len = board.flash_bootloader_from_lfs();
                sendSysex(FirmataStream, FM_FLASH_BOOT, 2, (byte *)&len);
                break;
#endif

#ifdef USE_KVDB_LFS
        case pb_firmata_cmd_FM_LIST_KEY:
            kfs.dir_buf(pb_firmata_cmd_FM_LIST_KEY, (const char *) &argv[8], 0, 16, this, FirmataStream);
            break;
        case pb_firmata_cmd_CB_READ_KEY:
            size_t vlen, name_len;
            name_len = strlen((const char *) argv);
            byte *value_crk;
            value_crk = (byte *) malloc(FDB_KV_NAME_MAX + FDB_STR_KV_VALUE_MAX_SIZE + 2);
            strncpy((char *) &value_crk[2], (const char *) argv, name_len);
            if (name_len < FDB_KV_NAME_MAX && name_len > 0) {
                buffer = (byte *) kvdb.get((const char *) argv);
                vlen = strlen((const char *) buffer);
                if (buffer && (vlen > 0) && (vlen < FDB_STR_KV_VALUE_MAX_SIZE)) {
                    strncpy((char *) &value_crk[name_len + 3], (const char *) buffer, vlen);
                    vlen += name_len + 4;
                    *(short *) value_crk = 0;
                } else {
                    vlen = name_len + 4;
                    *(short *) value_crk = pb_event_KV_VALUE_ILLEGAL;
                }
            } else {
                vlen = name_len + 4;
                *(short *) value_crk = pb_event_KV_VALUE_ILLEGAL;
            }
            sendSysex(FirmataStream, pb_firmata_cmd_CB_READ_KEY, vlen, (byte *) value_crk);
            free(value_crk);
            break;
        case pb_firmata_cmd_FM_READ_KEY_BYTES:
            data = (u8 *) malloc(256);
            len = kvdb.get((const char *) argv, (char *) data + 4, 256, (uint32_t *) data);
            sendSysex(FirmataStream, pb_firmata_cmd_FM_READ_KEY_BYTES, len + 4, (byte *) data);
            free(data);
            break;
        case pb_firmata_cmd_CB_WRITE_KEY:
            key_len = strlen((const char *) argv);
            int rw;
            rw = argc - key_len - 2;
            byte *value;
            value = (byte *) malloc(FDB_KV_NAME_MAX + FDB_STR_KV_VALUE_MAX_SIZE + 2);
            strncpy((char *) &value[2], (const char *) argv, key_len);
            if ((key_len < FDB_KV_NAME_MAX) && (rw < FDB_STR_KV_VALUE_MAX_SIZE)) {
                rw = kvdb.set((const char *) argv, (const char *) argv + key_len + 1, (int) rw, KV_STR_VALUE);
                buffer = (byte *) kvdb.get((const char *) argv);
                vlen = strlen((const char *) buffer);
                if (buffer && (vlen < FDB_STR_KV_VALUE_MAX_SIZE)) {
                    strncpy((char *) &value[key_len + 3], (const char *) buffer, vlen);
                    vlen += key_len + 4;
                } else {
                    vlen = name_len + 3;
                    *(short *) value = pb_event_KV_VALUE_ILLEGAL;
                }
            }
            sendSysex(FirmataStream, pb_firmata_cmd_CB_WRITE_KEY, vlen, (byte *) value);
            free(value);
            break;
        case pb_firmata_cmd_FM_WRITE_KEY_BYTES:
            key_len = strlen((const char *) argv);
            uint32_t type;
            type = *(uint32_t *) (argv + key_len + 1);
            len = kvdb.set((const char *) argv, (const char *) (argv + key_len + 1 + 4),
                           (int) (argc - key_len - 1 - 4), type);
            sendSysex(FirmataStream, pb_firmata_cmd_FM_WRITE_KEY_BYTES, 2, (byte *) &len);
            break;
        case pb_firmata_cmd_CB_RM_KEY:
            kvdb.remove((const char *) argv);
            sendSysex(FirmataStream, pb_firmata_cmd_CB_RM_KEY, 0, argv);
            break;
#endif
#ifdef USE_TSDB
        case pb_firmata_cmd_CB_SET_TSL_RANGE:
                tsl_query q;
                memset(&q, 0, sizeof(q));
                if (argc == 13)
                {
                    u8 db = argv[0];
                    start = *(uint32_t *)&argv[1];
                    end = *(uint32_t *)&argv[1 + 4];
                    state = (int)*(uint32_t *)&argv[1 + 8];
                    TSDB *tsdb = TSDB::db(db);
                    if (tsdb)
                    {
                        tsdb->query(start, end, (fdb_tsl_status)(state), &q);
                    }
                }
            sendSysex(FirmataStream, pb_firmata_cmd_CB_SET_TSL_RANGE, sizeof(tsl_query), (byte *) &q);
                break;
        case pb_firmata_cmd_CB_SET_TSL_STATUS:

                len = -1;
                if (argc == 13)
                {
                    u8 db = argv[0];
                    start = *(uint32_t *)&argv[1];
                    end = *(uint32_t *)&argv[1 + 4];
                    state = (int)*(uint32_t *)&argv[1 + 8];
                    TSDB *tsdb = TSDB::db(db);
                    if (tsdb)
                    {
                        len = tsdb->set_status(start, end, (fdb_tsl_status)(state));
                    }
                }
            sendSysex(FirmataStream, pb_firmata_cmd_CB_SET_TSL_STATUS, 2, (byte *) &len);
                break;
                // case CB_GET_TSL: {
                // char *tbuf;
                //     int tlen;
                //     int buf_sz = FirmataStream->tx_max_size();
                //     tbuf = (char *) malloc(buf_sz);
                //     memset(tbuf, 0, buf_sz);
                //     u8 db = argv[0];
                //     TSDB *tsdb = TSDB::db(db);
                //     if (tsdb) {
                //         tlen = tsdb->query_read((uint32_t *) &tbuf[1], (fdb_time_t *) &tbuf[6],
                //                                 (int *) (tbuf + 10),
                //                                 tbuf + 14, buf_sz - 14);
                //     }
                //     if (tlen < 0) {
                //         *(short *) tbuf = tlen;
                //         tlen = 0;
                //}
                //     sendSysex(FirmataStream, CB_GET_TSL, (byte) tlen + 14, (byte *) tbuf);
                // free(tbuf);
                // }
                // break;
        case pb_firmata_cmd_CB_GET_TSL_BY_ID:
            {
                char *tbuf;
                int tlen;
                u8 db = argv[0];
                int buf_sz = FirmataStream->tx_max_size() * 7 / 8;
                tbuf = (char *)malloc(buf_sz);
                tlen = 2;
                *(short *)tbuf = 0;
                TSDB *tsdb = TSDB::db(db);
                if (tsdb)
                {
                    tlen = tsdb->query_read_by_id(*(uint32_t *)&argv[1], (uint32_t *)&tbuf[2],
                                                  (fdb_time_t *)&tbuf[6],
                                                  (int *)(tbuf + 10),
                                                  tbuf + 14, buf_sz - 14);
                }
                if (tlen < 0)
                {
                    *(short *)tbuf = tlen;
                    tlen = 2;
                }
                else
                {
                    tlen += 14;
                }
                sendSysex(FirmataStream, pb_firmata_cmd_CB_GET_TSL_BY_ID, tlen, (byte *) tbuf);
                free(tbuf);
            }
            break;
        case pb_firmata_cmd_CB_TSL_CLEAR:
            {
                short len = 0;
                key_len = strlen((const char *)argv);
                // index = *(int *) &argv[key_len + 1];
                state = 0;
                u8 db = argv[0];
                TSDB *tsdb = TSDB::db(db);
                if (tsdb)
                {
                    sendSysex(FirmataStream, pb_firmata_cmd_CB_TSL_CLEAR, 2, (byte *) &len);
                    state = tsdb->clear();
                }
            }
            break;
#endif
        case pb_firmata_cmd_CB_CPU_USAGE:
            sendSysex(FirmataStream, pb_firmata_cmd_CB_CPU_USAGE, 1, (byte *) &rte.data.cpu_usage);
            break;
#ifdef USE_WIFI
            case CB_WIFI_LIST:
                sendSysex(FirmataStream, CB_WIFI_LIST, rte_data.wifi_size, (byte *)&rte_data.wifi_list);
                break;
            case CB_WIFI_SET_PASS:
                int plen;
                plen = strlen((const char *)argv);
                //                kvdb.set("wifi_pass", (const char *)argv, plen);
                wifi.reload();
                sendSysex(FirmataStream, CB_WIFI_SET_PASS, 0, argv);
                break;
#endif
#ifdef USE_MEMBLOCK
        case pb_firmata_cmd_FM_PUT_DATA_BLOCK: {
            rte.event(pb_event_APP_FLASH, true);
            int block = *(int *) &argv[0];
            if (block == 0) {
                {
                    uint32_t object = *(uint32_t *) &argv[4];
                    uint32_t data_address = *(uint32_t *) &argv[8];
                    uint32_t data_len = *(uint32_t *) &argv[12];

                    dev = mem_block::mems[object];
                    if (!dev) {
                        state = pb_event_NO_DEVICE;
                    } else {
                        state = dev->begin(&argv[16], argc - 16, data_address, data_len);
                        blocksize = (FirmataStream->tx_max_size() * 7 / 8 - 16) & (~0x3);
                        if ((state > 0) && (state > blocksize)) {
                            state = blocksize;
                        }
                        if (state > FIRMATA_BUFFER_SZ * 7 / 8 - 16) {
                            blocksize = (FIRMATA_BUFFER_SZ * 7 / 8 - 16) & (~0x3);
                            state = blocksize;
                        }
                        state &= ~0x3;
                        // logger.info("block 0 file = %s ,address=0x%x ,size= %d", &argv[16], data_address, data_len);
                    }
                }
            } else if (block == -1) {
                if (dev) {
                    if (dev->Shutdown() < 0) {
                        state = pb_event_DEVICE_SHUTDOWN_ERR;
                    } else {
                        state = 1;
                        rte.event(pb_event_APP_FLASH, false);
                        // logger.info("recv end.");
                        dev = nullptr;
                    }
                }
            } else {
                if (dev) {
                    //                        rte.event(PLC_STATUS::APP_FLASHING);
                    if (dev->Write(&argv[4], argc - 4) < 0) {
                        state = pb_event_DEVICE_WRITE_ERR;
                        logger.error("write error %d ,size= %d", block, argc - 8);
                    } else {
                        state = block;
                    }
                    //  logger.info("recv %d ,size= %d 0x%x 0x%x", block, argc - 4, argv[4], argv[5]);
                }
            }
        }
            sendSysex(FirmataStream, pb_firmata_cmd_FM_PUT_DATA_BLOCK, 4, (byte *) &state);
            break;
        case pb_firmata_cmd_FM_GET_DATA_BLOCK: {
            state = -1;
            int block = *(int *) &argv[0];
            if (block == 0) {
                uint32_t object = *(uint32_t *) &argv[4];
                uint32_t data_address = *(uint32_t *) &argv[8];
                uint32_t data_len = *(uint32_t *) &argv[12];
                dev = mem_block::mems[object];
                if (!dev) {
                    state = pb_event_NO_DEVICE;
                } else {
                    byte *tbuf = (byte *) malloc(16);
                    *(int *) &tbuf[0] = 0;
                    *(int *) &tbuf[4] = dev->begin_read(&argv[16], argc - 16, data_address, data_len);
                    blocksize = (FirmataStream->tx_max_size() * 7 / 8 - 16) & (~0x3);

                    if (blocksize > FIRMATA_BUFFER_SZ * 7 / 8 - 16) {
                        blocksize = (FIRMATA_BUFFER_SZ * 7 / 8 - 16) & (~0x3);
                    }

                    *(uint32_t *) &tbuf[8] = blocksize;
                    sendSysex(FirmataStream, pb_firmata_cmd_FM_GET_DATA_BLOCK, 12, tbuf);
                    free(tbuf);
                    break;
                }
            } else {
                if (dev) {
                    byte *tbuf = (byte *) malloc(blocksize + 16);
                    *(int *) &tbuf[0] = block;
                    *(uint32_t *) &tbuf[4] = dev->Read(0, blocksize * (block - 1), blocksize, &tbuf[8]);
                    if (*(uint32_t *) &tbuf[4] < blocksize) {
                        dev->Shutdown();
                        dev = nullptr;
                    }
                    sendSysex(FirmataStream, pb_firmata_cmd_FM_GET_DATA_BLOCK, *(uint32_t *) &tbuf[4] + 8, tbuf);
                    free(tbuf);
                    break;
                }
            }
            sendSysex(FirmataStream, pb_firmata_cmd_FM_GET_DATA_BLOCK, 4, (byte *) &state);
            break;
        }
#endif
#if defined(RTE_APP) || defined(PLC)
        case pb_firmata_cmd_FM_GET_LOC_SIZE:
            if (app.data.plc_curr_app) {
                sendSysex(FirmataStream, pb_firmata_cmd_FM_GET_LOC_SIZE, 2,
                          (byte *) &((plc_app_abi_t *) app.data.plc_curr_app)->l_sz);
            } else {
                sendSysex(FirmataStream, pb_firmata_cmd_FM_GET_LOC_SIZE, 0,
                          (byte *) &((plc_app_abi_t *) app.data.plc_curr_app)->l_sz);
            }
            break;
        case pb_firmata_cmd_FM_GET_LOC_TAB:
            uint32_t l_index;
            if (argc == 4) {
                l_index = *(uint32_t *) &argv[0];
                if (app.data.plc_curr_app && l_index < ((plc_app_abi_t *) app.data.plc_curr_app)->l_sz) {
                    plc_loc_tbl_t loc = ((plc_app_abi_t *) app.data.plc_curr_app)->l_tab[l_index];
                    len = (int) sizeof(plc_loc_dsc_t) + loc->a_size + loc->v_size;
                    byte *buffer = (byte *) malloc(len);
                    buffer[0] = loc->v_type;
                    buffer[1] = loc->v_size;
                    *(u16 *) &buffer[2] = loc->proto;
                    *(u16 *) &buffer[4] = loc->a_size;
                    memcpy(&buffer[6], loc->a_data, loc->a_size);
                    memcpy(&buffer[6 + loc->a_size], loc->v_buf, loc->v_size);
                    sendSysex(FirmataStream, pb_firmata_cmd_FM_GET_LOC_TAB, len, (byte *) buffer);
                    free(buffer);
                    break;
                }
            }
            sendSysex(FirmataStream, pb_firmata_cmd_FM_GET_LOC_TAB, 0,
                      (byte *) &((plc_app_abi_t *) app.data.plc_curr_app)->l_sz);
            break;
        case pb_firmata_cmd_FM_SET_LOC_TAB:
            if (argc == 5) {
                l_index = *(uint32_t *) &argv[0];
                if (app.data.plc_curr_app && l_index < ((plc_app_abi_t *) app.data.plc_curr_app)->l_sz) {
                    sendSysex(FirmataStream, pb_firmata_cmd_FM_SET_LOC_TAB, sizeof(plc_loc_tbl_t),
                              (byte *) &((plc_app_abi_t *) app.data.plc_curr_app)->l_tab[l_index]);
                    break;
                }
            }
            sendSysex(FirmataStream, pb_firmata_cmd_FM_SET_LOC_TAB, 0,
                      (byte *) &((plc_app_abi_t *) app.data.plc_curr_app)->l_sz);
            break;
#endif
#ifdef ONLINE_DEBUG
            case pb_firmata_cmd_FM_GET_DBG_SIZE:
                if (app.data.plc_curr_app) {
                    sendSysex(FirmataStream, pb_firmata_cmd_FM_GET_DBG_SIZE, 4,
                              (byte *) &((plc_app_abi_t *) app.data.plc_curr_app)->data->size_dbgvardsc);
                } else {
                    sendSysex(FirmataStream, pb_firmata_cmd_FM_GET_DBG_SIZE, 0, nullptr);
                }
                break;
            case pb_firmata_cmd_FM_GET_DBG:
                len = 0;
                if (argc == 4) {
                    l_index = *(uint32_t *) &argv[0];
                    if (app.data.plc_curr_app &&
                        l_index < ((plc_app_abi_t *) app.data.plc_curr_app)->data->size_dbgvardsc) {
                        len = (int) fill_dbg(((plc_app_abi_t *) app.data.plc_curr_app)->data->dbgvardsc, (int) l_index,
                                             argv);
                    }
                }
                sendSysex(FirmataStream, pb_firmata_cmd_FM_GET_DBG, len, argv);
                break;
            case pb_firmata_cmd_FM_SET_DBG:
                len = 0;
                if (argc > 5) {
                    l_index = *(uint32_t *) argv;
                    if (app.data.plc_curr_app &&
                        l_index < ((plc_app_abi_t *) app.data.plc_curr_app)->data->size_dbgvardsc) {
                        set_dbg(((plc_app_abi_t *) app.data.plc_curr_app)->data->dbgvardsc, l_index, &argv[4],
                                argc - 4);
                        len = (int) fill_dbg(((plc_app_abi_t *) app.data.plc_curr_app)->data->dbgvardsc, (int) l_index,
                                             argv);
                    }
                }
                sendSysex(FirmataStream, pb_firmata_cmd_FM_GET_DBG, len, argv);
                break;
#endif
#if defined(RTE_APP) || defined(PLC)
        case pb_firmata_cmd_FM_LOG_SET_LEVEL:
            rte_config.log_level = argv[0];
            sendSysex(FirmataStream, pb_firmata_cmd_FM_LOG_SET_LEVEL, 1, &argv[0]);
            break;
#endif
#ifdef ARDUINO_ARCH_STM32
        case pb_firmata_cmd_FM_GET_CPU_SN:
            sendSysex(FirmataStream, pb_firmata_cmd_FM_GET_CPU_SN, 12, (byte *) rte.data.sn);
            break;
#endif
        case pb_firmata_cmd_FM_READ_MEM:
            indexv = 0;
            len = 0;
            if (argc == 6) {
                indexv = *(uint32_t *) argv;
                len = *(u16 *) &argv[4];
            }
            sendSysex(FirmataStream, pb_firmata_cmd_FM_READ_MEM, len, (byte *) indexv);
            break;
        case pb_firmata_cmd_FM_WRITE_MEM:
            if (argc > 6) {
                indexv = *(uint32_t *) argv;
                len = *(u16 *) &argv[4];
                for (int i = 0; i < len; ++i) {
                    *((uint8_t *) indexv + i) = argv[6 + i];
                }
                sendSysex(FirmataStream, pb_firmata_cmd_FM_WRITE_MEM, len, (byte *) indexv);
            }
            break;
#ifndef THIS_IS_BOOTLOADER
        case pb_firmata_cmd_FM_READ_VALUE:
            u8 region, typ;
            indexv = 0;
            len = *(u16 *) &argv[6];
            if ((argc == 8 && argv[0] <= REGION_HOLDER) && (len < FirmataStream->tx_max_size())) {
                region = argv[0];
                indexv = *(uint32_t *) &argv[1];
                typ = argv[5];
                // byte *buffer;
                // buffer = (byte *) malloc(len * 4 + 10);
                sendBuffer[0] = region;
                sendBuffer[1] = typ;
                *(uint32_t *) &sendBuffer[2] = indexv;
                const char *p;
                switch (region) {
                    default:
                        p = (const char *) nullptr;
                        break;
                    case REGION_XI: // byte from 0
                        p = (const char *) rte_data.X;
                        len = (len + 7) / 8;
                        break;
                    case REGION_16: // analogValue
                        p = (const char *) rte_data.XA;
                        break;
                    case REGION_32: // analogValue32
                        p = (const char *) rte_data.xa32;
                        break;
                    case REGION_HOLDER: // holdValue
                        p = (const char *) &rte_config;
                        break;
                    case REGION_INFO:
                        p = (const char *) &rte_data;
                        break;
                    case REGION_CONFIG:
                        p = (const char *) &rte_config;
                        break;
                }
                memcpy(&sendBuffer[6], &p[indexv], len);
                sendSysex(FirmataStream, pb_firmata_cmd_FM_READ_VALUE_REP, len + 6, (byte *) sendBuffer);
                // free(buffer);
            } else {
                len = -1;
                sendSysex(FirmataStream, pb_firmata_cmd_FM_READ_VALUE_REP, 2, (byte *) &len);
            }
            break;
        case pb_firmata_cmd_FM_WRITE_VALUE:
            if (argc > 7) {
                region = argv[0];
                indexv = *(uint32_t *) &argv[1];
                len = *(u16 *) &argv[5];
                char *p;
                switch (region) {
                    default:
                        p = nullptr;
                        break;
                    case REGION_XI: // byte from 0
                        u8 v;
                        v = ((char *) &rte_data.X)[indexv / 8];
                        if (argv[7] == 1) {
                            argv[7] = v | (1 << (indexv % 8));
                        } else {
                            argv[7] = v & ~(1 << (indexv % 8));
                        }
                        indexv = indexv / 8;
                        len = (len + 7) / 8;
                        p = ((char *) &rte_data.X);
                        break;
                    case REGION_16: // analogValue
                        p = (char *) &rte_data.XA;
                        break;
                    case REGION_32: // analogValue32
                        p = (char *) &rte_data.xa32;
                        break;
                    case REGION_HOLDER: // holdValue
                        p = (char *) &rte_config;
                        break;
                    case REGION_INFO:
                        p = (char *) &rte_data;
                        break;
                    case REGION_CONFIG:
                        p = (char *) &rte_config;
                        break;
                }
                memcpy(p + indexv, &argv[7], len);
                sendSysex(FirmataStream, pb_firmata_cmd_FM_WRITE_VALUE_REP, 7, (byte *) argv);
            }
            break;
        case pb_firmata_cmd_FM_READ_BIT:
            indexv = 0;
            len = 0;
            if (argc == 6) {
                indexv = *(uint32_t *) argv;
                len = *(u16 *) &argv[4];
            }
            if (len > 0) {
                byte *buffer = (byte *) malloc(len / 8 + 6);
                *(uint32_t *) argv = indexv;
                buffer[4] = len;
                for (int i = 0; i < len; i++) {
                    u8 b = rte_data.X[indexv + i];
                    buffer[i / 8 + 5] |= b << (i % 8);
                }
                sendSysex(FirmataStream, pb_firmata_cmd_FM_READ_BIT_REP, len / 8 + 6, (byte *) buffer);
                free(buffer);
            }
            break;
        case pb_firmata_cmd_FM_WRITE_BIT:
            if (argc > 6) {
                indexv = *(uint32_t *) argv;
                len = *(u16 *) &argv[4];
                if (len < (argc - 6)) {
                    for (int i = 0; i < len; i++) {
                        *(((uint8_t *) rte_data.X) + indexv + i) = argv[6 + i];
                    }
                }
                sendSysex(FirmataStream, pb_firmata_cmd_FM_WRITE_BIT_REP, len,
                          (byte *) ((uint8_t *) rte_data.X) + indexv);
            }
            break;
#endif
#ifdef RTE_APP
        case pb_firmata_cmd_FM_READ_VALUE_REP:
            if (argc > 0) {
                memcpy(valueBuf, argv, argc);
                valueLen = argc;
            }
            break;
        case pb_firmata_cmd_FM_WRITE_VALUE_REP:
            if (argc > 0 && argc < 8) {
            }
            break;
        case pb_firmata_cmd_FM_READ_BIT_REP:
            if (argc > 0 && argc < 16) {
                memcpy(valueBuf, argv, argc);
                valueLen = argc;
            }
            break;
        case pb_firmata_cmd_FM_WRITE_BIT_REP:
            if (argc > 0 && argc < 8) {
            }
            break;
        case pb_firmata_cmd_FM_GET_LOCATION:
            len = 0;
            byte *buf_fgl;
            buf_fgl = (byte *) malloc(32);
            if (argc >= 4) {
                len = board.get_input(argv[1], argv[2], argv[3], 0, buf_fgl);
            }
            sendSysex(FirmataStream, pb_firmata_cmd_FM_GET_LOCATION, len, (byte *) buf_fgl);
            free(buf_fgl);
            break;
        case pb_firmata_cmd_FM_SET_LOCATION:
            len = -1;
            if (argc >= 6) {
                len = board.set_output(argv[1], argv[2], argv[3], 0, &argv[5], argv[4]);
            }
            sendSysex(FirmataStream, pb_firmata_cmd_FM_SET_LOCATION, 4, (byte *) &len);
            break;
#endif

        case pb_firmata_cmd_CB_RESET: {
            len = 0;
            if (argc > 0) {
#ifdef USE_IAP
                if (argv[0] == 1)
                    inlineCtrl.data->iap = pb_state_EXEC_IAP;
                else
#endif
                if (argv[0] == 2) {
#ifdef USE_FLASH_LFS_MV
                    inlineCtrl.data->rte_action = pb_state_flash_rte;
                    inlineCtrl.data->plc_action = pb_state_flash_app;
#endif
                }
                len = argv[0];
            }
            sendSysex(FirmataStream, pb_firmata_cmd_CB_RESET, 2, (byte *) &len);
            rte.event(pb_event_REQUEST_RESTART, 1);
        }
            break;
#ifndef THIS_IS_BOOTLOADER
        case pb_firmata_cmd_FM_IOT_LOGIN:
            switch (argv[0]) {
                case IOT_LOGIN_OK:
                    break;
            }
            break;
#ifdef ARDUINO_ARCH_STM32
        case pb_firmata_cmd_FM_INFO_SERIAL_RX: {
            kSerial *serial = kSerial::get_serial(argv[0]);
            if (nullptr == serial) {
                buffer = (byte *) malloc(4);
                len = 4;
                *(int *) buffer = pb_event_NO_DEVICE;
            } else {
                buffer = (byte *) malloc(serial->data.rx_buf_size + 16);
                *(int *) buffer = serial->data.rx_count;
                memcpy(buffer + 4, Rtos::queue_buf(serial->_serial.rx_buff), serial->data.rx_buf_size);
                len = serial->data.rx_buf_size + 4;
            }
            sendSysex(FirmataStream, pb_firmata_cmd_FM_INFO_SERIAL_RX, len, buffer);
            free(buffer);
        }
            break;
        case pb_firmata_cmd_FM_INFO_SERIAL_TX: {
            kSerial *serial = kSerial::get_serial(argv[0]);
            if (nullptr == serial) {
                buffer = (byte *) malloc(4);
                len = 4;
                *(int *) buffer = pb_event_NO_DEVICE;
            } else {
                buffer = (byte *) malloc(serial->data.tx_buf_size + 16);
                *(int *) buffer = serial->data.tx_count;
                memcpy(buffer + 4, serial->_serial.tx_buff + serial->_serial.tx_head,
                       serial->data.tx_buf_size - serial->_serial.tx_head);
                memcpy(buffer + 4, serial->_serial.tx_buff, serial->_serial.tx_head);
                len = serial->data.tx_buf_size + 4;
            }
            sendSysex(FirmataStream, pb_firmata_cmd_FM_INFO_SERIAL_TX, len, buffer);
            free(buffer);
        }
            break;
#endif
#endif
#ifdef USE_LFS
        case pb_firmata_cmd_FM_LFS_LS:
            if (argc > 8) {
                uint32_t since = *(uint32_t *) argv;
                uint32_t size = *(uint32_t *) &argv[4];
                len = kfs.dir_buf(pb_firmata_cmd_FM_LFS_LS, (const char *) &argv[8], since, size, this,
                                  FirmataStream);
            }
            break;
        case pb_firmata_cmd_FM_REMOVE_FILE:
            len = -1;
            if (argc > 1) {
                len = kfs.unlink((const char *) argv);
            }
            sendSysex(FirmataStream, pb_firmata_cmd_FM_REMOVE_FILE, 2, (byte *) &len);
            break;
#endif
#ifndef THIS_IS_BOOTLOADER
        // case pb_firmata_cmd_FM_GET_SERIAL_INFO:
        //     kSerial::get_info(this, FirmataStream, command, argc, argv);
        //     break;
#endif
        default: {
            short len = -1;
            logger.error("sysexCallback: %d argc=%d,argv=%p", command, argc, argv);
            // sendSysex(FirmataStream, command, 2, (byte *) &len);
        }
            break;
    }
    set_flag(command);
}

int mFirmata::read_rte_const(mFirmata *mf, nStream *pStream, pb_cmd cmd) {
    int res = 0;
//    logger.debug("read_rte_const");
    pb_ostream_t stream = pb_ostream_from_buffer(mf->sendBuffer, FIRMATA_BUFFER_SZ);
    mf->msg.object.rte_const = rteConst;
    mf->msg.which_object = pb_object_rte_const_tag;
    res = pb_encode(&stream, pb_object_fields, &mf->msg);
//    logger.debug("read_rte_const res=%d", res);
    if (!res) {
        const char *error = PB_GET_ERROR(&stream);
        logger.error("read_rte_const encode error: %s", error);
    }
    mf->sendSysex(pStream, pb_firmata_cmd_FM_PROTOBUF, stream.bytes_written, mf->sendBuffer);
    return 0;
}

int mFirmata::write_module(mFirmata *mf, nStream *pStream, pb_cmd cmd) {
    pb_field_iter_t iter;
    bool ok = false;
    pb_ostream_t *stream = mf->SysexStream(pStream, pb_firmata_cmd_FM_PROTOBUF);
    if (cmd.param < (rte.data.max_level)) {
        smodule *module = smodule::modules[cmd.param];
        if (module->iter(&iter))
            ok = true;
        int ret = module->encode(&mf->msg, stream);
        if (!ret) {
            const char *error = PB_GET_ERROR(stream);
            logger.error("write_module %d encode error: %s", cmd.param, error);
        }
    }
    if (!ok) {
        logger.error("write_module: %d", cmd.param);
        return -1;
    }
    if (pb_field_iter_find(&iter, cmd.tag)) {
        if (PB_ATYPE(iter.type) == PB_ATYPE_STATIC) {
            memcpy(iter.pData, cmd.data->bytes, cmd.data->size);
        } else if (PB_ATYPE(iter.type) == PB_ATYPE_POINTER) {
            memcpy(iter.pData, cmd.data->bytes, cmd.data->size);
        } else {
            logger.error("write_module: %d", cmd.param);
        }
    }
    int ret = pb_encode(stream, pb_object_fields, &mf->msg);
    if (!ret) {
        const char *error = PB_GET_ERROR(stream);
        logger.error("write_module encode error: %s", error);
    }
    mf->sendSysex(pStream, pb_firmata_cmd_FM_PROTOBUF, stream->bytes_written, mf->sendBuffer);
    return 0;
}

int mFirmata::goto_iap(mFirmata *mf, nStream *pStream, pb_cmd cmd) {
    pb_response response;
    response.result = 0;
    response.cmd = cmd.cmd;
    response.msg = "OK";
#ifdef USE_IAP
    inlineCtrl.data->iap = pb_state_EXEC_IAP;
    rte.event(pb_event_REQUEST_RESTART, 1);
#endif
    pb_ostream_t *stream = mf->SysexStream(pStream, pb_firmata_cmd_FM_PROTOBUF);
    int ret = pb_encode(stream, pb_response_fields, &response);
    if (!ret) {
        const char *error = PB_GET_ERROR(stream);
        logger.error("goto_iap encode error: %s", error);
    }
    mf->sendSysex(pStream, pb_firmata_cmd_FM_PROTOBUF, stream->bytes_written, mf->sendBuffer);
    return 0;
}

int mFirmata::reboot(mFirmata *mf, nStream *pStream, pb_cmd cmd) {
    pb_response response;
    response.result = 0;
    response.cmd = cmd.cmd;

    rte.event(pb_event_REQUEST_RESTART, 1);

    pb_ostream_t *stream = mf->SysexStream(pStream, pb_firmata_cmd_FM_PROTOBUF);
    int ret = pb_encode(stream, pb_response_fields, &response);
    if (!ret) {
        const char *error = PB_GET_ERROR(stream);
        logger.error("reboot encode error: %s", error);
    }
    mf->sendSysex(pStream, pb_firmata_cmd_FM_PROTOBUF, stream->bytes_written, mf->sendBuffer);
    return 0;
}

int mFirmata::goto_boot(mFirmata *mf, nStream *pStream, pb_cmd cmd) {
    pb_response response;
    response.result = 0;
    response.msg = "OK";
    response.cmd = cmd.cmd;
    pb_ostream_t *stream = mf->SysexStream(pStream, pb_firmata_cmd_FM_PROTOBUF);
    int ret = pb_encode(stream, pb_response_fields, &response);
    if (!ret) {
        const char *error = PB_GET_ERROR(stream);
        logger.error("goto_boot encode error: %s", error);
    }
    mf->sendSysex(pStream, pb_firmata_cmd_FM_PROTOBUF, stream->bytes_written, mf->sendBuffer);
    rte.event(pb_event_REQUEST_RESTART, 1);
    return 0;
}

int mFirmata::read_rte_data(mFirmata *mf, nStream *pStream, pb_cmd cmd) {
//    logger.debug("read_rte_data");
    pb_ostream_t stream = pb_ostream_from_buffer(mf->sendBuffer, FIRMATA_BUFFER_SZ);
    int res = pb_encode(&stream, pb_rte_data_fields, &rte_data);
//    logger.debug("read_rte_data res: %d", res);
    if (!res) {
        const char *error = PB_GET_ERROR(&stream);
        logger.error("read_rte_data encode error: %s", error);
    }
    mf->sendSysex(pStream, pb_firmata_cmd_FM_PROTOBUF, stream.bytes_written, mf->sendBuffer);
    return 0;
}

int mFirmata::read_rte_holder(mFirmata *mf, nStream *pStream, pb_cmd cmd) {
//    logger.debug("read_rte_holder");
    pb_ostream_t stream = pb_ostream_from_buffer(mf->sendBuffer, FIRMATA_BUFFER_SZ);
    int res = pb_encode(&stream, pb_rte_holder_fields, &rte_config);
//    logger.debug("read_rte_holder res: %d", res);
    if (!res) {
        const char *error = PB_GET_ERROR(&stream);
        logger.error("read_rte_holder encode error: %s", error);
    }
    mf->sendSysex(pStream, pb_firmata_cmd_FM_PROTOBUF, stream.bytes_written, mf->sendBuffer);
    return 0;
}

int mFirmata::write_rte_data(mFirmata *mf, nStream *pStream, pb_cmd cmd) {
    pb_field_iter_t iter;
    bool ok = false;
    pb_ostream_t *stream = mf->SysexStream(pStream, pb_firmata_cmd_FM_PROTOBUF);
    if (pb_field_iter_begin(&iter, pb_rte_data_fields, &rte_data))
        ok = true;
    if (!ok) {
        logger.error("write_rte_data: %d", cmd.param);
        return -1;
    }
    if (pb_field_iter_find(&iter, cmd.tag)) {
        if (PB_ATYPE(iter.type) == PB_ATYPE_STATIC) {
            memcpy(iter.pData, cmd.data->bytes, cmd.data->size);
        } else if (PB_ATYPE(iter.type) == PB_ATYPE_POINTER) {
            memcpy(iter.pData, cmd.data->bytes, cmd.data->size);
        } else {
            logger.error("write_rte_data: %d", cmd.param);
        }
    }
    int res = pb_encode(stream, pb_rte_data_fields, &rte_data);
    if (!res) {
        const char *error = PB_GET_ERROR(stream);
        logger.error("write_rte_data encode error: %s", error);
    }
    mf->sendSysex(pStream, pb_firmata_cmd_FM_PROTOBUF, stream->bytes_written, mf->sendBuffer);
    return 0;
}

int mFirmata::write_rte_holder(mFirmata *mf, nStream *pStream, pb_cmd cmd) {
    pb_field_iter_t iter;
    bool ok = false;
    pb_ostream_t *stream = mf->SysexStream(pStream, pb_firmata_cmd_FM_PROTOBUF);
    if (pb_field_iter_begin(&iter, pb_rte_holder_fields, &rte_config))
        ok = true;
    if (!ok) {
        logger.error("write_rte_holder: %d", cmd.param);
        return -1;
    }
    if (pb_field_iter_find(&iter, cmd.tag)) {
        if (PB_ATYPE(iter.type) == PB_ATYPE_STATIC) {
            memcpy(iter.pData, cmd.data->bytes, cmd.data->size);
        } else if (PB_ATYPE(iter.type) == PB_ATYPE_POINTER) {
            memcpy(iter.pData, cmd.data->bytes, cmd.data->size);
        } else {
            logger.error("write_rte_holder: %d", cmd.param);
        }
    }
    int res = pb_encode(stream, pb_rte_holder_fields, &rte_config);
    if (!res) {
        const char *error = PB_GET_ERROR(stream);
        logger.error("write_rte_holder encode error: %s", error);
    }
    mf->sendSysex(pStream, pb_firmata_cmd_FM_PROTOBUF, stream->bytes_written, mf->sendBuffer);
    return 0;
}

int mFirmata::read_rte_info(mFirmata *mf, nStream *pStream, pb_cmd cmd) {
    int res = 0;
//    logger.debug("read_rte_info");
    pb_ostream_t stream = pb_ostream_from_buffer(mf->sendBuffer, FIRMATA_BUFFER_SZ);
    mf->msg.object.rte_info = rte.data;
    mf->msg.which_object = pb_object_rte_info_tag;
    res = pb_encode(&stream, pb_object_fields, &mf->msg);
//    logger.debug("read_rte_info res: %d", res);
    if (!res) {
        const char *error = PB_GET_ERROR(&stream);
        logger.error("read_rte_info encode error: %s", error);
    }
    mf->sendSysex(pStream, pb_firmata_cmd_FM_PROTOBUF, stream.bytes_written, mf->sendBuffer);
    return 0;
}

int mFirmata::read_module(mFirmata *mf, nStream *pStream, pb_cmd cmd) {
    int res = 0;
//    logger.debug("read_module");
    pb_ostream_t stream = pb_ostream_from_buffer(mf->sendBuffer, FIRMATA_BUFFER_SZ);
    int index = cmd.param;
    if (index < (rte.data.max_level)) {
        smodule *module = smodule::modules[index];
        res = module->encode(&mf->msg, &stream);
    }
    res = pb_encode(&stream, pb_object_fields, &mf->msg);
    if (!res) {
        const char *error = PB_GET_ERROR(&stream);
        logger.error("read_module %d encode error: %s", index, error);
    }
    mf->sendSysex(pStream, pb_firmata_cmd_FM_PROTOBUF, stream.bytes_written, mf->sendBuffer);
    return 0;
}

int mFirmata::read_rte_ctrl(mFirmata *mf, nStream *pStream, pb_cmd cmd) {
    int res = 0;
#ifdef USE_BKP_SRAM
    pb_ostream_t *stream =   mf->SysexStream(pStream,pb_firmata_cmd_FM_PROTOBUF);
    int index = cmd.param;
    mf->msg.object.ctrl = *inlineCtrl.data;
    mf->msg.which_object = pb_object_ctrl_tag;
    res = pb_encode(stream, pb_object_fields, &mf->msg);

    if (!res)
    {
        const char *error = PB_GET_ERROR(stream);
        logger.error("read_rte_ctrl encode error: %s", error);
    }
    mf->sendSysex(pStream, pb_firmata_cmd_FM_PROTOBUF, stream->bytes_written, mf->sendBuffer);
#endif
    return 0;
}

int mFirmata::write_rte_ctrl(mFirmata *mf, nStream *pStream, pb_cmd cmd) {
#ifdef USE_BKP_SRAM
    pb_field_iter_t iter;
    bool ok = false;
    pb_ostream_t *stream =   mf->SysexStream(pStream,pb_firmata_cmd_FM_PROTOBUF);
    if (pb_field_iter_begin(&iter, pb_ctrl_fields, inlineCtrl.data))
        ok = true;
    if (!ok)
    {
        logger.error("write_rte_info: %d", cmd.param);
        return -1;
    }
    if (pb_field_iter_find(&iter, cmd.tag))
    {
        if (PB_ATYPE(iter.type) == PB_ATYPE_STATIC)
        {
            memcpy(iter.pData, cmd.data->bytes, cmd.data->size);
        }
        else if (PB_ATYPE(iter.type) == PB_ATYPE_POINTER)
        {
            memcpy(iter.pData, cmd.data->bytes, cmd.data->size);
        }
        else
        {
            logger.error("write_rte_info: %d", cmd.param);
        }
    }
    int res = pb_encode(stream, pb_ctrl_fields, inlineCtrl.data);
    if (!res)
    {
        const char *error = PB_GET_ERROR(stream);
        logger.error("write_rte_info encode error: %s", error);
    }
    mf->sendSysex(pStream, pb_firmata_cmd_FM_PROTOBUF, stream->bytes_written, mf->sendBuffer);
#endif
    return 0;
}

int mFirmata::write_rte_info(mFirmata *mf, nStream *pStream, pb_cmd cmd) {
    pb_field_iter_t iter;
    bool ok = false;
    pb_ostream_t *stream = mf->SysexStream(pStream, pb_firmata_cmd_FM_PROTOBUF);
    if (pb_field_iter_begin(&iter, pb_rte_data_fields, &rte.data))
        ok = true;
    if (!ok) {
        logger.error("write_rte_info: %d", cmd.param);
        return -1;
    }
    if (pb_field_iter_find(&iter, cmd.tag)) {
        if (PB_ATYPE(iter.type) == PB_ATYPE_STATIC) {
            memcpy(iter.pData, cmd.data->bytes, cmd.data->size);
        } else if (PB_ATYPE(iter.type) == PB_ATYPE_POINTER) {
            memcpy(iter.pData, cmd.data->bytes, cmd.data->size);
        } else {
            logger.error("write_rte_info: %d", cmd.param);
        }
    }
    int res = pb_encode(stream, pb_rte_data_fields, &rte.data);
    if (!res) {
        const char *error = PB_GET_ERROR(stream);
        logger.error("write_rte_info encode error: %s", error);
    }
    mf->sendSysex(pStream, pb_firmata_cmd_FM_PROTOBUF, stream->bytes_written, mf->sendBuffer);
    return 0;
}

int mFirmata::get_tsdb_info(mFirmata *mf, nStream *pStream, pb_cmd cmd) {
#if defined(USE_TSDB_DATA) || defined(USE_TSDB_LOG)
    int res = 0;
    pb_tsdb_infos infos;
    pb_tsdb_info *info = (pb_tsdb_info *)malloc(sizeof(pb_tsdb_info) * rteConst.tsdb_nrs);
    for (int i = 0; i < rteConst.tsdb_nrs; i++)
    {
        TSDB *db = TSDB::db(i);
        info[i] = db->data;
    }
    infos.tsdb = info;
    infos.tsdb_count = rteConst.tsdb_nrs;
    pb_ostream_t *stream =   mf->SysexStream(pStream,pb_firmata_cmd_FM_PROTOBUF);
    res = pb_encode(stream, pb_tsdb_infos_fields, &infos);
    if (!res)
    {
        const char *error = PB_GET_ERROR(stream);
        logger.error("get_tsdb_info encode error: %s", error);
    }
    mf->sendSysex(pStream, pb_firmata_cmd_FM_PROTOBUF, stream->bytes_written, mf->sendBuffer);
#endif
    return 0;
}


pb_ostream_t *mFirmata::SysexStream(nStream *nstream, pb_firmata_cmd cmd) {
    pb_Stream = nstream;

#ifdef PB_BUFFER_ONLY
    /* In PB_BUFFER_ONLY configuration the callback pointer is just int*.
     * NULL pointer marks a sizing field, so put a non-NULL value to mark a buffer stream.
     */
    static const int marker = 0;
    stream.callback = &marker;
#else
    stream.callback = &buf_write;
#endif
    stream.state = sendBuffer;
    stream.max_size = FIRMATA_BUFFER_SZ;
    stream.bytes_written = 0;
#ifndef PB_NO_ERRMSG
    stream.errmsg = NULL;
#endif
    return &stream;
}

bool mFirmata::buf_write(pb_ostream_t *stream, const pb_byte_t *buf, size_t count) {
    if (stream->bytes_written + count >= stream->max_size) {
        // logger.error("buf_write: %d", count);
        return false;
    }
    pb_byte_t *dest = (pb_byte_t *) stream->state;
    stream->state = dest + count;

    memcpy(dest, buf, count * sizeof(pb_byte_t));

    return true;
}

int mFirmata::get_module_info(mFirmata *mf, nStream *pStream, pb_cmd cmd) {
    int res = 0;
#ifndef THIS_IS_BOOTLOADER
    pb_module_info *info = (pb_module_info *) malloc(sizeof(pb_module_info) * rteConst.module_nrs);
    for (int i = 0; i < rteConst.module_nrs; i++) {
        info[i].id = i;
        info[i].name = (char *) smodule::modules[i]->name();
        info[i].type = smodule::modules[i]->type();
        info[i].state = smodule::modules[i]->state;
    }
    pb_module_infos infos;
    infos.module = info;
    infos.module_count = rteConst.module_nrs;
    size_t size;
    if (!pb_get_encoded_size(&size, pb_module_infos_fields, &infos)) {
        logger.error("get_module_info pb_get_encoded_size error");
    }
    if (size > FIRMATA_BUFFER_SZ) {
        logger.error("get_module_info buffer size error");
    }
    pb_ostream_t *stream = mf->SysexStream(pStream, pb_firmata_cmd_FM_PROTOBUF);
    res = pb_encode(stream, pb_module_infos_fields, &infos);
    if (!res) {
        const char *error = PB_GET_ERROR(stream);
        logger.error("get_tsdb_info encode error: %s", error);
    }
    mf->sendSysex(pStream, pb_firmata_cmd_FM_PROTOBUF, stream->bytes_written, mf->sendBuffer);
#endif
    return 0;
}
