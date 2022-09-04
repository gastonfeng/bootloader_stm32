#ifdef USE_FIRMATA

#include "mFirmata.h"
#include "rtos.h"
#include "plc_const.h"

#if defined(RTE_APP) || defined(PLC)

#include <plc_rte.h>
#include <iec_types.h>
#include <kSerial.h>

#include "hwboard.h"

#endif

#include <ctime>
#include <rte_rtc.h>
#include <plc_var_class.h>

#ifdef USE_SERVO
#include <Servo.h>
#endif

#ifdef USE_FIRMATA_WIRE
#include <Wire.h>
#endif
#ifdef FIRMATA_SERIAL_FEATURE

#include <SerialFirmata.h>

SerialFirmata *serialFeature;
#endif

#ifdef USE_FREERTOS

#ifdef USE_FREERTOS

#include "STM32FreeRTOS.h"

#endif
#ifdef USE_FIRMATA_WIRE
#endif
bool isI2CEnabled;
int i2cReadDelayTime;
byte i2cRxData[8];
#endif
int queryIndex;
byte analogInputsToReport[(IO_XA_NRS + IO_YA_NRS) / 8 + 1];
byte reportPINs[(IO_YO_NRS + IO_XI_NRS + 7) / 8]; // 1 = report this port, 0 = silence
byte previousPINs[(IO_YO_NRS + IO_XI_NRS + 7) / 8];                       // previous 8 bits sent
bool isResetting;

#ifdef USE_SERVO
Servo servos[MAX_SERVOS];
byte servoPinMap[IO_YO_NRS + IO_XI_NRS + IO_XA_NRS + IO_YA_NRS];
byte detachedServos[MAX_SERVOS];
byte detachedServoCount = 0;
byte servoCount = 0;
#endif
/* pins configuration */
byte portConfigInputs[TOTAL_PORTS]; // each bit: 1 = pin in INPUT, 0 = anything else
#ifndef ARDUINO
// from arduino
#define INPUT 0
#define OUTPUT 1
#endif

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
void enableI2CPins(firmata::FirmataClass *fm)
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
using namespace std;

void setPinModeCallback(firmata::FirmataClass *fm, byte pin, int mode);

int dbg_size();

void reportAnalogCallback(firmata::FirmataClass *fm, Stream *stream, byte analogPin, int value) {
#if defined(RTE_APP) || defined(PLC)
    if (analogPin < ANALOGVALUE_LENGTH) {
        if (value == 0) {
            analogInputsToReport[analogPin / 8] &= ~(1 << (analogPin % 8));
        } else {
            analogInputsToReport[analogPin / 8] |= (1 << (analogPin % 8));
            // prevent during system reset or all analog pin values will be reported
            // which may report noise for unconnected analog pins
            if (!isResetting) {
                // Send pin value immediately. This is helpful when connected via
                // ethernet, wi-fi or bluetooth so pin states can be known upon
                // reconnecting.
                fm->sendAnalog(stream, analogPin, plcVar.analogValue(analogPin));
            }
        }
    }
#endif
}

void reportDigitalCallback(firmata::FirmataClass *fm, Stream *, byte port, int value) {
#if defined(RTE_APP) || defined(PLC)
    if (port < IO_XI_NRS + IO_YO_NRS) {
        reportPINs[port] = value;
        // Send port value immediately. This is helpful when connected via
        // ethernet, wi-fi or bluetooth so pin states can be known upon
        // reconnecting.
        //        if (value)
        //            board.outputPort(port, readPort(port, portConfigInputs[port]), true);
    }
    // do not disable analog reporting on these 8 pins, to allow some
    // pins used for digital, others analog.  Instead, allow both types
    // of reporting to be enabled, but check if the pin is configured
    // as analog when sampling the analog inputs.  Likewise, while
    // scanning digital pins, portConfigInputs will mask off values from any
    // pins configured as analog
#endif
}

void setPinValueCallback(firmata::FirmataClass *fm, Stream *, byte pin, int value) {
#if defined(RTE_APP) || defined(PLC)
    mFirmata *mfm = (mFirmata *) fm;
    if (pin < IO_YO_NRS + IO_XI_NRS + IO_XA_NRS + IO_YA_NRS) //&& fm->getPinMode(pin) == OUTPUT
    {
        mfm->setPinState(pin, value);
        board.outputPort(pin, value);
    }
#endif
}

void systemResetCallback(firmata::FirmataClass *fm, Stream *) {
#if defined(RTE_APP) || defined(PLC)
    isResetting = true;
    logger.debug("systemResetCallback");
#ifdef FIRMATA_SERIAL_FEATURE
    serialFeature->reset();
#endif
#ifdef USE_FIRMATA_WIRE
    if (isI2CEnabled)
    {
        disableI2CPins();
    }
#endif
    for (unsigned char &reportPIN: reportPINs)
        reportPIN = 0; // by default, reporting off
    // by default, do not report any analog inputs
    memset(analogInputsToReport, 0, sizeof(analogInputsToReport));

    /* send digital inputs to set the initial state on the host computer,
     * since once in the loop(), this firmware will only send on change */

    isResetting = false;
#endif
}

#ifdef USE_SERVO

void attachServo(firmata::FirmataClass *fm, Stream *fs, byte pin, int minPulse, int maxPulse)
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
        fm->sendString(fs, "Max servos attached");
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
#ifdef ARDUINO

void setPinModeCallback(firmata::FirmataClass *fm, Stream *Fs, byte pin, int mode) {
    if (fm->getPinMode(pin) == PIN_MODE_IGNORE)
        return;
#ifdef USE_FIRMATA_WIRE
    if (fm->getPinMode(pin) == PIN_MODE_I2C && isI2CEnabled && mode != PIN_MODE_I2C)
    {
        // disable i2c so pins can be used for other functions
        // the following if statements should reconfigure the pins properly
        disableI2CPins();
    }
#endif
#ifdef USE_SERVO
    if (IS_PIN_DIGITAL(pin) && mode != PIN_MODE_SERVO && servoPinMap[pin] < MAX_SERVOS &&
        servos[servoPinMap[pin]].attached())
    {
        detachServo(pin);
    }
#endif
    if (IS_PIN_ANALOG(pin)) {
        reportAnalogCallback(fm, Fs, PIN_TO_ANALOG(pin), mode == PIN_MODE_ANALOG ? 1 : 0); // turn on/off reporting
    }
    if (IS_PIN_DIGITAL(pin)) {
        if (mode == INPUT || mode == PIN_MODE_PULLUP) {
            portConfigInputs[pin / 8] |= (1 << (pin & 7));
        } else {
            portConfigInputs[pin / 8] &= ~(1 << (pin & 7));
        }
    }
    fm->setPinState(pin, 0);
    switch (mode) {
        case PIN_MODE_ANALOG:
            if (IS_PIN_ANALOG(pin)) {
                if (IS_PIN_DIGITAL(pin)) {
                    //
                }
                fm->setPinMode(pin, PIN_MODE_ANALOG);
            }
            break;
        case INPUT:
            if (IS_PIN_DIGITAL(pin)) {
                fm->setPinMode(pin, INPUT);
            }
            break;
        case PIN_MODE_PULLUP:
            if (IS_PIN_DIGITAL(pin)) {
                fm->setPinMode(pin, PIN_MODE_PULLUP);
                fm->setPinState(pin, 1);
            }
            break;
        case OUTPUT:
            if (IS_PIN_DIGITAL(pin)) {
                if (fm->getPinMode(pin) == PIN_MODE_PWM) {
                    // Disable PWM if pin mode was previously set to PWM.
                    digitalWrite(PIN_TO_DIGITAL(pin), LOW);
                }
                fm->setPinMode(pin, OUTPUT);
            }
            break;
        case PIN_MODE_PWM:
            if (IS_PIN_PWM(pin)) {
                analogWrite(PIN_TO_PWM(pin), 0);
                fm->setPinMode(pin, PIN_MODE_PWM);
            }
            break;
#ifdef USE_SERVO

            case PIN_MODE_SERVO:
                if (IS_PIN_DIGITAL(pin))
                {
                    fm->setPinMode(pin, PIN_MODE_SERVO);
                    if (servoPinMap[pin] == 255 || !servos[servoPinMap[pin]].attached())
                    {
                        // pass -1 for min and max pulse values to use default values set
                        // by Servo library
                        attachServo(fm, fs, pin, -1, -1);
                    }
                }
                break;
#endif
#ifdef USE_FIRMATA_WIRE
            case PIN_MODE_I2C:
                if (IS_PIN_I2C(pin))
                {
                    // mark the pin as i2c
                    // the user must call I2C_CONFIG to enable I2C for a device
                    fm->setPinMode(pin, PIN_MODE_I2C);
                }
                break;
#endif
        case PIN_MODE_SERIAL:
#ifdef FIRMATA_SERIAL_FEATURE
            serialFeature->handlePinMode(fm, pin, PIN_MODE_SERIAL);
#endif
            break;
        default:
            fm->sendString(Fs, "Unknown pin mode");
    }
}

#endif


int soem_scan(firmata::FirmataClass *fm, Stream *);

void analogWriteCallback(firmata::FirmataClass *fm, Stream *, byte i, int val) {
#if defined(RTE_APP) || defined(PLC)
    auto v = (u16) val;
    board.set_aout(i, &v);
#endif
}

#ifdef USE_LFS

#include "flashFs.h"

extern FlashFs flash_fs;
#endif

void stringCallback(firmata::FirmataClass *fc, Stream *Fs, char *myString) {
#ifdef USE_LFS
    if (strncmp(myString, "rm ", 3) == 0)
    {
        if (FlashFs::unlink(&myString[3]) == 0)
            fc->sendString(Fs, "rm ok");
        else
            fc->sendString(Fs, "rm fail");
    }
    else
#endif
    fc->sendString(Fs, "unknown input");
}

int decodeByteStream(size_t bytec, const byte *bytev, byte *buf) {
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

extern "C" void core_debug_uart(bool v);

int fill_dbg(int index, u8 *buf);

void set_dbg(u32 index, byte *varp, int len);

void sysexCallback(firmata::FirmataClass *fm, Stream *FirmataStream, byte command, uint16_t argc, byte *argv) {
    int len_data;
    int index;
    char *buffer;
    int state;
    u32 start, end;
    u32 indexv;
    int decodedLen;
    int len;
    u8 *data;
    tm new_time{};
    struct {
        u32 build;
        char name[8];
    } info{};
    byte decodeBuf[16];
    auto *mfm = (mFirmata *) fm;
//    logger.debug("sysexCallback: %d", command);
    switch (command) {
        case firmata::ARE_YOU_THERE:
#if defined(RTE_APP) || defined(PLC)
            // core_debug_uart(false);
            // logger.disable(logger_t::LOGGER_SERIAL);
#endif
            fm->write(FirmataStream, START_SYSEX);
            fm->write(FirmataStream, firmata::I_AM_HERE);
            fm->write(FirmataStream, 1); // arduino_instance_id
            fm->write(FirmataStream, END_SYSEX);
            fm->flush(FirmataStream);
            break;
        case firmata::I_AM_HERE:
            if (mfm->i_am_here_cb)
                mfm->i_am_here_cb(mfm, FirmataStream);
            break;
#if defined(RTE_APP) || defined(PLC)
        case SAMPLING_INTERVAL:
            if (argc > 1) {
                plc_var.info.samplingInterval = (byte) (argv[0] + (argv[1] << 7));
                if (plc_var.info.samplingInterval < plc_var.config.MINIMUM_SAMPLING_INTERVAL) {
                    plc_var.info.samplingInterval = plc_var.config.MINIMUM_SAMPLING_INTERVAL;
                }
            } else {
                // sendString("Not enough data");
            }
            break;
        case REPORT_ANALOG:
            byte bu[2];
            if (argc == 3) {
                decodeByteStream(argc, (const byte *) argv, bu);
                reportAnalogCallback(fm, FirmataStream, bu[0], bu[1]);
            }
            break;
        case EXTENDED_ANALOG:
            if (argc > 1) {
                int al = argv[1];
                if (argc > 2)
                    al |= (argv[2] << 7);
                if (argc > 3)
                    al |= (argv[3] << 14);
                analogWriteCallback(fm, FirmataStream, argv[0], al);
            }
            break;
        case CAPABILITY_QUERY:
            fm->write(FirmataStream, START_SYSEX);
            fm->write(FirmataStream, CAPABILITY_RESPONSE);
            board.capability_query(fm, FirmataStream);
            fm->write(FirmataStream, END_SYSEX);
            fm->flush(FirmataStream);
            break;
        case PIN_STATE_QUERY:
            if (argc > 0) {
                byte pin = argv[0];
                fm->write(FirmataStream, START_SYSEX);
                fm->write(FirmataStream, PIN_STATE_RESPONSE);
                fm->write(FirmataStream, pin);
                if (pin < IO_YO_NRS + IO_XI_NRS) {
                    fm->write(FirmataStream, fm->getPinMode(pin));
                    fm->write(FirmataStream, (byte) fm->getPinState(pin) & 0x7F);
                    if (fm->getPinState(pin) & 0xFF80)
                        fm->write(FirmataStream, (byte) (fm->getPinState(pin) >> 7) & 0x7F);
                    if (fm->getPinState(pin) & 0xC000)
                        fm->write(FirmataStream, (byte) (fm->getPinState(pin) >> 14) & 0x7F);
                }
                fm->write(FirmataStream, END_SYSEX);
                fm->flush(FirmataStream);
            }
            break;
        case ANALOG_MAPPING_QUERY:
            fm->write(FirmataStream, START_SYSEX);
            fm->write(FirmataStream, ANALOG_MAPPING_RESPONSE);
            board.analog_mapping_query(fm, FirmataStream);

            fm->write(FirmataStream, END_SYSEX);
            fm->flush(FirmataStream);
            break;
#ifdef FIRMATA_SERIAL_FEATURE
            case SERIAL_MESSAGE:
                serialFeature->handleSysex(fm, FirmataStream, command, argc, argv);
                break;
#endif
        case CB_GET_REMAIN_MEM:
            fm->sendSysex(FirmataStream, CB_GET_REMAIN_MEM, 2, (byte *) &plc_var.info.remain_mem);
            break;
        case CB_GET_RTE_VERSION:
            fm->sendSysex(FirmataStream, CB_GET_RTE_VERSION, sizeof(rte_ver_t), (uint8_t *) &plc_var.config.rte_ver);
            break;
        case CB_PLC_START:
            rte.app_start();
            fm->write(FirmataStream, START_SYSEX);
            fm->write(FirmataStream, CB_PLC_START);
            fm->write(FirmataStream, 0);
            fm->write(FirmataStream, END_SYSEX);
            fm->flush(FirmataStream);
            break;
        case CB_PLC_STOP:
            rte.app_stop();
            fm->write(FirmataStream, START_SYSEX);
            fm->write(FirmataStream, CB_PLC_STOP);
            fm->write(FirmataStream, 0);
            fm->write(FirmataStream, END_SYSEX);
            fm->flush(FirmataStream);
            break;
        case REPORT_PLC_MD5:
            if (plc_var.info.plc_curr_app)
                fm->sendSysex(FirmataStream, REPORT_PLC_MD5, 32, (byte *) plc_var.info.plc_curr_app->id);
            else
                fm->sendSysex(FirmataStream, REPORT_PLC_MD5, 0, (byte *) "");
            break;
        case CB_PLC_LOAD:
            fm->write(FirmataStream, START_SYSEX);
            fm->write(FirmataStream, CB_PLC_LOAD);
            fm->write(FirmataStream, 0);
            fm->write(FirmataStream, END_SYSEX);
            fm->flush(FirmataStream);
            rte.app_stop();
            app.unload();
            rte.load_app();
            break;
        case CB_PLC_REPAIR:
            rte.app_stop();
            app.unload();
            fm->write(FirmataStream, START_SYSEX);
            fm->write(FirmataStream, CB_PLC_REPAIR);
            fm->write(FirmataStream, 0);
            fm->write(FirmataStream, END_SYSEX);
            fm->flush(FirmataStream);
            break;
        case FM_FLASH_CLEAR:
            len = 0;
            fm->sendSysex(FirmataStream, FM_FLASH_CLEAR, 4, (byte *) &len);
            board.flashClear();
            hwboard::reset();
            break;
#endif
#if defined(USE_RTC) || defined(USE_PCF8563)
        case CB_GET_RTC:
            fm->sendSysex(FirmataStream, CB_GET_RTC, sizeof(rtc_t), (byte *) &plc_var.info.rtc);
            break;
        case CB_SET_RTC:
            new_time.tm_year = *(u16 *) &argv[0];
            new_time.tm_mon = argv[2];
            new_time.tm_mday = argv[3];
            new_time.tm_hour = argv[4];
            new_time.tm_min = argv[5];
            new_time.tm_sec = argv[6];
            new_time.tm_wday = argv[7];
            rtc.set_time(&new_time);
            fm->sendSysex(FirmataStream, CB_SET_RTC, 0, nullptr);
            break;
#endif
#ifdef ARDUINO
#ifdef USE_LWIP
#ifdef USE_IP_MODIFY
        case CB_SET_IP:
            byte ip[4];
            decodeByteStream(argc, (const byte *) argv, ip);
            plc_var.config.ip.ip1 = ip[0];
            plc_var.config.ip.ip2 = ip[1];
            plc_var.config.ip.ip3 = ip[2];
            plc_var.config.ip.ip4 = ip[3];
            ETH_LWIP::set_ip();
            fm->sendSysex(FirmataStream, CB_SET_IP, 4, (byte *) (&plc_var.config.ip));
            break;
#endif
        case CB_GET_IP:
            fm->sendSysex(FirmataStream, CB_GET_IP, 4, (byte *) (&plc_var.config.ip));
            break;
        case FM_GET_NET_BUF_STAT:
            buffer = (char *) malloc(13 * MEMP_MAX);
            for (int i = 0; i < MEMP_MAX; i++) {
                *(u8 *) &buffer[0 + 13 * i] = memp_pools[i]->stats->avail;
                *(u8 *) &buffer[1 + 13 * i] = memp_pools[i]->stats->err;
                *(u8 *) &buffer[2 + 13 * i] = memp_pools[i]->stats->illegal;
                *(u8 *) &buffer[3 + 13 * i] = memp_pools[i]->stats->max;
                *(u8 *) &buffer[4 + 13 * i] = memp_pools[i]->stats->used;
                memcpy(&buffer[5 + 13 * i], memp_pools[i]->stats->name, 8);
            }
            fm->sendSysex(FirmataStream, FM_GET_NET_BUF_STAT, 13 * MEMP_MAX, (byte *) buffer);
            free(buffer);
            break;
#endif
#ifdef USE_FREERTOS
        case CB_THREAD_INFO:
            typedef struct {
                u32 xTaskNumber;           /* A number unique to the task. */
                uint32_t ulRunTimeCounter; /* The total run time allocated to the task so far, as defined by the run time stats clock.  See http://www.freertos.org/rtos-run-time-stats.html.  Only valid when configGENERATE_RUN_TIME_STATS is defined as 1 in FreeRTOSConfig.h. */
                u32 pxStackBase;           /* Points to the lowest address of the task's stack area. */
                uint16_t usStackHighWaterMark;
                char name[8];
                uint16_t eCurrentState;     /* The state in which the task existed when the structure was populated. */
                uint16_t uxCurrentPriority; /* The priority at which the task was running (maybe inherited) when the structure was populated. */
                uint16_t uxBasePriority;    /* The priority to which the task will return if the task's current priority has been inherited to avoid unbounded priority inversion when obtaining a mutex.  Only valid if configUSE_MUTEXES is defined as 1 in FreeRTOSConfig.h. */
            } task_info;
            u32 task_num;
            u32 TotalRunTime;
            TaskStatus_t *StatusArray;
            task_num = uxTaskGetNumberOfTasks();
            buffer = (char *) malloc(task_num * sizeof(task_info));
            StatusArray = (TaskStatus_t *) pvPortMalloc(task_num * sizeof(TaskStatus_t));
            if (StatusArray != nullptr) {

                uxTaskGetSystemState(StatusArray, (UBaseType_t) task_num, (uint32_t *) &TotalRunTime);
                for (int i = 0; i < task_num; i++) {
                    auto *d = (task_info *) &buffer[i * sizeof(task_info)];
                    auto *s = (TaskStatus_t *) &StatusArray[i];
                    if (s) {
                        strcpy(d->name, s->pcTaskName);
                        d->xTaskNumber = s->xTaskNumber;
                        d->eCurrentState = s->eCurrentState;
                        d->pxStackBase = (u32) s->pxStackBase;
                        d->ulRunTimeCounter = s->ulRunTimeCounter;
                        d->uxBasePriority = s->uxBasePriority;
                        d->usStackHighWaterMark = s->usStackHighWaterMark;
                        d->uxCurrentPriority = s->uxCurrentPriority;
                    }
                }
            }
            fm->sendSysex(FirmataStream, CB_THREAD_INFO, (task_num * sizeof(task_info)), (byte *) (buffer));
            vPortFree(StatusArray);
            free(buffer);
            break;
#endif
#endif
#if defined(RTE_APP) || defined(PLC)
        case CB_SET_FORCE:
            for (int i = 0; i < argc;) {
                const u16 *byte = (u16 *) &argv[i];
                len = argv[i + 2];
                index = *byte;
                if (plc_var.info.plc_state == (u8) PLC_STATUS::Started) {
                    plc_var.info.plc_curr_app->dbg_set_force(index, len ? &argv[i + 3] : nullptr);
                }
                i += len + 3;
            }
            fm->write(FirmataStream, START_SYSEX);
            fm->write(FirmataStream, CB_SET_FORCE);
            fm->write(FirmataStream, argc);
            fm->write(FirmataStream, END_SYSEX);
            fm->flush(FirmataStream);
            break;

        case CB_CLEAR_V:
            if (plc_var.info.plc_curr_app && (plc_var.info.plc_state == (u8) PLC_STATUS::Started)) {
                plc_var.info.plc_curr_app->dbg_vars_reset(__IEC_DEBUG_FLAG);
                logger.debug("monitor var reset.");
            } else {
                logger.debug("monitor var not reset.plc_state=0x%x ", plc_var.info.plc_state);
            }
            fm->write(FirmataStream, START_SYSEX);
            fm->write(FirmataStream, CB_CLEAR_V);
            fm->write(FirmataStream, 0);
            fm->write(FirmataStream, END_SYSEX);
            fm->flush(FirmataStream);
            break;
        case CB_SET_V:
            if (argc > 2) {
                byte *bufs;
                bufs = (byte *) malloc(argc);
                decodedLen = decodeByteStream(argc, argv, bufs);
                // logger.debug("set_v %d -> %d", argc, decodedLen);
                for (int i = 0; i < decodedLen; i += 2) {
                    const u16 *byte = (u16 *) &bufs[i];
                    indexv = *byte;
                    // logger.debug("%d %d", i, indexv);
                    if (plc_var.info.plc_curr_app) {
                        plc_var.info.plc_curr_app->dbg_var_register(indexv);
                    }
                }
                fm->write(FirmataStream, START_SYSEX);
                fm->write(FirmataStream, CB_SET_V);
                fm->write(FirmataStream, decodedLen);
                fm->write(FirmataStream, END_SYSEX);
                fm->flush(FirmataStream);
                free(bufs);
            }
            break;
        case CB_GET_V:
            len = 0;
            data = (u8 *) malloc(512);
            if (plc_var.info.plc_state == (u8) PLC_STATUS::Started) {
                void *b = &data[4];
                plc_var.info.plc_curr_app->dbg_data_get((u32 *) &data[0], (u32 *) &len, (void **) &b);
                memcpy(&data[4], b, len);
                plc_var.info.plc_curr_app->dbg_data_free();
            }
            fm->sendSysex(FirmataStream, CB_GET_V, len + 4, data);
            free(data);
            break;
#endif
#ifdef ARDUINO
        case CB_SET_SERIAL_RX:

            // port = *(uint16_t *) argv;
            //            kSerial::get_serial(port)->set_rx();
            break;
        case CB_SET_SERIAL_TX_HIGH:

            // port1 = *(uint16_t *) argv;
            //            kSerial::get_serial(port1)->set_high();
            break;
        case CB_SET_SERIAL_TX_LOW:

            // port2 = *(uint16_t *) argv;
            //            kSerial::get_serial(port2)->set_low();
            break;
#endif
#if defined(RTE_APP) || defined(PLC)
        case FM_GET_TASK_NRS:
            fm->sendSysex(FirmataStream, FM_GET_TASK_NRS, 1, &(plc_var.info.plc_task_cnt));
            break;
        case FM_GET_TASK_NAME:
            if (tasks[argv[0]]) {
                fm->sendSysex(FirmataStream, FM_GET_TASK_NAME, (byte) strlen(tasks[argv[0]]->name),
                              (byte *) tasks[argv[0]]->name);
            }
            break;
        case FM_GET_TASK_DETAIL:
            if (tasks[argv[0]]) {
                fm->sendSysex(FirmataStream, FM_GET_TASK_DETAIL, 24, tasks[argv[0]]->mata());
            }
            break;
        case FM_GET_PLC_STATE:
            fm->sendSysex(FirmataStream, FM_GET_PLC_STATE, 1, (byte *) (&plc_var.info.plc_state));
            break;
        case FM_GET_PLC_INFO:
            if (plc_var.info.plc_curr_app) {
                info.build = plc_var.info.plc_curr_app->buildnumber;
                strcpy(info.name, plc_var.info.plc_curr_app->app_name);
                fm->sendSysex(FirmataStream, FM_GET_PLC_INFO, sizeof(info), (byte *) &info);
            } else
                fm->sendSysex(FirmataStream, FM_GET_PLC_INFO, 0, (byte *) &info);
            break;
        case CB_GET_LOG_NUMBER:
            fm->sendSysex(FirmataStream, CB_GET_LOG_NUMBER, 5, (byte *) (&plc_var.info.plc_state));
            break;
        case CB_GET_LOG:
            fm->write(FirmataStream, START_SYSEX);
            fm->write(FirmataStream, CB_GET_LOG);
            fm->write(FirmataStream, 0);
            fm->write(FirmataStream, END_SYSEX);
            fm->flush(FirmataStream);
            break;
#ifdef USE_BOOTLOADER
            case CB_GET_BOOT_VERSION:
#ifdef BOOTINFO
                boot_t *b;
                b = (boot_t *)BOOTINFO; // platformio.ini中定义
                if (b)
                    fm->sendSysex(FirmataStream, CB_GET_BOOT_VERSION, sizeof(boot_t), (byte *)b);
                else
#endif
                {
                    fm->write(FirmataStream, START_SYSEX);
                    fm->write(FirmataStream, CB_GET_BOOT_VERSION);
                    fm->write(FirmataStream, 0);
                    fm->write(FirmataStream, END_SYSEX);
                    fm->flush(FirmataStream);
                }
                break;
            case FM_FLASH_BOOT:
                len = board.updateBootbin();
                fm->sendSysex(FirmataStream, FM_FLASH_BOOT, len, (byte *)&len);
                break;
#endif
#endif
#ifdef USE_KVDB
        case FM_LIST_KEY:
            kvdb.list(fm, FirmataStream);
            break;
        case CB_READ_KEY:
            size_t vlen;
            buffer = kvdb.get((const char *) argv);
            vlen = strlen(buffer);
            if (buffer && (vlen > 0))
                fm->sendSysex(FirmataStream, CB_READ_KEY, (byte) vlen, (byte *) buffer);
            else
                fm->sendSysex(FirmataStream, CB_READ_KEY, 0, (byte *) buffer);
            break;
        case FM_READ_KEY_BYTES:
            data = (u8 *) malloc(256);
            len = kvdb.get((const char *) argv, (char *) data + 4, 256, (u32 *) data);
            fm->sendSysex(FirmataStream, FM_READ_KEY_BYTES, len + 4, (byte *) data);
            free(data);
            break;
        case CB_WRITE_KEY:
            size_t key_len;
            key_len = strlen((const char *) argv);
            int rw;
            rw = kvdb.set((const char *) argv, (const char *) argv + key_len + 1, (int) (argc - key_len - 2));
            fm->sendSysex(FirmataStream, CB_WRITE_KEY, 4, (byte *) &rw);
            break;
        case FM_WRITE_KEY_BYTES:
            decodedLen = decodeByteStream(argc, argv, decodeBuf);
            key_len = strlen((const char *) decodeBuf);
            uint32_t type;
            type = *(uint32_t *) (decodeBuf + key_len + 1);
            len = kvdb.set((const char *) decodeBuf, (const char *) (decodeBuf + key_len + 1 + 4),
                           (int) (decodedLen - key_len - 1 - 4), type);
            fm->sendSysex(FirmataStream, FM_WRITE_KEY_BYTES, 4, (byte *) &len);
            break;
        case CB_RM_KEY:
            kvdb.remove((const char *) argv);
            fm->sendSysex(FirmataStream, CB_RM_KEY, 0, argv);
            break;
        case CB_SET_TSL_RANGE:
            tsl_query q;
            decodedLen = decodeByteStream(argc, argv, decodeBuf);
            key_len = strlen((const char *) decodeBuf);
            if (decodedLen == key_len + 13) {
                start = *(u32 *) &decodeBuf[key_len + 1];
                end = *(u32 *) &decodeBuf[key_len + 1 + 4];
                state = (int) *(u32 *) &decodeBuf[key_len + 1 + 8];
                memset(&q, 0, sizeof(q));
                tsdb.query((const char *) decodeBuf, start, end, (fdb_tsl_status) (state), &q);
            }
            fm->sendSysex(FirmataStream, CB_SET_TSL_RANGE, sizeof(tsl_query), (byte *) &q);
            break;
        case CB_SET_TSL_STATUS:
            decodedLen = decodeByteStream(argc, argv, decodeBuf);
            key_len = strlen((const char *) decodeBuf);
            len = -1;
            if (decodedLen == key_len + 13) {
                start = *(u32 *) &decodeBuf[key_len + 1];
                end = *(u32 *) &decodeBuf[key_len + 1 + 4];
                state = (int) *(u32 *) &decodeBuf[key_len + 1 + 8];
                len = tsdb.set_status((const char *) decodeBuf, start, end, (fdb_tsl_status) (state));
            }
            fm->sendSysex(FirmataStream, CB_SET_TSL_STATUS, 4, (byte *) &len);
            break;
        case CB_GET_TSL:
            char *tbuf;
            int tlen;
            tbuf = (char *) malloc(256);
            memset(tbuf, 0, 256);
            tlen = tsdb.query_read((const char *) argv, (u32 *) &tbuf[0], (fdb_time_t *) &tbuf[4],
                                   (int *) (tbuf + 8),
                                   tbuf + 12, 256 - 12);
            if (tlen < 0)
                tlen = 0;
            else {
                *(u32 *) &tbuf[tlen] = GenerateCRC32Sum((const u8 *) tbuf, tlen, 0);
                tlen += 4;
            }
            fm->sendSysex(FirmataStream, CB_GET_TSL, (byte) tlen, (byte *) tbuf);
            free(tbuf);
            break;
        case CB_TSL_CLEAR:
            key_len = strlen((const char *) argv);
            // index = *(int *) &argv[key_len + 1];

            fm->sendSysex(FirmataStream, CB_TSL_CLEAR, (byte) sizeof(state), (byte *) &state);
            state = TSL::clear((const char *) (argv));
            break;
#endif
#ifdef USE_SOEM
            case FM_SOEM_SCAN:
                soem_scan(fm, FirmataStream);
                break;
#endif
#if defined(RTE_APP) || defined(PLC)
        case CB_SET_PLC_FILE:
            app.setPLCDLL((char *) argv);
            fm->write(FirmataStream, START_SYSEX);
            fm->write(FirmataStream, CB_SET_PLC_FILE);
            fm->write(FirmataStream, 0);
            fm->write(FirmataStream, END_SYSEX);
            fm->flush(FirmataStream);
            break;
        case CB_CPU_USAGE:
            fm->sendSysex(FirmataStream, CB_CPU_USAGE, 1, (byte *) &plc_var.info.cpu_usage);
            break;
#endif
#ifdef USE_WIFI
            case CB_WIFI_LIST:
                fm->sendSysex(FirmataStream, CB_WIFI_LIST, plc_var.info.wifi_size, (byte *)&plc_var.info.wifi_list);
                break;
            case CB_WIFI_SET_PASS:
                int plen;
                plen = strlen((const char *)argv);
                kvdb.set("wifi_pass", (const char *)argv, plen);
                wifi_reload();
                fm->sendSysex(FirmataStream, CB_WIFI_SET_PASS, 0, argv);
                break;
#endif
#if defined(RTE_APP) || defined(PLC)
#ifndef PLC
        case FM_PUT_DATA_BLOCK:
            u32 crc, crc_r;
            byte *buffer_data;
            if (argc < 12)
                break;
            buffer_data = (byte *) malloc(argc);

            len_data = decodeByteStream(argc, argv, buffer_data);
            crc = GenerateCRC32Sum((const u8 *) buffer_data, len_data - 4, 0);
            crc_r = *(u32 *) (&buffer_data[len_data - 4]);
            if (crc_r != crc) {
                state = CRC_ERROR;
                logger.error("crc error block %d 0x%x 0x%x argc=%d len=%d", *(int *) &buffer_data[0], crc, crc_r, argc,
                             len_data);
            } else {
                rte.set_state(PLC_STATUS::APP_FLASH_BEGIN);
                int block = *(int *) &buffer_data[0];
                if (block == 0) {
                    // if (ifirmata.dev) {
                    //     state = DEV_IS_OPEN;
                    // } else
                    {
                        u32 object = *(u32 *) &buffer_data[4];

                        ifirmata.dev = mem_block::mems[object];
                        if (!ifirmata.dev) {
                            state = NO_DEVICE;
                        } else {
                            state = ifirmata.dev->begin(&buffer_data[8], len_data - 8);
                            if (state > 0 && state > ifirmata.parser.dataBufferSize * 7 / 8 - 4) {
                                state = (int) (ifirmata.parser.dataBufferSize * 7 / 8 - 4);
                            }
                            logger.info("recv %s ,size= %d", &buffer_data[12], *(u32 *) &buffer_data[8]);
                        }
                    }
                } else if (block == -1) {
                    if (ifirmata.dev) {
                        if (ifirmata.dev->Shutdown() < 0) {
                            state = DEVICE_SHUTDOWN_ERR;
                        } else {
                            state = 1;
                            rte.set_state(PLC_STATUS::APP_FLASH_END);
                            logger.info("recv end.");
                            ifirmata.dev = nullptr;
                        }
                    }
                } else {
                    if (ifirmata.dev) {
                        if (ifirmata.dev->Write(&buffer_data[4], len_data - 8) < 0) {
                            state = DEVICE_WRITE_ERR;
                        } else {
                            state = block;
                        }
                        logger.info("recv %d ,size= %d", block, len_data - 8);
                    }
                }
            }
            fm->sendSysex(FirmataStream, FM_PUT_DATA_BLOCK, 4, (byte *) &state);
            free(buffer_data);
            break;
#endif
        case FM_GET_LOC_SIZE:
            if (plc_var.info.plc_curr_app) {
                fm->sendSysex(FirmataStream, FM_GET_LOC_SIZE, 2, (byte *) &plc_var.info.plc_curr_app->l_sz);
            } else {
                fm->sendSysex(FirmataStream, FM_GET_LOC_SIZE, 0, (byte *) &plc_var.info.plc_curr_app->l_sz);
            }
            break;
        case FM_GET_LOC:
            u32 l_index;
            if (argc == 5) {
                decodeByteStream(argc, argv, (byte *) &l_index);
                if (plc_var.info.plc_curr_app && l_index < plc_var.info.plc_curr_app->l_sz) {
                    plc_loc_tbl_t loc = plc_var.info.plc_curr_app->l_tab[l_index];
                    len = (int) sizeof(plc_loc_dsc_t) + loc->a_size + loc->v_size;
                    buffer = (char *) malloc(len);
                    buffer[0] = loc->v_type;
                    buffer[1] = loc->v_size;
                    *(u16 *) &buffer[2] = loc->proto;
                    *(u16 *) &buffer[4] = loc->a_size;
                    memcpy(&buffer[6], loc->a_data, loc->a_size);
                    memcpy(&buffer[6 + loc->a_size], loc->v_buf, loc->v_size);
                    fm->sendSysex(FirmataStream, FM_GET_LOC, len, (byte *) buffer);
                    free(buffer);
                    break;
                }
            }
            fm->sendSysex(FirmataStream, FM_GET_LOC, 0, (byte *) &plc_var.info.plc_curr_app->l_sz);
            break;
        case FM_SET_LOC:
            if (argc == 5) {
                decodeByteStream(argc, argv, (byte *) &l_index);
                if (plc_var.info.plc_curr_app && l_index < plc_var.info.plc_curr_app->l_sz) {
                    fm->sendSysex(FirmataStream, FM_SET_LOC, sizeof(plc_loc_tbl_t),
                                  (byte *) &plc_var.info.plc_curr_app->l_tab[l_index]);
                    break;
                }
            }
            fm->sendSysex(FirmataStream, FM_SET_LOC, 0, (byte *) &plc_var.info.plc_curr_app->l_sz);
            break;
#endif
#ifdef ONLINE_DEBUG
            case FM_GET_DBG_SIZE:
                if (plc_var.info.plc_curr_app) {
                    fm->sendSysex(FirmataStream, FM_GET_DBG_SIZE, 4,
                                  (byte *) &plc_var.info.plc_curr_app->data->size_dbgvardsc);
                } else {
                    fm->sendSysex(FirmataStream, FM_GET_DBG_SIZE, 0, nullptr);
                }
                break;
            case FM_GET_DBG:
                len = 0;
                if (argc == 5) {
                    decodeByteStream(argc, argv, (byte *) &l_index);
                    if (plc_var.info.plc_curr_app && l_index < plc_var.info.plc_curr_app->data->size_dbgvardsc) {
                        len = (int) fill_dbg((int) l_index, decodeBuf);
                    }
                }
                fm->sendSysex(FirmataStream, FM_GET_DBG, len, decodeBuf);
                break;
            case FM_SET_DBG:
                len = 0;
                if (argc > 5) {
                    decodedLen = decodeByteStream(argc, argv, (byte *) &decodeBuf);
                    l_index = *(u32 *) decodeBuf;
                    if (plc_var.info.plc_curr_app && l_index < plc_var.info.plc_curr_app->data->size_dbgvardsc) {
                        set_dbg(l_index, &decodeBuf[4], decodedLen - 4);
                        len = (int) fill_dbg((int) l_index, decodeBuf);
                    }
                }
                fm->sendSysex(FirmataStream, FM_GET_DBG, len, decodeBuf);
                break;
#endif
#if defined(RTE_APP) || defined(PLC)
        case FM_LOG_SET_LEVEL:
            plc_var.config.log_level = argv[0];
            fm->sendSysex(FirmataStream, FM_LOG_SET_LEVEL, 1, &plc_var.config.log_level);
            break;
#endif
#ifdef ARDUINO_ARCH_STM32
        case FM_GET_CPU_SN:
            u32 sn[3];
            sn[0] = HAL_GetUIDw0();
            sn[1] = HAL_GetUIDw1();
            sn[2] = HAL_GetUIDw2();
            fm->sendSysex(FirmataStream, FM_GET_CPU_SN, 12, (byte *) sn);
            break;
#endif
#if defined(RTE_APP) || defined(PLC)
        case FM_READ_MEM:
            decodedLen = decodeByteStream(argc, argv, decodeBuf);
            indexv = 0;
            len = 0;
            if (decodedLen == 6) {
                indexv = *(u32 *) decodeBuf;
                len = *(u16 *) &decodeBuf[4];
            }
            fm->sendSysex(FirmataStream, FM_READ_MEM, len, (byte *) indexv);
            break;
        case FM_WRITE_MEM:
            if (argc > 6) {
                buffer = (char *) malloc(argc);
                decodedLen = decodeByteStream(argc, argv, (byte *) buffer);
                indexv = *(u32 *) buffer;
                len = *(u16 *) &buffer[4];
                for (int i = 0; i < len; i++) {
                    *((uint8_t *) indexv + i) = buffer[6 + i];
                }
                fm->sendSysex(FirmataStream, FM_WRITE_MEM, len, (byte *) indexv);
                free(buffer);
            }
            break;
        case FM_READ_VALUE:
            u8 region, typ;
            byte *rdbuf;
            rdbuf = (byte *) malloc(FirmataStream->tx_max_size());
            decodedLen = decodeByteStream(argc, argv, rdbuf);
            indexv = 0;
            len = 0;
            if (decodedLen == 8 && rdbuf[0] <= REGION_HOLDER) {
                region = rdbuf[0];
                indexv = *(u32 *) &rdbuf[1];
                typ = rdbuf[5];
                len = *(u16 *) &rdbuf[6];
                rdbuf[0] = region;
                rdbuf[1] = typ;
                *(u32 *) &rdbuf[2] = indexv;
                const char *p;
                switch (region) {
                    default:
                    case REGION_XI: // byte from 0
                    case REGION_DIGITAL: // digitalValue
                        p = (const char *) &plc_var.digitalValue;
                        break;
                    case REGION_16: // analogValue
                        p = (const char *) &plc_var.analogValue;
                        break;
                    case REGION_32: // analogValue32
                        p = (const char *) &plc_var.analogValue32;
                        break;
                    case REGION_HOLDER: // holdValue
                        p = (const char *) &plc_var.holdValue;
                        break;
                    case REGION_INFO:
                        p = (const char *) &plc_var.info;
                        break;
                    case REGION_CONFIG:
                        p = (const char *) &plc_var.config;
                        break;
                }
                memcpy(&rdbuf[6], &p[indexv], len);
                fm->sendSysex(FirmataStream, FM_READ_VALUE_REP, len + 6, (byte *) rdbuf);
            } else
                fm->sendSysex(FirmataStream, FM_READ_VALUE_REP, len, ((byte *) &plc_var) + indexv);
            free(rdbuf);
            break;
        case FM_WRITE_VALUE:
            if (argc > 7) {
                buffer = (char *) malloc(argc);
                decodedLen = decodeByteStream(argc, argv, (byte *) buffer);
                region = buffer[0];
                indexv = *(u32 *) buffer[1];
                len = *(u16 *) &buffer[5];
                const char *p;
                switch (region) {
                    default:
                    case REGION_XI: // byte from 0
                    case REGION_DIGITAL: // digitalValue
                        p = (const char *) &plc_var.digitalValue;
                        break;
                    case REGION_16: // analogValue
                        p = (const char *) &plc_var.analogValue;
                        break;
                    case REGION_32: // analogValue32
                        p = (const char *) &plc_var.analogValue32;
                        break;
                    case REGION_HOLDER: // holdValue
                        p = (const char *) &plc_var.holdValue;
                        break;
                    case REGION_INFO:
                        p = (const char *) &plc_var.info;
                        break;
                    case REGION_CONFIG:
                        p = (const char *) &plc_var.config;
                        break;
                }
                memcpy((void *) &p[indexv], (void *) &buffer[indexv], len);
                fm->sendSysex(FirmataStream, FM_WRITE_VALUE_REP, 7, (byte *) buffer);
                free(buffer);
            }
            break;
        case FM_READ_BIT:
            decodedLen = decodeByteStream(argc, argv, decodeBuf);
            indexv = 0;
            len = 0;
            if (decodedLen == 6) {
                indexv = *(u32 *) decodeBuf;
                len = *(u16 *) &decodeBuf[4];
            }
            if (len > 0) {
                buffer = (char *) malloc(len / 8 + 6);
                *(u32 *) buffer = indexv;
                buffer[4] = len;
                for (int i = 0; i < len; i++) {
                    u8 b = plcVar.digitalValue(indexv + i);
                    buffer[i / 8 + 5] |= b << (i % 8);
                }
                fm->sendSysex(FirmataStream, FM_READ_BIT_REP, len / 8 + 6, (byte *) buffer);
                free(buffer);
            }
            break;
        case FM_WRITE_BIT:
            if (argc > 6) {
                buffer = (char *) malloc(argc);
                decodedLen = decodeByteStream(argc, argv, (byte *) buffer);
                indexv = *(u32 *) buffer;
                len = *(u16 *) &buffer[4];
                if (len < (argc - 6)) {
                    for (int i = 0; i < len; i++) {
                        *(((uint8_t *) &plc_var) + indexv + i) = buffer[6 + i];
                    }
                }
                fm->sendSysex(FirmataStream, FM_WRITE_VALUE_REP, len,
                              (byte *) ((uint8_t *) &plc_var) + indexv);
                free(buffer);
            }
            break;
#endif
        case FM_READ_VALUE_REP:
            if (argc > 0) {
                decodedLen = decodeByteStream(argc, argv, decodeBuf);
                memcpy(mfm->valueBuf, decodeBuf, decodedLen);
                mfm->valueLen = decodedLen;
            }
            break;
        case FM_WRITE_VALUE_REP:
            if (argc > 0 && argc < 8) {
                decodedLen = decodeByteStream(argc, argv, decodeBuf);
            }
            break;
        case FM_READ_BIT_REP:
            if (argc > 0) {
                decodedLen = decodeByteStream(argc, argv, decodeBuf);
                memcpy(mfm->valueBuf, decodeBuf, decodedLen);
                mfm->valueLen = decodedLen;
            }
            break;
        case FM_WRITE_BIT_REP:
            if (argc > 0 && argc < 8) {
                decodedLen = decodeByteStream(argc, argv, decodeBuf);
            }
            break;
#if defined(RTE_APP) || defined(PLC)
        case FM_READ_LOC:
            if (argc < 5)
                break;
            decodedLen = decodeByteStream(argc, argv, (byte *) decodeBuf);
            if (decodedLen < 5)
                break;
            len = board.get_input(decodeBuf[0], decodeBuf[1], decodeBuf[2], decodeBuf[3],
                                  (char *) decodeBuf);
            fm->sendSysex(FirmataStream, FM_READ_LOC, len, (byte *) decodeBuf);
            break;
        case FM_WRITE_LOC:
            decodedLen = decodeByteStream(argc, argv, (byte *) decodeBuf);
            len = board.set_output(decodeBuf[0], decodeBuf[1], decodeBuf[2], decodeBuf[3],
                                   (char *) &decodeBuf[4],
                                   decodedLen - 4);
            fm->sendSysex(FirmataStream, FM_READ_LOC, len, (byte *) decodeBuf);
            break;
        case FM_GET_LOCATION:
            byte *buf_fgl;
            buf_fgl = (byte *) malloc(32);
            decodedLen = decodeByteStream(argc, argv, decodeBuf);
            len = 0;
            if (decodedLen >= 4) {
                len = board.get_input(decodeBuf[1], decodeBuf[2], decodeBuf[3], 0, (char *) buf_fgl);
            }
            fm->sendSysex(FirmataStream, FM_GET_LOCATION, len, (byte *) buf_fgl);
            free(buf_fgl);
            break;
        case FM_SET_LOCATION:
            decodedLen = decodeByteStream(argc, argv, decodeBuf);
            len = -1;
            if (decodedLen >= 6) {
                len = board.set_output(decodeBuf[1], decodeBuf[2], decodeBuf[3], 0, (char *) &decodeBuf[5],
                                       decodeBuf[4]);
            }
            fm->sendSysex(FirmataStream, FM_SET_LOCATION, 4, (byte *) &len);
            break;
#endif
        case CB_GOTO_IAP:
            fm->sendSysex(FirmataStream, CB_GOTO_IAP, 0, nullptr);
            boardBase::goto_iap();
        case CB_RESET:
            len = 0;
            fm->sendSysex(FirmataStream, CB_RESET, 4, (byte *) &len);
            RTE::reset();
            break;
        default:
            len = -1;
            fm->sendSysex(FirmataStream, command, 4, (byte *) &len);
            break;
    }
    mfm->set_flag(command);
}

void digitalWriteCallback(firmata::FirmataClass *fm, Stream *FirmataStream, byte port, int value) {
#if defined(RTE_APP) || defined(PLC)
    byte lastPin, pinValue, mask = 1, pinWriteMask = 0;

    if (port < IO_XI_NRS + IO_YO_NRS) {
        // create a mask of the pins on this port that are writable.
        lastPin = port * 8 + 8;
        if (lastPin > IO_YO_NRS + IO_XI_NRS + IO_XA_NRS + IO_YA_NRS)
            lastPin = IO_YO_NRS + IO_XI_NRS + IO_XA_NRS + IO_YA_NRS;
#ifdef ARDUINO
        for (byte pin = port * 8; pin < lastPin; pin++) {
            // do not disturb non-digital pins (eg, Rx & Tx)
            if (IS_PIN_DIGITAL(pin) && (fm->getPinMode(pin) == OUTPUT || fm->getPinMode(pin) == INPUT)) {
                pinValue = ((byte) value & mask) ? 1 : 0;
                if (fm->getPinMode(pin) == OUTPUT) {
                    pinWriteMask |= mask;
                } else if (fm->getPinMode(pin) == INPUT && pinValue == 1 && fm->getPinState(pin) != 1) {
                    // only handle INPUT here for backwards compatibility
#if ARDUINO > 100
                    // pinMode(pin, INPUT_PULLUP);
#else
                    // only write to the INPUT pin to enable pullups if Arduino v1.0.0 or earlier
                    pinWriteMask |= mask;
#endif
                }
                board.setPinState(fm, FirmataStream, pin, pinValue);
            }
            mask = (byte) (mask << 1);
        }
        writePort(port, (byte) value, pinWriteMask);
#endif
    }
#endif
}

int mFirmata::loop(Stream *FirmataStream) {
    while (available(FirmataStream)) {
        processInput(FirmataStream);
        plc_var.info.task_busy |= 0x2;
        last_tick = rtos::ticks();
    }
#if defined(RTE_APP) || defined(PLC)
    report(FirmataStream);
#endif
    return 0;
}

mFirmata::mFirmata() {
    disableBlinkVersion();
    setFirmwareVersion(FIRMATA_FIRMWARE_MAJOR_VERSION, FIRMATA_FIRMWARE_MINOR_VERSION);
    attach(STRING_DATA, stringCallback);
    attach(START_SYSEX, sysexCallback);
    //    attach(ANALOG_MESSAGE, analogWriteCallback);
    //    attach(DIGITAL_MESSAGE, digitalWriteCallback);
    //    attach(REPORT_ANALOG, reportAnalogCallback);
    //    attach(REPORT_DIGITAL, reportDigitalCallback);
    //    attach(SET_DIGITAL_PIN_VALUE, setPinValueCallback);
    //    attach(SET_PIN_MODE, setPinModeCallback);
    //    attach(SYSTEM_RESET, systemResetCallback);
    i_am_here_cb = nullptr;
#ifdef FIRMATA_SERIAL_FEATURE
    serialFeature = new SerialFirmata();
#endif
}

#if defined(RTE_APP) || defined(PLC)

void mFirmata::report(Stream *FirmataStream) {
    u32 currentMillis = rtos::ticks();

    if (currentMillis - previousMillis > plc_var.config.reportInterval) {
        previousMillis += plc_var.config.reportInterval;
        /* ANALOGREAD - do all analogReads() at the configured sampling interval */
        board.readAnalogValue(this, FirmataStream, analogInputsToReport, sizeof(analogInputsToReport));
        for (byte pin = 0; pin < IO_XI_NRS + IO_YO_NRS; pin++) {
            if (reportPINs[pin]) {
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

void mFirmata::outputPort(Stream *FirmataStream, byte portNumber, byte portValue, byte forceSend) {
    // pins not configured as INPUT are cleared to zeros
    //    portValue = portValue & portConfigInputs[portNumber];
    // only send if the value is different than previously sent
    if (portNumber < (IO_XI_NRS + IO_YO_NRS) &&
        (forceSend || (previousPINs[portNumber / 8] & (1 << (portNumber % 8))) != portValue)) {
        sendDigitalPort(FirmataStream, portNumber, portValue);
        flush(FirmataStream);
        if (portValue == 0) {
            previousPINs[portNumber / 8] &= ~(1 << (portNumber % 8));
        } else {
            previousPINs[portNumber / 8] |= (1 << (portNumber % 8));
        }
    }
}

#ifdef windows_x86
#include <pthread.h>
#include <minwindef.h>
#include <profileapi.h>
#include <sysinfoapi.h>
u32 ticks()
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
void Delay(u32 ms)
{
    struct timespec interval
    {
    };
    interval.tv_nsec = ms * 1000000;
    pthread_delay_np(&interval);
}
#endif

int mFirmata::setValue(Stream *FirmataStream, int index, void *valBuf, u8 size) {
    byte *buf;
    buf = (byte *) malloc(size + 4);
    *(int *) buf = index;
    memcpy(&buf[4], valBuf, size);
    clr_flag(FM_WRITE_VALUE_REP);
    sendSysex(FirmataStream, FM_WRITE_VALUE, size + 4, (byte *) buf);
    free(buf);
    u32 tick = rtos::ticks() + 1000;
    while (get_flag(FM_WRITE_VALUE_REP) == 0) {
        rtos::Delay(1);
        if (rtos::ticks() > tick)
            return TIMEOUT;
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
    if (cmd < FM_LAST) {
        respose[cmd / 8] |= (1 << (cmd % 8));
        return 0;
    }
    return -1;
}

int mFirmata::getValue(Stream *pStream, int index, u8 *value_buf, u16 len) {
    u8 buf[6];
    *(int *) buf = index;
    *(u16 *) &buf[4] = len;
    clr_flag(FM_READ_VALUE_REP);
    valueLen = 0;
    sendSysex(pStream, FM_READ_VALUE, 6, (byte *) buf);
    u32 tick = rtos::ticks() + 1000;
    while (get_flag(FM_READ_VALUE_REP) == 0) {
        rtos::Delay(1);
        if (rtos::ticks() > tick)
            return TIMEOUT;
    }
    if (valueLen == len) {
        memcpy(value_buf, valueBuf, valueLen);
        return valueLen;
    }
    return -1;
}

int mFirmata::getBit(Stream *pStream, int index, u8 *value_buf, u16 len) {
    u8 buf[6];
    *(int *) buf = index;
    *(u16 *) &buf[4] = len;
    clr_flag(FM_READ_BIT_REP);
    valueLen = 0;
    sendSysex(pStream, FM_READ_BIT, 6, (byte *) buf);
    u32 tick = rtos::ticks() + 1000;
    while (get_flag(FM_READ_BIT_REP) == 0) {
        rtos::Delay(1);
        if (rtos::ticks() > tick)
            return TIMEOUT;
    }
    if (valueLen == len) {
        for (int i = 0; i < len; i++) {
            value_buf[i] = (valueBuf[i / 8] & (1 << (i % 8))) ? 1 : 0;
        }
        return valueLen;
    }
    return -1;
}

int mFirmata::run(u32 tick) {
    if (tick > (last_tick + 60000)) {
        plc_var.info.task_busy &= (~0x2);
    }
    return 0;
}

int mFirmata::getPinState(byte pin) {
    return plcVar.digitalValue(pin);
}

void mFirmata::setPinState(byte pin, int state) {
    plcVar.digitalValue(pin, state);
}

#endif