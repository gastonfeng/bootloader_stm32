#include "mFirmata.h"
#include "plc_const.h"
#include "kFs.h"
#include "rte_rtc.h"
#include <plc_rte.h>
#include <ctime>
#include <cassert>

#if defined(RTE_APP) || defined(PLC)

#include <iec_types.h>

#endif

#ifdef USE_RTC

#include <rte_rtc.h>

#endif

#include <plc_var_class.h>

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

/* pins configuration */
byte portConfigInputs[TOTAL_PORTS];
// each bit: 1 = pin in INPUT, 0 = anything else
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
extern "C" u16 crc_16(u8 *buf, int cnt, u16 init);

void setPinModeCallback(mFirmata *fm, byte pin, int mode);

int dbg_size();

void mFirmata::reportAnalogCallback(nStream *stream, byte analogPin, int value) {
#if defined(RTE_APP) || defined(PLC)
    if (analogPin < ANALOGVALUE_LENGTH) {
        if (0 == value) {
            (plc_var.info.analogInputsToReport[analogPin / 8]) &= (u8) (~(1 << ((u32) analogPin % 8)));
        } else {
            plc_var.info.analogInputsToReport[analogPin / 8] |= (u8) (1 << ((u32) analogPin % 8));
            // prevent during system reset or all analog pin values will be reported
            // which may report noise for unconnected analog pins
            if (plc_var.info.plc_state < BOOT_WAIT_RESTART) {
                // Send pin value immediately. This is helpful when connected via
                // ethernet, wi-fi or bluetooth so pin states can be known upon
                // reconnecting.
                sendAnalog(stream, analogPin, plcVar.analogValue(analogPin));
            }
        }
    }
#endif
}

void mFirmata::reportDigitalCallback(Stream *, byte port, int value) {
#if defined(RTE_APP) || defined(PLC)
    if (port < IO_XI_NRS + IO_YO_NRS) {
        plc_var.info.reportPINs[port] = value;
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

void mFirmata::setPinValueCallback(Stream *, byte pin, int value) {
#if defined(RTE_APP) || defined(PLC)
    if (pin < IO_YO_NRS + IO_XI_NRS + IO_XA_NRS + IO_YA_NRS) //&& getPinMode(pin) == OUTPUT
    {
        setPinState(pin, value);
        hwboard::outputPort(pin, value);
    }
#endif
}

void mFirmata::systemResetCallback(Stream *) {
#if defined(RTE_APP) || defined(PLC)
    rte.set_state(BOOT_WAIT_RESTART);
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
    for (unsigned char &reportPIN: plc_var.info.reportPINs)
        reportPIN = 0; // by default, reporting off
    // by default, do not report any analog inputs
    memset(plc_var.info.analogInputsToReport, 0, sizeof(plc_var.info.analogInputsToReport));

    /* send digital inputs to set the initial state on the host computer,
     * since once in the loop(), this firmware will only send on change */

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
#ifdef ARDUINO

bool IS_PIN_ANALOG(byte pin);

byte PIN_TO_ANALOG(byte pin);

bool IS_PIN_DIGITAL(byte pin);

uint32_t PIN_TO_DIGITAL(byte pin);

bool IS_PIN_PWM(byte pin);

uint32_t PIN_TO_PWM(byte pin);

void mFirmata::setPinModeCallback(nStream *Fs, byte pin, int mode) {
    if (getPinMode(pin) == PIN_MODE_IGNORE)
        return;
#ifdef USE_FIRMATA_WIRE
    if (getPinMode(pin) == PIN_MODE_I2C && isI2CEnabled && mode != PIN_MODE_I2C)
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
        reportAnalogCallback(Fs, PIN_TO_ANALOG(pin), mode == PIN_MODE_ANALOG ? 1 : 0); // turn on/off reporting
    }
    if (IS_PIN_DIGITAL(pin)) {
        if (mode == INPUT || mode == PIN_MODE_PULLUP) {
            portConfigInputs[pin / 8] |= (1 << (pin & 7));
        } else {
            portConfigInputs[pin / 8] &= ~(1 << (pin & 7));
        }
    }
    setPinState(pin, 0);
    switch (mode) {
        case PIN_MODE_ANALOG:
            if (IS_PIN_ANALOG(pin)) {
                if (IS_PIN_DIGITAL(pin)) {
                    //
                }
                setPinMode(pin, PIN_MODE_ANALOG);
            }
            break;
        case INPUT:
            if (IS_PIN_DIGITAL(pin)) {
                setPinMode(pin, INPUT);
            }
            break;
        case PIN_MODE_PULLUP:
            if (IS_PIN_DIGITAL(pin)) {
                setPinMode(pin, PIN_MODE_PULLUP);
                setPinState(pin, 1);
            }
            break;
        case OUTPUT:
            if (IS_PIN_DIGITAL(pin)) {
                if (getPinMode(pin) == PIN_MODE_PWM) {
                    // Disable PWM if pin mode was previously set to PWM.
                    digitalWrite(PIN_TO_DIGITAL(pin), LOW);
                }
                setPinMode(pin, OUTPUT);
            }
            break;
        case PIN_MODE_PWM:
            if (IS_PIN_PWM(pin)) {
                analogWrite(PIN_TO_PWM(pin), 0);
                setPinMode(pin, PIN_MODE_PWM);
            }
            break;
#ifdef USE_SERVO

            case PIN_MODE_SERVO:
                if (IS_PIN_DIGITAL(pin))
                {
                    setPinMode(pin, PIN_MODE_SERVO);
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
                    setPinMode(pin, PIN_MODE_I2C);
                }
                break;
#endif
        case PIN_MODE_SERIAL:
#ifdef FIRMATA_SERIAL_FEATURE
            serialFeature->handlePinMode(this, pin, PIN_MODE_SERIAL);
#endif
            break;
        default:
            sendString(Fs, "Unknown pin mode");
    }
}

uint32_t PIN_TO_PWM(byte pin) {
    return 0;
}

bool IS_PIN_PWM(byte pin) {
    return false;
}

uint32_t PIN_TO_DIGITAL(byte pin) {
    return 0;
}

bool IS_PIN_DIGITAL(byte pin) {
    return false;
}

byte PIN_TO_ANALOG(byte pin) {
    return 0;
}

bool IS_PIN_ANALOG(byte pin) {
    return false;
}

#endif

int soem_scan(mFirmata *fm, Stream *);

void mFirmata::analogWriteCallback(Stream *, byte i, int val) {
#if defined(RTE_APP) || defined(PLC)
    auto v = (u16) val;
    plcVar.analogValue(i, v);
#endif
}


void mFirmata::stringCallback(nStream *Fs, char *myString) {
#ifdef USE_LFS
    if (strncmp(myString, "rm ", 3) == 0) {
        if (FlashFs::unlink(&myString[3]) == 0)
            sendString(Fs, "rm ok");
        else
            sendString(Fs, "rm fail");
    } else
#endif
        sendString(Fs, "unknown input");
}

int fill_dbg(int index, u8 *buf);

int set_dbg(u32 index, byte *varp, int len);

void writePort(byte port, byte i, byte mask);

int getPinMode(byte pin);

int getPinState(byte pin);

void mFirmata::sysexCallback(nStream *FirmataStream, byte command, uint16_t argc, byte *argv) {
    int index;
    int state;
    u32 start, end;
    u32 indexv;
    int len;
    u8 *data;
    tm new_time{};
    size_t key_len;
    byte *buffer;
    struct {
        u32 build;
        char name[8];
    } info{};
    // logger.debug("sysexCallback: %d argc=%d,argv=%p", command, argc, argv);
    switch (command) {
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
        case I_AM_HERE:
            break;
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
        case CB_GET_REMAIN_MEM:
            sendSysex(FirmataStream, CB_GET_REMAIN_MEM, 2, (byte *) &plc_var.info.remain_mem);
            break;
        case CB_GET_RTE_VERSION:
            sendSysex(FirmataStream, CB_GET_RTE_VERSION, sizeof(rte_ver_t), (uint8_t *) &plc_var.config.rte_ver);
            break;
#if defined(RTE_APP) || defined(PLC)
        case CB_PLC_START:
            rte.app_start();
            len = 0;
            sendSysex(FirmataStream, CB_PLC_START, 4, (byte *) &len);
            break;
        case CB_PLC_STOP:
            rte.app_stop();
            len = 0;
            sendSysex(FirmataStream, CB_PLC_STOP, 4, (byte *) &len);
            break;
        case REPORT_PLC_MD5:
            if (plc_var.info.plc_curr_app)
                sendSysex(FirmataStream, REPORT_PLC_MD5, 32, (byte *) plc_var.info.plc_curr_app->id);
            else
                sendSysex(FirmataStream, REPORT_PLC_MD5, 0, (byte *) "");
            break;
        case CB_PLC_LOAD:
            len = 0;
            sendSysex(FirmataStream, CB_PLC_LOAD, 4, (byte *) &len);
            rte.app_stop();
            app.unload();
            rte.load_app();
            break;
        case CB_PLC_REPAIR:
            rte.app_stop();
            app.unload();
            len = 0;
            sendSysex(FirmataStream, CB_PLC_REPAIR, 4, (byte *) &len);
            break;
#endif
        case FM_FLASH_CLEAR:
            len = 0;
            sendSysex(FirmataStream, FM_FLASH_CLEAR, 4, (byte *) &len);
            board.flashClear();
            hwboard::reset();
            break;
#if defined(USE_RTC) || defined(USE_PCF8563)
        case CB_GET_RTC:
            sendSysex(FirmataStream, CB_GET_RTC, sizeof(rtc_t), (byte *) &plc_var.info.rtc);
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
            sendSysex(FirmataStream, CB_SET_RTC, 0, nullptr);
            break;
#endif
#ifdef ARDUINO
#ifdef USE_LWIP
#ifdef USE_IP_MODIFY
        case CB_SET_IP:
            plc_var.config.ip.ip1 = argv[0];
            plc_var.config.ip.ip2 = argv[1];
            plc_var.config.ip.ip3 = argv[2];
            plc_var.config.ip.ip4 = argv[3];
            ETH_LWIP::set_ip();
            sendSysex(FirmataStream, CB_SET_IP, 4, (byte *) (&plc_var.config.ip));
            break;
#endif
        case CB_GET_IP:
            sendSysex(FirmataStream, CB_GET_IP, 4, (byte *) (&plc_var.config.ip));
            break;
        case FM_GET_NET_BUF_STAT: {
            buffer = (byte *) malloc(13 * MEMP_MAX);
            for (int i = 0; i < MEMP_MAX; i++) {
                *(u8 *) &buffer[0 + 13 * i] = memp_pools[i]->stats->avail;
                *(u8 *) &buffer[1 + 13 * i] = memp_pools[i]->stats->err;
                *(u8 *) &buffer[2 + 13 * i] = memp_pools[i]->stats->illegal;
                *(u8 *) &buffer[3 + 13 * i] = memp_pools[i]->stats->max;
                *(u8 *) &buffer[4 + 13 * i] = memp_pools[i]->stats->used;
                memcpy(&buffer[5 + 13 * i], memp_pools[i]->stats->name, 8);
            }
            sendSysex(FirmataStream, FM_GET_NET_BUF_STAT, 13 * MEMP_MAX, (byte *) buffer);
            free(buffer);
        }
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
            buffer = (byte *) malloc(task_num * sizeof(task_info));
            StatusArray = (TaskStatus_t *) malloc(task_num * sizeof(TaskStatus_t));
            if (StatusArray != nullptr) {

                uxTaskGetSystemState(StatusArray, (UBaseType_t) task_num, (uint32_t *) &TotalRunTime);
                for (int i = 0; i < task_num; i++) {
                    auto *d = (task_info *) &argv[i * sizeof(task_info)];
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
            sendSysex(FirmataStream, CB_THREAD_INFO, (task_num * sizeof(task_info)), (byte *) (argv));
            free(StatusArray);
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
            FirmataStream->write(START_SYSEX);
            FirmataStream->write(CB_SET_FORCE);
            FirmataStream->write(argc);
            FirmataStream->write(END_SYSEX);
            FirmataStream->flush();
            break;

        case CB_CLEAR_V:
            if (plc_var.info.plc_curr_app && (plc_var.info.plc_state == (u8) PLC_STATUS::Started)) {
                plc_var.info.plc_curr_app->dbg_vars_reset(__IEC_DEBUG_FLAG);
                logger.debug("monitor var reset.");
            } else {
                logger.debug("monitor var not reset.plc_state=0x%x ", plc_var.info.plc_state);
            }
            FirmataStream->write(START_SYSEX);
            FirmataStream->write(CB_CLEAR_V);
            FirmataStream->write((uint8_t) 0);
            FirmataStream->write(END_SYSEX);
            FirmataStream->flush();
            break;
        case CB_SET_V:
            if (argc > 2) {
                for (int i = 0; i < argc; i += 2) {
                    const u16 *byte = (u16 *) &argv[i];
                    indexv = *byte;
                    // logger.debug("%d %d", i, indexv);
                    if (plc_var.info.plc_curr_app) {
                        plc_var.info.plc_curr_app->dbg_var_register(indexv);
                    }
                }
                FirmataStream->write(START_SYSEX);
                FirmataStream->write(CB_SET_V);
                FirmataStream->write(argc);
                FirmataStream->write(END_SYSEX);
                FirmataStream->flush();
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
            sendSysex(FirmataStream, CB_GET_V, len + 4, data);
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
            sendSysex(FirmataStream, FM_GET_TASK_NRS, 1, &(plc_var.info.plc_task_cnt));
            break;
        case FM_GET_TASK_NAME:
            if (tasks[argv[0]]) {
                sendSysex(FirmataStream, FM_GET_TASK_NAME, (byte) strlen(tasks[argv[0]]->task_name),
                          (byte *) tasks[argv[0]]->task_name);
            }
            break;
        case FM_GET_TASK_DETAIL:
            if (argv[0] < PLC_TASK_NRS && tasks[argv[0]]) {
                sendSysex(FirmataStream, FM_GET_TASK_DETAIL, 24, tasks[argv[0]]->mata());
            }
            break;
#endif
        case FM_GET_PLC_STATE:
            sendSysex(FirmataStream, FM_GET_PLC_STATE, 1, (byte *) (&plc_var.info.plc_state));
            break;
        case FM_GET_PLC_INFO:
            if (plc_var.info.plc_curr_app) {
                info.build = plc_var.info.plc_curr_app->buildnumber;
                strcpy(info.name, plc_var.info.plc_curr_app->app_name);
                sendSysex(FirmataStream, FM_GET_PLC_INFO, sizeof(info), (byte *) &info);
            } else
                sendSysex(FirmataStream, FM_GET_PLC_INFO, 0, (byte *) &info);
            break;
        case CB_GET_LOG_NUMBER:
            sendSysex(FirmataStream, CB_GET_LOG_NUMBER, 5, (byte *) (&plc_var.info.plc_state));
            break;
        case CB_GET_LOG:
            sendSysex(FirmataStream, CB_GET_LOG, 0, (byte *) argv);
            break;
#ifdef USE_BOOTLOADER
        case CB_GET_BOOT_VERSION:
#ifdef BOOTINFO
            boot_t *b;
            b = (boot_t *) BOOTINFO; // platformio.ini中定义
            if (b)
                sendSysex(FirmataStream, CB_GET_BOOT_VERSION, sizeof(boot_t), (byte *) b);
            else
#endif
            {
                sendSysex(FirmataStream, CB_GET_BOOT_VERSION, 0, (byte *) b);
            }
            break;
        case FM_FLASH_BOOT:
            len = board.updateBootbin();
            sendSysex(FirmataStream, FM_FLASH_BOOT, len, (byte *) &len);
            break;
#endif

#ifdef USE_KVDB
        case FM_LIST_KEY:
            kvdb.list(this, FirmataStream);
            break;
        case CB_READ_KEY:
            size_t vlen, name_len;
            name_len = strlen((const char *) argv);
            byte *value_crk;
            value_crk = (byte *) malloc(FDB_KV_NAME_MAX + FDB_STR_KV_VALUE_MAX_SIZE + 2);
            memset(value_crk, 0, FDB_KV_NAME_MAX + FDB_STR_KV_VALUE_MAX_SIZE + 2);
            strncpy((char *) value_crk, (const char *) argv, name_len);
            if (name_len < FDB_KV_NAME_MAX && name_len > 0) {

                buffer = (byte *) kvdb.get((const char *) argv);
                vlen = strlen((const char *) buffer);
                if (buffer && (vlen > 0) && (vlen < FDB_STR_KV_VALUE_MAX_SIZE)) {
                    strncpy((char *) &value_crk[name_len + 1], (const char *) buffer, vlen);
                    vlen += name_len + 2;
                } else {

                    vlen = 4;
                    *(int *) value_crk = KV_VALUE_ILLEAGAL;
                }
            } else {
                vlen = 4;
                *(int *) value_crk = KV_NAME_ILLEAGL;
            }
            sendSysex(FirmataStream, CB_READ_KEY, vlen, (byte *) value_crk);
            free(value_crk);
            break;
        case FM_READ_KEY_BYTES:
            data = (u8 *) malloc(256);
            len = kvdb.get((const char *) argv, (char *) data + 4, 256, (u32 *) data);
            sendSysex(FirmataStream, FM_READ_KEY_BYTES, len + 4, (byte *) data);
            free(data);
            break;
        case CB_WRITE_KEY:
            key_len = strlen((const char *) argv);
            int rw;
            rw = argc - key_len - 2;
            byte *value;
            value = (byte *) malloc(FDB_KV_NAME_MAX + FDB_STR_KV_VALUE_MAX_SIZE + 2);
            memset(value, 0, FDB_KV_NAME_MAX + FDB_STR_KV_VALUE_MAX_SIZE + 2);
            strncpy((char *) value, (const char *) argv, key_len);
            if ((key_len < FDB_KV_NAME_MAX) && (rw < FDB_STR_KV_VALUE_MAX_SIZE)) {

                rw = kvdb.set((const char *) argv, (const char *) argv + key_len + 1, (int) rw, KV_STR_VALUE);
                buffer = (byte *) kvdb.get((const char *) argv);
                vlen = strlen((const char *) buffer);
                if (buffer && (vlen > 0) && (vlen < FDB_STR_KV_VALUE_MAX_SIZE)) {
                    strncpy((char *) &value[key_len + 1], (const char *) buffer, vlen);
                    vlen += key_len + 2;
                } else {
                    vlen = 4;
                    *(int *) value = KV_VALUE_ILLEAGAL;
                }
            }
            sendSysex(FirmataStream, CB_WRITE_KEY, vlen, (byte *) value);
            free(value);
            break;
        case FM_WRITE_KEY_BYTES:
            key_len = strlen((const char *) argv);
            uint32_t type;
            type = *(uint32_t *) (argv + key_len + 1);
            len = kvdb.set((const char *) argv, (const char *) (argv + key_len + 1 + 4),
                           (int) (argc - key_len - 1 - 4), type);
            sendSysex(FirmataStream, FM_WRITE_KEY_BYTES, 4, (byte *) &len);
            break;
        case CB_RM_KEY:
            kvdb.remove((const char *) argv);
            sendSysex(FirmataStream, CB_RM_KEY, 0, argv);
            break;
#endif
#ifdef USE_TSDB
        case CB_SET_TSL_RANGE:
            tsl_query q;
            key_len = strlen((const char *) argv);
            if (argc == key_len + 13) {
                start = *(u32 *) &argv[key_len + 1];
                end = *(u32 *) &argv[key_len + 1 + 4];
                state = (int) *(u32 *) &argv[key_len + 1 + 8];
                memset(&q, 0, sizeof(q));
                tsdb.query((const char *) argv, start, end, (fdb_tsl_status) (state), &q);
            }
            sendSysex(FirmataStream, CB_SET_TSL_RANGE, sizeof(tsl_query), (byte *) &q);
            break;
        case CB_SET_TSL_STATUS:
            key_len = strlen((const char *) argv);
            len = -1;
            if (argc == key_len + 13) {
                start = *(u32 *) &argv[key_len + 1];
                end = *(u32 *) &argv[key_len + 1 + 4];
                state = (int) *(u32 *) &argv[key_len + 1 + 8];
                len = tsdb.set_status((const char *) argv, start, end, (fdb_tsl_status) (state));
            }
            sendSysex(FirmataStream, CB_SET_TSL_STATUS, 4, (byte *) &len);
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
            sendSysex(FirmataStream, CB_GET_TSL, (byte) tlen, (byte *) tbuf);
            free(tbuf);
            break;
        case CB_GET_TSL_BY_ID:
            key_len = strlen((const char *) argv);
            tbuf = (char *) malloc(256);
            memset(tbuf, 0, 256);
            tlen = 4;
            *(int *) tbuf = -1;
            if (argc == key_len + 5) {
                tlen = tsdb.query_read_by_id((const char *) argv, *(u32 *) &argv[key_len + 1], (u32 *) &tbuf[0],
                                             (fdb_time_t *) &tbuf[4],
                                             (int *) (tbuf + 8),
                                             tbuf + 12, 256 - 12);
                if (tlen < 0)
                    tlen = 0;
                else {
                    *(u32 *) &tbuf[tlen] = GenerateCRC32Sum((const u8 *) tbuf, tlen, 0);
                    tlen += 4;
                }
            }
            sendSysex(FirmataStream, CB_GET_TSL_BY_ID, (byte) tlen, (byte *) tbuf);
            free(tbuf);
            break;
        case CB_TSL_CLEAR:
            key_len = strlen((const char *) argv);
            // index = *(int *) &argv[key_len + 1];
            state = 0;
            sendSysex(FirmataStream, CB_TSL_CLEAR, (byte) sizeof(state), (byte *) &state);
            state = TSL::clear((const char *) (argv));
            break;
#endif
#ifdef USE_SOEM
            case FM_SOEM_SCAN:
                soem_scan(this, FirmataStream);
                break;
#endif
        case CB_CPU_USAGE:
            sendSysex(FirmataStream, CB_CPU_USAGE, 1, (byte *) &plc_var.info.cpu_usage);
            break;
#ifdef USE_WIFI
            case CB_WIFI_LIST:
                sendSysex(FirmataStream, CB_WIFI_LIST, plc_var.info.wifi_size, (byte *) &plc_var.info.wifi_list);
                break;
            case CB_WIFI_SET_PASS:
                int plen;
                plen = strlen((const char *) argv);
                //                kvdb.set("wifi_pass", (const char *)argv, plen);
                wifi.reload();
                sendSysex(FirmataStream, CB_WIFI_SET_PASS, 0, argv);
                break;
#endif
#ifdef USE_MEMBLOCK
        case FM_PUT_DATA_BLOCK:
            u32 crc, crc_r;
            if (argc < 12)
                break;
            crc = GenerateCRC32Sum((const u8 *) argv, argc - 4, 0);
            crc_r = *(u32 *) (&argv[argc - 4]);
            if (crc_r != crc) {
                state = CRC_ERROR;
                logger.error("crc error block %d 0x%x 0x%x argc=%d len=%d", *(int *) &argv[0], crc, crc_r, argc,
                             argc);
            } else {
                rte.set_state(PLC_STATUS::APP_FLASH_BEGIN);
                int block = *(int *) &argv[0];
                if (block == 0) {
                    if (dev) {
                        state = DEV_IS_OPEN;
                    } else {
                        u32 object = *(u32 *) &argv[4];
                        u32 data_address = *(u32 *) &argv[8];
                        u32 data_len = *(u32 *) &argv[12];

                        dev = mem_block::mems[object];
                        if (!dev) {
                            state = NO_DEVICE;
                        } else {
                            state = dev->begin(&argv[16], argc - 16, data_address, data_len);
                            if (state > 0 && state > dataBufferSize * 7 / 8 - 4) {
                                state = (int) (dataBufferSize * 7 / 8 - 4);
                            }
                            logger.info("recv %s ,size= %d", &argv[12], *(u32 *) &argv[8]);
                        }
                    }
                } else if (block == -1) {
                    if (dev) {
                        if (dev->Shutdown() < 0) {
                            state = DEVICE_SHUTDOWN_ERR;
                        } else {
                            state = 1;
                            rte.set_state(PLC_STATUS::APP_FLASH_END);
                            logger.info("recv end.");
                            dev = nullptr;
                        }
                    }
                } else {
                    if (dev) {
                        if (dev->Write(&argv[4], argc - 8) < 0) {
                            state = DEVICE_WRITE_ERR;
                        } else {
                            state = block;
                        }
                        logger.info("recv %d ,size= %d", block, argc - 8);
                    }
                }
            }
            sendSysex(FirmataStream, FM_PUT_DATA_BLOCK, 4, (byte *) &state);
            break;
#endif
#if defined(RTE_APP) || defined(PLC)
        case FM_GET_LOC_SIZE:
            if (plc_var.info.plc_curr_app) {
                sendSysex(FirmataStream, FM_GET_LOC_SIZE, 2, (byte *) &plc_var.info.plc_curr_app->l_sz);
            } else {
                sendSysex(FirmataStream, FM_GET_LOC_SIZE, 0, (byte *) &plc_var.info.plc_curr_app->l_sz);
            }
            break;
        case FM_GET_LOC_TAB:
            u32 l_index;
            if (argc == 5) {
                l_index = *(u32 *) &argv[0];
                if (plc_var.info.plc_curr_app && l_index < plc_var.info.plc_curr_app->l_sz) {
                    plc_loc_tbl_t loc = plc_var.info.plc_curr_app->l_tab[l_index];
                    len = (int) sizeof(plc_loc_dsc_t) + loc->a_size + loc->v_size;
                    byte *buffer = (byte *) malloc(len);
                    buffer[0] = loc->v_type;
                    buffer[1] = loc->v_size;
                    *(u16 *) &buffer[2] = loc->proto;
                    *(u16 *) &buffer[4] = loc->a_size;
                    memcpy(&buffer[6], loc->a_data, loc->a_size);
                    memcpy(&buffer[6 + loc->a_size], loc->v_buf, loc->v_size);
                    sendSysex(FirmataStream, FM_GET_LOC_TAB, len, (byte *) buffer);
                    free(buffer);
                    break;
                }
            }
            sendSysex(FirmataStream, FM_GET_LOC_TAB, 0, (byte *) &plc_var.info.plc_curr_app->l_sz);
            break;
        case FM_SET_LOC_TAB:
            if (argc == 5) {
                l_index = *(u32 *) &argv[0];
                if (plc_var.info.plc_curr_app && l_index < plc_var.info.plc_curr_app->l_sz) {
                    sendSysex(FirmataStream, FM_SET_LOC_TAB, sizeof(plc_loc_tbl_t),
                              (byte *) &plc_var.info.plc_curr_app->l_tab[l_index]);
                    break;
                }
            }
            sendSysex(FirmataStream, FM_SET_LOC_TAB, 0, (byte *) &plc_var.info.plc_curr_app->l_sz);
            break;
#endif
#ifdef ONLINE_DEBUG
        case FM_GET_DBG_SIZE:
            if (plc_var.info.plc_curr_app) {
                sendSysex(FirmataStream, FM_GET_DBG_SIZE, 4,
                          (byte *) &plc_var.info.plc_curr_app->data->size_dbgvardsc);
            } else {
                sendSysex(FirmataStream, FM_GET_DBG_SIZE, 0, nullptr);
            }
            break;
        case FM_GET_DBG:
            len = 0;
            if (argc == 5) {
                l_index = *(u32 *) &argv[0];
                if (plc_var.info.plc_curr_app && l_index < plc_var.info.plc_curr_app->data->size_dbgvardsc) {
                    len = (int) fill_dbg((int) l_index, argv);
                }
            }
            sendSysex(FirmataStream, FM_GET_DBG, len, argv);
            break;
        case FM_SET_DBG:
            len = 0;
            if (argc > 5) {
                l_index = *(u32 *) argv;
                if (plc_var.info.plc_curr_app && l_index < plc_var.info.plc_curr_app->data->size_dbgvardsc) {
                    set_dbg(l_index, &argv[4], argc - 4);
                    len = (int) fill_dbg((int) l_index, argv);
                }
            }
            sendSysex(FirmataStream, FM_GET_DBG, len, argv);
            break;
#endif
#if defined(RTE_APP) || defined(PLC)
        case FM_LOG_SET_LEVEL:
            plc_var.config.log_level = argv[0];
            sendSysex(FirmataStream, FM_LOG_SET_LEVEL, 1, &plc_var.config.log_level);
            break;
#endif
#ifdef ARDUINO_ARCH_STM32
        case FM_GET_CPU_SN:
            u32 sn[3];
            sn[0] = HAL_GetUIDw0();
            sn[1] = HAL_GetUIDw1();
            sn[2] = HAL_GetUIDw2();
            sendSysex(FirmataStream, FM_GET_CPU_SN, 12, (byte *) sn);
            break;
#endif
        case FM_READ_MEM:
            indexv = 0;
            len = 0;
            if (argc == 6) {
                indexv = *(u32 *) argv;
                len = *(u16 *) &argv[4];
            }
            sendSysex(FirmataStream, FM_READ_MEM, len, (byte *) indexv);
            break;
        case FM_WRITE_MEM:
            if (argc > 6) {
                indexv = *(u32 *) argv;
                len = *(u16 *) &argv[4];
                for (int i = 0; i < len; ++i) {
                    *((uint8_t *) indexv + i) = argv[6 + i];
                }
                sendSysex(FirmataStream, FM_WRITE_MEM, len, (byte *) indexv);
            }
            break;
        case FM_READ_VALUE:
            u8 region, typ;
            indexv = 0;
            len = 0;
            if (argc == 8 && argv[0] <= REGION_HOLDER) {
                region = argv[0];
                indexv = *(u32 *) &argv[1];
                typ = argv[5];
                len = *(u16 *) &argv[6];
                byte *buffer;
                buffer = (byte *) malloc(len * 4 + 10);
                memset(buffer, 0, len * 4 + 10);
                buffer[0] = region;
                buffer[1] = typ;
                *(u32 *) &buffer[2] = indexv;
                const char *p;
                switch (region) {
                    default:
                        p = (const char *) &plc_var.digitalValue;
                        break;
                    case REGION_XI:      // byte from 0
                    case REGION_DIGITAL: // digitalValue
                        p = (const char *) &plc_var.digitalValue;
                        len = (len + 7) / 8;
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
                memcpy(&buffer[6], &p[indexv], len);
                sendSysex(FirmataStream, FM_READ_VALUE_REP, len + 6, (byte *) buffer);
                free(buffer);
            } else
                sendSysex(FirmataStream, FM_READ_VALUE_REP, len, ((byte *) &plc_var) + indexv);
            break;
        case FM_WRITE_VALUE:
            if (argc > 7) {
                region = argv[0];
                indexv = *(u32 *) &argv[1];
                len = *(u16 *) &argv[5];
                char *p;
                switch (region) {
                    default:
                        p = ((char *) &plc_var.digitalValue);
                        break;
                    case REGION_XI:      // byte from 0
                    case REGION_DIGITAL: // digitalValue
                        u8 v;
                        v = ((char *) &plc_var.digitalValue)[indexv / 8];
                        if (argv[7] == 1) {
                            argv[7] = v | (1 << (indexv % 8));
                        } else {
                            argv[7] = v & ~(1 << (indexv % 8));
                        }
                        indexv = indexv / 8;
                        len = (len + 7) / 8;
                        p = ((char *) &plc_var.digitalValue);
                        break;
                    case REGION_16: // analogValue
                        p = (char *) &plc_var.analogValue;
                        break;
                    case REGION_32: // analogValue32
                        p = (char *) &plc_var.analogValue32;
                        break;
                    case REGION_HOLDER: // holdValue
                        p = (char *) &plc_var.holdValue;
                        break;
                    case REGION_INFO:
                        p = (char *) &plc_var.info;
                        break;
                    case REGION_CONFIG:
                        p = (char *) &plc_var.config;
                        break;
                }
                memcpy(p + indexv, &argv[7], len);
                sendSysex(FirmataStream, FM_WRITE_VALUE_REP, 7, (byte *) argv);
            }
            break;
        case FM_READ_BIT:
            indexv = 0;
            len = 0;
            if (argc == 6) {
                indexv = *(u32 *) argv;
                len = *(u16 *) &argv[4];
            }
            if (len > 0) {
                byte *buffer = (byte *) malloc(len / 8 + 6);
                *(u32 *) argv = indexv;
                buffer[4] = len;
                for (int i = 0; i < len; i++) {
                    u8 b = plcVar.digitalValue(indexv + i);
                    buffer[i / 8 + 5] |= b << (i % 8);
                }
                sendSysex(FirmataStream, FM_READ_BIT_REP, len / 8 + 6, (byte *) buffer);
                free(buffer);
            }
            break;
        case FM_WRITE_BIT:
            if (argc > 6) {
                indexv = *(u32 *) argv;
                len = *(u16 *) &argv[4];
                if (len < (argc - 6)) {
                    for (int i = 0; i < len; i++) {
                        *(((uint8_t *) &plc_var) + indexv + i) = argv[6 + i];
                    }
                }
                sendSysex(FirmataStream, FM_WRITE_VALUE_REP, len,
                          (byte *) ((uint8_t *) &plc_var) + indexv);
            }
            break;
        case FM_READ_VALUE_REP:
            if (argc > 0) {
                memcpy(valueBuf, argv, argc);
                valueLen = argc;
            }
            break;
        case FM_WRITE_VALUE_REP:
            if (argc > 0 && argc < 8) {
            }
            break;
        case FM_READ_BIT_REP:
            if (argc > 0 && argc < 16) {
                memcpy(valueBuf, argv, argc);
                valueLen = argc;
            }
            break;
        case FM_WRITE_BIT_REP:
            if (argc > 0 && argc < 8) {
            }
            break;
#ifdef RTE_APP
        case FM_GET_LOCATION:
            len = 0;
            byte *buf_fgl;
            buf_fgl = (byte *) malloc(32);
            if (argc >= 4) {
                len = board.get_input(argv[1], argv[2], argv[3], 0, buf_fgl);
            }
            sendSysex(FirmataStream, FM_GET_LOCATION, len, (byte *) buf_fgl);
            free(buf_fgl);
            break;
        case FM_SET_LOCATION:
            len = -1;
            if (argc >= 6) {
                len = board.set_output(argv[1], argv[2], argv[3], 0, &argv[5], argv[4]);
            }
            sendSysex(FirmataStream, FM_SET_LOCATION, 4, (byte *) &len);
            break;
#endif

        case CB_RESET:
            len = 1;
            sendSysex(FirmataStream, CB_RESET, 4, (byte *) &len);
            rte.set_state(BOOT_WAIT_RESTART);
            break;
#ifndef THIS_IS_BOOTLOADER
#ifdef ARDUINO
            case CB_GOTO_IAP:
                len = 1;
                board.bpr_write(BOOTLOADER_REQUEST_BACKUP_REGISTER, EXEC_IAP);
                rte.set_state(BOOT_WAIT_IAP);
                sendSysex(FirmataStream, CB_GOTO_IAP, 4, (byte *) &len);
                break;
#endif
            case FM_IOT_LOGIN:
                switch (argv[0]) {
                    case IOT_LOGIN_OK:
                        break;
                }
                break;
#ifdef ARDUINO_ARCH_STM32
            case FM_INFO_SERIAL_RX: {
                kSerial *serial = kSerial::get_serial(argv[0]);
                if (nullptr == serial) {
                    buffer = (byte *) malloc(4);
                    len = 4;
                    *(int *) buffer = NO_DEVICE;
                } else {
                    buffer = (byte *) malloc(serial->rx_buf_size + 16);
                    *(int *) buffer = serial->rx_count;
                    memcpy(buffer + 4, rtos::queue_buf(serial->rx_buff), serial->rx_buf_size);
                    len = serial->rx_buf_size + 4;
                }
                sendSysex(FirmataStream, FM_INFO_SERIAL_RX, len, buffer);
                free(buffer);
            }
                break;
            case FM_INFO_SERIAL_TX: {
                kSerial *serial = kSerial::get_serial(argv[0]);
                if (nullptr == serial) {
                    buffer = (byte *) malloc(4);
                    len = 4;
                    *(int *) buffer = NO_DEVICE;
                } else {
                    buffer = (byte *) malloc(serial->tx_buf_size + 16);
                    *(int *) buffer = serial->tx_count;
                    memcpy(buffer + 4, serial->_serial.tx_buff + serial->_serial.tx_head,
                           serial->tx_buf_size - serial->_serial.tx_head);
                    memcpy(buffer + 4, serial->_serial.tx_buff, serial->_serial.tx_head);
                    len = serial->tx_buf_size + 4;
                }
                sendSysex(FirmataStream, FM_INFO_SERIAL_TX, len, buffer);
                free(buffer);
            }
                break;
#endif
#endif
#ifdef USE_LFS
        case FM_LFS_LS:
            if (argc > 0) {
                buffer = (byte *) malloc(1024);
                len = kfs.dir_buf((const char *) &argv[0], (char *) buffer, 1024);
                sendSysex(FirmataStream, FM_LFS_LS, len, buffer);
                free(buffer);
            }
            break;
#endif
        default:
            len = -1;
            logger.error("sysexCallback: %d argc=%d,argv=%p", command, argc, argv);
            // sendSysex(FirmataStream, command, 4, (byte *) &len);
            break;
    }
    set_flag(command);
}

void digitalWriteCallback(mFirmata *fm, nStream *FirmataStream, byte port, int value) {
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
            if (IS_PIN_DIGITAL(pin) && (getPinMode(pin) == OUTPUT || getPinMode(pin) == INPUT)) {
                pinValue = ((byte) value & mask) ? 1 : 0;
                if (getPinMode(pin) == OUTPUT) {
                    pinWriteMask |= mask;
                } else if (getPinMode(pin) == INPUT && pinValue == 1 && getPinState(pin) != 1) {
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

int getPinState(byte pin) {
    return 0;
}

int getPinMode(byte pin) {
    return 0;
}

void writePort(byte port, byte i, byte mask) {
}

int mFirmata::loop(nStream *FirmataStream) {
    processInput(FirmataStream);
    return 0;
}

void mFirmata::report(nStream *FirmataStream) {
    u32 currentMillis = rtos::ticks();

    if (currentMillis - previousMillis > plc_var.config.reportInterval) {
        previousMillis += plc_var.config.reportInterval;
        /* ANALOGREAD - do all analogReads() at the configured sampling interval */
        board.readAnalogValue(this, FirmataStream, plc_var.info.analogInputsToReport,
                              sizeof(plc_var.info.analogInputsToReport));
        for (byte pin = 0; pin < IO_XI_NRS + IO_YO_NRS; pin++) {
            if (plc_var.info.reportPINs[pin / 8] & (1 << (pin % 8))) {
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

void mFirmata::outputPort(nStream *FirmataStream, byte portNumber, byte portValue, byte forceSend) {
    // pins not configured as INPUT are cleared to zeros
    //    portValue = portValue & portConfigInputs[portNumber];
    // only send if the value is different than previously sent
    if (portNumber < (IO_XI_NRS + IO_YO_NRS) &&
        (forceSend || (plc_var.info.previousPINs[portNumber / 8] & (1 << (portNumber % 8))) != portValue)) {
        sendDigitalPort(FirmataStream, portNumber, portValue);
        FirmataStream->flush();
        if (portValue == 0) {
            plc_var.info.previousPINs[portNumber / 8] &= ~(1 << (portNumber % 8));
        } else {
            plc_var.info.previousPINs[portNumber / 8] |= (1 << (portNumber % 8));
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

int mFirmata::setValue(nStream *FirmataStream, int index, void *valBuf, u8 size) {
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

int mFirmata::getValue(nStream *pStream, int index, u8 *value_buf, u16 len) {
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

int mFirmata::getBit(nStream *pStream, int index, u8 *value_buf, u16 len) {
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

int mFirmata::getPinState(byte pin) {
    return plcVar.digitalValue(pin);
}

void mFirmata::setPinState(byte pin, int state) {
    plcVar.digitalValue(pin, state);
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
    u16 crc = 0;
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
    }
}

void mFirmata::marshaller_sendSysex(nStream *FirmataStream, uint8_t command, size_t bytec, uint8_t *bytev) {
    assert(FirmataStream);
    if (bytec > FIRMATA_BUFFER_SZ) {
        //        core_debug("FirmataMarshaller byte limit");
        return;
    }
    if (crc_en)
        FirmataStream->write(0xFA);
    else
        FirmataStream->write(START_SYSEX);
    FirmataStream->write(command);
    if (bytec > 0)
        encodeByteStream(FirmataStream, bytec, bytev, FIRMATA_BUFFER_SZ);
    if (use_sn)
        FirmataStream->write(0xE0);
    else
        FirmataStream->write(END_SYSEX);
    FirmataStream->flush();
}

void mFirmata::sendSysex(nStream *FirmataStream, byte command, uint16_t bytec, byte *bytev, bool _crc_en) {
    crc_en = _crc_en;
    if (FirmataStream->lock)
        rtos::mutex_lock(FirmataStream->lock);

    if (use_sn) {
        auto *c = (byte *) malloc(bytec + 4);
        *(uint32_t *) c = sn;
        memcpy(c + 4, bytev, bytec);
        marshaller_sendSysex(FirmataStream, command, bytec + 4, c);
        free(c);
    } else {
        marshaller_sendSysex(FirmataStream, command, bytec, bytev);
    }
    if (FirmataStream->lock)
        rtos::mutex_unlock(FirmataStream->lock);
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
    if (stream->flag & FLAG_FORWARD) {
        stream->forward(inputData);
        if (inputData == END_SYSEX || rtos::ticks() > stream->tick_forward) {
            stream->closeForward();
        }
        return;
    }
    if (forward > 0) {
        board.forwardMessage(forward, inputData);
        if (inputData == END_SYSEX + forward) {
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
            u16 crc1 = 0, crc = 0;
            if (crc_en && sysexBytesRead > 3) {
                crc1 = (dataBuffer[sysexBytesRead - 1] << 14) + (dataBuffer[sysexBytesRead - 2] << 7) +
                       dataBuffer[sysexBytesRead - 3];
                crc = crc_16(dataBuffer + 1, sysexBytesRead - 4, 0);
                sysexBytesRead -= 3;
            }
            if (crc == crc1) {
                processSysexMessage(stream);
            } else {
                logger.error("crc error %d %d", crc, crc1);
            }

            stream->flag &= ~FLAG_SYSEX;
        } else if (inputData == 0xE0) {
            use_sn = true;
            // stop sysex byte
            parsingSysex = false;
            u16 crc1 = 0, crc = 0;
            if (crc_en && sysexBytesRead > 3) {
                crc1 = (dataBuffer[sysexBytesRead - 1] << 14) + (dataBuffer[sysexBytesRead - 2] << 7) +
                       dataBuffer[sysexBytesRead - 3];
                crc = crc_16(dataBuffer + 1, sysexBytesRead - 4, 0);
                sysexBytesRead -= 3;
            }
            if (crc == crc1) {
                // fire off handler function
                processSysexMessage(stream);
            } else {
                logger.error("crc error %d %d", crc, crc1);
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
        if ((waitForData == 0) && executeMultiByteCommand) { // got the whole message
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
                crc_en = false;
                stream->flag |= FLAG_SYSEX;
                break;
            case 0xFA:
                use_sn = false;
                parsingSysex = true;
                sysexBytesRead = 0;
                crc_en = true;
                stream->flag |= FLAG_SYSEX;
            default:
#ifdef FORWARD_NRS
                if (command > START_SYSEX && command < START_SYSEX + FORWARD_NRS) {
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
    switch (dataBuffer[0]) { // first byte in buffer is command
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
                byte *buffer = (byte *) malloc(sysexBytesRead);
                int len = decodeByteStream(sysexBytesRead - 1, &dataBuffer[1], buffer);
                if (use_sn) {
                    sn = *(uint32_t *) buffer;
                }
                sysexCallback(stream, dataBuffer[0], len, buffer);
                free(buffer);
            } else {
                sysexCallback(stream, dataBuffer[0], sysexBytesRead - 1, &dataBuffer[1]);
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
}
