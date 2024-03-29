#ifndef THIS_IS_BOOTLOADER
/*
  SerialFirmata.cpp
  Copyright (C) 2016 Jeff Hoefs. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  See file LICENSE.txt for further information on licensing terms.

  This version of SerialFirmata.cpp differs from the ConfigurableFirmata
  version in the following ways:

  - handlePinMode calls Firmata::setPinMode

  Last updated March 16th, 2020
*/

#include <kSerial.h>
#include <rte_data.h>
#include "SerialFirmata.h"
#include "hwboard.h"
#include "plc_rte.h"

// The RX and TX hardware FIFOs of the ESP8266 hold 128 bytes that can be
// extended using interrupt handlers. The Arduino constants are not available
// for the ESP8266 platform.
#if !defined(SERIAL_RX_BUFFER_SIZE) && defined(UART_TX_FIFO_SIZE)
#define SERIAL_RX_BUFFER_SIZE UART_TX_FIFO_SIZE
#endif

SerialFirmata::SerialFirmata() {
#if defined(SoftwareSerial_h)
    swSerial0 = NULL;
    swSerial1 = NULL;
    swSerial2 = NULL;
    swSerial3 = NULL;
#endif

#if defined(FIRMATA_SERIAL_RX_DELAY)
    for (byte i = 0; i < SERIAL_READ_ARR_LEN; i++)
    {
      maxRxDelay[i] = FIRMATA_SERIAL_RX_DELAY; // @todo provide setter
    }
#endif
}

int MIN(int v1, int v2) {
    if (v1 < v2) {
        return v1;
    }
    return v2;
}
bool SerialFirmata::handlePinMode(mFirmata *fm, byte pin, int mode) {
    // used for both HW and SW serial
    if (mode == PIN_MODE_SERIAL) {
        fm->setPinMode(pin, PIN_MODE_SERIAL);
        return true;
    }
    return false;
}

void SerialFirmata::handleCapability(mFirmata *fm, nStream *FirmataStream, byte pin) {

    if (IS_PIN_SERIAL(pin)) {
        FirmataStream->write(PIN_MODE_SERIAL);
        FirmataStream->write(FirmataSerial::getSerialPinType(pin));
    }
}

bool SerialFirmata::handleSysex(mFirmata *fm, nStream *FirmataStream, byte command, byte argc, byte *argv) {
    //  logger.debug("SerialFirmata::handleSysex: fm=0x%x S=0x%x cmd=0x%x argc=%d argv=0x%x", fm, FirmataStream, command,
    //               argc, argv);
    if (command == SERIAL_MESSAGE) {

        kSerial *serialPort;
        byte mode = argv[0] & SERIAL_MODE_MASK;
        byte portId = argv[0] & SERIAL_PORT_ID_MASK;
        // logger.debug("mode =0x%x port=%d", mode, portId);
        if (portId >= SERIAL_READ_ARR_LEN) {
            return false;
        }

        switch (mode) {
            case SERIAL_CONFIG:
                if (argc != 6) {
                    break;
                }
                uint32_t baud;
                baud = *(uint32_t *) &argv[1];
                if ((baud > 115200) || (baud < 9600)) {
                    return false;
                }
                u8 format;
                format = argv[5];
#if defined(FIRMATA_SERIAL_RX_DELAY)
                lastBytesAvailable[portId] = 0;
                lastBytesReceived[portId] = 0;
#endif
                if (portId < 8) {
                    serialPort = getPortFromId(portId);
                    if (serialPort != nullptr) {
                        // pins = FirmataSerial::getSerialPinNumbers(portId);
                        // if (pins.rx != 0 && pins.tx != 0)
                        // {
                        //   fm->setPinMode(pins.rx, PIN_MODE_SERIAL);
                        //   fm->setPinMode(pins.tx, PIN_MODE_SERIAL);
                        //   // Fixes an issue where some serial devices would not work properly with Arduino Due
                        //   // because all Arduino pins are set to OUTPUT by default in StandardFirmata.
                        //   pinMode(pins.rx, INPUT);
                        // }
                        if (serialPort->flag & IS_OPEN) {
                            // serialPort->end();
                        } else {
                            ((kSerial *) serialPort)->begin(baud, format);
                            if (serialIndex + 1 >= SERIAL_NRS) {
                                break;
                            }

                            // read all available bytes per iteration of loop()
                            rte_data.firmata.serialBytesToRead[portId] = 0;
                            byte serialIndexToSkip = 0;
                            for (byte i = 0; i < serialIndex + 1; i++) {
                                if (rte_data.firmata.reportSerial[i] == portId) {
                                    serialIndexToSkip = 1;
                                    break;
                                }
                            }
                            if (0 == serialIndexToSkip) {
                                serialIndex++;
                                rte_data.firmata.reportSerial[serialIndex] = portId;
                            }
                        }
                        argv[0] = SERIAL_STATUS | portId;
                        if (serialPort->flag & IS_OPEN)
                            argv[1] = SERIAL_IS_OPEN;
                        else
                            argv[1] = SERIAL_IS_CLOSE;
                        fm->sendSysex(FirmataStream, SERIAL_MESSAGE, 2, argv);
                    }
                } else {
#if defined(SoftwareSerial_h)
                    byte swTxPin, swRxPin;
                    if (argc > 4)
                    {
                      swRxPin = argv[4];
                      swTxPin = argv[5];
                    }
                    else
                    {
                      // RX and TX pins must be specified when using SW serial
                      fm->sendString("Specify serial RX and TX pins");
                      return false;
                    }
                    switch (portId)
                    {
                    case SW_SERIAL0:
                      if (swSerial0 == NULL)
                      {
                        swSerial0 = new SoftwareSerial(swRxPin, swTxPin);
                      }
                      break;
                    case SW_SERIAL1:
                      if (swSerial1 == NULL)
                      {
                        swSerial1 = new SoftwareSerial(swRxPin, swTxPin);
                      }
                      break;
                    case SW_SERIAL2:
                      if (swSerial2 == NULL)
                      {
                        swSerial2 = new SoftwareSerial(swRxPin, swTxPin);
                      }
                      break;
                    case SW_SERIAL3:
                      if (swSerial3 == NULL)
                      {
                        swSerial3 = new SoftwareSerial(swRxPin, swTxPin);
                      }
                      break;
                    }
                    serialPort = getPortFromId(portId);
                    if (serialPort != NULL)
                    {
                      fm->setPinMode(swRxPin, PIN_MODE_SERIAL);
                      fm->setPinMode(swTxPin, PIN_MODE_SERIAL);
                      ((SoftwareSerial *)serialPort)->begin(baud);
                    }
#endif
                }
                break; // SERIAL_CONFIG

            case SERIAL_WRITE:
                serialPort = getPortFromId(portId);
                if (serialPort == nullptr) {
                    break;
                }
                if (serialPort->flag & IS_OPEN) {
                    serialPort->write(&argv[1], argc - 1);
                } else {
                    argv[0] = SERIAL_STATUS | portId;
                    argv[1] = SERIAL_IS_CLOSE;
                    fm->sendSysex(FirmataStream, SERIAL_MESSAGE, 2, argv);
                }
                break; // SERIAL_WRITE

            case SERIAL_READ:
                serialPort = getPortFromId(portId);
                if (serialPort == nullptr) {
                    break;
                }
                if ((serialPort->flag & IS_OPEN) == 0) {
                    argv[0] = SERIAL_STATUS | portId;
                    argv[1] = SERIAL_IS_CLOSE;
                    fm->sendSysex(FirmataStream, SERIAL_MESSAGE, 2, argv);
                    break;
                }
                if (argv[1] == SERIAL_READ_CONTINUOUSLY) {
                    // if (serialIndex + 1 >= SERIAL_NRS)
                    // {
                    //   break;
                    // }

                    // if (argc > 2)
                    // {
                    //   // maximum number of bytes to read from argvfer per iteration of loop()
                    //   rte_data.serialBytesToRead[portId] = argv[2];
                    // }
                    // else
                    // {
                    //   // read all available bytes per iteration of loop()
                    //   rte_data.serialBytesToRead[portId] = 0;
                    // }
                    // byte serialIndexToSkip = 0;
                    // for (byte i = 0; i < serialIndex + 1; i++)
                    // {
                    //   if (rte_data.reportSerial[i] == portId)
                    //   {
                    //     serialIndexToSkip = 1;
                    //     break;
                    //   }
                    // }
                    // if (0 == serialIndexToSkip)
                    // {
                    //   serialIndex++;
                    //   rte_data.reportSerial[serialIndex] = portId;
                    // }
                } else if (argv[1] == SERIAL_STOP_READING) {
                    byte serialIndexToSkip = 0;
                    if (serialIndex <= 0) {
                        serialIndex = -1;
                    } else {
                        for (byte i = 0; i < MIN(serialIndex + 1, SERIAL_NRS); i++) {
                            if (rte_data.firmata.reportSerial[i] == portId) {
                                serialIndexToSkip = i;
                                break;
                            }
                        }
                        // shift elements over to fill space left by removed element
                        for (byte i = serialIndexToSkip; i < MIN(serialIndex + 1, SERIAL_NRS); i++) {
                            if (i < (SERIAL_NRS - 1)) {
                                rte_data.firmata.reportSerial[i] = rte_data.firmata.reportSerial[i + 1];
                            }
                        }
                        serialIndex--;
                    }
                }
                break; // SERIAL_READ
            case SERIAL_CLOSE:
                serialPort = getPortFromId(portId);
                if (serialPort != nullptr) {
                    if (portId < 8) {
                        ((kSerial *) serialPort)->end();
                    } else {
#if defined(SoftwareSerial_h)
                        ((SoftwareSerial *)serialPort)->end();
                        if (serialPort != NULL)
                        {
                          free(serialPort);
                          serialPort = NULL;
                        }
#endif
                    }
                }
                break; // SERIAL_CLOSE
            case SERIAL_FLUSH:
                serialPort = getPortFromId(portId);
                if (serialPort != nullptr) {
                    getPortFromId(portId)->flush();
                }
                break; // SERIAL_FLUSH
#if defined(SoftwareSerial_h)
                case SERIAL_LISTEN:
                  // can only call listen() on software serial ports
                  if (portId > 7)
                  {
                    serialPort = getPortFromId(portId);
                    if (serialPort != NULL)
                    {
                      ((SoftwareSerial *)serialPort)->listen();
                    }
                  }
                  break; // SERIAL_LISTEN
#endif

            default:
                break;
        } // end switch
        return true;
    }
    return false;
}

void SerialFirmata::update(mFirmata *fm, nStream *FirmataStream) {
    checkSerial(fm, FirmataStream);
}

void SerialFirmata::reset() {
#if defined(SoftwareSerial_h)
    Stream *serialPort;
    // free memory allocated for SoftwareSerial ports
    for (byte i = SW_SERIAL0; i < SW_SERIAL3 + 1; i++)
    {
      serialPort = getPortFromId(i);
      if (serialPort != NULL)
      {
        free(serialPort);
        serialPort = NULL;
      }
    }
#endif

    serialIndex = -1;
    for (int i = 0; i < rte_data.firmata.serialBytesToRead_count; i++) {
        rte_data.firmata.serialBytesToRead[i] = 0;
#if defined(FIRMATA_SERIAL_RX_DELAY)
        lastBytesAvailable[i] = 0;
        lastBytesReceived[i] = 0;
#endif
    }
}

#if 0
// get a pointer to the serial port associated with the specified port id
Stream *SerialFirmata::getPortFromId(byte portId) {
    switch (portId) {
        case HW_SERIAL0:
            // block use of Serial (typically pins 0 and 1) until ability to reclaim Serial is implemented
            //return &Serial;
            return NULL;
#if defined(PIN_SERIAL1_RX)
            case HW_SERIAL1:
              return &Serial1;
#endif
#if defined(PIN_SERIAL2_RX)
            case HW_SERIAL2:
              return &Serial2;
#endif
#if defined(PIN_SERIAL3_RX)
            case HW_SERIAL3:
              return &Serial3;
#endif
#if defined(PIN_SERIAL4_RX)
            case HW_SERIAL4:
              return &Serial4;
#endif
#if defined(PIN_SERIAL5_RX)
            case HW_SERIAL5:
              return &Serial5;
#endif
#if defined(PIN_SERIAL6_RX)
            case HW_SERIAL6:
              return &Serial6;
#endif
#if defined(SoftwareSerial_h)
            case SW_SERIAL0:
              if (swSerial0 != NULL) {
                // instances of SoftwareSerial are already pointers so simply return the instance
                return swSerial0;
              }
              break;
            case SW_SERIAL1:
              if (swSerial1 != NULL) {
                return swSerial1;
              }
              break;
            case SW_SERIAL2:
              if (swSerial2 != NULL) {
                return swSerial2;
              }
              break;
            case SW_SERIAL3:
              if (swSerial3 != NULL) {
                return swSerial3;
              }
              break;
#endif
        default:
            break;
    }
    return NULL;
}
#endif

// Check serial ports that have READ_CONTINUOUS mode set and relay any data
// for each port to the device attached to that port.
void SerialFirmata::checkSerial(mFirmata *fm, nStream *FirmataStream) {
    byte portId;
    int serialData;
    int bytesToRead;
    int numBytesToRead;
    kSerial *serialPort;

    if (serialIndex > -1) {

#if defined(FIRMATA_SERIAL_RX_DELAY)
        unsigned long currentMillis = millis();
#endif

        // loop through all reporting (READ_CONTINUOUS) serial ports
        for (byte i = 0; i < MIN(serialIndex + 1, SERIAL_NRS); i++) {
            portId = rte_data.firmata.reportSerial[i];
            bytesToRead = rte_data.firmata.serialBytesToRead[portId];
            serialPort = getPortFromId(portId);
            if (serialPort == nullptr) {
                continue;
            }
#if defined(SoftwareSerial_h)
            // only the SoftwareSerial port that is "listening" can read data
            if (portId > 7 && !((SoftwareSerial *)serialPort)->isListening())
            {
              continue;
            }
#endif
            int bytesAvailable = serialPort->available();
            if (bytesAvailable > 0) {
#if defined(FIRMATA_SERIAL_RX_DELAY)
                if (bytesAvailable > lastBytesAvailable[portId])
                {
                  lastBytesReceived[portId] = currentMillis;
                }
                lastBytesAvailable[portId] = bytesAvailable;
#endif
                if (bytesToRead <= 0 || (bytesAvailable <= bytesToRead)) {
                    numBytesToRead = bytesAvailable;
                } else {
                    numBytesToRead = bytesToRead;
                }
#if defined(FIRMATA_SERIAL_RX_DELAY)
                if (maxRxDelay[portId] >= 0 && numBytesToRead > 0)
                {
                  // read and send immediately only if
                  // - expected bytes are unknown and the receive argvfer has reached 50 %
                  // - expected bytes are available
                  // - maxRxDelay has expired since last receive (or time counter wrap)
                  if (!((bytesToRead <= 0 && bytesAvailable >= SERIAL_RX_BUFFER_SIZE / 2) || (bytesToRead > 0 && bytesAvailable >= bytesToRead) || (maxRxDelay[portId] > 0 && (currentMillis < lastBytesReceived[portId] || (currentMillis - lastBytesReceived[portId]) >= maxRxDelay[portId]))))
                  {
                    // delay
                    numBytesToRead = 0;
                  }
                }
#endif
                // relay serial data to the serial device
                if (numBytesToRead > 0) {
#if defined(FIRMATA_SERIAL_RX_DELAY)
                    lastBytesAvailable[portId] -= numBytesToRead;
#endif
                    byte *buf = (byte *) malloc(numBytesToRead + 1);
                    // relay serial data to the serial device
                    buf[0] = SERIAL_REPLY | portId;
                    int count = 1;
                    count += serialPort->readBytes((char *) &buf[1], numBytesToRead);
                    fm->sendSysex(FirmataStream, SERIAL_MESSAGE, count, buf);
                    // fm->flush(FirmataStream);
                    free(buf);
                }
            }
        }
    }
}

bool SerialFirmata::IS_PIN_SERIAL(byte pin) {
    return false;
}

#endif
