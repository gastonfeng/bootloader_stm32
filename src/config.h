/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 LeafLabs LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *****************************************************************************/

/**
 *  @file config.h
 *
 *  @brief bootloader settings and macro defines
 *
 *
 */

#ifndef __CONFIG_H
#define __CONFIG_H

// #include "common.h"



/* Speed controls for strobing the LED pin */
#define BLINK_FAST 0x50000
#define BLINK_SLOW 0x100000


// The USB clock is the same for all boards
#define RCC_APB1ENR_USB_CLK   0x00800000

// Clocks for the backup domain registers
#define RCC_APB1ENR_PWR_CLK   0x10000000
#define RCC_APB1ENR_BKP_CLK   0x08000000


// Use the usb_description_strings_util.html to make new strngs for the next 3 arrays if you need to change the text.

#define ALT0_STR_LEN 0x80
#define ALT0_MSG_STR 'S',0,'T',0,'M',0,'3',0,'2',0,'d',0,'u',0,'i',0,'n',0,'o',0,' ',0,'b',0,'o',0,'o',0,'t',0,'l',0,'o',0,'a',0,'d',0,'e',0,'r',0,' ',0,'v',0,'1',0,'.',0,'0',0,' ',0,' ',0,'E',0,'R',0,'R',0,'O',0,'R',0,'.',0,' ',0,'U',0,'p',0,'l',0,'o',0,'a',0,'d',0,' ',0,'t',0,'o',0,' ',0,'R',0,'A',0,'M',0,' ',0,'n',0,'o',0,'t',0,' ',0,'s',0,'u',0,'p',0,'p',0,'o',0,'r',0,'t',0,'e',0,'d',0,'.',0

#define ALT1_STR_LEN 0x6C
#define ALT1_MSG_STR 'S',0,'T',0,'M',0,'3',0,'2',0,'d',0,'u',0,'i',0,'n',0,'o',0,' ',0,'b',0,'o',0,'o',0,'t',0,'l',0,'o',0,'a',0,'d',0,'e',0,'r',0,' ',0,'v',0,'1',0,'.',0,'0',0,' ',0,' ',0,'U',0,'p',0,'l',0,'o',0,'a',0,'d',0,' ',0,'t',0,'o',0,' ',0,'F',0,'l',0,'a',0,'s',0,'h',0,' ',0,'0',0,'x',0,'8',0,'0',0,'0',0,'5',0,'0',0,'0',0,'0',0

#define ALT2_STR_LEN 0x6C
#define ALT2_MSG_STR 'S',0,'T',0,'M',0,'3',0,'2',0,'d',0,'u',0,'i',0,'n',0,'o',0,' ',0,'b',0,'o',0,'o',0,'t',0,'l',0,'o',0,'a',0,'d',0,'e',0,'r',0,' ',0,'v',0,'1',0,'.',0,'0',0,' ',0,' ',0,'U',0,'p',0,'l',0,'o',0,'a',0,'d',0,' ',0,'t',0,'o',0,' ',0,'F',0,'l',0,'a',0,'s',0,'h',0,' ',0,'0',0,'x',0,'8',0,'0',0,'0',0,'2',0,'0',0,'0',0,'0',0

// Jump locations for legacy bootloader (0x8005000) and new / smaller bootloder (0x8002000)
#define USER_CODE_FLASH0X8005000    ((uint32_t)0x08005000)
#define USER_CODE_FLASH0X8002000    ((uint32_t)0x08002000)
#define USER_CODE_FLASH0X8008000    ((uint32_t)0x08008000)

#define LARGEST_FLASH_PAGE_SIZE 0x800

// Upload to RAM has been removed / depreacted so these values a not used any more
#define USER_CODE_RAM               ((uint32_t)0x20000C00)

// RAM_END, set ram end to the end of ram on the device wth the least RAM (STM32F103C)
#define RAM_END                     ((uint32_t)0x20005000)

/* Porting information Please read.

    These defineds are use to setup the hardware of the GPIO.
    See http://www.st.com/web/en/resource/technical/document/reference_manual/CD00171190.pdf

    Two GPIO pins need to be defined, the LED and the Button.

    For each pin, the following is required

    LED_BANK, this is the GPIO port, e.g. GPIOA,GPIOB, GPIOC etc etc etc
    LED_PIN, this is the pin number e.g PB1 = 1
    LED_ON_STATE is whether the pin needs to be 1 or 0 for the LED to be lit, this is needed because on some boards the led is wired between Vcc and the Pin
    instead of from the pin to GND

    BUTTON_BANK   Port name of the pin used for the button
    #define BUTTON_PIN
    #define BUTTON_PRESSED_STATE 1
*/


// Check if button pulldown should be enabled 
// Default to True as this was the default prior to needing to disable it
// in order to use the boot1 pin on the Blue Pill which has a very week pullup
#ifndef BUTTON_INPUT_MODE
	#define BUTTON_INPUT_MODE 	CR_INPUT_PU_PD
#endif

#define STARTUP_BLINKS 5
#ifndef BOOTLOADER_WAIT
#ifdef BUTTON_BANK
    #define BOOTLOADER_WAIT 6
#else
    #define BOOTLOADER_WAIT 30
#endif
#endif

// defines for USB (DONT CHANGE)
#define VEND_ID0 0xAF
#define VEND_ID1 0x1E
#define PROD_ID0 0x03
#define PROD_ID1 0x00

// Value to place in RTC backup register 10 for persistent bootloader mode
#define RTC_BOOTLOADER_FLAG 0x424C
#define RTC_BOOTLOADER_JUST_UPLOADED 0x424D
#endif
