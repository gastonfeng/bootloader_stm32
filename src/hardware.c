/* *****************************************************************************
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
 * ****************************************************************************/

/**
 *  @file hardware.c
 *
 *  @brief init routines to setup clocks, interrupts, also destructor functions.
 *  does not include USB stuff. EEPROM read/write functions.
 *
 */
#include "config.h"
#include "hardware.h"
/*
void setPin(uint32_t bank, uint8_t pin) {
    uint32_t pinMask = 0x1 << (pin);
    SET_REG(GPIO_BSRR(bank), pinMask);
}

void resetPin(uint32_t bank, uint8_t pin) {
    uint32_t pinMask = 0x1 << (16 + pin);
    SET_REG(GPIO_BSRR(bank), pinMask);
}
*/
void gpio_write_bit(uint32_t bank, uint8_t pin, uint8_t val) {
    val = !val;          // "set" bits are lower than "reset" bits
    SET_REG(GPIO_BSRR(bank), (1U << pin) << (16 * val));
}

bool readPin(uint32_t bank, uint8_t pin) {
    // todo, implement read
    if (GET_REG(GPIO_IDR(bank)) & (0x01 << pin)) {
        return 1;
    } else {
        return 0;
    }
}

bool readButtonState() {
    // todo, implement read
    bool state=0;
#if defined(BUTTON_BANK) && defined (BUTTON_PIN) && defined (BUTTON_PRESSED_STATE)
    if (GET_REG(GPIO_IDR(BUTTON_BANK)) & (0x01 << BUTTON_PIN))
    {
        state = 1;
    }

    if (BUTTON_PRESSED_STATE==0)
    {
        state=!state;
    }
#endif
    return state;
}

void strobePin(uint32_t bank, uint8_t pin, uint8_t count, uint32_t rate,uint8_t onState)
{
    gpio_write_bit( bank,pin,1-onState);

    uint32_t c;
    while (count-- > 0)
    {
        for (c = rate; c > 0; c--)
        {
            asm volatile("nop");
        }

        gpio_write_bit( bank,pin,onState);

        for (c = rate; c > 0; c--)
        {
            asm volatile("nop");
        }
        gpio_write_bit( bank,pin,1-onState);
    }
}

void systemReset(void) {
    SET_REG(RCC_CR, GET_REG(RCC_CR)     | 0x00000001);
    SET_REG(RCC_CFGR, GET_REG(RCC_CFGR) & 0xF8FF0000);
    SET_REG(RCC_CR, GET_REG(RCC_CR)     & 0xFEF6FFFF);
    SET_REG(RCC_CR, GET_REG(RCC_CR)     & 0xFFFBFFFF);
    SET_REG(RCC_CFGR, GET_REG(RCC_CFGR) & 0xFF80FFFF);

    SET_REG(RCC_CIR, 0x00000000);  /* disable all RCC interrupts */
}


void setupFLASH() {
    /* configure the HSI oscillator */
    if ((pRCC->CR & 0x01) == 0x00) {
        uint32_t rwmVal = pRCC->CR;
        rwmVal |= 0x01;
        pRCC->CR = rwmVal;
    }

    /* wait for it to come on */
    while ((pRCC->CR & 0x02) == 0x00) {}
}

bool checkUserCode(uint32_t usrAddr) {
    uint32_t sp = *(volatile uint32_t *) usrAddr;

    if ((sp & 0x2FFE0000) == 0x20000000) {
        return (1);
    } else {
        return (0);
    }
}

void setMspAndJump(uint32_t usrAddr) {
    // Dedicated function with no call to any function (appart the last call)
    // This way, there is no manipulation of the stack here, ensuring that GGC
    // didn't insert any pop from the SP after having set the MSP.
    typedef void (*funcPtr)(void);
    uint32_t jumpAddr = *(volatile uint32_t *)(usrAddr + 0x04); /* reset ptr in vector table */

    funcPtr usrMain = (funcPtr) jumpAddr;

    SET_REG(SCB_VTOR, (volatile uint32_t) (usrAddr));

    asm volatile("msr msp, %0"::"g"(*(volatile uint32_t *)usrAddr));

    usrMain();                                /* go! */
}


void jumpToUser(uint32_t usrAddr) {

    /* tear down all the dfu related setup */
    // disable usb interrupts, clear them, turn off usb, set the disc pin
    // todo pick exactly what we want to do here, now its just a conservative
    HAL_FLASH_Lock();
    // usbDsbISR();
    // nvicDisableInterrupts();

#ifndef HAS_MAPLE_HARDWARE
    // usbDsbBus();
#endif

// Does nothing, as PC12 is not connected on teh Maple mini according to the schemmatic     setPin(GPIOC, 12); // disconnect usb from host. todo, macroize pin
    systemReset(); // resets clocks and periphs, not core regs

    setMspAndJump(usrAddr);
}

void bkp10Write(uint16_t value)
{
        // Enable clocks for the backup domain registers
        pRCC->APB1ENR |= (RCC_APB1ENR_PWR_CLK | RCC_APB1ENR_BKP_CLK);

        // Disable backup register write protection
        pPWR->CR |= PWR_CR_DBP;

        // store value in pBK DR10
        pBKP->DR10 = value;

        // Re-enable backup register write protection
        pPWR->CR &=~ PWR_CR_DBP;
}

int checkAndClearBootloaderFlag()
{
    bool flagSet = 0x00;// Flag not used

    // Enable clocks for the backup domain registers
    pRCC->APB1ENR |= (RCC_APB1ENR_PWR_CLK | RCC_APB1ENR_BKP_CLK);

    switch (pBKP->DR10)
    {
        case RTC_BOOTLOADER_FLAG:
            flagSet = 0x01;
            break;
        case RTC_BOOTLOADER_JUST_UPLOADED:
            flagSet = 0x02;
            break;
    }

    if (flagSet!=0x00)
    {
        bkp10Write(0x0000);// Clear the flag
        // Disable clocks
        pRCC->APB1ENR &= ~(RCC_APB1ENR_PWR_CLK | RCC_APB1ENR_BKP_CLK);
    }



    return flagSet;
}




void systemHardReset(void) {
    SCB_TypeDef *rSCB = (SCB_TypeDef *) SCB_BASE;

    /* Reset  */
    rSCB->AIRCR = (uint32_t)AIRCR_RESET_REQ;

    /*  should never get here */
    while (1) {
        asm volatile("nop");
    }
}



#define FLASH_SIZE_REG 0x1FFFF7E0
int getFlashEnd(void)
{
    unsigned short *flashSize = (unsigned short *) (FLASH_SIZE_REG);// Address register
    return ((int)(*flashSize & 0xffff) * 1024) + 0x08000000;
}

int getFlashPageSize(void)
{

    unsigned short *flashSize = (unsigned short *) (FLASH_SIZE_REG);// Address register
    if ((*flashSize & 0xffff) > 128)
    {
        return 0x800;
    }
    else
    {
        return 0x400;
    }
}
