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

#ifndef __HARDWARE_H
#define __HARDWARE_H

#include <Arduino.h>
// #include "stm32f10x_type.h"
// #include "cortexm3_macro.h"
// #include "common.h"

/* macro'd register and peripheral definitions */
#define RCC   ((uint32_t)0x40021000)
#define FLASH ((uint32_t)0x40022000)
#define GPIOA ((uint32_t)0x40010800)
#define GPIOB ((uint32_t)0x40010C00)
#define GPIOC ((uint32_t)0x40011000)
#define GPIOD ((uint32_t)0x40011400)
#define GPIOE ((uint32_t)0x40011800)
#define GPIOF ((uint32_t)0x40011C00)
#define GPIOG ((uint32_t)0x40012000)
#define AFIO_BASE               ((uint32_t)0x40010000)
#define AFIO_MAPR               (AFIO_BASE + 0x04)


#define RCC_CR      RCC
#define RCC_CFGR    (RCC + 0x04)
#define RCC_CIR     (RCC + 0x08)
#define RCC_AHBENR  (RCC + 0x14)
#define RCC_APB2ENR (RCC + 0x18)
#define RCC_APB1ENR (RCC + 0x1C)

#define FLASH_ACR     (FLASH + 0x00)
#define FLASH_KEYR    (FLASH + 0x04)
#define FLASH_OPTKEYR (FLASH + 0x08)
#define FLASH_SR      (FLASH + 0x0C)
#define FLASH_CR      (FLASH + 0x10)
#define FLASH_AR      (FLASH + 0x14)
#define FLASH_OBR     (FLASH + 0x1C)
#define FLASH_WRPR    (FLASH + 0x20)

#define FLASH_KEY1     0x45670123
#define FLASH_KEY2     0xCDEF89AB
#define FLASH_RDPRT    0x00A5
#define FLASH_SR_BSY   0x01
#define FLASH_CR_PER   0x02
#define FLASH_CR_PG    0x01
#define FLASH_CR_START 0x40

#define GPIO_CRL(port)  port
#define GPIO_CRH(port)  (port+0x04)
#define GPIO_IDR(port)  (port+0x08)
#define GPIO_ODR(port)  (port+0x0c)
#define GPIO_BSRR(port) (port+0x10)
#define GPIO_CR(port,pin) (port + (0x04*(pin>7)))

#define CR_OUTPUT_OD        0x05
#define CR_OUTPUT_PP        0x01
#define CR_INPUT            0x04
#define CR_INPUT_PU_PD      0x08

#define SCS_BASE   ((uint32_t)0xE000E000)
#define NVIC_BASE  (SCS_BASE + 0x0100)
#define SCB_BASE   (SCS_BASE + 0x0D00)


#define SCS      0xE000E000
#define NVIC     (SCS+0x100)
#define SCB      (SCS+0xD00)
#define STK      (SCS+0x10)

#define SCB_VTOR (SCB+0x08)
#define STK_CTRL (STK+0x00)

#define TIM1_APB2_ENB ((uint32_t)0x00000800)
#define TIM1          ((uint32_t)0x40012C00)
#define TIM1_PSC      (TIM1+0x28)
#define TIM1_ARR      (TIM1+0x2C)
#define TIM1_RCR      (TIM1+0x30)
#define TIM1_CR1      (TIM1+0x00)
#define TIM1_CR2      (TIM1+0x04)
#define TIM1_DIER     (TIM1+0x0C)
#define TIM1_UP_IRQ_Channel ((uint8_t)0x19)

#define USB_HP_IRQ  ((uint8_t)0x13)
#define USB_LP_IRQ  ((uint8_t)0x14)
#define TIM2_IRQ    ((uint8_t)0x1C)


/* AIRCR  */
#define AIRCR_RESET         0x05FA0000
#define AIRCR_RESET_REQ     (AIRCR_RESET | (uint32_t)0x04);

/* temporary copyage of example from kiel */
#define __VAL(__TIMCLK, __PERIOD) ((__TIMCLK/1000000UL)*__PERIOD)
#define __PSC(__TIMCLK, __PERIOD)  (((__VAL(__TIMCLK, __PERIOD)+49999UL)/50000UL) - 1)
#define __ARR(__TIMCLK, __PERIOD) ((__VAL(__TIMCLK, __PERIOD)/(__PSC(__TIMCLK, __PERIOD)+1)) - 1)

// SWD and JTAG DEBUGGING
#define AFIO_MAPR_SWJ_CFG                      (0x7 << 24)
#define AFIO_MAPR_SWJ_CFG_FULL_SWJ             (0x0 << 24)
#define AFIO_MAPR_SWJ_CFG_FULL_SWJ_NO_NJRST    (0x1 << 24)
#define AFIO_MAPR_SWJ_CFG_NO_JTAG_SW           (0x2 << 24)
#define AFIO_MAPR_SWJ_CFG_NO_JTAG_NO_SW        (0x4 << 24)




// more bit twiddling to set Control register bits
#define CR_SHITF(pin) ((pin - 8*(pin>7))<<2)

#define SET_REG(addr,val) do { *(volatile uint32_t*)(addr)=val; } while(0)
#define GET_REG(addr)     (*(volatile uint32_t*)(addr))


/* todo: there must be some major misunderstanding in how we access
   regs. The direct access approach (GET_REG) causes the usb init to
   fail upon trying to activate RCC_APB1 |= 0x00800000. However, using
   the struct approach from ST, it works fine...temporarily switching
   to that approach */
typedef struct {
    volatile uint32_t CR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t APB2RSTR;
    volatile uint32_t APB1RSTR;
    volatile uint32_t AHBENR;
    volatile uint32_t APB2ENR;
    volatile uint32_t APB1ENR;
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
} RCC_RegStruct;
#define pRCC ((RCC_RegStruct *) RCC)

typedef struct {
    volatile uint32_t ISER[2];
    uint32_t  RESERVED0[30];
    volatile uint32_t ICER[2];
    uint32_t  RSERVED1[30];
    volatile uint32_t ISPR[2];
    uint32_t  RESERVED2[30];
    volatile uint32_t ICPR[2];
    uint32_t  RESERVED3[30];
    volatile uint32_t IABR[2];
    uint32_t  RESERVED4[62];
    volatile uint32_t IPR[15];
} NVIC_TypeDef;

typedef struct {
    uint8_t NVIC_IRQChannel;
    uint8_t NVIC_IRQChannelPreemptionPriority;
    uint8_t NVIC_IRQChannelSubPriority;
    bool NVIC_IRQChannelCmd; /* TRUE for enable */
} NVIC_InitTypeDef;

typedef struct {
    volatile unsigned long CPUID;
    volatile uint32_t ICSR;
    volatile uint32_t VTOR;
    volatile uint32_t AIRCR;
    volatile uint32_t SCR;
    volatile uint32_t CCR;
    volatile uint32_t SHPR[3];
    volatile uint32_t SHCSR;
    volatile uint32_t CFSR;
    volatile uint32_t HFSR;
    volatile uint32_t DFSR;
    volatile uint32_t MMFAR;
    volatile uint32_t BFAR;
    volatile uint32_t AFSR;
} SCB_TypeDef;


/** Power interface register map. */
typedef struct pwr_reg_map {
    volatile uint32_t CR;      /**< Control register */
    volatile uint32_t CSR;     /**< Control and status register */
} pwr_reg_map;

/** Power peripheral register map base pointer. */
#define pPWR                        ((struct pwr_reg_map*)0x40007000)

/** Disable backup domain write protection bit */
#define PWR_CR_DBP                  (1 << 8)


/** Backup peripheral register map type. */
typedef struct bkp_reg_map {
    const uint32_t RESERVED1; ///< Reserved
    volatile uint16_t DR1;            ///< Data register 1
    const uint16_t RESERVED2;
    volatile uint16_t DR2;            ///< Data register 2
    const uint16_t RESERVED3;
    volatile uint16_t DR3;            ///< Data register 3
    const uint16_t RESERVED4;
    volatile uint16_t DR4;            ///< Data register 4
    const uint16_t RESERVED5;
    volatile uint16_t DR5;            ///< Data register 5
    const uint16_t RESERVED6;
    volatile uint16_t DR6;            ///< Data register 6
    const uint16_t RESERVED7;
    volatile uint16_t DR7;            ///< Data register 7
    const uint16_t RESERVED8;
    volatile uint16_t DR8;            ///< Data register 8
    const uint16_t RESERVED9;
    volatile uint16_t DR9;            ///< Data register 9
    const uint16_t RESERVED10;
    volatile uint16_t DR10;           ///< Data register 10
    const uint16_t RESERVED11;
    volatile uint32_t RTCCR;          ///< RTC control register
    volatile uint32_t CR;             ///< Control register
    volatile uint32_t CSR;            ///< Control and status register
} bkp_reg_map;

/** Backup peripheral register map base pointer. */
#define pBKP ((struct bkp_reg_map *)0x40006C00)
#if defined(__cplusplus)
extern "C"
{
    #endif
    void bkp10Write(uint16_t value);

    //void setPin(uint32_t bank, uint8_t pin);
    //void resetPin(uint32_t bank, uint8_t pin);
    void gpio_write_bit(uint32_t bank, uint8_t pin, uint8_t val);
    unsigned int crMask(int pin);

    bool readPin(uint32_t bank, uint8_t pin);
    void strobePin(uint32_t bank, uint8_t pin, uint8_t count, uint32_t rate, uint8_t onState);

    void systemHardReset(void);
    void systemReset(void);
    void setupCLK(void);
    void setupLEDAndButton(void);
    void setupFLASH(void);
    bool checkUserCode(uint32_t usrAddr);
    void jumpToUser(uint32_t usrAddr);
    int checkAndClearBootloaderFlag();

    bool flashWriteWord(uint32_t addr, uint32_t word);
    bool flashErasePage(uint32_t addr);
    bool flashErasePages(uint32_t addr, uint16_t n);
    void flashLock(void);
    void flashUnlock(void);

    int getFlashEnd(void);
    int getFlashPageSize(void);
#if defined(__cplusplus)
}
#endif

#endif
