#include <Arduino.h>
#include "flashFs.h"
#include "ch423.h"
#include "hardware.h"
#include "config.h"
ch423 ch;

#define BLOCKSIZE 256
u8_t buf[BLOCKSIZE];
void setup()
{
    ch.setChar('B', 3);
    ch.update();
    FlashFile *fd = new FlashFile("porg.flag", SPIFFS_RDONLY);
    if (fd->fd)
    {
        ch.setChar('P', 3);
        ch.update();
        FlashFile *bin = new FlashFile("prog.bin", SPIFFS_RDONLY);
        setupFLASH();
        HAL_FLASH_Unlock();
        // Clear lower memory so that we can check on cold boot, whether the last upload was to 0x8002000 or 0x8005000
        // flashErasePage((uint32_t)USER_CODE_FLASH0X8002000);
        bkp10Write(RTC_BOOTLOADER_JUST_UPLOADED);
        for (int i = 0; i < bin->size(); i++)
        {
            ch.setChar('W', 3);
            ch.update();
            bin->read((char *)buf, BLOCKSIZE);
            for (int j = 0; j < BLOCKSIZE; j += 4)
                HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, USER_CODE_FLASH0X8008000 + BLOCKSIZE * i + j, buf[j]);
        }
        bin->close();
        fd->close();
        fd->remove();
    }
    ch.setChar('C', 3);
    ch.update();
}
void loop()
{
    if (checkUserCode(USER_CODE_FLASH0X8008000))
    {
        jumpToUser(USER_CODE_FLASH0X8008000);
    }
    else
    {
        if (checkUserCode(USER_CODE_FLASH0X8005000))
        {
            jumpToUser(USER_CODE_FLASH0X8005000);
        }
        else
        {
            // Nothing to execute in either Flash or RAM
            // strobePin(LED_BANK, LED_PIN, 5, BLINK_FAST,LED_ON_STATE);
            systemHardReset();
        }
    }
}