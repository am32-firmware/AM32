/*
 *   modified by TempersLee June 21,2024 for CH32V203
 * */

#include "eeprom.h"
#include <string.h>
#include "debug.h"


#define page_size 0x100 // 1 kb for f051

#define FLASH_KEY1                 ((uint32_t)0x45670123)
#define FLASH_KEY2                 ((uint32_t)0xCDEF89AB)

void save_flash_nolib(uint8_t* data, int length, uint32_t add)
{
    volatile uint32_t start_addr/*,page_num*/;
    uint16_t data_to_FLASH[128];
    memset(data_to_FLASH, 0, 128);
    while ((FLASH->STATR & FLASH_STATR_BSY) != 0)
    {
        /*  add time-out*/
    }
    {
        /* Authorize the FPEC of Bank1 Access */
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;

        /* Fast program mode unlock */
        FLASH->MODEKEYR = FLASH_KEY1;
        FLASH->MODEKEYR = FLASH_KEY2;
    }

    // erase page if address even divisable by 256
    if((add % 256) == 0)
    {
        FLASH_ErasePage_Fast(add);
    }

    start_addr = (add & 0xFFFFFF00);  //256字节对齐  0x08001002
    for(int i=0;i<128;i++)
    {
        data_to_FLASH[i] = *(  (volatile uint16_t *)( start_addr + 2*i )  ); //读取原来的值
    }

    for(int j=0;j<length/2;j++)
    {
        data_to_FLASH[(add-start_addr)/2 + j] = data[j*2+1] << 8 | data[j*2];
    }

    FLASH_ProgramPage_Fast(start_addr,((uint32_t *)data_to_FLASH));
    FLASH_Lock( );
}

void read_flash_bin(uint8_t* data, uint32_t add, int out_buff_len)
{
    // volatile uint32_t read_data;
    for (int i = 0; i < out_buff_len; i++) {
        data[i] = *(uint8_t*)(add + i);
    }
}
