/*
 * peripherals.c
 *
 *  Created on: 4. 24, 2026
 *      Author: Nong Jun
 */
#include "eeprom.h"
#include "stdlib.h"
#include <string.h>

void save_flash_nolib(uint8_t *data, int length, uint32_t address)
{
    uint32_t index = 0;
    uint32_t pAddress = address;
    uint32_t data_length = length / 4;
    
    uint32_t *pFlashBuff =(uint32_t *) malloc(sizeof(uint32_t)*data_length);
    if(pFlashBuff==NULL)
    {
        return ;
    }

    __disable_irq();

    memset(pFlashBuff, 0, length/4);
    for(int i = 0; i < length / 4 ; i ++ )
    {
        pFlashBuff[i] =  (data[i*4+3]<<24)|(data[i*4+2]<<16)|
                            (data[i*4+1]<<8) | data[i*4];
    }
    
    DDL_FLASH_RKEY_Unlock();
    DDL_FLASH_MKEY_Unlock();
    
    if(pAddress%512==0)
    {
        while(DDL_FLASH_IsActiveFlag_BUSY());
        DDL_FLASH_SetOperationMode(DDL_FLASH_OPERATE_SECTORERASE);
        *((volatile uint32_t *)pAddress) = 0xA5A5;
        while(DDL_FLASH_IsActiveFlag_BUSY());
    }
    while(index < data_length)
    {
        if( *(__IO uint32_t*)pAddress == 0xFFFFFFFF)
        {
            DDL_FLASH_SetOperationMode(DDL_FLASH_OPERATE_WRITE);
            *((uint32_t *)pAddress) = pFlashBuff[index];
            while (DDL_FLASH_IsActiveFlag_BUSY());
        }
        
        index++;
        pAddress += 4;
    }
    
    DDL_FLASH_MKEY_Lock();
    DDL_FLASH_RKEY_Lock();
    
    free(pFlashBuff);
    __enable_irq();
}

void read_flash_bin(uint8_t* data, uint32_t add, int out_buff_len)
{
    // volatile uint32_t read_data;
    for (int i = 0; i < out_buff_len; i++) {
        data[i] = *(uint8_t*)(add + i);
    }
}
