#include "targets.h"
#include "eeprom.h"
#include "stm32h563xx.h"

#include <string.h>
// !!!!!!!!!!!!!!!!!!!
// Per reference manual:
// The application software must not unlock an
// already unlocked register, otherwise this
// register remains locked until the next system reset.
// !!!!!!!!!!!!!!!!!!!!

// #define APP_START (uint32_t)0x08001000
// #define FLASH_STORAGE 0x08005000  // at the 31kb mark
// #define page_size 0x1800 // 1 kb for f051
// #define page_size 0x2000 // 1 kb for f051

uint32_t FLASH_FKEY1 = 0x45670123;
uint32_t FLASH_FKEY2 = 0xCDEF89AB;

void save_flash_nolib(uint8_t* data, int length, uint32_t add)
{
    uint16_t data_to_FLASH[length / 2];
    memset(data_to_FLASH, 0, length / 2);
    for (int i = 0; i < length / 2; i++) {
        data_to_FLASH[i] = data[i * 2 + 1] << 8 | data[i * 2]; // make 16 bit
    }
    volatile uint32_t data_length = length / 2;

    while (flash_busy());

    // flash_erase_sector((add - FLASH_BASE)/FLASH_PAGE_SIZE);
    
    flash_erase_sector((add - EEPROM_BASE)/EEPROM_PAGE_SIZE);
    
    volatile uint32_t write_cnt = 0, index = 0;
    while (index < data_length) {
        flash_program_word(data_to_FLASH[index], (add + write_cnt));
        write_cnt += 2;
        index++;
    }
}

void read_flash_bin(uint8_t* data, uint32_t add, int out_buff_len)
{
    // FLASH->EDATA1R_CUR |= 1 << 15;
    uint16_t readData[10];
    // uint32_t readData[10];
    int length = out_buff_len / 2;
    // volatile uint32_t read_data;
    for (int i = 0; i < length; i++) {
        readData[i] = *(uint16_t*)(add + i*2);
        // readData[i] = *(uint32_t*)(add + i);
    }
}

void flash_erase_sector(uint8_t sector)
{
    // sector out of range
    // if (sector > FLASH_MAX_SECTORS - 1) {
    //     return;
    // }
    // sector must be aligned to page size
    if (sector%EEPROM_PAGE_SIZE) {
        return;
    }

    while (flash_busy());
    while (flash_dbne());


    flash_unlock();

    FLASH->NSCR &= ~FLASH_CR_SNB_Msk;
    FLASH->NSCR |= FLASH_CR_SER |
    120 << FLASH_CR_SNB_Pos;
    // sector << FLASH_CR_SNB_Pos;
    FLASH->NSCR |= FLASH_CR_START;

    while (flash_busy());

    FLASH->NSCR &= ~FLASH_CR_SER;
    flash_lock();
}

bool flash_busy()
{
    return FLASH->NSSR & FLASH_SR_BSY;
}

bool flash_dbne()
{
    return FLASH->NSSR & FLASH_SR_DBNE;
}

void flash_unlock()
{
    // unlock the flash
    if ((FLASH->NSCR & FLASH_CR_LOCK) != 0) {
        FLASH->NSKEYR = FLASH_FKEY1;
        FLASH->NSKEYR = FLASH_FKEY2;
    }
}

void flash_lock()
{
    SET_BIT(FLASH->NSCR, FLASH_CR_LOCK);
}

void flash_program_word(uint16_t word, uint32_t address)
{
    while (flash_busy());
    while (flash_dbne());

    while (flash_wbne());

    flash_unlock();

    flash_enable_write();

    *(__IO uint16_t*)address = word;
    while (flash_busy());
    flash_disable_write();
    flash_lock();
}

void flash_enable_write()
{
    FLASH->NSCR |= FLASH_CR_PG;
}

void flash_disable_write()
{
    FLASH->NSCR &= ~FLASH_CR_PG;
}
bool flash_wbne()
{
    // return FLASH->NSSR & FLASH_SR_WBNE;
    return false;
}
