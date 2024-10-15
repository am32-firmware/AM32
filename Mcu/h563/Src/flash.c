#include "flash.h"
#include "stm32h5xx.h"

static const uint32_t FLASH_FKEY1 = 0x45670123;
static const int32_t FLASH_FKEY2 = 0xCDEF89AB;
#define FLASH_MAX_SECTORS (128)
#define EEPROM_PAGE_SIZE (0x2000)
void flash_erase_sector(uint8_t sector)
{
    // sector out of range
    if (sector > FLASH_MAX_SECTORS - 1) {
        return;
    }
    // // sector must be aligned to page size
    // if (sector%EEPROM_PAGE_SIZE) {
    //     return;
    // }

    while (flash_busy());
    while (flash_dbne());


    flash_unlock();

    FLASH->NSCR &= ~FLASH_CR_SNB_Msk;
    FLASH->NSCR |= FLASH_CR_SER | (sector << FLASH_CR_SNB_Pos);
    // sector << FLASH_CR_SNB_Pos;
    FLASH->NSCR |= FLASH_CR_START;

    while (flash_busy());
    while (flash_dbne());

    // maybe not necessary
    // FLASH->NSCR &= ~FLASH_CR_SNB_Msk;

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
    while (FLASH->NSCR & FLASH_CR_LOCK)
    {};
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
