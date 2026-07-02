/*
 * eeprom_stm32g.h
 *
 * Shared STM32G0/G4 flash driver body (save_flash_nolib / read_flash_bin),
 * double-word programming, erase-by-page-number via FLASH_CR_PER + PNB.
 *
 * #included at the end of Mcu/{g031,g071,g431}/Src/eeprom.c. Not a
 * standalone translation unit. G0 names the flash busy flag FLASH_SR_BSY1,
 * G4 names it FLASH_SR_BSY — same bit position, aliased below.
 */

#if defined(FLASH_SR_BSY1)
#define EEPROM_FLASH_SR_BUSY FLASH_SR_BSY1
#else
#define EEPROM_FLASH_SR_BUSY FLASH_SR_BSY
#endif

uint32_t FLASH_FKEY1 = 0x45670123;
uint32_t FLASH_FKEY2 = 0xCDEF89AB;

void save_flash_nolib(uint8_t* data, int length, uint32_t add)
{
    uint32_t data_to_FLASH[length / 4];
    memset(data_to_FLASH, 0, length / 4);
    for (int i = 0; i < length / 4; i++) {
        data_to_FLASH[i] = data[i * 4 + 3] << 24 | data[i * 4 + 2] << 16 | data[i * 4 + 1] << 8 | data[i * 4]; // make 16 bit
    }
    volatile uint32_t data_length = length / 4;

    // unlock flash

    while ((FLASH->SR & EEPROM_FLASH_SR_BUSY) != 0) {
        /*  add time-out*/
    }
    if ((FLASH->CR & FLASH_CR_LOCK) != 0) {
        FLASH->KEYR = FLASH_FKEY1;
        FLASH->KEYR = FLASH_FKEY2;
    }

    // erase page if address even divisable by 2048
    if ((add % 2048) == 0) {
        FLASH->CR |= FLASH_CR_PER;
        FLASH->CR |= (add / 2048) << 3;
        FLASH->CR |= FLASH_CR_STRT;
        while ((FLASH->SR & EEPROM_FLASH_SR_BUSY) != 0) {
            /*  add time-out */
        }
        if ((FLASH->SR & FLASH_SR_EOP) != 0) {
            FLASH->SR = FLASH_SR_EOP;
        } else {
            /* error */
        }
        FLASH->CR &= ~FLASH_CR_PER;
    }

    volatile uint32_t write_cnt = 0, index = 0;
    while (index < data_length) {
        FLASH->CR |= FLASH_CR_PG; /* (1) */
        *(__IO uint32_t*)(add + write_cnt) = data_to_FLASH[index];
        *(__IO uint32_t*)(add + write_cnt + 4) = data_to_FLASH[index + 1];
        while ((FLASH->SR & EEPROM_FLASH_SR_BUSY) != 0) { /*  add time-out  */
        }
        if ((FLASH->SR & FLASH_SR_EOP) != 0) {
            FLASH->SR = FLASH_SR_EOP;
        } else {
            /*  error  */
        }
        FLASH->CR &= ~FLASH_CR_PG;
        write_cnt += 8;
        index += 2;
    }
    SET_BIT(FLASH->CR, FLASH_CR_LOCK);
}

void read_flash_bin(uint8_t* data, uint32_t add, int out_buff_len)
{
    for (int i = 0; i < out_buff_len; i++) {
        data[i] = *(uint8_t*)(add + i);
    }
}
