/*
 * bootloader.c
 *
 *  Created on: Mar. 25, 2020
 *      Author: Alka
 *
 */

#include "eeprom.h"
#include <string.h>
#include <targets.h>

#define page_size 0x800                   // 2 kb for l431
uint32_t FLASH_FKEY1 =0x45670123;
uint32_t FLASH_FKEY2 =0xCDEF89AB;

/*
  write to flash, done as a RAM function to allow for DroneCAN fw update
 */
void save_flash_nolib(uint8_t *data, int length, uint32_t add)
{
  if ((add & 0x7) != 0 || (length & 0x7)) {
    // address and length must be on 8 byte boundary
    return;
  }
  // we need to flash on 32 bit boundaries
  uint32_t data_length = length / 4;
  volatile FLASH_TypeDef *flash = FLASH;

  // clear errors
  flash->SR |= FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR |
               FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR | FLASH_SR_FASTERR |
               FLASH_SR_RDERR | FLASH_SR_OPTVERR;

  // unlock flash
  while ((flash->SR & FLASH_SR_BSY) != 0) ;

  if ((flash->CR & FLASH_CR_LOCK) != 0) {
    flash->KEYR = FLASH_FKEY1;
    flash->KEYR = FLASH_FKEY2;
  }

  // erase page if address is divisable by page size
  if ((add % page_size) == 0) {
    flash->CR = FLASH_CR_PER;
    flash->CR |= (add/page_size) << 3;
    flash->CR |= FLASH_CR_STRT;
    while ((flash->SR & FLASH_SR_BSY) != 0) ;
  }

  uint32_t index = 0;
  volatile uint32_t *fdata = (volatile uint32_t *)add;

  while (index < data_length) {
    // flash two words at a time
    uint32_t words[2];
    memcpy((void*)&words[0], &data[index*4], sizeof(words));

    flash->CR = FLASH_CR_PG;

    fdata[index] = words[0];
    fdata[index+1] = words[1];

    while ((flash->SR & FLASH_SR_BSY) != 0) ;

    flash->SR |= FLASH_SR_EOP;
    flash->CR = 0;
    index += 2;
  }

  // lock flash again
  SET_BIT(flash->CR, FLASH_CR_LOCK);
}

void read_flash_bin(uint8_t*  data, uint32_t add, int out_buff_len)
{
  memcpy(data, (const void*)add, out_buff_len);
}
