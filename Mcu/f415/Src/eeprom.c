#include "eeprom.h"
#include "targets.h"

#include <string.h>
#include <stdbool.h>

//#pragma GCC optimize("O0")

#include "at32f415_flash.h"

/*
  the F415 can be either 1k or 2k sector size. The 256k flash part has
  2k sector size. We only support ESCs with 128k flash or less, so 1k
  sector size, but it is useful to work with 2k sector size when using
  a F415 dev board like the AT-Start F415, so we detect here.
 */
static inline uint32_t sector_size()
{
  const uint16_t *F_SIZE = (const uint16_t *)0x1FFFF7E0;
  if (*F_SIZE <= 128) {
    // 1k sectors for 128k flash or less
    return 1024;
  }
  // 256k flash is 2k sectors
  return 2048;
}

void save_flash_nolib(uint8_t* data, int length, uint32_t add)
{
  if ((add & 0x3) != 0 || (length & 0x3) != 0) {
    return;
  }
  /*
    we need the data to be 32 bit aligned
   */
  const uint32_t word_length = length / 4;

  // unlock flash
  flash_unlock();

  // erase page if address even divisable by sector size
  if ((add % sector_size()) == 0) {
    flash_sector_erase(add);
  }

  uint32_t index = 0;
  while (index < word_length) {
    uint32_t word;
    memcpy(&word, &data[index*4], sizeof(word));
    flash_word_program(add + (index * 4), word);
    flash_flag_clear(FLASH_PROGRAM_ERROR | FLASH_EPP_ERROR | FLASH_OPERATE_DONE);
    index++;
  }
  flash_lock();
}

void read_flash_bin(uint8_t* data, uint32_t add, int out_buff_len)
{
  memcpy(data, (void*)add, out_buff_len);
}
