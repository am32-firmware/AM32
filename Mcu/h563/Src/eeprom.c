#include "targets.h"
#include "eeprom.h"
#include "flash.h"
#include "stm32h563xx.h"

#include <string.h>
// !!!!!!!!!!!!!!!!!!!
// Per reference manual:
// The application software must not unlock an
// already unlocked register, otherwise this
// register remains locked until the next system reset.
// !!!!!!!!!!!!!!!!!!!!

void save_flash_nolib(uint8_t* data, int length, uint32_t add)
{
    // copy submitted data to native data width array
    // ie we can only program 16 bit data to flash
    // TODO this copy is slow
    uint16_t data_to_FLASH[length / 2];
    memset(data_to_FLASH, 0, length / 2);
    for (int i = 0; i < length / 2; i++) {
        data_to_FLASH[i] = data[i * 2 + 1] << 8 | data[i * 2]; // make 16 bit
    }

    // iterate by 16 bit words
    volatile uint32_t data_length = length / 2;

    while (flash_busy());

    // erase page if address even divisable by page size
    if ((add % EEPROM_PAGE_SIZE) == 0) {
        flash_erase_sector(126);
    }
    
    volatile uint32_t write_cnt = 0, index = 0;
    while (index < data_length) {
        flash_program_word(data_to_FLASH[index], (add + write_cnt));
        write_cnt += 2;
        index++;
    }
}

// see https://community.st.com/t5/stm32-mcus-products/stm32h5-high-cycle-data-read-more-than-16bit-at-once/td-p/584258
// "If the application reads an OTP data or flash high-cycle data not previously written,
// a double ECC error is reported and only a word full of set bits is returned"
void read_flash_bin(uint8_t* data, uint32_t add, int out_buff_len)
{
    for (int i = 0; i < out_buff_len; i++) {
        data[i] = *(uint8_t*)(add + i);
    }
}
