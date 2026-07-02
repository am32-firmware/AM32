/*
 * eeprom.c
 *
 * AT32F421 flash driver — shared GD32/AT32 body over the AT flash library.
 */

#include "eeprom.h"

#include <string.h>

#include "at32f421_flash.h"

#define EEPROM_FLASH_UNLOCK() flash_unlock()
#define EEPROM_PAGE_ERASE(add) flash_sector_erase(add)
#define EEPROM_WORD_PROGRAM(addr, val) flash_word_program(addr, val)
#define EEPROM_CLEAR_FLAGS() flash_flag_clear(FLASH_PROGRAM_ERROR | FLASH_EPP_ERROR | FLASH_OPERATE_DONE)
#define EEPROM_FLASH_LOCK() flash_lock()

#include "../../shared/eeprom_gd_at32.h"
