/*
 * eeprom.c
 *
 * GD32E230 flash driver — shared GD32/AT32 body over the GD FMC library.
 */

#include "eeprom.h"

#include <string.h>

#include "gd32e23x_fmc.h"

#define EEPROM_FLASH_UNLOCK() fmc_unlock()
#define EEPROM_PAGE_ERASE(add) fmc_page_erase(add)
#define EEPROM_WORD_PROGRAM(addr, val) fmc_word_program(addr, val)
#define EEPROM_CLEAR_FLAGS() fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR)
#define EEPROM_FLASH_LOCK() fmc_lock()

#include "../../shared/eeprom_gd_at32.h"
