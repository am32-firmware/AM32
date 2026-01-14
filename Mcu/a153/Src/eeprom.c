/*
 * eeprom.c
 *
 *  Created on: Mar. 25, 2020
 *      Author: Alka
 *
 */

#include "eeprom.h"
#include "main.h"
#include <string.h>

//8192 bytes sector size, 128 bytes page, 16 bytes phrase for MXCA133
#define sector_size 0x2000

uint32_t status = 0;
flash_config_t s_flashDriver;

/*
  write to flash, done as a RAM function to allow for DroneCAN fw update
 */
void save_flash_nolib(uint8_t *data, int length, uint32_t add){
	//Check if address and length is within 128 bytes boundary
    if ((add % 8192) != 0) {
    	return;
    }

    __disable_irq();

    uint32_t pflashBlockBase  = 0U;
    uint32_t pflashTotalSize  = 0U;
    uint32_t pflashSectorSize = 0U;
    uint32_t PflashPageSize   = 0U;

    /* Get flash properties kFLASH_ApiEraseKey */
    FLASH_API->flash_get_property(&s_flashDriver, kFLASH_PropertyPflashBlockBaseAddr, &pflashBlockBase);
    FLASH_API->flash_get_property(&s_flashDriver, kFLASH_PropertyPflashSectorSize, &pflashSectorSize);
    FLASH_API->flash_get_property(&s_flashDriver, kFLASH_PropertyPflashTotalSize, &pflashTotalSize);
    FLASH_API->flash_get_property(&s_flashDriver, kFLASH_PropertyPflashPageSize, &PflashPageSize);

    //Erase last sector
	status = FLASH_API->flash_erase_sector(&s_flashDriver, add, pflashSectorSize, kFLASH_ApiEraseKey);
	if (status) {
		__asm volatile ("nop");
	}

	//Verify sector erase
	status = FLASH_API->flash_verify_erase_sector(&s_flashDriver, add, pflashSectorSize);
	if (status) {
		__asm volatile ("nop");
	}

	//Program data
	status = FLASH_API->flash_program_page(&s_flashDriver, add, data, length);
	if (status) {
		__asm volatile ("nop");
	}

	//Verify programmed data
	uint32_t failed_data_addr 	= 0;
	uint32_t failed_data 		= 0;
	status = FLASH_API->flash_verify_program(&s_flashDriver, add, length, data, &failed_data_addr, &failed_data);
	if (status) {
		__asm volatile ("nop");
	}

	//Clear cache
	modifyReg32(&SYSCON->LPCAC_CTRL, 0, SYSCON_LPCAC_CTRL_CLR_LPCAC(1));

	//Unclear cache
	modifyReg32(&SYSCON->LPCAC_CTRL, SYSCON_LPCAC_CTRL_CLR_LPCAC(1), 0);

	//Check if verify program found failed data
	if ((failed_data_addr != 0) || (failed_data != 0)) {
		__asm volatile ("nop");
	}

	__enable_irq();
}

void read_flash_bin(uint8_t*  data , uint32_t add, int out_buff_len) {
	//Read flash at given address
	status = FLASH_API->flash_read(&s_flashDriver, add, data, out_buff_len);
	if (status) {
		__asm volatile ("nop");
	}
}
