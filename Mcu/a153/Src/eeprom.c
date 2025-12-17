/*
 * eeprom.c
 *
 *  Created on: Mar. 25, 2020
 *      Author: Alka
 *
 */

#include "eeprom.h"
//#include "mcxa153_rom_api.h"
#include "main.h"
#include <string.h>
//#include <targets.h>

//#define page_size 0x800                   // 2 kb for l431
#define sector_size 0x2000			//8192 bytes sector size, 128 bytes page, 16 bytes phrase for MXCA133
uint32_t FLASH_FKEY1 =0x45670123;
uint32_t FLASH_FKEY2 =0xCDEF89AB;

uint32_t status = 0;
//static flash_config_t s_flashDriver;
flash_config_t s_flashDriver;

/*
  write to flash, done as a RAM function to allow for DroneCAN fw update
 */
void save_flash_nolib(uint8_t *data, int length, uint32_t add){
	//Check if address and length is within 128 bytes boundary
    if ((add % 8192) != 0) {
    	return;
    }

//    __disable_irq();
//
////    memset(&s_flashDriver, 0, sizeof(flash_config_t));
////
////    //Check if init went successful
////    status = FLASH_API->flash_init(&s_flashDriver);
////    if (status) {
////    	__asm volatile ("nop");
////    }
//
//    uint32_t pflashBlockBase  = 0U;
//    uint32_t pflashTotalSize  = 0U;
//    uint32_t pflashSectorSize = 0U;
//    uint32_t PflashPageSize   = 0U;
//
//    /* Get flash properties kFLASH_ApiEraseKey */
//    FLASH_API->flash_get_property(&s_flashDriver, kFLASH_PropertyPflashBlockBaseAddr, &pflashBlockBase);
//    FLASH_API->flash_get_property(&s_flashDriver, kFLASH_PropertyPflashSectorSize, &pflashSectorSize);
//    FLASH_API->flash_get_property(&s_flashDriver, kFLASH_PropertyPflashTotalSize, &pflashTotalSize);
//    FLASH_API->flash_get_property(&s_flashDriver, kFLASH_PropertyPflashPageSize, &PflashPageSize);
////
//////    uint32_t eeprom_address = s_flashDriver.PFlashBlockBase + (s_flashDriver.PFlashTotalSize - (1 * s_flashDriver.PFlashSectorSize));
////    uint32_t dest_addr = pflashBlockBase + (pflashTotalSize - pflashSectorSize);
//
//    //Erase last sector
//	status = FLASH_API->flash_erase_sector(&s_flashDriver, add, pflashSectorSize, kFLASH_ApiEraseKey);
//	if (status) {
//		__asm volatile ("nop");
//	}
//
//	//Verify sector erase
//	status = FLASH_API->flash_verify_erase_sector(&s_flashDriver, add, pflashSectorSize);
//	if (status) {
//		__asm volatile ("nop");
//	}
//
////	data[0] = 0xaa;
////	data[1] = 0x55;
////	data[2] = 0xaa;
////	data[3] = 0x55;
//
//	uint32_t num_flash_pages = length / PflashPageSize + ((length % PflashPageSize) > 0);
//
//	//Program data
//	status = FLASH_API->flash_program_page(&s_flashDriver, add, data, num_flash_pages);
//	if (status) {
//		__asm volatile ("nop");
//	}
//
////	uint8_t readout[400];
////	status = FLASH_API->flash_read(&s_flashDriver, add, readout, num_flash_pages);
////	if (status) {
////		__asm volatile ("nop");
////	}
//
//	//Verify programmed data
//	uint32_t failed_data_addr 	= 0;
//	uint32_t failed_data 		= 0;
//	status = FLASH_API->flash_verify_program(&s_flashDriver, add, num_flash_pages, data, &failed_data_addr, &failed_data);
//	if (status) {
//		__asm volatile ("nop");
//	}
//
//	//Check if verify program found failed data
//	if (failed_data_addr != 0) {
//		__asm volatile ("nop");
//	}
//
//	__enable_irq();
}

void read_flash_bin(uint8_t*  data , uint32_t add, int out_buff_len) {
//    memcpy(data, (const void*)add, out_buff_len);

	status = FLASH_API->flash_read(&s_flashDriver, add, data, out_buff_len);
	if (status) {
		__asm volatile ("nop");
	}
}
