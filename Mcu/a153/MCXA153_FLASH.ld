/*
 * GENERATED FILE - DO NOT EDIT
 * Copyright 2008-2013 Code Red Technologies Ltd,
 * Copyright 2013-2024 NXP
 * Generated linker script file for MCXA153
 * Created from linkscript.ldt by FMCreateLinkLibraries
 * Using Freemarker v2.3.30
 * MCUXpresso IDE v11.10.0 [Build 3148] [2024-07-03] on Nov 8, 2024, 2:13:22 PM
 */
 
ENTRY(ResetISR)

MEMORY
{
  /* Define each memory region */
  BOOT_PROGRAM 	(rx)  : ORIGIN = 0x0, LENGTH = 0x4000
  FLASH_VECTAB  (rx)  : ORIGIN = ORIGIN(BOOT_PROGRAM) + LENGTH(BOOT_PROGRAM), LENGTH = 512
  FLASH_VERSION (rx)  : ORIGIN = ORIGIN(FLASH_VECTAB) + LENGTH(FLASH_VECTAB), LENGTH = 16
  PROGRAM_FLASH (rx)  : ORIGIN = ORIGIN(FLASH_VERSION) + LENGTH(FLASH_VERSION), LENGTH = 0x10000 - LENGTH(BOOT_PROGRAM) - LENGTH(FLASH_VECTAB) - LENGTH(FLASH_VERSION) - 32 - 0x2000 /* 64K bytes (alias Flash) */
  FILE_NAME     (rx)  : ORIGIN = ORIGIN(PROGRAM_FLASH) + LENGTH(PROGRAM_FLASH), LENGTH = 32
  EEPROM        (rx)  : ORIGIN = ORIGIN(FILE_NAME) + LENGTH(FILE_NAME), LENGTH = 0x2000  /*0xe000 - 0x10000*/
  SRAM 			(rwx) : ORIGIN = 0x20000000, LENGTH = 0x6000 /* 24K bytes (alias RAM) */  
  SRAMX0 		(rwx) : ORIGIN = 0x4000000, LENGTH = 0x2000 /* 8K bytes (alias RAM2) */  
  SRAMX1 		(rwx) : ORIGIN = 0x4002000, LENGTH = 0x1000 /* 4K bytes (alias RAM3) */  
}

  /* Define a symbol for the top of each memory region */
  /* Used for reference */
  __base_PROGRAM_FLASH = 0x0  ; /* PROGRAM_FLASH */  
  __base_Flash = 0x0 ; /* Flash */  
  __top_PROGRAM_FLASH = 0x0 + 0x20000 ; /* 128K bytes */  
  __top_Flash = 0x0 + 0x20000 ; /* 128K bytes */  
  __base_SRAM = 0x20000000  ; /* SRAM */  
  __base_RAM = 0x20000000 ; /* RAM */  
  __top_SRAM = 0x20000000 + 0x6000 ; /* 24K bytes */  
  __top_RAM = 0x20000000 + 0x6000 ; /* 24K bytes */  
  __base_SRAMX0 = 0x4000000  ; /* SRAMX0 */  
  __base_RAM2 = 0x4000000 ; /* RAM2 */  
  __top_SRAMX0 = 0x4000000 + 0x2000 ; /* 8K bytes */  
  __top_RAM2 = 0x4000000 + 0x2000 ; /* 8K bytes */  
  __base_SRAMX1 = 0x4002000  ; /* SRAMX1 */  
  __base_RAM3 = 0x4002000 ; /* RAM3 */  
  __top_SRAMX1 = 0x4002000 + 0x1000 ; /* 4K bytes */  
  __top_RAM3 = 0x4002000 + 0x1000 ; /* 4K bytes */  

SECTIONS
{
	
    /* The startup code into "FLASH" Rom type memory */
    .isr_vector : ALIGN(4)
  	{
        FILL(0xff)
        __vectors_start__ = ABSOLUTE(.) ;
    	KEEP(*(.isr_vector)) /* Startup code */
    	. = ALIGN(4);
    } >FLASH_VECTAB

    /* The firmware version and name - at a fixed address to make it possible to read it from firmware files. */
    .firmware_version :
    {
    	. = ALIGN(4);
		KEEP (*(.firmware_info))
		. = ALIGN(4);
    } >FLASH_VERSION
	
     /* MAIN TEXT SECTION */
    .text : ALIGN(4)
    {
        __section_table_start = .;
        __data_section_table = .;
        LONG(LOADADDR(.data));
        LONG(    ADDR(.data));
        LONG(  SIZEOF(.data));
        LONG(LOADADDR(.data_RAM2));
        LONG(    ADDR(.data_RAM2));
        LONG(  SIZEOF(.data_RAM2));
        LONG(LOADADDR(.data_RAM3));
        LONG(    ADDR(.data_RAM3));
        LONG(  SIZEOF(.data_RAM3));
        __data_section_table_end = .;
        __bss_section_table = .;
        LONG(    ADDR(.bss));
        LONG(  SIZEOF(.bss));
        LONG(    ADDR(.bss_RAM2));
        LONG(  SIZEOF(.bss_RAM2));
        LONG(    ADDR(.bss_RAM3));
        LONG(  SIZEOF(.bss_RAM3));
        __bss_section_table_end = .;
        __section_table_end = . ;
        /* End of Global Section Table */

        *(.after_vectors*)

       *(.text*)
       *(.rodata .rodata.* .constdata .constdata.*)
       . = ALIGN(4);
    } > PROGRAM_FLASH
    /*
     * for exception handling/unwind - some Newlib functions (in common
     * with C++ and STDC++) use this.
     */
    .ARM.extab : ALIGN(4)
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > PROGRAM_FLASH

    .ARM.exidx : ALIGN(4)
    {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } > PROGRAM_FLASH
 
    _etext = .;
    
	/* The file name */
	.file_name :
	{
        . = ALIGN(4);
		KEEP (*(.file_name))
	    . = ALIGN(4);
	} >FILE_NAME
        
    /* DATA section for SRAMX0 */

    .data_RAM2 : ALIGN(4)
    {
        FILL(0xff)
        PROVIDE(__start_data_RAM2 = .) ;
        PROVIDE(__start_data_SRAMX0 = .) ;
        *(.ramfunc.$RAM2)
        *(.ramfunc.$SRAMX0)
        *(.data.$RAM2)
        *(.data.$SRAMX0)
        *(.data.$RAM2.*)
        *(.data.$SRAMX0.*)
        . = ALIGN(4) ;
        PROVIDE(__end_data_RAM2 = .) ;
        PROVIDE(__end_data_SRAMX0 = .) ;
     } > SRAMX0 AT>PROGRAM_FLASH

    /* DATA section for SRAMX1 */

    .data_RAM3 : ALIGN(4)
    {
        FILL(0xff)
        PROVIDE(__start_data_RAM3 = .) ;
        PROVIDE(__start_data_SRAMX1 = .) ;
        *(.ramfunc.$RAM3)
        *(.ramfunc.$SRAMX1)
        *(.data.$RAM3)
        *(.data.$SRAMX1)
        *(.data.$RAM3.*)
        *(.data.$SRAMX1.*)
        . = ALIGN(4) ;
        PROVIDE(__end_data_RAM3 = .) ;
        PROVIDE(__end_data_SRAMX1 = .) ;
     } > SRAMX1 AT>PROGRAM_FLASH

    /* MAIN DATA SECTION */
    .uninit_RESERVED (NOLOAD) : ALIGN(4)
    {
        _start_uninit_RESERVED = .;
        KEEP(*(.bss.$RESERVED*))
       . = ALIGN(4) ;
        _end_uninit_RESERVED = .;
    } > SRAM AT> SRAM

    /* Main DATA section (SRAM) */
    .data : ALIGN(4)
    {
       FILL(0xff)
       _data = . ;
       PROVIDE(__start_data_RAM = .) ;
       PROVIDE(__start_data_SRAM = .) ;
       *(vtable)
       *(.ramfunc*)
       KEEP(*(CodeQuickAccess))
       KEEP(*(DataQuickAccess))
       *(RamFunction)
       *(.data*)
       . = ALIGN(4) ;
       _edata = . ;
       PROVIDE(__end_data_RAM = .) ;
       PROVIDE(__end_data_SRAM = .) ;
    } > SRAM AT>PROGRAM_FLASH

    /* BSS section for SRAMX0 */
    .bss_RAM2 (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_bss_RAM2 = .) ;
       PROVIDE(__start_bss_SRAMX0 = .) ;
       *(.bss.$RAM2)
       *(.bss.$SRAMX0)
       *(.bss.$RAM2.*)
       *(.bss.$SRAMX0.*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_RAM2 = .) ;
       PROVIDE(__end_bss_SRAMX0 = .) ;
    } > SRAMX0 AT> SRAMX0

    /* BSS section for SRAMX1 */
    .bss_RAM3 (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_bss_RAM3 = .) ;
       PROVIDE(__start_bss_SRAMX1 = .) ;
       *(.bss.$RAM3)
       *(.bss.$SRAMX1)
       *(.bss.$RAM3.*)
       *(.bss.$SRAMX1.*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_RAM3 = .) ;
       PROVIDE(__end_bss_SRAMX1 = .) ;
    } > SRAMX1 AT> SRAMX1

    /* MAIN BSS SECTION */
    .bss (NOLOAD) : ALIGN(4)
    {
        _bss = .;
        PROVIDE(__start_bss_RAM = .) ;
        PROVIDE(__start_bss_SRAM = .) ;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4) ;
        _ebss = .;
        PROVIDE(__end_bss_RAM = .) ;
        PROVIDE(__end_bss_SRAM = .) ;
        PROVIDE(end = .);
    } > SRAM AT> SRAM

    /* NOINIT section for SRAMX0 */
    .noinit_RAM2 (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_noinit_RAM2 = .) ;
       PROVIDE(__start_noinit_SRAMX0 = .) ;
       *(.noinit.$RAM2)
       *(.noinit.$SRAMX0)
       *(.noinit.$RAM2.*)
       *(.noinit.$SRAMX0.*)
       . = ALIGN(4) ;
       PROVIDE(__end_noinit_RAM2 = .) ;
       PROVIDE(__end_noinit_SRAMX0 = .) ;
    } > SRAMX0 AT> SRAMX0

    /* NOINIT section for SRAMX1 */
    .noinit_RAM3 (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_noinit_RAM3 = .) ;
       PROVIDE(__start_noinit_SRAMX1 = .) ;
       *(.noinit.$RAM3)
       *(.noinit.$SRAMX1)
       *(.noinit.$RAM3.*)
       *(.noinit.$SRAMX1.*)
       . = ALIGN(4) ;
       PROVIDE(__end_noinit_RAM3 = .) ;
       PROVIDE(__end_noinit_SRAMX1 = .) ;
    } > SRAMX1 AT> SRAMX1

    /* DEFAULT NOINIT SECTION */
    .noinit (NOLOAD): ALIGN(4)
    {
        _noinit = .;
        PROVIDE(__start_noinit_RAM = .) ;
        PROVIDE(__start_noinit_SRAM = .) ;
        *(.noinit*)
         . = ALIGN(4) ;
        _end_noinit = .;
       PROVIDE(__end_noinit_RAM = .) ;
       PROVIDE(__end_noinit_SRAM = .) ;        
    } > SRAM AT> SRAM

    /* Reserve and place Heap within memory map */
    _HeapSize = 0x600;
    .heap (NOLOAD) :  ALIGN(4)
    {
        _pvHeapStart = .;
        . += _HeapSize;
        . = ALIGN(4);
        _pvHeapLimit = .;
    } > SRAM

     _StackSize = 0x600;
     /* Reserve space in memory for Stack */
    .heap2stackfill (NOLOAD) :
    {
        . += _StackSize;
    } > SRAM
    /* Locate actual Stack in memory map */
    .stack ORIGIN(SRAM) + LENGTH(SRAM) - _StackSize - 0 (NOLOAD) :  ALIGN(4)
    {
        _vStackBase = .;
        . = ALIGN(4);
        _vStackTop = . + _StackSize;
    } > SRAM

    /* Provide basic symbols giving location and size of main text
     * block, including initial values of RW data sections. Note that
     * these will need extending to give a complete picture with
     * complex images (e.g multiple Flash banks).
     */
    _image_start = LOADADDR(.text);
    _image_end = LOADADDR(.data) + SIZEOF(.data);
    _image_size = _image_end - _image_start;
}
