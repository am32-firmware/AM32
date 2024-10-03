ENTRY( _start )

__stack_size = 2048;
__resv_size  = 256;
PROVIDE( _stack_size = __stack_size );
PROVIDE( _resv_size = __resv_size );

/*
  SRAM          (xrw) : ORIGIN = 0x20000000, LENGTH = 192
  RAM           (xrw) : ORIGIN = 0x200000C0, LENGTH = 8K - 192
  FLASH_VECTAB  (rx)  : ORIGIN = 0x08001000, LENGTH = 192
  FLASH_VERSION (rx)  : ORIGIN = 0x080010C0, LENGTH = 14
  FLASH         (rx)  : ORIGIN = ORIGIN(FLASH_VERSION) + LENGTH(FLASH_VERSION), LENGTH = 27K - (LENGTH(FLASH_VECTAB) + LENGTH(FLASH_VERSION))
  EEPROM        (rx)  : ORIGIN = 0x080007C00, LENGTH = 1K
*/


MEMORY
{  
        RESET_FLASH(rx)   : ORIGIN = 0x00001000, LENGTH = 192
        FLASH_VERSION(rx) : ORIGIN = 0x000010C0, LENGTH = 16
        FLASH (rx)     : ORIGIN = 0x000010D0, LENGTH = 56K - 0xD0
        FILE_NAME (rx) : ORIGIN = 0x0000f800 - 32, LENGTH = 32
	RAM (xrw)     : ORIGIN = 0x20000000, LENGTH = 20K
}



SECTIONS
{

	.init :
	{
		_sinit = .;
		. = ALIGN(4);
		KEEP(*(SORT_NONE(.init)))
		. = ALIGN(4);
		_einit = .;
	} >RESET_FLASH AT>RESET_FLASH

  	/* The firmware version and name - at a fixed address to make it possible to read it from firmware files. */
  	.firmware_version :
  	{
    	. = ALIGN(4);
		KEEP (*(.firmware_info))
  	} >FLASH_VERSION
  
    .vector :
    {
      . = ALIGN(4);
      *(.vector);
	  . = ALIGN(4);
    } >FLASH AT>FLASH

	.text :
	{
		. = ALIGN(4);
		*(.text)
		*(.text.*)
		*(.rodata)
		*(.rodata*)
		*(.gnu.linkonce.t.*)
		. = ALIGN(4);
	} >FLASH AT>FLASH 

  	.file_name :
  	{
    	. = ALIGN(4);
		KEEP (*(.file_name))
		. = ALIGN(4);
  	} >FILE_NAME

	.fini :
	{
		KEEP(*(SORT_NONE(.fini)))
		. = ALIGN(4);
	} >FLASH AT>FLASH

	PROVIDE( _etext = . );
	PROVIDE( _eitcm = . );	

	.preinit_array  :
	{
	  PROVIDE_HIDDEN (__preinit_array_start = .);
	  KEEP (*(.preinit_array))
	  PROVIDE_HIDDEN (__preinit_array_end = .);
	} >FLASH AT>FLASH 
	
	.init_array     :
	{
	  PROVIDE_HIDDEN (__init_array_start = .);
	  KEEP (*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
	  KEEP (*(.init_array EXCLUDE_FILE (*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o ) .ctors))
	  PROVIDE_HIDDEN (__init_array_end = .);
	} >FLASH AT>FLASH 
	
	.fini_array     :
	{
	  PROVIDE_HIDDEN (__fini_array_start = .);
	  KEEP (*(SORT_BY_INIT_PRIORITY(.fini_array.*) SORT_BY_INIT_PRIORITY(.dtors.*)))
	  KEEP (*(.fini_array EXCLUDE_FILE (*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o ) .dtors))
	  PROVIDE_HIDDEN (__fini_array_end = .);
	} >FLASH AT>FLASH 
	
	.ctors          :
	{
	  /* gcc uses crtbegin.o to find the start of
	     the constructors, so we make sure it is
	     first.  Because this is a wildcard, it
	     doesn't matter if the user does not
	     actually link against crtbegin.o; the
	     linker won't look for a file to match a
	     wildcard.  The wildcard also means that it
	     doesn't matter which directory crtbegin.o
	     is in.  */
	  KEEP (*crtbegin.o(.ctors))
	  KEEP (*crtbegin?.o(.ctors))
	  /* We don't want to include the .ctor section from
	     the crtend.o file until after the sorted ctors.
	     The .ctor section from the crtend file contains the
	     end of ctors marker and it must be last */
	  KEEP (*(EXCLUDE_FILE (*crtend.o *crtend?.o ) .ctors))
	  KEEP (*(SORT(.ctors.*)))
	  KEEP (*(.ctors))
	} >FLASH AT>FLASH 
	
	.dtors          :
	{
	  KEEP (*crtbegin.o(.dtors))
	  KEEP (*crtbegin?.o(.dtors))
	  KEEP (*(EXCLUDE_FILE (*crtend.o *crtend?.o ) .dtors))
	  KEEP (*(SORT(.dtors.*)))
	  KEEP (*(.dtors))
	} >FLASH AT>FLASH 
	
	/* The file name */
	.file_name :
	{
	  . = ALIGN(4);
	  KEEP (*(.file_name))
    } >FILE_NAME


	.dalign :
	{
		. = ALIGN(4);
		PROVIDE(_data_vma = .);
	} >RAM AT>FLASH	

	.dlalign :
	{
		. = ALIGN(4); 
		PROVIDE(_data_lma = .);
	} >FLASH AT>FLASH

	.data :
	{
    	*(.gnu.linkonce.r.*)
    	*(.data .data.*)
    	*(.gnu.linkonce.d.*)
		. = ALIGN(8);
    	PROVIDE( __global_pointer$ = . + 0x800 );
    	*(.sdata .sdata.*)
		*(.sdata2.*)
    	*(.gnu.linkonce.s.*)
    	. = ALIGN(8);
    	*(.srodata.cst16)
    	*(.srodata.cst8)
    	*(.srodata.cst4)
    	*(.srodata.cst2)
    	*(.srodata .srodata.*)
    	. = ALIGN(4);
		PROVIDE( _edata = .);
	} >RAM AT>FLASH

	.bss :
	{
		. = ALIGN(4);
		PROVIDE( _sbss = .);
  	    *(.sbss*)
        *(.gnu.linkonce.sb.*)
		*(.bss*)
     	*(.gnu.linkonce.b.*)		
		*(COMMON*)
		. = ALIGN(4);
		PROVIDE( _ebss = .);
	} >RAM AT>FLASH

	PROVIDE( _end = _ebss);
	PROVIDE( end = . );

    .stack ORIGIN(RAM) + LENGTH(RAM) - __stack_size - __resv_size :
    {
        PROVIDE( _heap_end = . );   
        . = ALIGN(4);
        PROVIDE(_susrstack = . );
        . = . + __stack_size;
        PROVIDE( _eusrstack = .);
        . = . + __resv_size;
        PROVIDE( _endram = .);
    } >RAM 

}



