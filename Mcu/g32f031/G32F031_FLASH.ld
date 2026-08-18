/*
       linker file for AM32 firmware on G32F031
       Bootloader occupies 0x00000000 - 0x00000FFF (4KB)
       Firmware starts at 0x00001000
*/

ENTRY(Reset_Handler)

_end_stack = 0x20002000;
_heap_size = 0x200;
_stack_size = 0x400;

MEMORY
{
FLASH (rx)      : ORIGIN = 0x00001000, LENGTH = 32K
RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 8K
}

SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM : {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array     :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH
  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH
  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH

  _start_address_init_data = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _start_address_data = .;
    *(.data)
    *(.data*)

    . = ALIGN(4);
    _end_address_data = .;
  } >RAM AT> FLASH

  . = ALIGN(4);
  .bss :
  {
    _start_address_bss = .;
    __bss_start__ = _start_address_bss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _end_address_bss = .;
    __bss_end__ = _end_address_bss;
  } >RAM

  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _heap_size;
    . = . + _stack_size;
    . = ALIGN(8);
  } >RAM

  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }
}