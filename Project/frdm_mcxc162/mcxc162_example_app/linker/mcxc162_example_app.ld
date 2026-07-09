/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

ENTRY(ResetISR)

MEMORY
{
  PROGRAM_FLASH (rx) : ORIGIN = 0x4000, LENGTH = 0x6000
  DATA_FLASH (rx) : ORIGIN = 0x1c000, LENGTH = 0x4000
  SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x3000
}

__base_PROGRAM_FLASH = ORIGIN(PROGRAM_FLASH);
__base_Flash = ORIGIN(PROGRAM_FLASH);
__top_PROGRAM_FLASH = ORIGIN(PROGRAM_FLASH) + LENGTH(PROGRAM_FLASH);
__top_Flash = ORIGIN(PROGRAM_FLASH) + LENGTH(PROGRAM_FLASH);
__base_DATA_FLASH = ORIGIN(DATA_FLASH);
__base_Flash2 = ORIGIN(DATA_FLASH);
__top_DATA_FLASH = ORIGIN(DATA_FLASH) + LENGTH(DATA_FLASH);
__top_Flash2 = ORIGIN(DATA_FLASH) + LENGTH(DATA_FLASH);
__base_SRAM = ORIGIN(SRAM);
__base_RAM = ORIGIN(SRAM);
__top_SRAM = ORIGIN(SRAM) + LENGTH(SRAM);
__top_RAM = ORIGIN(SRAM) + LENGTH(SRAM);

APP_IMAGE_HEADER_ADDRESS = ORIGIN(PROGRAM_FLASH) + 0x200;
APP_IMAGE_HEADER_SIZE = 0x40;

SECTIONS
{
    .text_Flash2 : ALIGN(4)
    {
        FILL(0xff)
        *(.text_Flash2)
        *(.text_DATA_FLASH)
        *(.text.$Flash2)
        *(.text.$DATA_FLASH)
        *(.text_Flash2.*)
        *(.text_DATA_FLASH.*)
        *(.text.$Flash2.*)
        *(.text.$DATA_FLASH.*)
        *(.rodata.$Flash2)
        *(.rodata.$DATA_FLASH)
        *(.rodata.$Flash2.*)
        *(.rodata.$DATA_FLASH.*)
    } > DATA_FLASH

    .text : ALIGN(4)
    {
        FILL(0xff)
        __vectors_start__ = ABSOLUTE(.);
        KEEP(*(.isr_vector))

        . = ALIGN(4);
        __section_table_start = .;
        __data_section_table = .;
        LONG(LOADADDR(.data));
        LONG(ADDR(.data));
        LONG(SIZEOF(.data));
        __data_section_table_end = .;
        __bss_section_table = .;
        LONG(ADDR(.bss));
        LONG(SIZEOF(.bss));
        __bss_section_table_end = .;
        __section_table_end = .;

        . = ALIGN(4);
        ASSERT(. <= APP_IMAGE_HEADER_ADDRESS, "MCXC162 app vectors exceed image header offset");
    } > PROGRAM_FLASH

    .image_header APP_IMAGE_HEADER_ADDRESS : ALIGN(4)
    {
        FILL(0xff)
        __image_header_start__ = .;
        . += APP_IMAGE_HEADER_SIZE;
        __image_header_end__ = .;
    } > PROGRAM_FLASH

    .text_after_header : ALIGN(4)
    {
        FILL(0xff)

        *(.after_vectors*)
        *(.text*)
        *(.rodata .rodata.* .constdata .constdata.*)
        . = ALIGN(4);
    } > PROGRAM_FLASH

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

    .uninit_RESERVED (NOLOAD) : ALIGN(4)
    {
        _start_uninit_RESERVED = .;
        KEEP(*(.bss.$RESERVED*))
        . = ALIGN(4);
        _end_uninit_RESERVED = .;
    } > SRAM AT> SRAM

    .data : ALIGN(4)
    {
        FILL(0xff)
        _data = .;
        PROVIDE(__start_data_RAM = .);
        PROVIDE(__start_data_SRAM = .);
        *(vtable)
        *(.ramfunc*)
        KEEP(*(CodeQuickAccess))
        KEEP(*(DataQuickAccess))
        *(RamFunction)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
        PROVIDE(__end_data_RAM = .);
        PROVIDE(__end_data_SRAM = .);
    } > SRAM AT> PROGRAM_FLASH

    .bss (NOLOAD) : ALIGN(4)
    {
        _bss = .;
        PROVIDE(__start_bss_RAM = .);
        PROVIDE(__start_bss_SRAM = .);
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        PROVIDE(__end_bss_RAM = .);
        PROVIDE(__end_bss_SRAM = .);
        PROVIDE(end = .);
    } > SRAM AT> SRAM

    .noinit (NOLOAD) : ALIGN(4)
    {
        _noinit = .;
        PROVIDE(__start_noinit_RAM = .);
        PROVIDE(__start_noinit_SRAM = .);
        *(.noinit*)
        . = ALIGN(4);
        _end_noinit = .;
        PROVIDE(__end_noinit_RAM = .);
        PROVIDE(__end_noinit_SRAM = .);
    } > SRAM AT> SRAM

    _HeapSize = 0x300;
    .heap (NOLOAD) : ALIGN(4)
    {
        _pvHeapStart = .;
        . += _HeapSize;
        . = ALIGN(4);
        _pvHeapLimit = .;
    } > SRAM

    _StackSize = 0x300;
    .heap2stackfill (NOLOAD) :
    {
        . += _StackSize;
    } > SRAM

    .stack ORIGIN(SRAM) + LENGTH(SRAM) - _StackSize - 0 (NOLOAD) : ALIGN(4)
    {
        _vStackBase = .;
        . = ALIGN(4);
        _vStackTop = . + _StackSize;
    } > SRAM

    _image_start = LOADADDR(.text);
    _image_end = LOADADDR(.data) + SIZEOF(.data);
    _image_size = _image_end - _image_start;
}

GROUP (
  "libcr_nohost_nf.a"
  "libcr_c.a"
  "libcr_eabihelpers.a"
  "libgcc.a"
)
