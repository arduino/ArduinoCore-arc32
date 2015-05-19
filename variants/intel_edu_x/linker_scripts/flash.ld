/* linker.cmd - Linker command/script file */

/*
 * Copyright (c) 2014 Wind River Systems, Inc.
 *
 * The right to copy, distribute, modify or otherwise make use
 * of this software may be licensed only pursuant to the terms
 * of an applicable Wind River license agreement.
 */

/*
modification history
--------------------
03Nov14,j_b  written
16Apr15,dod  trimmed down for Atlas-Edge version
*/

/*
DESCRIPTION
Linker script for the Atlas Peak ARC BSPs.
*/

OUTPUT_FORMAT("elf32-littlearc", "elf32-bigarc", "elf32-littlearc")

MEMORY
    {
    FLASH                 (rx) : ORIGIN = 0x40000000, LENGTH = 192K
    SRAM                  (wx) : ORIGIN = 0xa8010000, LENGTH = 16K
    DCCM                  (wx) : ORIGIN = 0x80000000, LENGTH = 8K
    }

/* Putting stack at end of SRAM for now */
__stack_start = ORIGIN(SRAM)+LENGTH(SRAM);

/* Allocating heap size of 2048 bytes for now */
__HEAP_SIZE = 2048;

SECTIONS
    {
/* FLASH Start */

    version_header_section :
    {
        *(.version_header)
        KEEP(*(".version_header*"))
    } > FLASH

    text : ALIGN(1024)
	{
	__text_start = .;

/* when !XIP, .text is in RAM, and vector table must be at its very start */
	KEEP(*(.int_vector_table))
	KEEP(*(".int_vector_table.*"))

	*(.text)
	*(".text.*")
	__text_end = .;
	} > FLASH

    ctors :
        {
        /*
         * The compiler fills the constructor pointers table below, hence symbol
         * __ctor_table_start must be aligned on 4 byte boundary.
         * To align with the C++ standard, the first element of the array
         * contains the number of actual constructors. The last element is
         * NULL.
         */
        . = ALIGN(4);
        __CTOR_LIST__ = .;
        LONG((__CTOR_END__ - __CTOR_LIST__) / 4 - 2)
        KEEP(*(SORT(".ctors*")))
        LONG(0)
        __CTOR_END__ = .;
        } > FLASH

    rodata :
	{
	*(.rodata)
	*(".rodata.*")
	} > FLASH

    __data_rom_start = ALIGN(4);    /* XIP imaged DATA ROM start addr */

/* FLASH End */

/* SRAM Start */

    datas : AT(__data_rom_start)
	{

/* when XIP, .text is in ROM, but vector table must be at start of .data */

	__data_ram_start = .;
	*(.data)
	*(".data.*")
	} > SRAM

    sdata :
        {
         __SDATA_BEGIN__ = .;
        *(.sdata .sdata.* .gnu.linkonce.s.*)
        } > SRAM

    __data_ram_end = .;

    bss (NOLOAD) :
	{
        /*
         * For performance, BSS section is assumed to be 4 byte aligned and
         * a multiple of 4 bytes
         */
        . = ALIGN(4);
	__bss_start = .;
	*(.bss)
	*(".bss.*")
	*(COMMON)
        /*
         * BSP clears this memory in words only and doesn't clear any
         * potential left over bytes.
	 */
	__bss_end = ALIGN(4);
	} > SRAM

    noinit (NOLOAD) :
        {
        /*
         * This section is used for non-intialized objects that
         * will not be cleared during the boot process.
         */
        *(.noinit)
        *(".noinit.*")
	/*
	 * The seg_rxtx section is used by the Host IO module for
	 * allocating RX/TX buffers. If a specific memory location is
	 * required for these buffers, then a MEMORY definition should be
	 * made.
	 */
	*(.seg_rxtx)
	*(".seg_rxtx.*")
        } > SRAM

     heap (NOLOAD) :
     {
         . = ALIGN(4);
         __start_heap = . ;
         . = . + __HEAP_SIZE ;
         __end_heap = . ;
     } > SRAM

    /* Define linker symbols */

    _end = .; /* end of image */

/* SRAM End */

	/* Data Closely Coupled Memory (DCCM) */
/* DCCM Start */
/* DCCM End */

    }
