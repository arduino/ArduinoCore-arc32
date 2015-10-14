/* linker.cmd - Linker command/script file */

/*
 * Copyright (c) 2014 Wind River Systems, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1) Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2) Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3) Neither the name of Wind River Systems nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
DESCRIPTION
Linker script for the Arduino 101 ARC BSPs.
*/

OUTPUT_FORMAT("elf32-littlearc", "elf32-bigarc", "elf32-littlearc")

MEMORY
    {
    FLASH                 (rx) : ORIGIN = 0x40034000, LENGTH = 152K
    SRAM                  (wx) : ORIGIN = 0xa800e000, LENGTH = 24K
    DCCM                  (wx) : ORIGIN = 0x80000000, LENGTH = 8K
    }

/* Define default stack size and FIRQ stack size.
 * See below stack section for __stack_start and __firq_stack_start
 */
__stack_size = 2048;
__firq_stack_size = 512;

/* Minimum heap size to allocate
 * Actual heap size might be bigger due to page size alignment */
__HEAP_SIZE_MIN = 8192;
/* This should be set to the page size used by the malloc implementation */
__PAGE_SIZE = 4096;

SECTIONS
    {
/* FLASH Start */

    text : ALIGN(1024)
	{
	__text_start = .;

/* when !XIP, .text is in RAM, and vector table must be at its very start */
	KEEP(*(.int_vector_table))
	KEEP(*(".int_vector_table.*"))

	*(.version_header)
	KEEP(*(".version_header*"))

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
        __start_heap = .;
        . = . + __HEAP_SIZE_MIN;
        __end_heap = ALIGN(__PAGE_SIZE);
    } > SRAM

    stack :
    {
        . += __firq_stack_size;
        . = ALIGN(4);
        __firq_stack_start = .;

        . += __stack_size;
        . = ALIGN(4);
        __stack_start = .;
    } > SRAM

    /* Define linker symbols */

    _end = .; /* end of image */

/* SRAM End */

	/* Data Closely Coupled Memory (DCCM) */
/* DCCM Start */
/* DCCM End */

    }
