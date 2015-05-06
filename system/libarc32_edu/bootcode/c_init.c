#include <stdint.h>
#include <string.h>

#include "interrupt.h"

/* Application main() function prototype */
extern int main (void);

/* C++ constructor function prototype */
typedef void (*cpp_ctor_fn) (void);
/* C++ constructor list */
extern cpp_ctor_fn __CTOR_LIST__[];

/* BSS section markers */
extern char __bss_start[];
extern char __bss_end[];

/* DATA section markers */
extern char __data_rom_start[];
extern char __data_ram_start[];
extern char __data_ram_end[];

static void _exec_ctors (void)
{
    unsigned long i, nctors = (unsigned long)(__CTOR_LIST__[0]);

    for (i = nctors; i > 0; i--)
        __CTOR_LIST__[i]();
}

 __attribute__((__noreturn__)) void _main (void)
{
    /* Zero BSS section */
    memset(__bss_start, 0, __bss_end - __bss_start);
    /* Relocate DATA section to RAM */
    memcpy(__data_ram_start, __data_rom_start, __data_ram_end - __data_ram_start);
    /* Execute C++ Constructors */
    _exec_ctors();
    /* Init the the interrupt unit device - disable all the interrupts; The
     * default value of IRQ_ENABLE is 0x01 for all configured interrupts */
    interrupt_unit_device_init();
    /* Jump to application main() */
    main ();
    /* Never reached */
    __builtin_unreachable();
}
