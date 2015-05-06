#include "aux_regs.h"

.globl _do_isr
.type _do_isr,%function

.globl _IsrTable
.type _IsrTable,%object

.text
.balign 4
_do_isr:
    lr r0, [ARC_V2_ICAUSE]
    sub r0, r0, 16

    mov r1, _IsrTable
    add3 r0, r1, r0   /* table entries are 8-bytes wide */

    ld r1, [r0, 4] /* ISR into r1 */
    jl_s.d [r1]
    ld_s r0, [r0] /* delay slot: ISR parameter into r0  */

    /* back from ISR */
    rtie
    nop
