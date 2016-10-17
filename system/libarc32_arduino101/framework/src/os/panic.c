
#include "os/os.h"

#include "infra/log.h"
#include "aux_regs.h"

extern void _do_fault();
void panic(int x)
{
    _do_fault();
}


void __assert_fail()
{
	panic(-10);
}


void __attribute__((weak)) _Fault(void)
{
    uint32_t exc_addr = aux_reg_read(ARC_V2_EFA);
    uint32_t ecr = aux_reg_read(ARC_V2_ECR);

    pr_error(0, "Exception vector: 0x%x, cause code: 0x%x, parameter 0x%x\n",
           ARC_V2_ECR_VECTOR(ecr),
           ARC_V2_ECR_CODE(ecr),
           ARC_V2_ECR_PARAMETER(ecr));
    pr_error(0, "Address 0x%x\n", exc_addr);
    while (1);  // Sid.  Acknowledge KW warning.
}

