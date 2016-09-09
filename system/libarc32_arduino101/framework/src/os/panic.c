
#include "os/os.h"

extern void _do_fault();
void panic(int x)
{
    _do_fault();
}


void __assert_fail()
{
	panic(-10);
}


