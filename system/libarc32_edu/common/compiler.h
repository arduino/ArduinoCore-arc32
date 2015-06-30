#ifndef _COMPILER_H__
#define _COMPILER_H__

#ifdef __GNUC__

#define _Usually(x) __builtin_expect(!!((x)), 1)
#define _Rarely(x) __builtin_expect(!!((x)), 0)
#define _sr(_src_, _reg_) __builtin_arc_sr((unsigned int)_src_, _reg_)
#define _lr(_reg_) __builtin_arc_lr(_reg_)
#define _nop()

#endif

#endif
