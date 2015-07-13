// TODO - replace with a proper implementation of panic()
#define panic(x) _do_fault();
#define force_panic() panic(-1)
