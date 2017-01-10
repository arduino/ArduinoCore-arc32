## MemoryFree library for Arduino/Genuino101 and tinyTILE

This library is a re-write of http://playground.arduino.cc/Code/AvailableMemory
specifically for Curie-based devices. This library defines the same header file
`MemoryFree.h`, and provides the function `freeMemory()`. In addition, two extra
functions are provided; `freeHeap()`, for heap space only, and `freeStack()`,
for free stack space. `freeStack()` can also be used to detect a stack overflow.

The rewrite is necessary for two reasons:

* AVR-based boards use a different implementation of `malloc()` than Curie-based
  boards, and in order to determine the amount of free heap space, we depend
  on specific symbols defined by the `malloc()` implementation. The `malloc()`
  implementation used by Curie-based devices
  [can be seen here](https://github.com/foss-for-synopsys-dwc-arc-processors/glibc).

* Curie-based boards have a different memory layout than AVR-based boards. See
  the [linker script](https://github.com/01org/corelibs-arduino101/blob/master/variants/arduino_101/linker_scripts/flash.ld)
  for Arduino/Genuino 101 for details of the memory layout
  for Curie-based devices.

## Functions


`int freeHeap (void)`

Returns the number of bytes free in the heap, i.e. the number of bytes free
to be allocated using `malloc()`.

`int freeStack (void)`

Returns the number of bytes free in the stack, i.e. the space between the
current stack frame and the end of the stack area. This function will return
a negative number in the event of a stack overflow, representing the size of
the overflow; for example, a return value of -20 means that the current stack
frame is 20 bytes past the end of the stack area.

`int freeMemory (void)`

Returns the number of bytes free in both the stack and the heap. If a stack
overflow has occurred, only the number of bytes free in the heap is returned.
