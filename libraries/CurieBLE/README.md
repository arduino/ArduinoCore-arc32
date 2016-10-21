# Enable debug interface on Serail1

* Default disable the debug interface.

If you want to enable debug trace on Serial1 to debug corelib,  follow these instructions.

1. Shut down the IDE
2. Go to Arduino15 directory
  * Windows: `C:\Users\<user>\AppData\Roaming\Arduino15`
  * OS X: `~/Library/Arduino15`
  * Linux: `~/.arduino15`
3. Modify the platform.txt
  * Find `compiler.c.flags` and add `-DCONFIGURE_DEBUG_CORELIB_ENABLED` at the end of this line
  * Find `compiler.cpp.flags` and add `-DCONFIGURE_DEBUG_CORELIB_ENABLED` at the end of this line
4. Initial Serial1 in your sketch
  * Add `Serial1.begin(115200);` in your `setup()`
5. Adjust the output level at log_init function in log.c

