# corelibs-arduino101

The contents of this repo is distributed through releases in Arduino IDE.    
`Tools > Board > Boards Manager > Intel Curie Boards by Intel`

If you wish to use the latest **untested** changes, follow these instructions.

1. Install the latest `Intel Curie Boards by Intel` from `Boards Manager`
2. Download the [latest snapshot](https://github.com/01org/corelibs-arduino101/archive/master.zip) of this repo
3. Shut down the IDE
4. Go to Arduino15 directory
  * Windows: `C:\Users\<user>\AppData\Roaming\Arduino15`
  * OS X: `~/Library/Arduino15`
  * Linux: `~/.arduino15`
5. Go to `packages/Intel/hardware/arc32/<version>/`
6. Delete the content of the directory from step 5, and replace it with the content of the zip from step 2

Future upgrades may fail since the internal contents were modified by hand. In order to recover, shut down the IDE, delete the entire `Arduino15` directory, then restart the IDE.
