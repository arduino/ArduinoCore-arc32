# corelibs-arduino101

The contents of this repo is distributed through releases in Arduino IDE.    
`Tools > Board > Boards Manager > Intel Curie Boards by Intel`

If you wish to use the latest **untested** changes, follow these instructions.

1. Install the latest `Intel Curie Boards by Intel` from `Boards Manager`
2. Download the [latest snapshot](https://github.com/01org/corelibs-arduino101/archive/master.zip)
   of this repo
3. Shut down the IDE
4. Go to Arduino15 directory
  * Windows: `C:\Users\<user>\AppData\Roaming\Arduino15`
  * OS X: `~/Library/Arduino15`
  * Linux: `~/.arduino15`
5. Go to `packages/Intel/hardware/arc32/<version>/`
6. Delete the content of the directory from step 5, and replace it with the
   content of the "corelibs-arduino101-master" folder in the zip from step 2.

Future upgrades may fail since the internal contents were modified by hand. In
order to recover, shut down the IDE, delete the entire `Arduino15` directory,
then restart the IDE.

# Pull Requests

Before submitting a pull request, please see our
[guidelines](https://github.com/01org/corelibs-arduino101/wiki/Writing-a-commit-message)
for writing a considerate commit message.

# Support & Issues

If you have found a bug, or you believe a new feature should be added, please
use the Github issue tracker (click "Issues" above) to provide details about
the bug or feature. If you need product support (e.g. have a question about /
are having problems with the Arduino IDE or the Arduino API), please direct
them to the [support forum](https://forum.arduino.cc/index.php?board=103).

## Examples of things that should go in the Issue tracker

> "I noticed that your DoSomeThing library doesn't support all the same
> modes as the library from SomeOtherGuy: https://link-to-relevant-thing.com
> Can you add support for these modes?"

> "If I run example sketch X on an Arduino 101 board, I get result Y. But if I
> run the same sketch on an Arduino UNO board, I get result Z. This looks like
> a bug to me."

## Examples of things that should go in the support forum

> "I'm having trouble downloading the Arduino 101 boards package in the Arduino
> IDE Boards Manager"

> "How do I use this library?"

> "I can't get this example sketch to work. What am I doing wrong?"
