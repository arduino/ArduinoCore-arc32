Table of Contents
=================

   * [CuriePower](#curiepower)
   * [CuriePower API reference](#curiepower-api-reference)
      * [Functions](#functions)
         * [CuriePower.doze()](#curiepowerdoze)
         * [CuriePower.doze(int duration)](#curiepowerdozeint-duration)
         * [CuriePower.idle()](#curiepoweridle)
         * [CuriePower.idle(int duration)](#curiepoweridleint-duration)
         * [CuriePower.sleep()](#curiepowersleep)
         * [CuriePower.sleep(int duration)](#curiepowersleepint-duration)
         * [CuriePower.deepSleep()](#curiepowerdeepsleep)
         * [CuriePower.deepSleep(int duration)](#curiepowerdeepsleepint-duration)
         * [CuriePower.attachInterruptWakeup(uint32_t pin, voidFuncPtr callback, uint32_t mode)](#curiepowerattachinterruptwakeupuint32_t-pin-voidfuncptr-callback-uint32_t-mode)
         * [CuriePower.detachInterruptWakeup(uint32_t pin)](#curiepowerdetachinterruptwakeupuint32_t-pin)
   * [Tutorials](#tutorials)
      * [Tutorial #1: TimedWakeup Example](#tutorial-1-timedwakeup-example)
      * [Tutorial #2: WakeFromIMU Example](#tutorial-2-wakefromimu-example)

# CuriePower
CuriePower is a Power Management library for Curie based boards such as the Arduino101/Genuino101 and tinyTILE

# CuriePower API reference

## Functions

### ``CuriePower.doze()``

```
void CuriePower.doze()
```

Places the SoC in "doze" mode which switches the system clock to the internal 32.768 kHz RTC oscillator.

*Parameters*

none

*Return value*

none

### ``CuriePower.doze(int duration)``

```
void CuriePower.doze(int duration)
```


Places the SoC in "doze" mode which switches the system clock to the internal 32.768 kHz RTC oscillator for `duration` milliseconds


*Parameters*

1. `int duration` : number in milliseconds to doze

*Return value*

none

### ``CuriePower.idle()``

```
void CuriePower.idle()
```

Places the SoC into "doze" mode then enters an infinite loop, effectively stopping all operations until a wake interrupt is generated.

*Parameters*

none

*Return value*

none

### ``CuriePower.idle(int duration)``

```
void CuriePower.idle(int duration)
```


Places the SoC in "doze" mode and enters an infinite loop for `duration` milliseconds


*Parameters*

1. `int duration` : number in milliseconds to idle

*Return value*

none

### ``CuriePower.sleep()``

```
void CuriePower.sleep()
```

Places the SoC into a sleep state, stopping all operations, until a wake interrupt is generated

*Parameters*

none

*Return value*

none

### ``CuriePower.sleep(int duration)``

```
void CuriePower.sleep(int duration)
```


Places the SoC into a sleep state for `duration` milliseconds


*Parameters*

1. `int duration` : number in milliseconds to sleep

*Return value*

none

### ``CuriePower.deepSleep()``

```
void CuriePower.deepSleep()
```

Places the SoC into a deep sleep state, stopping all operations, until a wake interrupt is generated

*Parameters*

none

*Return value*

none

### ``CuriePower.deepSleep(int duration)``

```
void CuriePower.deepSleep(int duration)
```


Places the SoC into a deep sleep state for `duration` milliseconds


*Parameters*

1. `int duration` : number in milliseconds to deep sleep

*Return value*

none

### ``CuriePower.attachInterruptWakeup(uint32_t pin, voidFuncPtr callback, uint32_t mode)``

```
void CuriePower.attachInterruptWakeup(uint32_t pin, voidFuncPtr callback, uint32_t mode)
```

Attaches a wakeup interrupt to digital pin `pin`. `callback` is the function to be called when the wakeup interrupt occurs. 

*Parameters*

none

*Return value*

none

### ``CuriePower.detachInterruptWakeup(uint32_t pin)``

```
void CuriePower.detachInterruptWakeup(uint32_t pin)
```

Removes any wakeup interrupt attached to digital pin `pin`. 

*Parameters*

none

*Return value*

none

# Tutorials

## Tutorial #1: TimedWakeup Example

This sketch demonstrates the simplest way to use the CuriePower library. It blinks the LED a few times, goes to sleep for a certain amount of time then goes back at the start of loop()

```cpp
#include <Power.h>

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  for(int i = 0; i < 5; i++)
  { 
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
  PM.sleep(1000);
}
```

The line of code that is most interesting here is:
```cpp
PM.sleep(1000);
```
This puts the SoC into sleep, drawing significantly less power, for 1000ms or 1s.

In what situations is this most useful?

Let's says you have a battery powered project that reads a sensor value once every second and saves it to an SD card.
Simply using delay() will work but you will notice that you will run out of battery pretty fast. That is because even though the code is not doing much inside delay, it is still running everything at full clock speed and all peripherals are turned on.
When we put the SoC to sleep, several things are turned off. This includes most of the peripherals. voltage rails, and some clocks. Basically, it draws much less power and no code is running until a wake interrupt is generated.

Many Arduino projects typically have a loop where you read a sensor, do something with that reading, and then delay for a set amount of time.
In most cases, reading the sensor and doing something with that reading takes very little time, much smaller than the delay duration. This means that we are wasting a lot of power inside the delay doing nothing.
By placing the SoC to sleep instead of just waiting inside the delay, we can save a considerable amount of power in most applications.


## Tutorial #2: WakeFromIMU Example

This sketch uses CuriePower library and the CurieIMU library together and demonstrates the use of an interrupt to wake the Curie SoC from sleep.

```cpp
#include <Power.h>
#include "CurieIMU.h"


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  CurieIMU.begin();
  CurieIMU.attachInterrupt(wakeup);
  CurieIMU.setDetectionThreshold(CURIE_IMU_MOTION, 20);      // 100mg
  CurieIMU.setDetectionDuration(CURIE_IMU_MOTION, 10);       // trigger times of consecutive slope data points
  CurieIMU.interrupts(CURIE_IMU_MOTION);
  
}

void loop() {
  PM.sleep();
  digitalWrite(LED_BUILTIN, HIGH);
  delay(3000);
  digitalWrite(LED_BUILTIN, LOW);
}

void wakeup()
{
  PM.wakeFromDoze();
  // This function will be called once on device wakeup
  // You can do some little operations here (like changing variables which will be used in the loop)
  // Remember to avoid calling delay() and long running functions since this functions executes in interrupt context
}
```

If you look at the beginning of loop() you will see:
```cpp
PM.sleep();
```
This line of code puts the SoC into a sleep state. The SoC will remain in a sleep state until it is woken by an interrupt.
If no interrupt triggers the SoC to wake up, it will stay in a sleep state forever(or until the battery runs out).
This is specifically useful in applications that are not periodic, like in this example sketch where the SoC stays at a sleep state until the Arduino101 detects motion, or more precisely the Bosch BMI160 [6-Axis Accelerometer/Gyroscope] sensor detects the motion and triggers an interrupt to wake the Curie Soc from sleep.

Inside setup() we have:
```cpp
CurieIMU.attachInterrupt(wakeup);
CurieIMU.setDetectionThreshold(CURIE_IMU_MOTION, 20);      // 100mg
CurieIMU.setDetectionDuration(CURIE_IMU_MOTION, 10);       // trigger times of consecutive slope data points
CurieIMU.interrupts(CURIE_IMU_MOTION);
```
These lines of code attaches a method called wakeup() which is called whenever motion is detected. Since the interrupt signal generated by the Bosch BMI160 sensor is already internally connected to AON(Always On) interrupt of the SoC, it automatically wakes the SoC from sleep. 
However, since we are using the attachInterrupt() method of the CurieIMU Library instead of the one from the CuriePower Library we still need to do one more thing after the SoC is taken out of a sleep state.

Inside the wakeup() method:
```cpp
PM.wakeFromDoze();
```
This line of code simply takes the SoC out of the Doze state, switching it from using the internal RTC 32.768 KHz as the main clock, back to the 32Mhz oscillator.
At this point the SoC runs back at full speed ready to do stuff quickly and then go back to sleep to save power.
