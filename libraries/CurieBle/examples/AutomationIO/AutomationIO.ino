/*
 * Copyright (c) 2015 Intel Corporation.  All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.

 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */
#include <CurieBle.h>

/*
 * This sketch partially implements the standard Bluetooth Low-Energy "Automation IO" service.
 * It is a relatively complex example which demonstrates use of BLE descriptors and
 * multi-instance characteristics for data input and data output.
 *
 * For more information: https://developer.bluetooth.org/gatt/services/Pages/ServicesHome.aspx
 */

/* Local name: Appears in advertising packets. Must not exceed 20 characters in length */
#define LOCAL_NAME                "AE_IO"
/* UUID for Automation I/O service */
#define SERVICE_UUID_AUTOMATIONIO (0x1815)
/* UUID for digital characteristic */
#define CHAR_UUID_DIGITAL         (0x2A56)
/* UUID for analog characteristic */
#define CHAR_UUID_ANALOG          (0x2A58)
/* UUID for number_of_digitals descriptor */
#define DESC_UUID_NUMDIGITALS     (0x2909)

/* Structure to encapsulate information for each pin
 * This helps us to build lists of pins below to allow
 * flexible pin assignments to different modes of operation
 */
struct DigitalPinConfig {
  unsigned          pin;
  const char        *name;
  BleCharacteristic characteristic;
  BleDescriptor     numDigitalsDesc;
  uint8_t           val;
};

struct AnalogPinConfig {
  unsigned          pin;
  const char        *name;
  BleCharacteristic characteristic;
  uint16_t          val;
};

/* Macros to simplify the definition of a new PinConfig struct for a given pin number
 * Note that input pins are only readable by the remote device, while output pins are
 * only writable.  Different characteristic UUIDs are used for digital and analog pins */
#define DIGITAL_INPUT_PINCONFIG(pin) \
  { (pin), #pin, {CHAR_UUID_DIGITAL, sizeof(uint8_t), BLE_CLIENT_ACCESS_READ_ONLY, BLE_CLIENT_NOTIFY_ENABLED}, {DESC_UUID_NUMDIGITALS, sizeof(uint8_t), BLE_CLIENT_ACCESS_READ_ONLY} }
#define DIGITAL_OUTPUT_PINCONFIG(pin) \
  { (pin), #pin, {CHAR_UUID_DIGITAL, sizeof(uint8_t), BLE_CLIENT_ACCESS_WRITE_ONLY, BLE_CLIENT_NOTIFY_DISABLED}, {DESC_UUID_NUMDIGITALS, sizeof(uint8_t), BLE_CLIENT_ACCESS_READ_ONLY} }
#define ANALOG_INPUT_PINCONFIG(pin) \
  { (pin), #pin, {CHAR_UUID_ANALOG, sizeof(uint16_t), BLE_CLIENT_ACCESS_READ_ONLY, BLE_CLIENT_NOTIFY_ENABLED} }
#define ANALOG_OUTPUT_PINCONFIG(pin) \
  { (pin), #pin, {CHAR_UUID_ANALOG, sizeof(uint16_t), BLE_CLIENT_ACCESS_WRITE_ONLY, BLE_CLIENT_NOTIFY_DISABLED} }

/* The following lists of pins are configured and presented to
 * the remote BLE device as digital/analog input/output pins
 *
 * These lists can be changed, but with the following caveats:
 * - each pin should only be included once, and cannot be included by multiple lists
 * - only valid pins should be used
 *   - pins 0-19 are valid for digital input/output
 *   - pins 14-19 are valid for analog input
 *   - pins 3,5,6,9 are valid for analog output
 * - a maximum of 16 pins are currently supported (limited by number of characteristics)
 */
struct DigitalPinConfig digitalInputPins[] = {
  DIGITAL_INPUT_PINCONFIG(2),
  DIGITAL_INPUT_PINCONFIG(4),
  DIGITAL_INPUT_PINCONFIG(7),
};

struct DigitalPinConfig digitalOutputPins[] = {
  DIGITAL_OUTPUT_PINCONFIG(8),
  DIGITAL_OUTPUT_PINCONFIG(10),
  DIGITAL_OUTPUT_PINCONFIG(11),
  DIGITAL_OUTPUT_PINCONFIG(12),
  DIGITAL_OUTPUT_PINCONFIG(13),
};

struct AnalogPinConfig analogInputPins[] = {
  ANALOG_INPUT_PINCONFIG(14),
  ANALOG_INPUT_PINCONFIG(15),
  ANALOG_INPUT_PINCONFIG(16),
  ANALOG_INPUT_PINCONFIG(17),
};

struct AnalogPinConfig analogOutputPins[] = {
  ANALOG_OUTPUT_PINCONFIG(3),
  ANALOG_OUTPUT_PINCONFIG(5),
  ANALOG_OUTPUT_PINCONFIG(6),
  ANALOG_OUTPUT_PINCONFIG(9),
};

/* BLE Peripheral Device (this Intel Curie device) */
BlePeripheral blePeripheral;

/* BLE Automation I/O Service */
BleService ioService(SERVICE_UUID_AUTOMATIONIO);

/* The standard allows for multiple digital pins to be included per characteristic,
 * where the pin states are encapsulated as an array of 2-bit values
 *
 * This simplified implementation assumes just one pin per characteristic for now.
 */
#define DIGITAL_PIN_STATE_TO_VAL(pin, state) (uint8_t)((state) ? 0x1 << 6 : 0)
#define VAL_TO_DIGITAL_PIN_STATE(pin, val)   (uint8_t)((val) ? HIGH : LOW)

#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x)[0])

/* Serial port to use for printing informational messages to the user */
#define LOG_SERIAL Serial

/* For convenience, this macro will invoke a specified function call and will
 * check the status value returned to ensure it is successful.  If not, it will
 * print an error message to the serial port and will return from the current function
 */
#define CHECK_STATUS(op)                               \
  do {                                                 \
    BleStatus status = op;                             \
    if (BLE_STATUS_SUCCESS != status) {                \
      LOG_SERIAL.print(#op" returned error status: "); \
      LOG_SERIAL.println(status);                      \
      return;                                          \
    }                                                  \
  } while(0)

/* This function will be called when a BLE GAP event is detected by the
 * Intel Curie BLE device */
void blePeripheralEventCb(BlePeripheral &peripheral, BlePeripheralEvent event, void *arg)
{
  if (BLE_PERIPH_EVENT_CONNECTED == event) {
    LOG_SERIAL.println("Got CONNECTED event");
  } else if (BLE_PERIPH_EVENT_DISCONNECTED == event) {
    LOG_SERIAL.println("Got DISCONNECTED event");
  } else {
    LOG_SERIAL.println("Got UNKNOWN peripheral event");
  }
}

/* This function will be called when a connected remote peer sets a new value for a digital output characteristic */
void digitalOutputCharEventCb(BleCharacteristic &characteristic, BleCharacteristicEvent event, void *arg)
{
  unsigned pin = (unsigned)arg;
  uint8_t val;

  if (BLE_CHAR_EVENT_WRITE == event) {
    /* The remote client has updated the value for this pin, get the current value */
    characteristic.getValue(val);
    /* Update the state of the pin to reflect the new value */
    digitalWrite(pin, VAL_TO_DIGITAL_PIN_STATE(pin, val));
  } else
    LOG_SERIAL.println("Got UNKNOWN characteristic event");
}

/* This function will be called when a connected remote peer sets a new value for an analog output characteristic */
void analogOutputCharEventCb(BleCharacteristic &characteristic, BleCharacteristicEvent event, void *arg)
{
  unsigned pin = (unsigned)arg;
  uint16_t val;

  if (BLE_CHAR_EVENT_WRITE == event) {
    /* The remote client has updated the value for this pin, get the current value */
    characteristic.getValue(val);
    /* Update the state of the pin to reflect the new value */
    analogWrite(pin, val);
  }
  else
    LOG_SERIAL.println("Got UNKNOWN characteristic event");
}

void setup() {
  LOG_SERIAL.begin(115200);

  /* Set a name for the BLE device */
  CHECK_STATUS(blePeripheral.setLocalName(LOCAL_NAME));

  /* First, initialise the BLE device */
  CHECK_STATUS(blePeripheral.init());

  /* Set a function to be called whenever a BLE GAP event occurs */
  blePeripheral.setEventCallback(blePeripheralEventCb);

  CHECK_STATUS(blePeripheral.setAdvertisedServiceUuid(ioService.uuid()));

  /* Add the Automation I/O Service, and include the UUID in BLE advertising data */
  CHECK_STATUS(blePeripheral.addPrimaryService(ioService));

  /* Add characteristics for the Digital Inputs */
  for (unsigned i = 0; i < ARRAY_SIZE(digitalInputPins); i++) {
    DigitalPinConfig *pin = &digitalInputPins[i];

    /* Configure this pin as an input */
    pinMode(pin->pin, INPUT);

    /* Add the User-Description for this pin */
    CHECK_STATUS(pin->characteristic.addUserDescription(pin->name));
    /* Add the Presentation-Format for this pin
     *   format:      0x2    (unsigned 2-bit integer)
     *   exponent:    0      (Not Applicable)
     *   unit:        0x2700 (BLE GATT Unit UUID: unitless)
     *   nameSpace:   0x1    (Bluetooth standard namespace)
     *   description: pin+1  (Instance number of this characteristic)
     */
    CHECK_STATUS(pin->characteristic.addPresentationFormat(0x2, 0, 0x2700, 0x1, pin->pin + 1));
    /* Add the characteristic for this pin */
    CHECK_STATUS(ioService.addCharacteristic(pin->characteristic));
    /* Set an initial value for this characteristic; refreshed later in the loop() function */
    pin->val = digitalRead(pin->pin);
    CHECK_STATUS(pin->characteristic.setValue(DIGITAL_PIN_STATE_TO_VAL(pin->pin, pin->val)));
    /* Add a number_of_digitals descriptor for this characteristic */
    CHECK_STATUS(pin->characteristic.addDescriptor(pin->numDigitalsDesc));
    CHECK_STATUS(pin->numDigitalsDesc.setValue((uint8_t) 1));
  }

  /* Add characteristics for the Digital Outputs */
  for (unsigned i = 0; i < ARRAY_SIZE(digitalOutputPins); i++) {
    DigitalPinConfig *pin = &digitalOutputPins[i];

    /* Configure this pin as an output */
    pinMode(pin->pin, OUTPUT);

    /* Add the User-Description for this pin */
    CHECK_STATUS(pin->characteristic.addUserDescription(pin->name));
    /* Add the Presentation-Format for this pin
     *   format:      0x2    (unsigned 2-bit integer)
     *   exponent:    0      (Not Applicable)
     *   unit:        0x2700 (BLE GATT Unit UUID: unitless)
     *   nameSpace:   0x1    (Bluetooth standard namespace)
     *   description: pin+1  (Instance number of this characteristic)
     */
    CHECK_STATUS(pin->characteristic.addPresentationFormat(0x2, 0, 0x2700, 0x1, pin->pin + 1));
    /* Add the characteristic for this pin */
    CHECK_STATUS(ioService.addCharacteristic(pin->characteristic));
    /* Add a callback to be triggered if the remote device updates the value for this pin */
    pin->characteristic.setEventCallback(digitalOutputCharEventCb, (void*)pin->pin);
    /* Add a number_of_digitals descriptor for this characteristic */
    CHECK_STATUS(pin->characteristic.addDescriptor(pin->numDigitalsDesc));
    CHECK_STATUS(pin->numDigitalsDesc.setValue((uint8_t) 1));
  }

  /* Add characteristics for the Analog Inputs */
  for (unsigned i = 0; i < ARRAY_SIZE(analogInputPins); i++) {
    AnalogPinConfig *pin = &analogInputPins[i];

    /* Add the User-Description for this pin */
    CHECK_STATUS(pin->characteristic.addUserDescription(pin->name));
    /* Add the Presentation-Format for this pin
     *   format:      0x6    (unsigned 16-bit integer)
     *   exponent:    0      (Not Applicable)
     *   unit:        0x2700 (BLE GATT Unit UUID: unitless)
     *   nameSpace:   0x1    (Bluetooth standard namespace)
     *   description: pin+1  (Instance number of this characteristic)
     */
    CHECK_STATUS(pin->characteristic.addPresentationFormat(0x6, 0, 0x2700, 0x1, pin->pin + 1));
    /* Add the characteristic for this pin */
    CHECK_STATUS(ioService.addCharacteristic(pin->characteristic));
    /* Set an initial value for this characteristic; refreshed later in the loop() function */
    pin->val = analogRead(pin->pin);
    CHECK_STATUS(pin->characteristic.setValue(pin->val));
  }

  /* Add characteristics for the Analog Outputs */
  for (unsigned i = 0; i < ARRAY_SIZE(analogOutputPins); i++) {
    AnalogPinConfig *pin = &analogOutputPins[i];

    /* Add the User-Description for this pin */
    CHECK_STATUS(pin->characteristic.addUserDescription(pin->name));
    /* Add the Presentation-Format for this pin
     *   format:      0x6    (unsigned 16-bit integer)
     *   exponent:    0      (Not Applicable)
     *   unit:        0x2700 (BLE GATT Unit UUID: unitless)
     *   nameSpace:   0x1    (Bluetooth standard namespace)
     *   description: pin+1  (Instance number of this characteristic)
     */
    CHECK_STATUS(pin->characteristic.addPresentationFormat(0x6, 0, 0x2700, 0x1, pin->pin + 1));
    /* Add the characteristic for this pin */
    CHECK_STATUS(ioService.addCharacteristic(pin->characteristic));
    /* Add a callback to be triggered if the remote device updates the value for this pin */
    pin->characteristic.setEventCallback(analogOutputCharEventCb, (void*)pin->pin);
  }

  /* Now activate the BLE device.  It will start continuously transmitting BLE
   * advertising packets and thus become visible to remote BLE central devices
   * (e.g smartphones) until it receives a new connection */
  blePeripheral.begin();
  LOG_SERIAL.println("Bluetooth device active, waiting for connections...");
}

void loop() {
  blePeripheral.poll();
  
  /* Update the digital input characteristics based on current pin reading */
  for (unsigned i = 0; i < ARRAY_SIZE(digitalInputPins); i++) {
    DigitalPinConfig *pin = &digitalInputPins[i];
    uint8_t newVal = digitalRead(pin->pin);
    if (newVal != pin->val) {
      CHECK_STATUS(pin->characteristic.setValue(DIGITAL_PIN_STATE_TO_VAL(pin->pin, newVal)));
      pin->val = newVal;
    }
  }
  /* Update the analog input characteristics based on current pin reading*/
  for (unsigned i = 0; i < ARRAY_SIZE(analogInputPins); i++) {
    AnalogPinConfig *pin = &analogInputPins[i];
    uint16_t newVal = analogRead(pin->pin);
    if (newVal != pin->val) {
      CHECK_STATUS(pin->characteristic.setValue(newVal));
      pin->val = newVal;
    }
  }

  /* Repeat the loop every 500ms - can be changed if desired */
  delay(500);
}
