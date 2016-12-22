/* Written by Oren Levy (auxren.com; @auxren) while competing on
   America's Greatest Makers with help from Intel.
   MIDI over BLE info from: https://developer.apple.com/bluetooth/Apple-Bluetooth-Low-Energy-MIDI-Specification.pdf

  This sketch plays a random MIDI note (between 0 and 127) every 400ms.
  For a 'smarter' sketch, check out my Airpeggiator example.
  The Airpeggiator uses the Curie's IMU to allow you to play
  an imaginary harp in the air. I included a quantizer so you can
  select a key and scale so you can jam along with friends.
  https://github.com/auxren/MIDIBLE101/tree/master/Airpeggiator

  I have only tested MIDI over BLE using Apple devices. Android doesn't
  support native MIDI over BLE yet and I haven't had much of a chance
  to test with Windows machines.

  To connect on a Mac, search for Audio MIDI Setup.
  Click 'Window' on the top menu and choose 'Show MIDI Studio'.
  Double click 'Bluetooth' and the bluetooth configuration window
  will pop up. After loading the MIDIBLE sketch on your Arduino 101
  you should see it advertising as Auxren. Click connect and the device
  will be available as MIDI device in all your audio software like Garageband.

  There are a few ways to connect using an iOS device. One way to to open
  up Garageband. Click on the wrench icon in the upper right and choose 'Advanced'
  Towards the bottom of advanced, you will see 'Bluetooth MIDI devices'.
  You should see your Arduino 101 advertising in the list. Connect to
  your device and it should be available to all other iOS MIDI apps you have.

  To send data, you use the following line: char.setValue(d, n); where char is
  the BLE characteristic (in our case, midiCha), d is the data, and n is the
  number of bytes of data.
  The first 2 bytes of data are the header byte and timestamp byte. If you want,
  you can figure out the timestamping scheme, but I just left it with a generic value
  since I haven't worked on anything timeing sensitive yet (like a sequencer).
  The third, fourth, and fifth bytes are standard MIDI bytes. You can send more bytes
  if you would like as long as it is complies to the standard MIDI spec.

  The MIT License (MIT)

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

*/
#include <CurieBLE.h>

#define TXRX_BUF_LEN              20 //max number of bytes
#define RX_BUF_LEN                20 //max number of bytes
uint8_t rx_buf[RX_BUF_LEN];
int rx_buf_num, rx_state = 0;
uint8_t rx_temp_buf[20];
uint8_t outBufMidi[128];

//Buffer to hold 5 bytes of MIDI data. Note the timestamp is forced
uint8_t midiData[] = {0x80, 0x80, 0x00, 0x00, 0x00};

//Loads up buffer with values for note On
void noteOn(char chan, char note, char vel) //channel 1
{
  midiData[2] = 0x90 + chan;
  midiData[3] = note;
  midiData[4] = vel;
}

//Loads up buffer with values for note Off
void noteOff(char chan, char note) //channel 1
{
  midiData[2] = 0x80 + chan;
  midiData[3] = note;
  midiData[4] = 0;
}

//BLEPeripheral midiDevice; // create peripheral instance

BLEService midiSvc("03B80E5A-EDE8-4B33-A751-6CE34EC4C700"); // create service

// create switch characteristic and allow remote device to read and write
BLECharacteristic midiChar("7772E5DB-3868-4112-A1A9-F2669D106BF3",  BLEWrite | BLEWriteWithoutResponse | BLENotify | BLERead,5);

void setup() {
  Serial.begin(9600);

  BLESetup();
    // advertise the service
  BLE.advertise();
  Serial.println(("Bluetooth device active, waiting for connections..."));
}

void BLESetup()
{
  BLE.begin();
  // set the local name peripheral advertises
  BLE.setLocalName("Auxren");

  // set the UUID for the service this peripheral advertises
  BLE.setAdvertisedServiceUuid(midiSvc.uuid());

  // add service and characteristic
  
  midiSvc.addCharacteristic(midiChar);
  BLE.addService(midiSvc);

  // assign event handlers for connected, disconnected to peripheral
  BLE.setEventHandler(BLEConnected, midiDeviceConnectHandler);
  BLE.setEventHandler(BLEDisconnected, midiDeviceDisconnectHandler);

  // assign event handlers for characteristic
  midiChar.setEventHandler(BLEWritten, midiCharacteristicWritten);
  // set an initial value for the characteristic
  midiChar.setValue(midiData, 5);


}

void loop() {

  /*Simple randome note player to test MIDI output
     Plays random note every 400ms
  */
  BLE.poll();
  int note = random(0, 127);
  //readMIDI();
  noteOn(0, note, 127); //loads up midiData buffer
  midiChar.setValue(midiData, 5);//midiData); //posts 5 bytes
  delay(200);
  noteOff(0, note);
  midiChar.setValue(midiData, 5);//midiData); //posts 5 bytes
  delay(200);
}


void midiDeviceConnectHandler(BLEDevice central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
}

void midiDeviceDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
}

void midiCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  // central wrote new value to characteristic, update LED
  Serial.print("Characteristic event, written: ");
}
