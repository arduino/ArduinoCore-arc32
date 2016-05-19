//SerialFlash library API: https://github.com/PaulStoffregen/SerialFlash

#include <SerialFlash.h>
#include <SPI.h>

#define FSIZE 256

const char *filename = "myfile.txt";
const char *contents = "0123456789ABCDEF";

const int FlashChipSelect = 21; // digital pin for flash chip CS pin

void setup() {
  Serial.begin(9600);

  // wait for Arduino Serial Monitor
  while (!Serial) ;
  delay(100);

  // Init. SPI Flash chip
  if (!SerialFlash.begin(FlashChipSelect)) {
    Serial.println("Unable to access SPI Flash chip");
  }

  SerialFlashFile file;

  // Create the file if it doesn't exist
  if (!create_if_not_exists(filename)) {
    Serial.println("Not enough space to create file " + String(filename));
    return;
  }

  // Open the file and write test data
  file = SerialFlash.open(filename);
  file.write(contents, strlen(contents) + 1);
  Serial.println("String \"" + String(contents) + "\" written to file " + String(filename));
}

bool create_if_not_exists (const char *filename) {
  if (!SerialFlash.exists(filename)) {
    Serial.println("Creating file " + String(filename));
    return SerialFlash.create(filename, FSIZE);
  }

  Serial.println("File " + String(filename) + " already exists");
  return true;
}

void loop() {
}
