#include <CurieTime.h>

void setup() {
  while (!Serial);
  Serial.begin(9600);

  // set the current time to 14:27:00, December 14th, 2015
  setTime(14, 27, 00, 14, 12, 2015);
}

void loop() {
  Serial.print("Time now is: ");

  print2digits(hour());
  Serial.print(":");
  print2digits(minute());
  Serial.print(":");
  print2digits(second());

  Serial.print(" ");

  Serial.print(day());
  Serial.print("/");
  Serial.print(month());
  Serial.print("/");
  Serial.print(year());

  Serial.println();

  delay(1000);
}

void print2digits(int number) {
  if (number >= 0 && number < 10) {
    Serial.print('0');
  }
  Serial.print(number);
}
