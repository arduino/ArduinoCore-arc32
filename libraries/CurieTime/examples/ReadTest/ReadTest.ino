#include <CurieTime.h>

void setup() {
  while (!Serial);
  Serial.begin(9600);

  Serial.println("CurieTime Read Test");
  Serial.println("-------------------");
}

void loop() {
  Serial.print("Ok, Time = ");
  print2digits(hour());
  Serial.write(':');
  print2digits(minute());
  Serial.write(':');
  print2digits(second());
  Serial.print(", Date (D/M/Y) = ");
  Serial.print(day());
  Serial.write('/');
  Serial.print(month());
  Serial.write('/');
  Serial.print(year());
  Serial.println();
  delay(1000);
}

void print2digits(int number) {
  if (number >= 0 && number < 10) {
    Serial.write('0');
  }
  Serial.print(number);
}
