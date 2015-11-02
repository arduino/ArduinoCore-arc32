#include <Time.h>
#include <CurieRTC.h>

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("CurieRTC Read Test");
  Serial.println("-------------------");
}

void loop() {
  tmElements_t tm;
  time_t t = RTC.get();
  breakTime(t, tm);
  
  Serial.print("Ok, Time = ");
  print2digits(tm.Hour);
  Serial.write(':');
  print2digits(tm.Minute);
  Serial.write(':');
  print2digits(tm.Second);
  Serial.print(", Date (D/M/Y) = ");
  Serial.print(tm.Day);
  Serial.write('/');
  Serial.print(tm.Month);
  Serial.write('/');
  Serial.print(tmYearToCalendar(tm.Year));
  Serial.println();
  delay(1000);
}

void print2digits(int number) {
  if (number >= 0 && number < 10) {
    Serial.write('0');
  }
  Serial.print(number);
}
