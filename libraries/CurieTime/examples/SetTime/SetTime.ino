#include <CurieTime.h>

const char *monthName[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

int Hour, Min, Sec;
int Day, Month, Year;

void setup() {
  while (!Serial);
  Serial.begin(9600);

  // get the date and time the compiler was run
  if (getDate(__DATE__) && getTime(__TIME__)) {
    Serial.print("Curie configured Time=");
    Serial.print(__TIME__);
    Serial.print(", Date=");
    Serial.println(__DATE__);

    setTime(Hour, Min, Sec, Day, Month, Year);
  } else {
    Serial.print("Could not parse info from the compiler, Time=\"");
    Serial.print(__TIME__);
    Serial.print("\", Date=\"");
    Serial.print(__DATE__);
    Serial.println("\"");
  }
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

bool getTime(const char *str)
{
  // Get time from string of format HH:MM:SS
  String s = str;

  int firstColonIndex = s.indexOf(":");
  int lastColonIndex = s.lastIndexOf(":");

  if (firstColonIndex == -1) {
    // first colon not found
    return false;
  }

  if (lastColonIndex == -1) {
    // last colon not found
    return false;
  }

  if (firstColonIndex == lastColonIndex) {
    // only one colon, first and last index are the same
    return false;
  }

  String hourString = s.substring(0, firstColonIndex);
  String minuteString = s.substring(firstColonIndex + 1, lastColonIndex);
  String secondString = s.substring(lastColonIndex + 1);

  Hour = hourString.toInt();
  Min = minuteString.toInt();
  Sec = secondString.toInt();

  return true;
}

bool getDate(const char *str)
{
  // Get Date from string of format MMM DD YYYY
  String s = str;

  int firstSpaceIndex = s.indexOf(" ");
  int lastSpaceIndex = s.lastIndexOf(" ");

  if (firstSpaceIndex == -1) {
    // first space not found
    return false;
  }

  if (lastSpaceIndex == -1) {
    // last space not found
    return false;
  }

  if (firstSpaceIndex == lastSpaceIndex) {
    // only one space, first and last index are the same
    return false;
  }

  String monthString = s.substring(0, firstSpaceIndex);
  String dayString = s.substring(firstSpaceIndex + 1, lastSpaceIndex);
  String yearString = s.substring(lastSpaceIndex + 1);

  for (Month = 1; Month <= 12; Month++) {
    if (monthString.equals(monthName[Month - 1])) {
      break;
    }
  }

  Day = dayString.toInt();
  Year = yearString.toInt();

  return (Month <= 12);
}

void print2digits(int number) {
  if (number >= 0 && number < 10) {
    Serial.print('0');
  }
  Serial.print(number);
}
