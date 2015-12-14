#include <CurieRTC.h>

const char *monthName[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

int Hour, Min, Sec;
int Day, Month, Year;

void setup() {
  while(!Serial);
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
  if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3) return false;
  return true;
}

bool getDate(const char *str)
{
  char monthString[12];

  if (sscanf(str, "%s %d %d", monthString, &Day, &Year) != 3) {
    return false;
  }

  for (Month = 1; Month <= 12; Month++) {
    if (strcmp(monthString, monthName[Month - 1]) == 0) {
      break;
    }
  }

  return (Month <= 12);
}

void print2digits(int number) {
  if (number >= 0 && number < 10) {
    Serial.print('0');
  }
  Serial.print(number);
}
