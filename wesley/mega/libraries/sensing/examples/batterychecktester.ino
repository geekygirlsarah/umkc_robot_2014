// Battery Checker Tester
// prints out voltage on A0 analog pin. If it goes below tolerance, then 
// it prints out warning too.

#include <batterychecker.h>

BatteryChecker bc;

void setup()
{
  Serial.begin(9600);
  analogReference(DEFAULT);
  bc.init(A0);
}

void loop()
{
  bc.update();
  bc.printDebug();
}

