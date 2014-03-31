

//test prog to compare raw/actual ditances in the ir sensor
//author vicky wu

//Currently
//organizing this out into a classs thingy (done)

//TODO
//add in the ir sensor //thingy formula thingy
//add in using 3 ir sensors, and the slow down thingy
#include <Distance2D120X.h>
#include <gapfinder.h>
#include <irsensor_tester.h>
#include <magellan_edgesensors.h>

//WITH THE thigy -> IN INCHES!!! (problem - when you don't have anything... it dfaults to 8 inches

IRSensorTester tester;

void setup()
{
  Serial.begin(9600); 
  tester.init(1,A0,A1,A2); 

}

void loop()
{
  tester.printDebugRaw();
  tester.printDebugCM();
  
}

