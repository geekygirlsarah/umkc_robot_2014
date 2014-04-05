#include <DistanceSensor.h>
#include <Distance2D120X.h>
#include <AnalogDistanceSensor.h>

#include "wallfollower.h"

const byte pin1 = A0;
const byte pin2 = A1;
const byte pin3 = A2;
const int port 9600;

WallFollower wf;

void setup(){
  Serial.begin(9600);
  wf.init(pin1,pin2,pin3);
}
void loop(){
  wf.checkWall();
  wf.printDebug();
}
