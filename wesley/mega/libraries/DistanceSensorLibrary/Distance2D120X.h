//UMKC 2014 Robotics
//
//IR library for Sharp 2D120X
//using formula experiemntally derived by Eric Gonzalez  (eric.gonzalez@mail.umkc.edu)

// Arduino library for distance sensors
// Copyright 2011-2013 Jeroen Doggen (jeroendoggen@gmail.com)

#ifndef Distance2D120X_h
#define Distance2D120X_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#include <pins_arduino.h>
#endif

#include "AnalogDistanceSensor.h"

class Distance2D120X: public AnalogDistanceSensor
{

private:
	int prevDistance;
	int currentDistance;
public:
    Distance2D120X();
    int getDistanceCentimeter();
	int getSmoothedDistanceCM(float alpha);
};
#endif
