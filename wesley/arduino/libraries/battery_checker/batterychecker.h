/**
 * March 2014
 * Author: Sarah W
 *
 * Battery checker, reports if the voltage drops below TOLERANCE 
 **/

#ifndef BATTERYCHECKER_H 
#define BATTERYCHECKER_H

// Tolerance in volts (ex: 3.6)
#define TOLERANCE 4.0

#include <Arduino.h>

class BatteryChecker {
private:

    int analogPin;
    float lastReadValue;
    bool safeOperatingVoltage;


public:


    // Parameter for analog pin that voltage is connected to
    void init(int inputPin)  {
        analogPin = inputPin;
        lastReadValue = 0;
        safeOperatingVoltage = true;
        
        analogReference(DEFAULT);

    }

    // print input voltage
    void printDebug()  {
		if(lastReadValue < TOLERANCE)
			Serial.print("***WARNING!*** ");
        Serial.print("Current voltage: ");
        Serial.println(lastReadValue);
    }
  

    bool update() {
        int value = analogRead(analogPin);
        lastReadValue = 0.0049 * value;

        if (lastReadValue < TOLERANCE)
        {
            safeOperatingVoltage = false;        
        }
        
        return safeOperatingVoltage; 
    }
    
    bool isSafe() {
        return safeOperatingVoltage;
    }

    void reset()
    {
        safeOperatingVoltage = true;        
    }

};

#endif

