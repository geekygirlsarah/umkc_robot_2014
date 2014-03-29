/**
 * March 2014
 * Author: Sarah W
 *
 * Detect if it's driven through a gap with IR sensors
 **/

// Sarah recommends a tolerance value of 23 cm

#include <Distance2D120X.h>

#ifndef GAPCROSSEDDETECTOR_H 
#define GAPCROSSEDDETECTOR_H


class GapCrossedDetector {
private:
    // Distances in CM from each sensor 
    int distance1,distance2,distance3;
    
    // T/F flags if each sensor has been tripped
    bool tripped1, tripped2, tripped3;
    
    // minimum distance each sensor should read before tripping the flag
    int tolerance;
    
    // Sensors
    Distance2D120X ir1;
    Distance2D120X ir2;  
    Distance2D120X ir3;
    
    // T/F state if it has crossed the gap
    bool hasCompletedGapDrive;


public:


    //Which 3 pins are you using to find this gap? (num is how many you will actually use) 
    //must be in order, from right or left doesn't matter
    // Tolerance is minimum CM IR sensors have to reach before setting flags
    void init(int pin1, int pin2, int pin3, int theTolerance)  {
        ir1.begin(pin1);
        ir2.begin(pin2);
        ir3.begin(pin3);
        tolerance = theTolerance; 
        
        tripped1 = false;
        tripped2 = false;
        tripped3 = false; 
        hasCompletedGapDrive = false;    
    }

    //find and print the distances w/CM
    void printDebugCM()  {
        distance1 = ir1.getDistanceCentimeter();
        distance2 = ir2.getDistanceCentimeter();
        distance3 = ir3.getDistanceCentimeter();
        
        Serial.print("Dist cm #1: ");
        Serial.print(distance1);
        Serial.print("     Dist cm #2: ");
        Serial.print(distance2);    
        Serial.print("     Dist cm #3: ");
        Serial.println(distance3);
        Serial.print("Detected wave yet? ");
        if(hasCompletedGapDrive)
            Serial.print("yes");
        else
            Serial.print("no");
        
        Serial.println();    
    }
  
    //find and print the distances w/ raw values
    void printDebugRaw()  {
        distance1 = ir1.getDistanceRaw();
        distance2 = ir2.getDistanceRaw();
        distance3 = ir3.getDistanceRaw();
        
        Serial.print("Dist cm #1: ");
        Serial.print(distance1);
        Serial.print("     Dist cm #2: ");
        Serial.print(distance2);    
        Serial.print("     Dist cm #3: ");
        Serial.println(distance3);
        Serial.print("Detected wave yet? ");
        if(hasCompletedGapDrive)
            Serial.print("yes");
        else
            Serial.print("no");
        
        Serial.println();    
    }

    void update() {
        /*****************
         * The logic:
         * 1) Grab distances
         * 2) If any of them go below a hard-coded value, note it
         * 3) If three of them trip, we drove through a gap                             
         *****************/     
        // If we found the gap, don't even check anymore.
                                      
        if(!hasCompletedGapDrive)
        {
            distance1 = ir1.getDistanceCentimeter();
            distance2 = ir2.getDistanceCentimeter();
            distance3 = ir3.getDistanceCentimeter();

            // The idea is that at this point only one sensor can detect a wave, the others
            // should detect "infinity", so if any of them go below tolerance, count it as 
            // just one            
            if (distance1 < tolerance)
                tripped1 = true;
            else if(distance2 < tolerance) 
                tripped2 = true;
            else if (distance3 < tolerance)
                tripped3 = true;
            
            if(tripped1 && tripped2 && tripped3)
            {
                hasCompletedGapDrive = true;            
            } 
        }    
    }

    bool haveDroveThroughGap()
    {
        return hasCompletedGapDrive;
    }
    
    void reset()
    {
        tripped1 = false;
        tripped2 = false;
        tripped3 = false;
        hasCompletedGapDrive = false;        
    }

};

#endif

