


/**
 * Jan 2014
 * Author: Victoria Wu
 *
 * Testing IR sensors
 **/
#include <Distance2D120X.h>


#ifndef IRSENSORTESTER_H 
#define IRSENSORTESTER_H 



class IRSensorTester {
private:
  int distance1,distance2,distance3;  //used for debugging

  Distance2D120X Dist1;
  Distance2D120X Dist2;  
  Distance2D120X Dist3;
  
  int irNumber;


public:


  //Which 3 pins are you using to find this gap? (num is how many you will actually use) 
  //must be in order, from right or left doesn't matter
  void init(int num, int pin1, int pin2, int pin3)  {
    Dist1.begin(pin1);
    Dist2.begin(pin2);
    Dist3.begin(pin3);
    irNumber = num;
  }

  //find and print the distances
  void printDebugCM()  {
    distance1 = Dist1.getDistanceCentimeter();
    distance2 = Dist2.getDistanceCentimeter();
    distance3 = Dist3.getDistanceCentimeter();
    
    if(irNumber >=1)  {
        Serial.print("Dist cm #1: ");
        Serial.println(distance1);
    }
    if(irNumber >= 2)  {
        Serial.print("Dist cm #2: ");
        Serial.println(distance2);    
    }
    if(irNumber >= 3)  {
        Serial.print("Dist cm #3: ");
        Serial.println(distance3); 
    }
   
    Serial.println();
    delay(500); //make it readable
    
  }
  
    void printDebugRaw()  {
    distance1 = Dist1.getDistanceRaw();
    distance2 = Dist2.getDistanceRaw();
    distance3 = Dist3.getDistanceRaw();
    //difference = distance1 - distance2;
    
    if(irNumber >=1)  {
        Serial.print("Distance raw #1: ");
        Serial.println(distance1);
    }
    if(irNumber >= 2)  {
        Serial.print("Distance raw #2: ");
        Serial.println(distance2);    
    }
    if(irNumber >= 3)  {
        Serial.print("Distance raw #3: ");
        Serial.println(distance3); 
    }
    delay(500); //make it readable
  }



};

#endif

