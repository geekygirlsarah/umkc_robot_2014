/**
* GapFinder
* Jan 2014
* Author: Victoria Wu, Andrew Cunningham
*
* Given three IR sensors in a row, finds a gap thingy
* written for the 2014 ieee region 5 comp
**/

#ifndef GAPFINDER_H
#define GAPFINDER_H

#include <DistanceGP2Y0A21YK.h>  //ir sensor madness
  



class GapFinder	{
private:
  enum ternary  { no_gap, maybe_gap, yes_gap };
  ternary gap_status;
  
  int check;        //how many times in a row must the sensors read a hole
  int distance1,distance2,distance3;  //used for debugging
  
  DistanceGP2Y0A21YK Dist1;
  DistanceGP2Y0A21YK Dist2;  
  DistanceGP2Y0A21YK Dist3;
  
  const static int threshold = 20;  //threshold for ping sensor detecting a hole (cm)
  
  bool gapPresent()  {
    return Dist1.isFarther(threshold) && Dist2.isFarther(threshold);
  }
  
public:
  //Which 3 pins are you using to find this gap?
  void init(int pin1, int pin2, int pin3)  {
    Dist1.begin(pin1);
    Dist2.begin(pin2);
    Dist3.begin(pin3);
    check = 3;
    gap_status = no_gap;    //we start out assuming no hole
  }
  
  //find and print the distances
  void debug()  {
    distance1 = Dist1.getDistanceCentimeter();
    distance2 = Dist2.getDistanceCentimeter();
    distance3 = Dist3.getDistanceCentimeter();
    //difference = distance1 - distance2;
    
    Serial.println("Distance in centimers #1: ");
    Serial.print(distance1);
    Serial.println("Distance in centimers #2: ");
    Serial.print(distance2);    
    Serial.println("Distance in centimers #3: ");
    Serial.print(distance3); 
    delay(500); //make it readable
  }
   
  //State machine - to make sure there really is a gap, and not just a sensor misreading
  void findGap()  {
    switch (gap_status)  {
      case no_gap:
        if(gapPresent())
          gap_status = maybe_gap;
        break;
      case maybe_gap:
        //maybe check for gap many many times? or is it better to just delay and wait a bit?
        check = 3;
        while(check-- > 0)  {
          delay(20);    //delay just in case
          if(gapPresent())
            gap_status = yes_gap;
          else
            gap_status = no_gap;
           break;    //only if 3 in a row read back HOLE, will we move to yes gap
        }
        break;
      case yes_gap:
        if(!gapPresent())
          gap_status = no_gap;
        break;
    
    }
    if(gap_status == no_gap)
      Serial.println("Nope");
    else if (gap_status == maybe_gap)
      Serial.println("Maybe");
    else
      Serial.println("GAP!");
  }
  

};

#endif
