/**
 * GapFinder
 * Jan 2014
 * Author: Victoria Wu, Andrew Cunningham
 *
 * Given three IR sensors in a row, finds a gap thingy
 * written for the 2014 ieee region 5 comp
 *
 * TODO: Put in the IR sensor modification thingy, the IR sensor values are jumping EVERWHERE
 * TODO: refine the error checking... i don't want to ever mistakenly get to GAP stage.
 *   strange floaty issues - will vascilllate between MAYBE and def maybe state. Need to make those state transitions more defined 
 * TODO: interface with the outside world - return a thingy from gapFind
 **/

#ifndef GAPFINDER_H
#define GAPFINDER_H

#include <DistanceGP2Y0A21YK.h>  //ir sensor madness
#include <stdio.h>


class GapFinder	{
private:
  //Using 3 IR sensors along the side. 
  //no gap - none of the 3 IR sensors indicate more 
  //maybe gap - YES no no | no no YES (ignore middle sensor)
  //definitely maybe gap - YES YES no | no YES YES (Time to slow down!!)
  //YES GAP - YES YES YES (we're at a gap)
  enum internal  { 
    no_gap, maybe_gap, def_maybe_gap, yes_gap };
  internal gap_status;

  int check;        //how many times in a row must the sensors read a hole
  int distance1,distance2,distance3;  //used for debugging

  DistanceGP2Y0A21YK Dist1;
  DistanceGP2Y0A21YK Dist2;  
  DistanceGP2Y0A21YK Dist3;

  const static int threshold = 20;  //threshold for ping sensor detecting a hole (cm)


  
  //checks side IR sensors for YES, to move from no_gap state to maybe_gap
  bool checkMaybeGap()  {
    return Dist1.isFarther(threshold) || Dist3.isFarther(threshold);
  }
  
  //checks if 2 IR sensors say YES, to move from maybe_gap to def_maybe_gap state 
  bool checkDefMaybeGap()  {
    return Dist2.isFarther(threshold) && (Dist1.isFarther(threshold) || Dist3.isFarther(threshold)); 
  }
  
  //checks if all 3 IR sensors say YES, to move from def_maybe_gap to yes_gap
  bool checkYesGap()  {
    return Dist1.isFarther(threshold) && Dist2.isFarther(threshold) && Dist3.isFarther(threshold);
  }


public:

  //FindGap returns this enum. 
  //NO_GAP -> nothing
  //MAYBE_GAP -> Slow down! There is a gap ahead.
  //YES_GAP -> Stop, there's definitely a gap right here!
  enum ternary {NO_GAP, MAYBE_GAP, YES_GAP};  
  


  //Which 3 pins are you using to find this gap?
  //must be in order, from right or left doesn't matter
  void init(int pin1, int pin2, int pin3)  {
    Dist1.begin(pin1);
    Dist2.begin(pin2);
    Dist3.begin(pin3);
    check = 3;
    gap_status = no_gap;    //we start out assuming no hole
  }

  //find and print the distances
  void printDebug()  {
    distance1 = Dist1.getDistanceCentimeter();
    distance2 = Dist2.getDistanceCentimeter();
    distance3 = Dist3.getDistanceCentimeter();
    //difference = distance1 - distance2;

    Serial.print("Distance in centimers #1: ");
    Serial.println(distance1);
    Serial.print("Distance in centimers #2: ");
    Serial.println(distance2);    
    Serial.print("Distance in centimers #3: ");
    Serial.println(distance3); 
    delay(500); //make it readable
  }

  //State machine - to make sure there really is a gap, and not just a sensor misreading
  ternary findGap()  {
    switch (gap_status)  {
    case no_gap:
      if(checkMaybeGap())
        gap_status = maybe_gap;
      break;

    case maybe_gap:
      //maybe check for gap many many times? or is it better to just delay and wait a bit?
      check = 2;

      if(checkDefMaybeGap())
          gap_status = def_maybe_gap;
      else if (check-- == 0)
          gap_status = no_gap;
      break;    //only if 3 in a row read back HOLE, will we move to yes gap
      
      
    case def_maybe_gap:  
      //SLOW DOWN!!
      check = 3;
      while(check-- >0)  {
       if(checkYesGap())
        gap_status = yes_gap;
       else if (!checkDefMaybeGap())
        gap_status = maybe_gap;
      }
     
      break;
    case yes_gap:
      if(!checkYesGap())
        gap_status = no_gap;
      break;

    }
    if(gap_status == no_gap)  {
      Serial.println("Nope");
      return NO_GAP;
    }
    else if (gap_status == maybe_gap)  {
      Serial.println("Maybe - 1 sensor");
      return NO_GAP;
    }
    else if (gap_status == def_maybe_gap)  {
      
      Serial.println("Definitely maybe - 2 sensor");
      return MAYBE_GAP;
    }
    else  {
      Serial.println("GAP! - 3 Sensors");
      return YES_GAP;
    }
  }


  void report(byte* packet) {
    packet[0] = Dist1.getDistanceCentimeter();
    packet[1] = Dist2.getDistanceCentimeter();
    packet[2] = Dist3.getDistanceCentimeter();
  } 
};

#endif

