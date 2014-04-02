


/**
 * GapFinder
 * Jan 2014
 *
 * Given three IR sensors in a row, finds a gap thingy
 * written for the 2014 ieee region 5 comp
 *
 * TODO: Put in the IR sensor modification thingy, the IR sensor values are jumping EVERWHERE (DONE)
 * TODO: refine the error checking... i don't want to ever mistakenly get to GAP stage. (not done yet)
 *   strange floaty issues - will vascilllate between MAYBE and def maybe state. Need to make those state transitions more defined 
 * TODO: interface with the outside world - return a thingy from gapFind (DONE)
 **/
#include <Distance2D120X.h>
#include <DistSmoother.h>

#include <motor_cmd.h>

#ifndef GAPFINDER_H
#define GAPFINDER_H

#define NUM_GAP_CHECKS 3

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
  int outerloopcount;	//how many times the update() method will run thru the state loop
  int distance1,distance2,distance3;  //used for debugging

  DistSmoother Dist1;
  DistSmoother Dist2;  
  DistSmoother Dist3;

  motor_cmd *saber;

  const static int threshold_default = 10;  //threshold for ping sensor detecting a hole (cm)
  int threshold;  //threshold for ping sensor detecting a hole (cm)


  
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
  void init(int pin1, int pin2, int pin3 , motor_cmd* s)  {
    Dist1.init(pin1);
    Dist2.init(pin2);
    Dist3.init(pin3);
    check = 3;
    threshold = threshold_default;
    gap_status = no_gap;    //we start out assuming no hole

    saber = s;

  }


  //Which 3 pins are you using to find this gap?
  //must be in order, from right or left doesn't matter
  void init(int pin1, int pin2, int pin3, int thresh, motor_cmd* s)  {
    Dist1.init(pin1);
    Dist2.init(pin2);
    Dist3.init(pin3);
    check = 3;
    threshold = thresh;
    gap_status = no_gap;    //we start out assuming no hole
    
    saber = s;
  }

  //find and print the distances
  void printDebug()  {
    distance1 = Dist1.getAccurateDistCM();
    distance2 = Dist2.getAccurateDistCM();
    distance3 = Dist3.getAccurateDistCM();
    //difference = distance1 - distance2;

    Serial.print("dist(cm):\t");
    Serial.print(distance1);
    Serial.print("\t");
    Serial.print(distance2);    
    Serial.print("\t");
    Serial.println(distance3); 
    //delay(500); //make it readable
  }

  void printGapStatus()  {
   if(gap_status == no_gap)  {
      Serial.println("gap_status: Nope");
    }
    else if (gap_status == maybe_gap)  {
      Serial.println("gap_status: Maybe - 1 sensor");
    }
    else if (gap_status == def_maybe_gap)  {
      
      Serial.println("gap_status: Definitely maybe - 2 sensor");
    }
    else  {
      Serial.println("gap_status: GAP! - 3 Sensors");
    }
  }

  //State machine - to make sure there really is a gap, and not just a sensor misreading
  ternary update()  {
    
	//wrapping this entire thing in a while... hopefully speed this up		  
	outerloopcount= 5;
	while(outerloopcount-- > 0)	{
			switch (gap_status)  {
			case no_gap:
			  if(checkMaybeGap())
				gap_status = maybe_gap;
			  break;

			case maybe_gap:
			  //maybe check for gap many many times? or is it better to just delay and wait a bit?
			  //check = 2;
		  //TODO TODO make this nicer instead of vascillating between maybegap and no gap
			  if(checkDefMaybeGap())
				  gap_status = def_maybe_gap;
			  else 
				  gap_status = no_gap;
			  break;    //only if 3 in a row read back HOLE, will we move to yes gap
			  
			case def_maybe_gap:  
			  //SLOW DOWN!!
			  check = 5;
			  gap_status = yes_gap;	//assume that there is a gap.
			  while(check-- >0)  {
				if(!checkYesGap())	//want to check if yes gap check times in a ROW
					gap_status = maybe_gap;
			  }
			 
			  break;
			case yes_gap:
			
			saber->all_stop();	//attempting to STOP as soon as gap is reached
			//  if(!checkYesGap())
			//	gap_status = no_gap;
			  break;

			}
	}
    if(gap_status == no_gap)  {
      return NO_GAP;
    }
    else if (gap_status == maybe_gap)  {
      return NO_GAP;
    }
    else if (gap_status == def_maybe_gap)  {
      return MAYBE_GAP;
    }
    else  {
      return YES_GAP;
    }
  }

  //this one values quickness more than accuracy.
  //ok to be moving
  bool gapPresent()	{
 	return gap_status == yes_gap;	
  }

  //this one assumes you are already stopped, checks in a row.
  //DO NOT BE MOVING WHEN YOU DO THIS
  bool gapPresentThorough()	{
 	for(int i = 0; i < NUM_GAP_CHECKS; i++)
 		{        
		 	if(!checkYesGap())
 				return false; 
 		}    
 	return true;
  }

  bool maybeGapPresent()	{
  	return gap_status == def_maybe_gap;
  }

  bool gapNotPresent()	{
  	return (gap_status == no_gap || gap_status == maybe_gap);
  }

  //reset back to no gap state. 
  void reset()	{
 	gap_status = no_gap; 
  }

};

#endif

