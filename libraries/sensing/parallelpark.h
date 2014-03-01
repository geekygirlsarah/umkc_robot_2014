


/**
 * parallel park 
 * feb 2014
 *
 * Given three IR sensors in a row, indicate which way to turn in order to align robot parallel
 * written for the 2014 ieee region 5 comp
 *
 * TOD TO DO TOTOTODODODO
  -> -> If I am confused (35 |35 ), i should NEVER go into parallel victory mode.
 
 * baaad :(
   //TODOTODODODODO 
  //problem - parallel sometimes drifts off. vascillates between parallel and start, 
  // and isParallel() should ideally account for that .
  //
  // broke. why is this happening?
  // movement::forever turn
  // dist(cm): 	8	16	16
  // park_status: ccw
  // movement::forever turn
  // dist(cm): 	11	14	35
  // park_status: ccw
  // movement::forever turn
  // dist(cm): 	10	18	35
  // park_status: ccw
  //    ::CMD:: stopping
  //    its parallel!
  //
  //
  //    TODO - do min/max range thingy
  //	//measuing stuff as it happens
  //	moving the erroc checking / averaging shenanigans to the functions and away from the state mahcine
  //
  //
  //
  //	MORE PROEBLSMSDLFKJDSL:Fd
  //	TODO - CCW -> CW... too fast. it skips the parallel, and just keeps being stuck in CCW stage
  //
 **/
#include <Distance2D120X.h>


#ifndef PARALLELPARK_H 
#define PARALLELPARK_H 


class ParallelPark {
private:
  //Using 3 IR sensors along the side. 
  // parallel - we're good and parallel to the wall.
  // maybe_cw - might need to go clockwise. unsure. must check
  // maybe_ccw - might need to go counterclockwise. unsure. must check
  // maybe_parallel - we might have started out parallel. gotta make sure 
  // cw - need to turn clockwise to get parallel. 
  // ccw - need to turn counter clockwise to get parallel
  // start - always restart here, if you're not sure whether to turn cw or ccw.want to choose wisely
  // confused - If at any time my sensors read the maximum distance i will be confused. CONFUSEDDDD! 
  enum internal  { 
    parallel, maybe_cw, maybe_ccw, maybe_parallel, cw, ccw, start, confused}; 
  internal park_status;

  bool flag;		//used for assumption
  int check;        //how many times in a row must the sensors read 
  int outerloopcount;	//how many times the update() method will run thru the state loop
  int distance1,distance2,distance3;  //used for debugging
//  double prevDistance1, prevDistance3;	//trying out the running average

  Distance2D120X Dist1;
  Distance2D120X Dist2;  
  Distance2D120X Dist3;

  const static int difference = 5;  //the minimum difference between far left and far right sensor to indicate not parallel
  const static int upperBound= 25;	//Distrust all readings above this value, will switch to confused mode.  (note our sensors will max out on 35cm).
  const static float alpha = .80;	//used for exponential moving avg filter						
  const static float countInARow= 10;	//test - used to set the check for each stage, how many times checkCW () in a row it must be good


 
  
  //checks side IR sensors - do I need to turn clockwise? (listing CCW) 
  bool checkClockwise()  {
   // distance1 = Dist1.getDistanceCentimeter();
   // distance3 = Dist3.getDistanceCentimeter();
	  distance1 = Dist1.getSmoothedDistanceCM(alpha);
      distance3 = Dist3.getSmoothedDistanceCM(alpha);

	if(distance1 - distance3 > difference)
		return true;
	else
		return false;
  }

  //checks side IR sensors - do I need to turn counter clockwise? (listing CW) 
  bool checkCounterClockwise()	{
 	  distance1 = Dist1.getSmoothedDistanceCM(alpha);
      distance3 = Dist3.getSmoothedDistanceCM(alpha);

	if(distance3 - distance1 > difference)
		return true;
	else
		return false;

  }
 //checks side ir sensors - am I parallel ish?
 bool checkParallel()	{
	  distance1 = Dist1.getSmoothedDistanceCM(alpha);
      distance3 = Dist3.getSmoothedDistanceCM(alpha);

	if(distance1 >  upperBound|| distance3 > upperBound)
		return false;

	if(abs(distance3 - distance1)  <  difference)
		return true;
	else
		return false;

  } 

 //if both of my sensors are maxed out, indicate your confusion. 
 bool checkConfused()	{
 	check = 10;
	while(check -- > 0)	{
			if( distance1 > upperBound && distance3 > upperBound)
					return true;
	}
	return false;
 }
 
public:


  //FindGap returns this enum. 
  //NO_GAP -> nothing
  //MAYBE_GAP -> Slow down! There is a gap ahead.
  //YES_GAP -> Stop, there's definitely a gap right here!
//  enum ternary {NO_GAP, MAYBE_GAP, YES_GAP};  
  


  //Which 3 pins are you using to find this gap?
  //must be in order. pin1 must be the leftmost sensor facing out, pin3 must be rightmost sensor facing out 
  void init(int pin1, int pin2, int pin3)  {
    Dist1.begin(pin1);
    Dist2.begin(pin2);
    Dist3.begin(pin3);
    check = 3;
    park_status = start;    //we start out assuming no hole
  }

  //find and print the distances
  void printDebug()  {
   /*
	distance1 = Dist1.getDistanceCentimeter();
    distance2 = Dist2.getDistanceCentimeter();
    distance3 = Dist3.getDistanceCentimeter();
    */

	distance1 = Dist1.getSmoothedDistanceCM(alpha);
    distance2 = Dist2.getSmoothedDistanceCM(alpha);
    distance3 = Dist3.getSmoothedDistanceCM(alpha);

	//difference = distance1 - distance2;

    Serial.print("dist(cm): \t");
    Serial.print(distance1);
    Serial.print("\t");
    Serial.print(distance2);    
    Serial.print("\t");
    Serial.print(distance3); 
	Serial.println();
    //delay(500); //make it readable
  }

  void printStatus()  {
   if(park_status == start)  {
      Serial.println("park_status: start");
    }
    else if (park_status == cw)  {
      Serial.println("park_status: cw");
    }
    else if (park_status == ccw)  {
      
      Serial.println("park_status: ccw"); 
    }
     else if (park_status == maybe_ccw)  {
      
      Serial.println("park_status: maybe_ccw"); 
    }
	 else if (park_status == maybe_ccw)  {
      
      Serial.println("park_status: maybe_ccw"); 
    }else if (park_status == maybe_parallel)  {
      
      Serial.println("park_status: maybe_parallel"); 
    }
	else if (park_status == confused)
			Serial.println("park_status: confused ---------- " );

    else  {
      Serial.println("park_status: victory - parallel"); 
    }
  }

  void reset()	{
  	park_status = start;
  }

  //State machine - to make sure there really is a gap, and not just a sensor misreading
  void update()  {
	//so my code works when i call debug(), when i comment it out it doesn't.
	//so my hunch is i need to call the smoothdistance more often

	distance1 = Dist1.getSmoothedDistanceCM(alpha);
    distance2 = Dist2.getSmoothedDistanceCM(alpha);
    distance3 = Dist3.getSmoothedDistanceCM(alpha);   
	//wrapping this entire thing in a while... hopefully speed this up		  
	outerloopcount= 5;
	while(outerloopcount-- > 0)	{
			switch (park_status)  {
			case start:
				if(checkClockwise())
					park_status = maybe_cw;
				else if (checkCounterClockwise())
					park_status = maybe_ccw;
				else if (checkParallel())
					park_status = maybe_parallel;
				else if (checkConfused())
					park_status = confused;
				//derp. what if it starts as parallel???
			break;

			//both side sensors are reading max. this is not good :(
			case confused:
				//stay here until confusion is resolved.
				if(!checkConfused())
					park_status = start;

			break;
			case maybe_parallel:
				check = countInARow;
				flag = true;
				while(check-- > 0)	{
					if(!checkParallel())	{
						park_status = start;
						flag = false;
					}
				}
				if(flag) //if status HASN"T CHANGED we should be ok with our assume
					park_status = parallel;
				if(checkConfused())
					park_status = confused;
	break;


			case maybe_cw:
				check = countInARow;
				flag = true;
				while(check-- > 0)	{
					if(!checkClockwise())	{
						park_status = start;
						flag = false;
					}
				}

				if(flag) //if status HASN"T CHANGED we should be ok
					park_status = cw;
				if(checkConfused())
					park_status = confused;
			break;

			case maybe_ccw:
				check = countInARow;
				flag = true;
				while(check-- > 0)	{
					if(!checkCounterClockwise())	{
						flag = false;
						park_status = start;
					}
				}
				if(flag) //if status HASN"T CHANGED we should be ok
					park_status = ccw;
				if(checkConfused())
					park_status = confused;
			break;

			case cw:
				//stay here until you are parallel
				check = countInARow;
				flag = true;
				while(check --> 0)	{
					if(!checkParallel())	{
						park_status = cw;
						flag = false;
					}
				}
				if(flag) //if status HASN"T CHANGED we should be ok
					park_status = parallel;
				if(checkConfused())
					park_status = confused;
			break;

			case ccw:
				//stay here until you are parallel
				check = countInARow;
				flag = true;
				while(check --> 0)	{
					if(!checkParallel())	{
						flag = false;
						park_status = ccw;
					}
				}
				if(flag) //if status HASN"T CHANGED we should be ok
					park_status = parallel;
				if(checkConfused())
					park_status = confused;
			break;
			
			case parallel:
				//have to check!!! those pesky pesky incorrect values
				check = countInARow;
				park_status = parallel;
				//while(check -- > 0)	{
				//	if(!checkParallel())
				//		park_status = start;
				//}
			break;
		}
	}
  }

 
  bool isParallel()	{
 	return park_status == parallel;	
  }
 
  bool needToTurnCW()	{
 	return (!isParallel() || park_status == cw);	
  }
 
  bool needToTurnCCW()	{
 	return (!isParallel() || park_status == ccw);	
  }


};

#endif

