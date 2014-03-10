


/**
 * parallel park  i simpleified version
 * feb 2014
 *
 * Given three  (nope two) IR sensors in a row, indicate which way to turn in order to align robot parallel
 * written for the 2014 ieee region 5 comp
 *
 
 //
  //
 **/
#include <Distance2D120X.h>
#include <DistSmoother.h>

#include <motor_cmd.h>
#include <movement.h>

#ifndef PARALLELPARKSIMPLE_H 
#define PARALLELPARKSIMPLE_H 


class ParallelPark {
private:

//
//ok time for some sloppy hacky coding. 
//hopefully this works
//
	motor_cmd* saber; 
  //Using 3 IR sensors along the side. 
  // confused - If at any time any of  my sensors read the maximum distance. IE, i haven't caught a wave at all 
  // turn_cw -  whoah you need to turn clockwise
  // turn_ccw -  whoah you need to turn counterclockwise
  // parallel - we're good and parallel to the wall.
  enum internal  { 
    parallel, turn_ccw, turn_cw, confused}; 
  internal park_status;

  bool flag;		//used for assumption
  int check;        //how many times in a row must the sensors read 
  int outerloopcount;	//how many times the update() method will run thru the state loop
  int distance1,distance2,distance3;  //used for debugging
//  double prevDistance1, prevDistance3;	//trying out the running average

  bool isPar;
  DistSmoother Dist1;
  DistSmoother Dist2;  
  DistSmoother Dist3;

  const static int difference = 5;  //the minimum difference between far left and far right sensor to indicate not parallel
  const static int upperBound= 25;	//Distrust all readings above this value, will switch to confused mode.  (note our sensors will max out on 35cm).
  const static float alpha = .80;	//used for exponential moving avg filter						
  const static float countInARow= 5;	//test - used to set the check for each stage, how many times checkCW () in a row it must be good


  const static int countAverage= 2;	//how many times  to average readings from distsmoother

  //if countAcceptable out of countCheck readings are "valid" or parallel -> then we assume it is
  int countAcceptableActual;	//how many readings I have seen that are acceptable
  const static int countAcceptableRequired = 5;	//how many readings I must be over to be acceptable overall
  const static int numGroupReadings = 7;	//how many readings (grouped) to take in total 

  bool readings[numGroupReadings];	//instead of taking groups of readings at once -> take a running set
  int indexReadings;	
  
  bool actuallyParallel()	{
	
	readings[indexReadings] = checkParallel();
	indexReadings++;
	if(indexReadings >= numGroupReadings)	//if we're at end of array, wrap around
		indexReadings = 0;

	//now let's count the # of parallel readings
	countAcceptableActual = 0;
	for(int i = 0; i< numGroupReadings; i++)	{
		if(readings[indexReadings])
			countAcceptableActual++;
	}
	return (countAcceptableActual> countAcceptableRequired);
  }

  //calls checkParallel() multiple times... sees from there 
  //call THIS ONE from the outside
/*
   bool actuallyParallel()	{
	countAcceptableActual = 0;
	for(int i =0; i< numGroupReadings; i++)	{
		if(checkParallel()) 	{
			countAcceptableActual++;	
		}
		else	{
			//spin and do nothin
		}
  
	}
	return (countAcceptableActual> countAcceptableRequired);
  }
  */
 //checks side ir sensors - am I parallel ish?
 //
 //nope . no hackyness ok EVEN MORE HACK-Y. ASSUMING I am turning Clockwise... I am "parallel" as SOON AS they are kinda similar OR one sensor > greater than another. 
 bool checkParallel()	{
	 
	//take average of a couple readings just to be sure
	distance1 = 0;
	distance3 = 0;
	for(int i = 0; i< countAverage; i++)	{	
		distance1 += Dist1.getFilteredDistanceCM();
		distance3 += Dist3.getFilteredDistanceCM();
	}
	distance1 /= countAverage;
	distance3 /= countAverage;
	if(!isDataTrusted(distance1,distance3))
		return false;
	return (abs(distance3 - distance1)  <  difference);
///	return distance1-distance3 < difference || distance3 > distance1;	//this will be SUPER SUPER JUMPy
  } 
 
 //do i really need to turn ccw?
 bool needToTurnCCW()	{
	 
	//take average of a couple readings just to be sure
	distance1 = 0;
	distance3 = 0;
	for(int i = 0; i< countAverage; i++)	{	
		distance1 += Dist1.getFilteredDistanceCM();
		distance3 += Dist3.getFilteredDistanceCM();
	}
	distance1 /= countAverage;
	distance3 /= countAverage;
	if(!isDataTrusted(distance1,distance3))
		return false;
	return distance1 < distance3;
  } 

 //do i really need to turn cw?
 bool needToTurnCW()	{
	 
	//take average of a couple readings just to be sure
	distance1 = 0;
	distance3 = 0;
	for(int i = 0; i< countAverage; i++)	{	
		distance1 += Dist1.getFilteredDistanceCM();
		distance3 += Dist3.getFilteredDistanceCM();
	}
	distance1 /= countAverage;
	distance3 /= countAverage;
	if(!isDataTrusted(distance1,distance3))
		return false;
	return distance1 > distance3;
  } 
 //if all three, two, or an edge sensor are consistently maxed out => haven't caught a wave yet 
 bool checkConfused()	{
	distance1 = Dist1.getFilteredDistanceCM();
	distance3 = Dist3.getFilteredDistanceCM();
	if(!isDataTrusted(distance1, distance3)) 
		return true;
	
	return false;
 }

 bool isDataTrusted(int d1, int d2)	{
 	
	return !( distance1 > upperBound || distance3 > upperBound);
 }
  
 //state machine - to make sure there really is a gap, and not just a sensor misreading
  internal update()  {
	//so my code works when i call debug(), when i comment it out it doesn't.
	//so my hunch is i need to call the smoothdistance more often

		  /*
	distance1 = Dist1.getFilteredDistanceCM();
    distance2 = Dist2.getFilteredDistanceCM();
    distance3 = Dist3.getFilteredDistanceCM();   
	*/
	//wrapping this entire thing in a while... hopefully speed this up		  
	outerloopcount= 1;
	while(outerloopcount-- > 0)	{
			switch (park_status)  {
			case confused:
				if(needToTurnCW())
					park_status = turn_cw; 
				else if (needToTurnCCW())
					park_status = turn_ccw;
					
			break;

			case turn_cw:
				if(actuallyParallel())
					park_status = parallel;
				else if (needToTurnCCW())
					park_status = turn_ccw;
				else if (checkConfused())
					park_status = confused;
			break;
			
			case turn_ccw:
				if(actuallyParallel())
					park_status = parallel;
				else if (needToTurnCW())
					park_status = turn_cw;
				else if (checkConfused())
					park_status = confused;
			break;


		
			case parallel:
				//have to check!!! those pesky pesky incorrect values
//				check = countInARow;
//				park_status = parallel;
				//while(check -- > 0)	{
				//	if(!checkParallel())
				//		park_status = start;
				//}
			break;
		}
	}
	return park_status;
  }


public:



  //Which 3 pins are you using to find this gap?
  //must be in order. pin1 must be the leftmost sensor facing out, pin3 must be rightmost sensor facing out 
  
 void init(int pin1, int pin2, int pin3, motor_cmd* s)  {
    
	//init reading array
	for(int i = 0; i<  numGroupReadings; i++)	{
		readings[i] = false;	
	}
	Dist1.init(pin1);
    Dist2.init(pin2);
    Dist3.init(pin3);
    check = 3;
    park_status = confused;    //we start out confused
	saber = s;
  }

  //find and print the distances
  void printDebug()  {
   /*
	distance1 = Dist1.getDistanceCentimeter();
    distance2 = Dist2.getDistanceCentimeter();
    distance3 = Dist3.getDistanceCentimeter();
    */

	distance1 = Dist1.getFilteredDistanceCM();
    distance2 = Dist2.getFilteredDistanceCM();
    distance3 = Dist3.getFilteredDistanceCM();

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
   if(park_status == confused)  {
      Serial.println("park_status: confused======");
    }
     else if (park_status == turn_cw)  {
      Serial.println("park_status: turn_cw"); 
    }
     else if (park_status == turn_ccw)  {
      Serial.println("park_status: turn_ccw"); 
	 }
	else if (park_status ==  parallel)	{
      Serial.println("park_status: victory - parallel"); 
    }
  }

  void reset()	{
  	park_status = confused;
  }

  //hack and slash code.
  //keep moving until its parallel...
  //will take care of the movement junk as well
  void parallelPark()	{
 		 //keep turning.. until you are parallel

		//... so it's like a little state machine consuming the other machine T.T 
		saber->turnCW();	//arbitraily just turn cw

		while(update() != parallel)	{
				switch(park_status)	{
					case( confused):
						saber->turnCW();
					break;
					case(turn_cw):
						saber->turnCW();
					break;
					case(turn_ccw):
						saber->turnCCW();
					break;
					case(parallel):
						//shouldl never come here
					break;
				}
				printDebug();
				printStatus();
		}
		//it's parallel!	
		saber->all_stop();

  
  }

  /*
  bool isParallel()	{
 	return park_status == parallel;	
  }
*/
};

#endif

