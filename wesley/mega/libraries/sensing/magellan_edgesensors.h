
/**
 * Magellan - Edge sensor class 
 * Jan 2014
 * Author: Victoria Wu
 * UMKC Robotics 2014
 *
 * Find out if you're at the edge of the world or not. 
 * Used with sensors that point at an angle, downwards. Don't want to fall off the edge of the world.. no walls around 
 *
 * Future: Instead of flat, hardcoded distance, perhaps a change in distance? The only thing is, would have to test with the tilting of going over the wavethingies
 *
 * implementing it woot woot -> to account for the sumo wrestling thingamajiggers
 *
 **/
#include <Distance2D120X.h>
#include <DistSmoother.h>

#ifndef MAGELLAN_H 
#define MAGELLAN_H


class Magellan {
private:

  //findEdge returns this enum. 
  //ok - we're good. no edges about to run over
  //front_danger - STOP! front sensor detects an edge
  //back_danger - STOP! back sensor detects an edge
  enum edge_danger_state { 
    ok, front_danger,  back_danger };
 
  edge_danger_state danger_status;

  int check;        //how many times in a row must the sensors NOT read a potential edge to go back to OK
  int currentSafeCount;  //used to keep track of how many times in a row sensor has read OK?
  int distanceFront,distanceBack;	//used for debugging

  int previousDistFront, previousDistBack;
  DistSmoother front;
  DistSmoother back;  

  
  //using just sa pure (farther than this) threshold
  int threshold_front;  
  int threshold_back;  

  int outerloopcount;

  // using a hard coded distance T
  const static int threshold_front_default= 20;	//the ideal sensor reading for safetey - ie how far the sensor reads usually.  hard coded to our specific thingy
  const static int threshold_back_default = 18;	//the ideal sensor reading for safetey - ie how far the sensor reads usually. 
//  const static int difference = 5;	//once a reading reads within +- difference of the baseline, that is an abrupt change and you should stop
 // const static double alpha = 0.80;	//alpha - how much you value current reading over previous readings

  bool safeness;
  //int runningCountFront;	//running count of how many times a difference is tracked 
  //int runningCountBack;	//running count of how many times a difference is tracked 
  //int runningCountExecution;	//count of how many times ive taken reading when determining safeness
  //const static int countThreshold = 2;	//how many times in a row a difference must be seen in order to say YES it's not safe 
  //const static int countExecution= 5;	//how many times in a row to take a reading from a sensor to determine if it is safe 
  
  //registering abruupt CHANGE< not just a a "this is greater than.
 /*
 bool isFrontSensorSafe()  {
	
	// i want to execute this multiple times	  
	runningCountExecution = 0;
	safeness = true;
	while(runningCountExecution++ < countExecution)	{
	//how to account for first time prevDistFront
			distanceFront = front.getAccurateDistCM();	

			//if this is the first time that thedistanceFront is triggered by difference, mark a flag and say it's still ok
			//if this is the SECOND time that the distance front is triggered by difference, IT"S NOT OK ABORT! 
			if(abs(distanceFront - baselineFront) >= difference)	{
				runningCountFront++;	
				if(runningCountFront > countThreshold)
						safeness = false;
			}
			else	{
				runningCountFront = 0;
			}
	}
	return safeness;

	//i dont think i need this prev thing
//	prevDistFront = distanceFront;
   //return front.isCloser(threshold);
  }
  
  bool isBackSensorSafe()  {
	runningCountExecution = 0;
	safeness = true;
	while(runningCountExecution++ < countExecution)	{
			distanceBack = back.getAccurateDistCM();

			//if this is the first time that thedistanceFront is triggered by difference, mark a flag and say it's still ok
			//if this is the SECOND time that the distance front is triggered by difference, IT"S NOT OK ABORT! 
			if(abs(distanceBack- baselineBack) >= difference)	{
				runningCountBack++;	
				if(runningCountBack > countThreshold)
						safeness = false;
			}
			else	{
				runningCountBack = 0;
			}
	}
	return safeness;


   //return back.isCloser(threshold);
  }
  */


  
public:

  bool isFrontSensorSafe()  {
   return front.isCloser(threshold_front);
  }
  
  bool isBackSensorSafe()  {
   return back.isCloser(threshold_back);
  }
 
 
  //initializer that only takes in pin #s, defaults to 15CM threshold
  void init(int pin1, int pin2) {
    init(threshold_front_default, threshold_back_default, pin1, pin2);
  }

  //takes in pin numbers of the front, and back sensor 
  //as well as the threshold in cm for detecting an edge
  void init(int thresh_front, int thresh_back, int pin1, int pin2) {
    threshold_front = thresh_front;
    threshold_back= thresh_back;
    front.init(pin1);
    back.init(pin2);
    check =3;
  }

  void printDebugDifference()  {
    distanceFront = front.getAccurateDistCM();
    distanceBack = back.getAccurateDistCM();
	int localDifference = abs(distanceFront - distanceBack);
    Serial.print("diff(cm) #:\t ");

	if(localDifference < 9)
			Serial.print("0");
    Serial.println(localDifference);
//	delay(50);
  }

  //find and print the distances
  void printDebug()  {
    distanceFront = front.getAccurateDistCM();
    distanceBack = back.getAccurateDistCM();

    Serial.print("dist(cm) #f|b :\t ");
    Serial.print(distanceFront);
    Serial.print("\t ");
    Serial.println(distanceBack);    
//	delay(50);
  }
  
  void printEdgeStatus()  {
    if (danger_status == ok)
      Serial.println("edge_status: ok");
    else if (danger_status == front_danger)
      Serial.println("edge_status: front_danger"); 
    else if (danger_status == back_danger)
      Serial.println("edge_status: back_danger");
    Serial.println();
  }
  
  bool isFrontSafe()	{
 	return !(danger_status == front_danger); 
  }

  bool isBackSafe()	{
  	return !(danger_status == back_danger);
  }
   
  //returns true if it's ok.
  //false if noot ok
  bool update()  {


	outerloopcount = 5;
	while(outerloopcount -- > 0)	{
			//note to self - i hope the front + back sensors DON"T read edge at the same time :( then we have no where to go, no where!!
			switch(danger_status)  {
			  //Want to be quick and jumpy. As soon as sensor reads upsafe, jump. Rather a false positive than a false negative.
			  case ok:
				if(!isFrontSensorSafe())
				  danger_status = front_danger;
				else if (!isBackSensorSafe())
				  danger_status = back_danger;
				break;
				
			  //Must read a safe reading for at least "check" counts in a row.
			  case front_danger:
				if(isFrontSensorSafe())  {
				  if(currentSafeCount == check)  {
					danger_status = ok;
					currentSafeCount = 0;
				  }
				  else
					currentSafeCount++; 
				}
				else   //front still reads edge!!
				  currentSafeCount = 0;
				break;
			  case back_danger:
				if(isBackSensorSafe())  {
				  if(currentSafeCount == check)  {
					danger_status = ok;
					currentSafeCount = 0;
				  }
				  else
					currentSafeCount++;
				}
				else  //back ain't safe
				  currentSafeCount = 0;
				break;
			   
			} 
	}
	if(danger_status == ok)
    	return true;
	else
		return false;
    
  
  }

};

#endif

