


/**
 * fronteyes 
 * feb 2014
 * Author: vicky wu 
 *
 * ONe IR sensor. I donw' want to crash into a wave/oil right.  
 * written for the 2014 ieee region 5 comp
 *
 **/
#include <Distance2D120X.h>


#ifndef FRONTEYES_H 
#define FRONTEYES_H 



class FrontEyes {
private:
	//Is sthere an obstacle in front? or NOT?
  enum internal  { 
    NO_OBST, MAYBE_OBST,  YES_OBST};
  internal obst_status;

  int check;			//how many times in a row must the sensor detect a thingy
  int outerloopcount;	//how many times the update() method will run thru the state loop
  int distance1;  //used for debugging

  Distance2D120X Dist1;

  int threshold;  //threshold for ping sensor detecting an obstacle (cm), dfaults to 10cm

  bool checkObst() {
  	return Dist1.isCloser(threshold); 
  }
 
public:

  //Which pin is connected to the ir sensor in question 
  //threshold defaults to 10CM
  void init(int pin1)  {
    Dist1.begin(pin1);
	threshold = 10;
    obst_status = YES_OBST;    //we start out assuming a thing in the front 
  }


  //Which pin is connected to the ir sensor in question 
  //also what is the threshold you want for detecting an obstacle
  void init(int pin1, int thresh)  {
    Dist1.begin(pin1);
	threshold = thresh;
    obst_status = YES_OBST;    //we start out assuming a thing in the front 
  }

  //find and print the distances
  void printDebug()  {
    distance1 = Dist1.getDistanceCentimeter();
    //difference = distance1 - distance2;

    Serial.print("dist(cm)#1: ");
    Serial.println(distance1);
  }

  void printObstStatus()  {
   if(obst_status == NO_OBST)  {
      Serial.println("obst_status: Nope");
    }
    else if (obst_status== MAYBE_OBST)  {
      Serial.println("obst_status: maybe");
	}
    else if (obst_status== YES_OBST)  {
      Serial.println("obst_status: Yes!");
    }
  }

  //State machine - to make sure there really is a obst, and not just a sensor misreading
  void update()  {
    
	//wrapping this entire thing in a while... hopefully speed this up		  
	outerloopcount= 5;
	while(outerloopcount-- > 0)	{
			switch (obst_status)  {
			case NO_OBST:
			  if(checkObst())
				obst_status = MAYBE_OBST;
			  break;
			case MAYBE_OBST:
			  check = 5;
			  //assume there is no obstacle- want it to be trigger happy and have 5 in a row with NO obstacle to back down
			  obst_status = NO_OBST;
			  while(check-- >0)	{
			  	if(checkObst())
					obst_status = YES_OBST;
			  }
			  break;
			case YES_OBST:
			  //want it to have X in a row no obstacles to back down to no_obst
			  check = 5;
			  while(check-- >0)	{
				if(!checkObst())
					obst_status = NO_OBST;
			  }
			  break;

			}
	}
  }

  bool obstaclePresent()	{
 	return obst_status == YES_OBST;	
  }

  bool obstacleNotPresent()	{
  	return (obst_status == NO_OBST|| obst_status == MAYBE_OBST);
  }

};

#endif

