#include <DistanceSensor.h>
#include <Distance2D120X.h>
#include <AnalogDistanceSensor.h>
#include <stdlib.h>

/**
 * Wall following module by Chris Denniston.
 * The documentation should work well with doxygen.
 * Make sure to turn on documentation of private members.
 */

#ifndef WALLFOLLOWER_H
#define WALLFOLLOWER_H

class WallFollower{
 private:
  int straightConfidence; /**< The difference between the first and last sensors. 0 is the highest value, meaning that it is definitely straight. */
  int turnOffset; /**< The difference in distance between the first and last sensor. pin1 being greater in distance results in a positive value.*/
  int timeBeforeConcern; /**< The number of function calls to checkWall() before the robot will move to low concern. */
  int timeLowConfidence; /**< The number of function calls spent in low confidence of going straight */
  Distance2D120X Dist1;
  Distance2D120X Dist2;  
  Distance2D120X Dist3;
  int distance1;
  int distance2;
  int distance3;
  
 public:
  enum straightConfidenceLevel {perfect = 0, high = 1, medium = 2, low=3}; /**< The confidence leel in a more general form */
  straightConfidenceLevel confidenceEnum; /**< The confidence level in a more general form*/
  /**
   * Initializes the wall follower object. The three pins shouldbe the IR sensors.
   * @param pin1 The first pin to connect to. 
   * @param pin2 The second pin to connect to. Should be the middle pin.
   * @param pin3 The third pin to connect to.
   */
  void init(int pin1, int pin2, int pin3){
    Dist1.begin(pin1);
    Dist2.begin(pin2);
    Dist3.begin(pin3);
    distance1 = 0;
    distance2 = 0;
    distance3 = 0;
    straightConfidence = 0;
    timeBeforeConcern = 5; //Should proably be changed.
    timeLowConfidence = 0;
    turnOffset = 0;
  }
   /**
   * Initializes the wall follower object. The three pins shouldbe the IR sensors.
   * @param timeBeforeWallConcern The amount of calls to the checkWall() function before the robot will move to the low concern.
   * @param pin1 The first pin to connect to. 
   * @param pin2 The second pin to connect to. Should be the middle pin.
   * @param pin3 The third pin to connect to.
   */
  void init(int timeBeforeWallConcern, int pin1, int pin2, int pin3){
    init(pin1,pin2,pin3);
    timeBeforeConcern = timeBeforeWallConcern;
  }
  /**
   * Prints debug information for the module. 
   * Copied from gapfinder mostly.
   */
  void printDebug(){
    distance1 =  Dist1.getDistanceCentimeter();
    distance2 = Dist2.getDistanceCentimeter();
    distance3 = Dist3.getDistanceCentimeter();
    //difference = distance1 - distance2;

    Serial.print("dist(cm)#1: ");
    Serial.println(distance1);
    Serial.print("dist(cm)#2: ");
    Serial.println(distance2);    
    Serial.print("dist(cm)#3: ");
    Serial.println(distance3); 
    Serial.print("Straight confidence: ");
    Serial.println(straightConfidence);
    Serial.print("Time low confidence: ");
    Serial.println(timeLowConfidence);
    delay(500); //make it readable
  }
  /**
   *Detremines straihgt confidence. Nothing should start caring until in low confidence. Robot shouldn't be concerned until in low confidence. \n
   * Algorithm: Get distance from 3 IR pins. -> distance variables \n
   * take the difference between the first and last distances -> turn offset \n
   * absolute value of the difference between the first and last values -> straight confidence \n
   * determine level in enum -> store in enum \n
   * if not in low confidence:
   * \t set low conf counter to 0 \n
   * if in low confidence: \n
   * \t increment low conf counter \n 
   * \t if low conf counter is greater than time before low confidence:
   * \t return low confidence\n 
   * else: \n
   * \t return confidence level 
   * @return The current straightConfidenceLevel
   */
  straightConfidenceLevel checkWall(){
    distance1 = Dist1.getDistanceCentimeter();
    distance2 = Dist2.getDistanceCentimeter();
    distance3 = Dist3.getDistanceCentimeter();
    turnOffset = distance1 - distance3;
    straightConfidence = abs(turnOffset);
    
    if(straightConfidence > high){
      timeLowConfidence=0;
      return perfect; 
    }
    else if(straightConfidence > medium){  
      timeLowConfidence=0;
      return high;
    }
    else if(straightConfidence > low){ 
      timeLowConfidence=0;
      return medium;
    }
    else{
      timeLowConfidence++;
      if(timeLowConfidence > timeBeforeConcern){
	return low;
      }
      else{
	return medium;
      }
      
    }
  }   
};



#endif
