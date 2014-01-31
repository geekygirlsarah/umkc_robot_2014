
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
 **/
#include <Distance2D120X.h>


#ifndef MAGELLAN_H 
#define MAGELLAN_H


class Magellan {
  
public:


  //detectEdge returns this enum. 
  //ok - we're good. no edges about to run over
  //front_danger - STOP! front sensor detects an edge
  //back_danger - STOP! back sensor detects an edge
  enum edge_danger_state { 
    ok, front_danger,  back_danger };
 
 
  //initializer that only takes in pin #s, defaults to 15CM threshold
  void init(int pin1, int pin2) {
    init(15, pin1, pin2);
  }

  //takes in pin numbers of the front, and back sensor 
  //as well as the threshold in cm for detecting an edge
  void init(int thresh, int pin1, int pin2) {
    front.begin(pin1);
    back.begin(pin2);
    check = 3;              //start out needing 3 negative readings to go back to OK state
    currentSafeCount = 0;
    danger_status = ok;    //we start out assuming not on the edge
    threshold = thresh;
  }

  //find and print the distances
  void printDebug()  {
    distance1 = front.getDistanceCentimeter();
    distance2 = back.getDistanceCentimeter();

    Serial.print("dist(cm) #front: ");
    Serial.println(distance1);
    Serial.print("dist(cm) #back: ");
    Serial.println(distance2);    
    delay(500); //make it readable
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
  
  edge_danger_state detectEdges()  {

 
    //note to self - i hope the front + back sensors DON"T read edge at the same time :( then we have no where to go, no where!!
    switch(danger_status)  {
      //Want to be quick and jumpy. As soon as sensor reads upsafe, jump. Rather a false positive than a false negative.
      case ok:
        if(!isFrontSafe())
          danger_status = front_danger;
        else if (!isBackSafe())
          danger_status = back_danger;
        break;
        
      //Must read a safe reading for at least "check" counts in a row.
      case front_danger:
        if(isFrontSafe())  {
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
        if(isBackSafe())  {
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

    return danger_status;
    
  
  }
private:
  edge_danger_state danger_status;

  int check;        //how many times in a row must the sensors NOT read a potential edge to go back to OK
  int currentSafeCount;  //used to keep track of how many times in a row sensor has read OK?
  int distance1,distance2;	//used for debugging

  Distance2D120X front;
  Distance2D120X back;  

  int threshold;  //threshold for ping sensor detecting an EDGE (cm)


 
  bool isFrontSafe()  {
   return front.isCloser(threshold);
  }
  
  bool isBackSafe()  {
   return back.isCloser(threshold);
  }
 


};

#endif


