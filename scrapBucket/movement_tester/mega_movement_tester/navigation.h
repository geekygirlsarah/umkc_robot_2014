
/*
 * navigation
 * top level functions for navigating
 * look at the state diagram
 * 2014 umkc robotics 
 */


#include <AnalogDistanceSensor.h>
#include <Distance2D120X.h>
#include <DistanceSensor.h>

#include <gapfinder.h>
#include <irsensor_tester.h>
#include <magellan_edgesensors.h>
#include <fronteyes.h>

#include <motor_cmd.h>

#include <movement.h>

#ifndef NAVIGATION_H
#define NAVIGATION_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#include <pins_arduino.h>
#endif

class Navigation {
    private:
                motor_cmd sabertooth;
                FrontEyes eyes;
                movement mov;
                GapFinder gapfind;
                QuadEncoder encoders; 
    public:
                void init()  {
                  sabertooth.begin(2);        //motor controller on serial2
                  encoders.init();
                  mov.init(&sabertooth);
                  Serial.println("ready");
                  gapfind.init(A5,A6,A7);
                  eyes.init(A0,11);
                }
                
                
                //Stuff that needs to be updated goes in here.
                void update()  {
                //keep going forward until you find gap
                    if(!gapfind.gapPresent())
                      gapfind.update();  
                    //gapfind.printGapStatus();
                    //gapfind.printDebug();
                    //sabertooth.forward();
                                
                    eyes.update();
                    //if(eyes.obstaclePresent())
                    //   sabertooth.all_stop();
                
                } 
                
                void takeOff()  {
                  sabertooth.reverse(20);
                }
                
                //returns if gap is found.
                bool lookingForGap()  {
                  sabertooth.reverse(20);
                  //sabertooth.forward(20);
                  //console.println("moving \t checking gap");
                  //check if a gap has been found
                  
                  //gap found? move forward a set amount to center self
                  if(gapfind.gapPresent())  {
                    //Serial.println("moving \t GAP FOUND!!");
      
                    
                    int32_t start_ticks = positionFL;
                    while(true)  {
                      if(abs(positionFL - start_ticks) > 500)
                         break; 
                     }
                    sabertooth.all_stop();
                    return true;
                  }
                  return false;
                }
                
                void turnTowardsGap()  {
                  mov.turn90degreesCW(0x20,0x60);
        
                  sabertooth.all_stop();
                  delay(500);
                }
                
                bool crossGap()  {
                  sabertooth.reverse(20);
                   if(eyes.obstaclePresent())  {
                     sabertooth.all_stop();
                     Serial.println("dont crash");
                     return true;
                   }
                   return false;
                }
                
                void sleep()  {
                  sabertooth.all_stop();
                }
               
                
		
			
};




#endif
