
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
#include <parallelpark.h>

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
                ParallelPark par; 
    public:
                void init()  {
                  sabertooth.begin(2);        //motor controller on serial2
                  encoders.init();
                  mov.init(&sabertooth);
                  Serial.println("ready");
                  gapfind.init(A0,A1,A2);
                  eyes.init(A7,15);
                  par.init(A2,A1,A0);
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
                  gapfind.printDebug();
                  gapfind.printGapStatus();
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
                   eyes.update();
                   eyes.printDebug();
                   if(eyes.obstaclePresent())  {
                     sabertooth.all_stop();
                     Serial.println("dont crash");
                     return true;
                   }
                   return false;
                }
                
                void parallelpark()  {
                 /*
                  while(true)    { 
                   par.printDebug(); 
                   par.update();
                   //par.printStatus();
                 }
                 */
                  
                  //WHOAAAAH
                  //change of paradigm.
                  //instead of going until you stop, you should stop a bit and then go if it's bad. so default to stopping
                  
                  mov.turn(0x60,0x20);
                  while(!par.isParallel())  {
                    mov.turn(0x60,0x20);
                    //keep going
                    par.printDebug();
                    par.printStatus();
                    par.update();
                  }
                  
                  /*
                  sabertooth.all_stop();
                  
                  while(par.needToTurnCW())  {
                     mov.turn(0x20,0x60);
                     par.printDebug();
                     par.printStatus();
                     par.update();
                     delay(100);
                     sabertooth.all_stop(); 
                  }
                  */
                  
                  //sabertooth.all_stop();
                  
                  //jump out when it is parallel hopefully
                }
                void sleep()  {
                  sabertooth.all_stop();
                  delay(500);
                }
               
                
		
			
};




#endif
