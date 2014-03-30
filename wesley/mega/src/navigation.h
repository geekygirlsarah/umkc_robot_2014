
/*
 * navigation
 * top level functions for navigating
 * look at the state diagram
 * 2014 umkc robotics 
 */
//woohoo ros time !!!



#define HARDCODE_TICKS_LASTGAP 5500
#define HARDCODE_TICKS_GAP_ADJUST 1000
#define HARDCODE_TICKS_TRAVEL_TO_TOOLS 20000
#define GAPFINDER_THRESHOLD 25


#include <ros.h>
#include <mega_caretaker/MegaPacket.h>


#include <AnalogDistanceSensor.h>
#include <Distance2D120X.h>
#include <DistanceSensor.h>

#include <gapfinder.h>
#include <irsensor_tester.h>
#include <magellan_edgesensors.h>
#include <fronteyes.h>
//#include <parallelpark.h>
#include <parallelpark_simple.h>
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
                Magellan mag;
                
          
                
                const static int32_t ticksFor90 = 3450;
                bool goingForward;
    public:
                void init()  {
                  sabertooth.begin(2);        //motor controller on serial2
                  encoders.init();
                  mov.init(&sabertooth);
                  Serial.println("ready");
                  gapfind.init(A0,A1,A2,GAPFINDER_THRESHOLD);
                  eyes.init(A4,8);
                  mag.init(10,10,A6,A7);
                  par.init(A2,A1,A0, &sabertooth);
                
                  
                  goingForward = true;  //... this is from the point of view of robot. it will always start going forward.
                                        //it'xs just that the motor library has it as "reverse()" as our going forward XD
                }
                
                
                //used to test basic power reqs
                void simpleMovementTest()  {
                  Serial.println("simple movement test");
                  //delay(5000);  //wait 5 seconds
                  sabertooth.reverse(20);
                  delay(500);  //go for 2 secs
                  sabertooth.all_stop();
                  delay(2000);
                  sabertooth.forward(20);
                  delay(500);
                  sabertooth.all_stop();
                }
                
                //Stuff that needs to be updated goes in here.
                //NO NO NO NO NONONONONONONO
                //update-y things need to GO IN THE THING ITSELF BLAHHHHHHHHH or be called in the nav functions themselves nonono
                /*
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
                */
              
                //toggle in forward or backwards when magellans see seomthing
                
                void traveling()  {
                   mag.printDebug();
                   //mag.printDebugDifference();
                   mag.printEdgeStatus();
                   mag.update();
                   //if it senses ANYTHING reverse direction quick!
                  /*
                  //twitches and just twitches forward and backward
                   if( !mag.isFrontSafe() || !mag.isBackSafe())  {
                     Serial.println("ACK SOMETHING!");
                     sabertooth.all_stop();
                     delay(500);
                     goingForward = !goingForward;
                   } 
                   */
                   
                   if(!mag.isFrontSafe() && goingForward)  {
                     Serial.println("ACK SOMETHING!");
                     sabertooth.all_stop();
                     delay(500);
                     goingForward = false;
                     
                  
                   }
                   if(!mag.isBackSafe() && !goingForward)  {
                     Serial.println("ACK SOMETHING!");
                     sabertooth.all_stop();
                     delay(500);
                     goingForward = true;
                   }
                   if(goingForward)  {
                     Serial.println("go forward");
                     sabertooth.reverse(20);
                   }
                   else  {
                     Serial.println("go reverse");
                     sabertooth.forward(20);
                   }
                }
                
                //returns if gap is found.
                bool lookingForGap()  {
                  //sabertooth.reverse(20);
                  //sabertooth.forward(20);
                  //console.println("moving \t checking gap");
                  //check if a gap has been found
                  
                  
                  //gapfind.printDebug();
                  //gapfind.printGapStatus();
                  
                  gapfind.update();
                  
                  
                  //gap found? move forward a set amount to center self
                  if(gapfind.gapPresent())  {
                    Serial.println("moving \t GAP FOUND!!");
                    gapfind.reset();
                     //unfortunately right now i have no ticks.. so it stays here forever
                    
                    
                    sabertooth.all_stop();
                    return true;
                  }
                  return false;
                }
                
                // check if really at a gap
                // should be ran once
                bool doubleCheckGap()   {
                    // By this point gap should have been detected and robot stopped. We 
                    // should double check this and align up properly
                    
                    // update position first
                    //gapfind.update();
                    if(gapfind.gapPresent())
                        return true;
                    
                    // Might or might not be at gap, so return false
                    return false;                                        
                
                
                
                }
                
                
                //use ticks to adjust.. as soon as we find gap, needs to go forward some ticks
                bool adjustToGap()  {
                  //500 taped, trying 900 untaped
                    goForwardForever();
                    int32_t start_ticks = positionFR;
                    while(true)  {
                      if(abs(positionFR - start_ticks) > HARDCODE_TICKS_GAP_ADJUST)
                         break; 
                     }
                     stopNow();
                     
                }
                
                //hardcode the thing T.T
                bool crossLastGap()  {
                  goForwardForever();
                   int32_t start_ticks = positionFR;
                    while(true)  {
                      if(abs(positionFR - start_ticks) > HARDCODE_TICKS_LASTGAP)
                         break; 
                     }
                     return true;
                     stopNow();
                }
                
                
                bool crossGap()  {
                  
                  goForwardForever();
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
                
                  
                  //WHOAAAAH
                  //change of paradigm.
                  //instead of going until you stop, you should stop a bit and then go if it's bad. so default to stopping
                  // lets be lazy and hack-y. Gonna hard code a turn for 45degrees, then make the par park thing work. 
                 //mov.turn90degreesCW(0x60,0x20,ticksFor90/2);
                 //sabertooth.all_stop();
                 
                  
                  //mov.turn(0x60,0x20);
                  
// T.T tears. this doesn't work... not now... nooooo                
                  par.parallelPark();
                  
                  //while(!par.isParallel())  {
                    //mov.turn(0x60,0x20);
                    //keep going
                    //par.printDebug();
                    //par.printStatus();
                    //par.update();
                  //}
                  
                  /*
                  while(true)  {
                    par.printDebug();
                  }
                  */
                  
                  par.reset();
                  
                  
                  
                  sabertooth.all_stop();
               
       
                }
                
                
                
                
                void turnClockwiseForever()  {
                  sabertooth.turnCW();
                }
                void turnCounterClockwiseForever()  {
                  sabertooth.turnCCW();
                }
                void stopNow()  {
                  sabertooth.all_stop();
                }
                void goForwardForever()  {
                  Serial.println("takeoff/goForwardForever");

                  //sabertooth.forward();
                  sabertooth.reverse();
                  goingForward = true;
                }
                
                void goBackwardForever()  {
                  //sabertooth.reverse();
                  sabertooth.forward();
                  goingForward = false;
                }
                
                void travelToTools()  {
                  int32_t start_ticks = positionFR;
                    while(true)  {
                      if(abs(positionFR - start_ticks) > HARDCODE_TICKS_TRAVEL_TO_TOOLS)
                         break; 
                     }
                   sabertooth.all_stop();
                }
                
                void stop_sleep(int duration)  {
                  sabertooth.all_stop();
                  delay(duration);
                }
               
               //just go backwards and find the edge. 
               //return true once you foind the edge (this is make it a lot easier to separate so i can find gap separetely find finding edge
               //TODO optimize :D
                 bool atEdge()    {
                   mag.update();
                   return !mag.isSafe();
                   
                 }

                /*  basically a duplicate of goForwardForever, so removing 		  
                void takeOff()  {
                  goForwardForever();
                  
                }
                */
                
              
                
			
};




#endif

