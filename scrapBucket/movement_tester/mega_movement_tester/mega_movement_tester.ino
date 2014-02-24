#include <QuadEncoder.h>

/* mega movement tester
 * written by: victoria wu
 * date: 1/17/14 
 *
 * what: make the robot keep going until the gap is found. then stop. (going to put it all on the mega, no bridge just yet
 * FIRST - just getting the robot to move. NO comms between mini and mega. just everything on the mega, ignoring encoders right now
 * PURPOSE: 
 
 * .... pro tip: if you get the error "serial2 is not declared in this scope" check that the board you're programming for IS THE MEGA
 * -> first starting. .the motors just go crazy  a bit 
 */

#include "navigation.h"

#include <AnalogDistanceSensor.h>
#include <Distance2D120X.h>
#include <DistanceSensor.h>

#include <gapfinder.h>
#include <irsensor_tester.h>
#include <magellan_edgesensors.h>
#include <fronteyes.h>

#include <motor_cmd.h>

#include <movement.h>



Navigation nav;

enum state_top { start, moving, gapfound, crossingwave, theend };
state_top current_status;

void setup() {
	Serial.begin(9600);
	nav.init();
        current_status = start;
}

void loop() {
            
        switch (current_status) {
          case start:
            //let's keep going
            //console.println("start \t go forward!");
            nav.takeOff();
            current_status = moving;
            break;
          case moving:
            if(nav.lookingForGap())
              current_status = gapfound;
            break;
          case gapfound:
            //console.println("gapfound \t ");
			//now to turn 90 degrees
            Serial.println("starting 90 deg turn");
            nav.turnTowardsGap();
            current_status = crossingwave;
            break;
          case crossingwave:
             if(nav.crossGap())
               current_status = theend;
             
            break;
          case theend:
            nav.sleep();
            
            break;
            
        
        }
       
        //delay(50);  //make it readable
        //gapfind.printDebug();
      
      
      /*
  Serial.print("FL: ");
  Serial.println(positionFL);
  Serial.print(" BL: ");
  Serial.println(positionBL);
  Serial.print(" FR: ");
  Serial.println(positionFR);
  Serial.print(" BR: ");
  Serial.println(positionBR);  
     */  
     
}

