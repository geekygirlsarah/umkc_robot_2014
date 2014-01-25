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

#include <AnalogDistanceSensor.h>
#include <Distance2D120X.h>
#include <DistanceSensor.h>

#include <gapfinder.h>
#include <irsensor_tester.h>
#include <magellan_edgesensors.h>

#include <motor_cmd.h>
#include <wesley_mega_bridge.h> 

#include <Encoder.h>
#define console Serial


motor_cmd sabertooth;
GapFinder gapfind;


enum state { start, moving, stopped };
state current_status;

void setup() {
	console.begin(9600);
	sabertooth.begin(2);        //motor controller on serial2
	console.println("ready");
        gapfind.init(A5,A6,A7);
        current_status = start;
}

void loop() {
        
        //keep going forward until you find gap
        if(!gapfind.gapPresent())
          gapfind.update();  
        //gapfind.printGapStatus();
        //gapfind.printDebug();
        
                    
        
        switch (current_status) {
          case start:
            //let's keep going
            //console.println("start \t go forward!");
            sabertooth.reverse(20);
            current_status = moving;
            break;
          case moving:
            sabertooth.reverse(20);
            //console.println("moving \t checking gap");
            //check if a gap has been found
            if(gapfind.gapPresent())  {
              console.println("moving \t GAP FOUND!!");
              current_status = stopped;
            }
            break;
          case stopped:
            //console.println("stopped \t ");
            //stay here forever
            sabertooth.all_stop();
            break;
            
        
        }
        
        
        //delay(50);  //make it readable
        //gapfind.printDebug();
        
       
     
}
