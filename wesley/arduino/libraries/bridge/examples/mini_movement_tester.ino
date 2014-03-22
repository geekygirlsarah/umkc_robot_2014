





/* WESLEY_SKETCH_MINI
 * written by: eric gonzalez, vicky wu 
 * date: 1/8/14, 1/17/14 
 *
 * PURPOSE: This sketch contains the response side of IR_CLASS_MEGA and
 *          resides on the mini.
 *
 *          REMEMBER TO MAKE SURE THAT THE SERIAL LINE-SPEED USED HERE
 *          MATCHES WHAT IS USED IN IR_CLASS_MEGA!
 *
 *  		
 */
//#include "wesley_mini_bridge.h"



#include <wesley_mini_bridge.h>
//sensing
#include <gapfinder.h>
#include <irsensor_tester.h>
#include <magellan_edgesensors.h>

//ir sensors
#include <AnalogDistanceSensor.h>
#include <Distance2D120X.h>
#include <DistanceSensor.h>

GapFinder gap_state;
GapFinder::ternary tern;
// from mini-->mega

Distance2D120X Dist1;
int distance, raw;
mini_bridge mega_br;

byte no_of_sensors;

void setup() {
	no_of_sensors = 3;
        Dist1.begin(A7);
	gap_state.init(A5,A6,A7);
	mega_br.begin(no_of_sensors);
}

void loop() {
	
        distance = Dist1.getDistanceCentimeter();
        raw = Dist1.getDistanceRaw();
        //Serial.println(distance);
        //Serial.println(raw);
        mega_br.speed(mega_br.HALF_SPEED);
        
        
        /*
        if(distance < 10)
          mega_br.speed(mega_br.HALF_SPEED);
        else
          mega_br.speed(mega_br.FULL_STOP);
        */  
    
        //tern = gap_state.findGap();
        //gap_state.printDebug();
        //gap_state.printGapStatus(); 
    
        // commented off for testing;
	/*-
        if (tern == gap_state.MAYBE_GAP) {
		mega_br.speed(mega_br.HALF_SPEED);
	} 
        else if (tern == gap_state.YES_GAP) {
		mega_br.speed(mega_br.FULL_STOP);
	}
        else if (tern == gap_state.NO_GAP)  {
                mega_br.speed(mega_br.HALF_SPEED);
        }
        
        //mega_br.stupidSpeed(mega_br.HALF_SPEED);
        
        //delay(5);
        //if the mega is polling for a thing of distances 
       /*
        else  if (mega_br.cmd_waiting()) {  
		byte cmd = mega_br.cmd();
		switch(cmd) {
			case '$': {
				// package and send off an array of cm distances
				gap_state.report(mega_br.data_packet);
				mega_br.tunnel();
				break;
			}
			default: {
				break;
			}
		}
	}
*/	

	//delay(random(500, 1400));
	//mega_br.speed(random(0,30));
//	Serial.print("mini: ");
//	Serial.print(random(0, 50), DEC);
//	Serial.println();
}
