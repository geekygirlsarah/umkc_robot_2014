#include <motor_cmd.h>

#include <gapfinder.h>
#include <irsensor_tester.h>
#include <magellan_edgesensors.h>

#include <QuadEncoder.h>

#include <AnalogDistanceSensor.h>
#include <Distance2D120X.h>
#include <DistanceSensor.h>



/*
thursday 2/6/14 after snooooow

At speed 20
Without any sensing. just encoders at speed20
------------------
Time taken: 10012
FL: 34119

Time taken: 10011
FL: 33947

Time taken: 10011
FL: 34014

Time taken: 10012
FL: 34103

-- ticks > 10K
Time taken: 3002
FL: 10104



gap checker + encoders at speed20
---------------
Time taken: 10011
FL: 34062

Time taken: 10012
FL: 34113

gap checker + magellan + encoder at speed20
--------------
Time taken: 10012
FL: 34120

---- -> not on time, but on ticks > 10K
Time taken: 2985
FL: 10116



trying it out a little faster, say 30
=======================
just encoders
Time taken: 10012
FL: 52477

with gap checker + encoder
Time taken: 10012
FL: 52553

with gap checker + mag + encoder
Time taken: 10018
FL: 53003

==1.813 inch off


now at full speed.. full speed is a BAD idea
========================
only encoders:
Time taken: 10018
FL: 82417

gap checker + encoders 
Time taken: 10016
FL: 79221


--- not on time, but on ticks 10k

with nothing
Time taken: 1379
FL: 10215


with everything
Time taken: 1342
FL: 10270


*/




//want to overload the mega. 
//have all sensors running,
//and encoders... see if it handle the load
GapFinder gapf;
motor_cmd sabertooth;
Magellan mag;
QuadEncoder encoders;

int sensoryOverload_state;

unsigned long start;
unsigned long elapsed;

void setup()
{
  start = millis();
  Serial.begin(9600);
  Serial.println("begin");
  sabertooth.begin(2); 
  encoders.init();
  gapf.init(A0,A1,A2);
  mag.init(A3,A4);
  
  state = 0;
}

void loop()
{
   
   switch (sensoryOverload_state)  {
     case(0):
       //start  
       Serial.println("state 0");
       //sabertooth.forward(20);
       sabertooth.reverse_full();
       sensoryOverload_state = 1;
       break;
     case (1):
       Serial.println("state 1");
       //just keep swimming
        
        Serial.print("FL: ");
        Serial.println(positionFL);
       // Serial.print(" BL: ");
        //Serial.println(positionBL);
        //Serial.print(" FR: ");
         //Serial.println(positionFR);
        //Serial.print(" BR: ");
        //Serial.println(positionBR);
       elapsed = millis() - start;
       //if(elapsed > 10000)  {
       if(positionFL > 10000)  {   
         Serial.print("Time taken: ");
          Serial.println(elapsed);
          Serial.print("FL: ");
          Serial.println(positionFL);
          sabertooth.all_stop();
          sensoryOverload_state = 2;
        }
       break;
     case (2):
       break;
     
   
   //Magellan::edge_danger_state edge_status;
   //mag.printDebug();
   //mag.printEdgeStatus();
   //edge_status = mag.detectEdges();
   
   
   //GapFinder::ternary gap_status;
   //gapf.printDebug();
   //gapf.printGapStatus();
   //gap_status = gapf.update(); 
   
   
  
  
 
  
   }

  
}


