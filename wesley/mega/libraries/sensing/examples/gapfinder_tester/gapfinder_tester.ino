

//Test program to find a gap among the waves.

//Currently
//organizing this out into a classs thingy (done)

//TODO
//add in the ir sensor //thingy formula thingy
//add in using 3 ir sensors, and the slow down thingy
#include <Distance2D120X.h> 
#include <gapfinder.h>
#include <irsensor_tester.h>
#include <magellan_edgesensors.h>
//#include <parallelpark.h>
//#include <parallelpark_simple.h>
#include <DistSmoother.h>


//WITH THE thigy -> IN INCHES!!! (problem - when you don't have anything... it dfaults to 8 inches
///ParallelPark par;
GapFinder gapf;
Magellan mag;
IRSensorTester tester;

//DistSmoother dist1;

void setup()
{
  Serial.begin(9600); 
  //tester.init(1,A0,A1,A2);
   //par.init(A2,A1,A0); 
  gapf.init(A0,A1,A2,15);
  //mag.init(A6,A7);
  //dist1.init(A0);

}

void loop()
{
  
  //tester.printDebugRaw();
  //tester.printDebugCM();
  
      
   //mag.printDebug();
   //mag.printEdgeStatus();
   //mag.update();
   
   
   GapFinder::ternary gap_status;
   gapf.printDebug();
   gapf.printGapStatus();
   gapf.update(); 

  
  /*
  par.printDebug();
  par.printStatus();
  par.update();
*/
/*
  Serial.print(dist1.getRawCM());
  Serial.print("\t");
  Serial.println(dist1.getFilteredDistanceCM());
*/

}

