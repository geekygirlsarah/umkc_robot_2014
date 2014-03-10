


//Currently
//organizing this out into a classs thingy (done)

//TODO
//add in the ir sensor //thingy formula thingy
//add in using 3 ir sensors, and the slow down thingy
#include <Distance2D120X.h>
#include <gapfinder.h>
#include <irsensor_tester.h>
#include <magellan_edgesensors.h>

//WITH THE thigy -> IN INCHES!!! (problem - when you don't have anything... it dfaults to 8 inches
GapFinder gapf;
Magellan mag;
IRSensorTester tester;

void setup()
{
  Serial.begin(9600); 
 // tester.init(1,A0,A1,A2); 
  //gapf.init(A0,A1,A2);
  mag.init(25,A0,A2);  //testing with a 25 cm threshold
}

void loop()
{
 // tester.printDebugRaw();
 // tester.printDebugCM();
  

   mag.printDebug();
   //mag.printEdgeStatus();
   mag.update();
   if(!mag.isFrontSafe())  Serial.println("EDGE FRONT");
   if(!mag.isBackSafe())  Serial.println("EDGE BACK");
   /*
   GapFinder::ternary gap_status;
   gapf.printDebug();
   gapf.printGapStatus();
   gap_status = gapf.findGap(); 
*/
}

