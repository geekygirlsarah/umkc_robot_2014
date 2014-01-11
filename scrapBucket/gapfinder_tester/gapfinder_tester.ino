//Test program to find a gap among the waves.
//Author: Vicky Wu + Andrew Cunningham

//Currently
//organizing this out into a classs thingy (done)

//TODO
//add in the ir sensor //thingy formula thingy
//add in using 3 ir sensors, and the slow down thingy
#include <Distance2D120X.h>
#include "gapfinder.h"


//WITH THE thigy -> IN INCHES!!! (problem - when you don't have anything... it dfaults to 8 inches
GapFinder gapf;

void setup()
{
  Serial.begin(9600);  
  gapf.init(A0,A1,A2);
}

void loop()
{

   GapFinder::ternary gap_status;
   gapf.printDebug();
   gap_status = gapf.findGap(); 

}

