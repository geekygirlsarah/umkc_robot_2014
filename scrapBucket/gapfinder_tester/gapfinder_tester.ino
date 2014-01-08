//Test program to find a gap among the waves.
//Author: Vicky Wu + Andrew Cunningham

//Currently
//organizing this out into a classs thingy (done)

//TODO
//add in the ir sensor //thingy formula thingy
//add in using 3 ir sensors, and the slow down thingy


#include <DistanceGP2Y0A21YK.h>
#include "gapfinder.h"

GapFinder gapf;

void setup()
{
  Serial.begin(9600);  
  gapf.init(A0,A1,A2);
}

void loop()
{
   gapf.findGap(); 
}

