// Arduino library for distance sensors
// Copyright 2011-2013 Jeroen Doggen (jeroendoggen@gmail.com)

//problem - when it returns 8 inches... it might be 8 inches or just a wall of nothingness

#include <Distance2D120X.h>

/// Constructor
Distance2D120X::Distance2D120X()
{
}

//Spec sheet says from 4CM to 30CM accuracy -> experimentally verify?
/// getDistanceCentimeter(): Returns the distance in centimeters
int Distance2D120X::getDistanceCentimeter()
{
  int adcValue=getDistanceRaw();
  if (adcValue > 600)                             // lower boundary: 4 cm (3 cm means under the boundary)
  {
    return (4);
  }

  if (adcValue < 90 )                             //upper boundary: with the formula, about 40 cm cutoff (updated 1/25/14)
  {
    return (35);
  }

  else
  {
	//SPREADSHEET + FORMULA IS IN INCHES D:
	float inches_to_cm = 2.54;
 	return ( inches_to_cm *5059.11401 * pow((float)adcValue,  -1.29861 )); //the in house one IN INCHES iNCHES INCHES
  }
}
