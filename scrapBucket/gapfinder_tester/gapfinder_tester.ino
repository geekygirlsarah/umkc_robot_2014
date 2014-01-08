//Test program to find a gap among the waves.
//Author: Vicky Wu + Andrew Cunningham

//TODO
//add in the ir sensor thingy formula thingy



#include <DistanceGP2Y0A21YK.h>




DistanceGP2Y0A21YK Dist1;
DistanceGP2Y0A21YK Dist2;
int distance1;
int distance2;

int check = 3;        //how many times in a row must the two sensors read a hole

int threshold = 20;   //threshold for ping sensor detecting a hole (cm)

enum ternary  { no_gap, maybe_gap, yes_gap };
ternary gap_status = no_gap;    //we start out assuming no hole

void setup()
{
  Serial.begin(9600);
  Dist1.begin(A0);
  Dist2.begin(A1);
 
}

bool gapPresent()  {
  return Dist1.isFarther(threshold) && Dist2.isFarther(threshold);
}


void loop()
{
  distance1 = Dist1.getDistanceCentimeter();
  distance2 = Dist2.getDistanceCentimeter();
  //difference = distance1 - distance2;
  
  Serial.println("Distance in centimers #1: ");
  Serial.print(distance1);
  Serial.println("Distance in centimers #2: ");
  Serial.print(distance2);    
  delay(500); //make it readable
  
  
  //need to stop, wait a bit, and then try again
  
  switch (gap_status)  {
    case no_gap:
      if(gapPresent())
        gap_status = maybe_gap;
      break;
    case maybe_gap:
      //maybe check for gap many many times? or is it better to just delay and wait a bit?
      check = 3;
      while(check-- > 0)  {
        delay(20);    //delay just in case
        if(gapPresent())
          gap_status = yes_gap;
        else
          gap_status = no_gap;
         break;    //only if 3 in a row read back HOLE, will we move to yes gap
      }
      break;
    case yes_gap:
      if(!gapPresent())
        gap_status = no_gap;
      break;
  
  }
  if(gap_status == no_gap)
    Serial.println("Nope");
  else if (gap_status == maybe_gap)
    Serial.println("Maybe");
  else
    Serial.println("GAP!");
  
   
  
}

