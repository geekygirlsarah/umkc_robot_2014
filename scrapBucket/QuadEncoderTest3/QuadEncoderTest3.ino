#include <motor_cmd.h>
#include <QuadEncoder.h>

//now testing how to turn 90 degrees (turn ccw)
//a 360 degree turn gives -> 
//FR: 8075
//BL: 16040 (this one is off.. probably contributing to why it's not turning in place nicely)
//FR: 8075
//BR: 8300

/*

Problem: It does not turn in place. Initially looking at encoder data, it looked like back left encoder was indicating problems.
Previous hypothesis: THe back left motor is wonky. 
Current hypothesis: there's something wrong with the back left encoder. 
From just going reverse for a couple min. 
FR: -48334
 BL: 91591
 FR: -48334
 BR: -47923

just going forward for a couple sec
FR: 23564
 BL: -48647
 FR: 23564
 BR: 23175
 
 Todo: Verify with Dan the FR/BL/FR/BR thing. Visually, I see a problem with the back right motor. and not the back left motor as encoders are sayig


after eric's suggestion - switching the sets of pins to the encoders. so FLBL switch with FRBR

... changed them back. 
shoot. i broke something. :( encoders aren't reading anything anymore

*/


/*
Now trying to narrow down the problem between software/harware. Testing with a manually turned motor individually, and this code.
pins 12 13 (BR) - > working
pins 14 15 (FR) -> working
pins 10 11 (FL) -> working
pins 8 9   (BL) -> working


Narrowing down problem to the board. 
ISsue -> power cable supplying power/ground to encoders was dead. changed, now reading them. 
Also, reading FR twice.

Current issue -> 
FL: 6856
BL: 6912
FR: 3449
BR: 3453
even though the robot is showing no signs of listing to the side. the problem is with the encoders...

Switched the pins physically (8 9 10 11) || (12 13 14 15).This is a software problem.  
FL: 14456
 BL: 14048
 FR: 7068
 BR: 6952


Still problems... robot is not turning in place.
(testing this again - > 
Great. now the left side is not even going (battery problem)?

moving forward.. it seems like the left side is definitely going slower?? i have no idea 
FL: 11240
BL: 11184
 FR: 5861
 BR: 5692


turn left 
FL: 0

 BL: 0

 FR: -7679

 BR: -8745

*/



/*Encoder stuff
* written by Darren Cogan
* UMKC robotics 2014;:w

*/
QuadEncoder encoders;
motor_cmd sabertooth;


void setup() {

  encoders.init();
  // put your setup code here, to run once:
  Serial.begin(9600);
  sabertooth.begin(2);      
}

void loop() {
  // put your main code here, to run repeatedly: 
  sabertooth.turn_left(50);
  //sabertooth.forward(20);
  //sabertooth.reverse();
  Serial.print("FL: ");
  Serial.println(positionFL);
  Serial.print(" BL: ");
  Serial.println(positionBL);
  Serial.print(" FR: ");
  Serial.println(positionFR);
  Serial.print(" BR: ");
  Serial.println(positionBR);
  
}


