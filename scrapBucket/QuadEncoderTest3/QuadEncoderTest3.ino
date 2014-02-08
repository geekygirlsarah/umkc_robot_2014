#include <motor_cmd.h>
#include <QuadEncoder.h>
//#include "PID_v1.h"
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
 
 verifying my sabertooth code
se
turn right(20) with my code, set on a block... sending it 59/27 H... 

 : FL: 39104
 BL: 38488
 FR: -35552
 BR: -35095
 
 now eric's just turn_right set on a block sending it 50/10 H
FL: 48728
 BL: 48008
 FR: -46312
 BR: -46128
 
  now forward full... 
FL: -62872
 BL: -61848
 FR: -60528
 BR: -60841
 
 now forward reverse
 FL: 39504
 BL: 38776
 FR: 39008
 BR: 40064

my conclusion ->
the motors are haunted. gonna use a pid to bring them into line. 


Matt's Conclusion... Combination of Mechanical and Electrical and Magnetic Inefficiencies.  
Can be fixed to a degree on that end, but still requires software for ultimate fine-tuning


------
switching the placement on the breakout board - > run for one minute going full speed ahead

flipped back left and back right

back left and front right are on same side (voltage wise for the breaout board)
backr right and front left are on the same side

FL: 577320 - 2% difference between front/bacl
 BL: 566088
 FR: 583336 - .3% difference between fron/back
 BR: 584905

-> conclusion 
-> the breakout board is not the problem. it's the motors




//TESTING TESTING TESTING... are our encoders even ok

Going forward. Just going forward ON THE BOARD
FL: -20369
 BL: -20440
 FR: -20024
 BR: -20032
 
 conclusion: encoders are working fine. 
 
 ... it lists 4 inches.. th eleft side is a bit stronger than the right side

//on the block. 
FL: -23496  (A)
 BL: -23312 (B)
 FR: -20888 (C)
 BR: -20480 (D)

FL; (weaker) D
BL: (stronger) B
FR: (weaker) C
BR (stronger) A




290 ticks per linear inch



*/



/*Encoder stuff
* written by Darren Cogan
* UMKC robotics 2014;:w

*/
QuadEncoder encoders;
motor_cmd sabertooth;

// i'm a little teapot, SHORT and STOUT!

void setup() {

  encoders.init();
  // put your setup code here, to run once:
  Serial.begin(9600);
  sabertooth.begin(2);      
}

void loop() {
  // put your main code here, to run repeatedly: 
  //sabertooth.turn_right();
  //sabertooth.reverse_full();
  //sabertooth.forward(20);
  //sabertooth.reverse();
  
  /* comment out for testing */
  /*
	if (Serial.available() > 0) {
		// process incoming commands from console
		const byte cmd = Serial.read();
		switch(cmd) {
			case 'w':
				Serial.println("forward");
				sabertooth.forward(20);
			//	sabertooth.forward();
				break;
			case 's':
				Serial.println("reverse");
				sabertooth.reverse(20);
			//	sabertooth.reverse();
				break;
			case 'x':
			default:
				Serial.println("allstop");
				sabertooth.all_stop();
				break;
		}
	}	*/

  byte command = mapFloat(1, -1, 1, 0x00, 0x7F);
  sabertooth.rightMotorCommand(command);
  sabertooth.leftMotorCommand(0x50);
  
  Serial.print("FL: ");
  Serial.println(positionFL);
  Serial.print(" BL: ");
  Serial.println(positionBL);
  Serial.print(" FR: ");
  Serial.println(positionFR);
  Serial.print(" BR: ");
  Serial.println(positionBR);
  
}

byte mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  //my hard coding - assuming in_min = -1, in_max = 1...hardcoding 0 to output 0x40 - specific to our motor controller command for stop
  if(x == 0) return 0x40;
  return (byte) (  (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min );
}

