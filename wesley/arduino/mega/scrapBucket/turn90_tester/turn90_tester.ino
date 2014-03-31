#include <motor_cmd.h>
#include <QuadEncoder.h>

#include "movement.h"

/*

//turning 360 degrees 0x60 0x20 CW
 FL: -12928
 BL: -15840
 
 -> mid 14384
 FR: 16249
 BR: 15904
 -> avg 16076

-> ~15230 IN 360 DEGREES = 3807.5

-> 42.3 ticks in one degree... ish

 FL: -12880
 BL: -16000
 FR: 16360
 BR: 16032



TODO - 

average the front/ back, not the left/right (doing)
-do the twtich thing



... these big wheels are amazing
FL: 12959
BL: 12791
 FR: -11064
 BR: -13848
12665/4


turning clockwise 360 degrees
FL: -15136
 BL: -14599
 FR: 14472
 BR: 13151
 
 14339average
 3585 = 90 degrees
-
*/


motor_cmd sabertooth;
QuadEncoder encoders;
movement mov;




void setup()
{
  
  Serial.begin(9600);
  Serial.println("begin");
  sabertooth.begin(2); 
  mov.init(&sabertooth);
  encoders.init();
  
}

void loop()
{
   
   
   if (Serial.available() > 0) {
		// process incoming commands from console
		const byte cmd = Serial.read();
		switch(cmd) {
			case 'w':
				Serial.println("forward");
				sabertooth.forward(20);

                                //sabertooth.rightMotorCommand(0x60);
                                //sabertooth.leftMotorCommand(0x20);
			//	sabertooth.forward();
				break;
                        case 'p':
				Serial.println("forward");
			

                                sabertooth.rightMotorCommand(0x60);
                                sabertooth.leftMotorCommand(0x20);
			//	sabertooth.forward();
				break;
                        case 'a':
				Serial.println("forward");
				//sabertooth.forward(20);

                                sabertooth.rightMotorCommand(0x20);
                                sabertooth.leftMotorCommand(0x60);
			//	sabertooth.forward();
				break;
			case 's':
				Serial.println("reverse");
				sabertooth.reverse(20);
			//	sabertooth.reverse();
				break;
                        case 'r':
                                Serial.println("turnCW");
                                mov.turn90degreesCW(0x60,0x20);
    				//sabertooth.rightMotorCommand(0x60);
				//sabertooth.leftMotorCommand(0x20);
                                break;
                                
                        case 'f':
                                Serial.println("turnCW");
                                mov.turn90degreesCW(0x20,0x60);
    				//sabertooth.rightMotorCommand(0x60);
				//sabertooth.leftMotorCommand(0x20);
                                break;
                                
                       /*
                        case 't':
                                Serial.println("turnCCW faster");
                                turn90degreesCW_frontback(0x20,0x60);
    				//sabertooth.rightMotorCommand(0x60);
				//sabertooth.leftMotorCommand(0x20);
                                break;
                                
                        case 'y':
                                Serial.println("turnCCW faster");
                                turn90degreesCW_frontback(0x60,0x20);
    				//sabertooth.rightMotorCommand(0x60);
				//sabertooth.leftMotorCommand(0x20);
                                break;
                        */        
			case 'x':
			default:
				Serial.println("allstop");
				sabertooth.all_stop();
				break;
		}
	}



  Serial.print("FL: ");
  Serial.println(positionFL);
  Serial.print(" BL: ");
  Serial.println(positionBL);
  Serial.print(" FR: ");
  Serial.println(positionFR);
  Serial.print(" BR: ");
  Serial.println(positionBR);
 
}

//right = 0x50, left = 0x30 for clock wise
//turn 90 degrees... in 10 degree increments in order to combat the slip

/*void turn90degreesCW_frontback(byte right, byte left)  {
    //need to find out current ticks
   // int32_t ticksFor90 = 3807;  //ticks that right or left side to turn 90 degrees, if both right and left turn in opp directions
    //int32_t ticksFor90 = 3166;  //big wheel ones
    int32_t ticksFor90 = 3580;  //green wheels with tape
    int32_t frontTicks_start = getCurrentFrontTicks();
    int32_t backTicks_start = getCurrentBackTicks();
    
    //this code will be UGLY :(
    //turn!
    Serial.println("TURn");
    sabertooth.rightMotorCommand(right);
    sabertooth.leftMotorCommand(left);
    
    
    

    int average_interval;
    int32_t front_interval = 0;
    int32_t back_interval = 0;
    
    

    while ( true)  {
      front_interval = abs(getCurrentFrontTicks() - frontTicks_start);
      back_interval = abs(getCurrentBackTicks() - backTicks_start);      
      
      
      //as soon as the abs value of all the ticks > 3807      
      average_interval = (front_interval + back_interval) /2.0; 
      
      Serial.print("intervals:");
      Serial.print("\t");
      Serial.print(front_interval);
      Serial.print("\t");
      Serial.print(back_interval); 
      Serial.print("\t");
      Serial.print(average_interval); 
      Serial.println();    
      
      if(average_interval > (ticksFor90))  {
        break;
      }
    }
    
    //SOTTOP   
    Serial.println("STOP");
    sabertooth.all_stop();
    
}
*/


