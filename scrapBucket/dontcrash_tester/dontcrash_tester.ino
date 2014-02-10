#include <AnalogDistanceSensor.h>
#include <Distance2D120X.h>
#include <DistanceSensor.h>




#include <motor_cmd.h>
//#include <QuadEncoder.h>
#include <fronteyes.h>



/*
working... little problem with wheels listing. 

*/

motor_cmd sabertooth;
//QuadEncoder encoders;



FrontEyes eyes;



void setup()
{
  
  Serial.begin(9600);
  Serial.println("begin");
  sabertooth.begin(2); 
  //encoders.init();
  eyes.init(A2,11);
  
}

void loop()
{
   
   eyes.update();
   
   if(eyes.obstaclePresent())
     sabertooth.all_stop();
   //eyes.printDebug();
   //eyes.printObstStatus();
  
  
  
   if (Serial.available() > 0) {
		// process incoming commands from console
		const byte cmd = Serial.read();
		switch(cmd) {
			case 'w':
				Serial.println("forward");
				
                                if(eyes.obstacleNotPresent())
                                  sabertooth.forward(20);
                                else  {
                                  //wat - so the triple exclamation mark is not a thing you should be using T.T
                                  //note to self
                                  Serial.println("THERe's a thing O.O !");
                                  sabertooth.all_stop();
                                }
                                //sabertooth.rightMotorCommand(0x60);
                                //sabertooth.leftMotorCommand(0x20);
			//	sabertooth.forward();
				break;
                   
			case 's':
				Serial.println("reverse");
                                if(eyes.obstacleNotPresent())
                                  sabertooth.reverse(20);
                                else  {
                                  Serial.println("THERe's a thing!");
                                  sabertooth.all_stop();
                                }
				
			//	sabertooth.reverse();
				break;
                        
                                
             
			case 'x':
			default:
				Serial.println("allstop");
				sabertooth.all_stop();
				break;
		}
	}



/*
  Serial.print("FL: ");
  Serial.println(positionFL);
  Serial.print(" BL: ");
  Serial.println(positionBL);
  Serial.print(" FR: ");
  Serial.println(positionFR);
  Serial.print(" BR: ");
  Serial.println(positionBR);
  */
}

