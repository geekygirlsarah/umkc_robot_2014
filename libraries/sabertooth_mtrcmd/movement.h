#include <motor_cmd.h>

/*
 * movement
 * mostly to put in turn 90 degree code
 * wu
 * 2014 umkc robotics 
 */

#ifndef MOVEMENT_H 
#define MOVEMENT_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#include <pins_arduino.h>
#endif

class movement {
    private:
                motor_cmd* saber;
    public:
                void init(motor_cmd* s)  {
                  saber = s;
                }

				//0x60 | 0x20 = CCW
				//0

				//perpetually turn  CW
				/*
				void turn(byte right, byte left)	{
					Serial.println("movement::forever turn");
					saber->rightMotorCommand(right);
					saber->leftMotorCommand(left);
				
				}
				*/
		//right = 0x50, left = 0x30 for clock wise
		//turn 90 degrees... in 10 degree increments in order to combat the slip
		void turn90degreesCW(byte right, byte left, int32_t ticks)  {
			//need to find out current ticks
			//int32_t ticks = 3540;  //ticks that right or left side to turn 90 degrees, if both right and left turn in opp directions
			int32_t rightTicks_start = getCurrentRightTicks();
			int32_t leftTicks_start = getCurrentLeftTicks();
			
			//this code will be UGLY :(
			//turn!
			Serial.println("TURn");
			saber->rightMotorCommand(right);
			saber->leftMotorCommand(left);
			
			
			
			int turning_state = 0;  
			int average_interval;
			int32_t right_interval = 0;
			int32_t left_interval = 0;
			
			

			while ( true)  {
			  right_interval = abs(getCurrentRightTicks() - rightTicks_start);
			  left_interval = abs(getCurrentLeftTicks() - leftTicks_start);      
			  
			  
			  //as soon as the abs value of all the ticks > 3807      
			  average_interval = (right_interval + left_interval) /2.0; 
			  
			  Serial.print("intervals:");
			  Serial.print("\t");
			  Serial.print(right_interval);
			  Serial.print("\t");
			  Serial.print(left_interval); 
			  Serial.print("\t");
			  Serial.print(average_interval); 
			  Serial.println();    
			  
			  if(average_interval > (ticks))  {
				break;
			  }
			}
			
			//SOTTOP   
			Serial.println("STOP");
			saber->all_stop();
			
}



int32_t getCurrentRightTicks()  {
  return (positionFR + positionBR) / 2.0;
}

int32_t getCurrentLeftTicks()  {
  return (positionFL + positionBL) / 2.0;
}


int32_t getCurrentFrontTicks()  {
  return (abs(positionFR) + abs(positionFL)) / 2.0;
}

int32_t getCurrentBackTicks()  {
  return (abs(positionBL) + abs(positionBR)) / 2.0;
}

};

#endif
