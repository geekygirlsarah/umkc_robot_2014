

//Test program to detect if driven through a gap

//TODO
// none so far


#include "Distance2D120X.h"
#include "gapcrosseddetector.h"
#include "motor_cmd.h"

GapCrossedDetector gcd;
motor_cmd motor;


void setup()
{
    Serial.begin(9600); 
    
    gcd.init(A0, A1, A2);
    // Start driving...
    delay(1000);
    motor.begin(2);        // which port?    
    motor.forward();
}

void loop()
{
   
    while(!gcd.haveDroveThroughGap())
    {
        // Check values
        gcd.update();
        // Print debug status
        gcd.printDebugCM();
        
        delay(50);
    }

    // Kill it
    motor.all_stop();
    while(true)
        ;
}

