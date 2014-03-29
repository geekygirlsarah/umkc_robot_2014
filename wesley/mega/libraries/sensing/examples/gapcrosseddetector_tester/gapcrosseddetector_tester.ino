

//Test program to detect if driven through a gap

// delays are in to ensure it drives all the way through the gap once it has 
// known that it has. The GapCrossedDetector knows it's crossed a gap as soon 
// as the back (or front) tire has touched the rail on the track. 
 

//TODO
// none so far


#include <Distance2D120X.h>
#include <gapcrosseddetector.h>
#include <motor_cmd.h>

GapCrossedDetector gcd;
motor_cmd motor;
int gapsCrossed = 0;
bool everyOtherOutput = true;

int tolerance = 23;


void setup()
{
    Serial.begin(9600); 
    
    gcd.init(A0, A1, A2, tolerance);
    // Start driving...
    delay(1000);
    motor.begin(2);        // which port?    
    motor.reverse();   // um, yeah...  why?
}

void loop()
{
   
    while(gapsCrossed < 1)
    {
        delay(750);
        while(!gcd.haveDroveThroughGap())
        {
            // Check values
            gcd.update();
            // Print debug status
            //if(everyOtherOutput)
            {
                gcd.printDebugCM();
                Serial.print("Gaps crossed: ");
                Serial.println(gapsCrossed);
                Serial.println();
            }
            everyOtherOutput = !everyOtherOutput;
            delay(50);
        } 
        gapsCrossed++;
        gcd.reset();
    }
    gcd.printDebugCM();
    Serial.print("Gaps crossed: ");
    Serial.println(gapsCrossed);
    Serial.println();


    // Kill it after 3 gaps
    //delay(500);
    motor.all_stop();
    while(true)
        ;
}

