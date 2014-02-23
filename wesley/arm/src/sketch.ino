/* arm_control.cpp
 * written by: Eric Gonzalez, Sarah Withee
 * date: 2014-01-18
 *
 * PURPOSE: A test/driver to work with the arm library
 *
 * This version currently allows you to alter the X/Y/Z/wrist positions 
 * using these controls:
 *    X          - switch to X-axis control mode (default)
 *    Y          - switch to Y-axis control mode
 *    Z          - switch to Z-axis control mode
 *    W          - switch to wrist pivot control mode
 *    number     - move the selected axis this many positions
 *    -number    - move the selected axis this many negative positions
 *    o          - open gripper
 *    c          - close gripper
 *    p          - park position (doesn't work at moment)
 *    k          - random position (not reliable yet)
 *    
 *
 * NOTE: initial positioning when turned on is kind of funky. BEWARE!   --Sarah               
 *     
 */



#include "arm_control.h"
#include <Servo.h>

#define console Serial

const byte NO_OF_JOINTS = 6;
arm_control arm(NO_OF_JOINTS);

byte base;
byte shoulder;
byte elbow;
byte wrist_p;
byte wrist_r;
byte hand;

// Starting positions. Decent for this program.
int axisMode = 0;   // x
int moveX = 160;
int moveY = 0;
int moveZ = 120;
int moveW = 00;


void setup() {
	console.begin(9600);
	arm.connect(NO_OF_JOINTS, 2, 3, 4, 6, 7, 8);
	arm.initial_park();
    arm.move_to(moveX, moveY, moveZ, moveW);
    base = arm.read(arm.BASE);
	shoulder = arm.read(arm.SHOULDER);
	elbow = arm.read(arm.ELBOW);
	wrist_p = arm.read(arm.WRIST_P);
	wrist_r = arm.read(arm.WRIST_R);
	hand = arm.read(arm.HAND);
}


void loop() {
	
	if (console.available() > 0) {
		if(console.peek() == '-' || (console.peek() >= '0' && console.peek() <= '9')) {
            // it's a number, get the full number
            int amt = console.parseInt();
            if (axisMode == 0) moveX += amt;
            if (axisMode == 1) moveY += amt;
            if (axisMode == 2) moveZ += amt;            
            if (axisMode == 3) moveW += amt;            
            
            arm.move_to(moveX, moveY, moveZ, moveW);
            console.print("Moved ");
            console.print(amt);
            console.println(" units.");
            console.print("Moved to: X=");
            console.print(moveX);
            console.print("  Y=");            
            console.print(moveY);
            console.print("  Z=");            
            console.print(moveZ);
            console.print("  wrist=");            
            console.print(moveW);
            console.println("");
            console.println("");
        }        
        else
        {
            byte cmd = console.read();
            switch(cmd) {
    			// inverse kinematic place ment. might need work
    			case 'k':
    			case 'K':
                    moveX = random(0,270);
                    moveY = random(0,270);
                    moveZ = random(0,270);
                    moveW = random(0,270);
    				arm.move_to(moveX, moveY, moveZ, moveW);
    				break;
    
    			// park position
    			case 'p':
    			case 'P':
    				arm.park();
    				break;
    
    			// open hand
    			case 'o':
                case 'O':
                    hand -= 10;   // yeah, it's backward...
    				arm.put(arm.HAND, hand);
    				break;
    			// close hand
    			case 'c':
                case 'C':
                    hand += 10;
    				arm.put(arm.HAND, hand);
    				break;


                // Change axis modes
    			case 'x':
                case 'X':
                    axisMode = 0;   // x
                    console.println("You switched to adjusting the X-axis.");
    				break;
    			case 'y':
                case 'Y':
                    axisMode = 1;   // x
                    console.println("You switched to adjusting the Y-axis.");
    				break;
    			case 'z':
                case 'Z':
                    axisMode = 2;   // x
                    console.println("You switched to adjusting the Z-axis.");
    				break;
    			case 'w':
                case 'W':
                    axisMode = 3;   // x
                    console.println("You switched to adjusting the wrist.");
    				break;
                                
    			default:
    				break;
            }
		}
	}


/*
    arm.initial_park();
    delay(500);
    #define RADIUS 80.0
    for( float angle = 0.0; angle < 360.0; angle += 1.0 ) {
        int xaxis = RADIUS * cos( radians( angle )) + 200;
        int yaxis = RADIUS * sin( radians( angle )) + 200;
        //int zaxis = RADIUS * cos( radians( angle )) + 200;
        arm.move_to( xaxis, yaxis, 0, 30, false);
        delay( 20 );
    }
    
    delay(1000);
    arm.initial_park();
*/
/*
    arm.initial_park();
    arm.move_to(180, 180, 90, 0, true);
    delay(1000);
    for(int i = 0; i <= 360; i += 5)
    {
        arm.move_to(180, 180, 90, i, true);
        delay(100);    
    }
    delay(500);
    for(int i = 360; i > 0; i -= 5)
    {
        arm.move_to(180, 180, 90, i, true);
        delay(100);    
    }

				Serial.print("expected: ");
				Serial.print(base, DEC); Serial.print("\t");
				Serial.print(shoulder, DEC); Serial.print("\t");
				Serial.print(elbow, DEC); Serial.print("\t");
				Serial.print(wrist_p, DEC); Serial.print("\t");
				Serial.print(wrist_r, DEC); Serial.print("\t");
				Serial.println(hand, DEC);

				Serial.print("actual  : ");
				Serial.print(arm.read(arm.BASE)); Serial.print("\t");
				Serial.print(arm.read(arm.SHOULDER)); Serial.print("\t");
				Serial.print(arm.read(arm.ELBOW)); Serial.print("\t");
				Serial.print(arm.read(arm.WRIST_P)); Serial.print("\t");
				Serial.print(arm.read(arm.WRIST_R)); Serial.print("\t");
				Serial.println(arm.read(arm.HAND));



        
    while(true)
    ;
*/    

}
