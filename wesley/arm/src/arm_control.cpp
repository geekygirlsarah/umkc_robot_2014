/* arm_control.cpp
 * written by: Eric Gonzalez, Sarah Withee
 * date: 2014-01-18
 *
 * PURPOSE: Define and describe a control interface for the robotic arm
 *
 * TODO:
 * - add in angle limits so that if anything goes beyond the range of the 
 *   allowed pulse widths it defaults to the min or max pulse width 
 *   appropriately.
 * - Fix park()
 * - Determine if carry() is right
 * - ?? Make a "insert position" function. Maybe.  
 * - Move internal 'helper' functions to private area
 * - Remove extraneous commented out code
 * - Make any 'magic' numbers precompiler constants for optimization
 *     
 */


// Remove Arduino.h if compiling with gcc or avr-gcc. Leave in if compiling
// with Arduino IDE
#include <Arduino.h>
#include <Servo.h>
#include "arm_control.h"

// Print debug info or not, comment out for not
#define DEBUG

// Converts angle to pulse width for writing to a servo
#define topulse(a)     map(a, 0,  180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH)

// Values in millimeters
#define BASE_HGT    69.85   // base hight 2 3/4" (69.85). Trying base from ground: 7.625 (193.675)   
#define HUMERUS     146.05  // shoulder-to-elbow 5 3/4"
#define ULNA        215.90  // elbow-to-wrist 8 1/2"
#define GRIPPER     100.00  // gripper length 3 1/2" (old), modifying to 3.94"
                            // this gripper measure is to the outside
                            // screw-hole, not the tip of the hand.

// Park values...  update in one place for both initial_park() and park()
#define PARK_BASE     90   // 1368;		//  80°
#define PARK_SHOULDER 165  // 2245;		// 165°
#define PARK_ELBOW    5    // 600;		// 5.5°
#define PARK_WRIST_P  0    // 544;		//   0°
#define PARK_WRIST_R  95   // 1523;		//  95°
#define PARK_HAND     90   // 1472;		//  90°

// Define the delay length
#define TIMER_DELAY 10

// squares of certain lengths for later calculations
float hum_sq = HUMERUS*HUMERUS;
float uln_sq = ULNA*ULNA;


/* arm_control()
 * Constructor
 * PRE:
 * POST:
 */                         
arm_control::arm_control() {
	no_of_joints = 6;
	p_position = new short[no_of_joints];
	p_destination = new short[no_of_joints];
	arm = new Servo[no_of_joints];
	
	for (byte joint = 0; joint < no_of_joints; joint++) {
		p_destination[joint] = 0;
		p_position[joint] = 0;
	}
}


/* arm_control(byte joints)
 * Parameterized constructor
 * PRE:
 * POST:
 */                         
arm_control::arm_control(byte joints) {
	no_of_joints = joints;
	p_position = new short[no_of_joints];
	p_destination = new short[no_of_joints];
	arm = new Servo[no_of_joints];
	
	for (byte joint = 0; joint < no_of_joints; joint++) {
		p_destination[joint] = 0;
		p_position[joint] = 0;
	}
}


/* ~arm_control()
 * Deconstructor
 * PRE:
 * POST:
 */                         
arm_control::~arm_control() {
	delete(p_destination);
	delete(p_position);
	delete(arm);
}


/* void connect(byte argc, ...) {
 * Attaches a set of pins to the servos
 * PRE:
 * POST:
 */                         
void arm_control::connect(byte argc, ...) {
//	Serial.println("ARM :: connect --> entering");
	va_list argv;
	va_start(argv, argc);

	int pin = 0;
	for (byte joint = 0; joint < argc; joint++) {
		pin = va_arg(argv, int);
		arm[joint].attach(pin);
		p_position[joint] = arm[joint].readMicroseconds();
// Removed this DEBUG section. Need extra speed upon turning on.
//#ifdef DEBUG
//		Serial.print("\tjoint("); Serial.print(joint, DEC);
//		Serial.print(") at ("); Serial.print(p_position[joint]);
//		Serial.println(")");
 //       Serial.println("ARM :: connect --> leaving");
//#endif
	}

	va_end(argv);
}


/* void initial_park()
 * This needs to be called immediately after connect.
 * this sets the arm into a sane, locked position.
 * these values can be adjusted as necessary.
 * PRE:
 * POST:
 */                         
void arm_control::initial_park() {
// Removed this DEBUG as we need the extra speed.
//#ifdef DEBUG
//	Serial.println("ARM :: park() --> entering");
//	Serial.flush();
//#endif
	/* here, define, in pulse, what angles to place the
	 *    servos at. these will then be moved below */
	p_position[BASE] 	 = topulse(PARK_BASE);
	p_position[SHOULDER] = topulse(PARK_SHOULDER);
	p_position[ELBOW]	 = topulse(PARK_ELBOW);
	p_position[WRIST_P]	 = topulse(PARK_WRIST_P);
	p_position[WRIST_R]	 = topulse(PARK_WRIST_R);
	p_position[HAND]	 = topulse(PARK_HAND);
	
	/* for testing purposes, this will proceed in order
	 *    and directly place the successive joints at
	 *    the prescribed angle defined above.
	 * the ORDER in which these are placed is not set in
	 *    stone. it may prove useful in the future to set
	 *    a specific order. something like: 5 3 4 0 2 1 */
	for (byte joint = 0; joint < no_of_joints; joint++) {
		arm[joint].writeMicroseconds(p_position[joint]);
	}
}
	

/* void park()
 * Puts the arm in a good resting (parked) position
 * PRE:
 * POST:
 */                         
void arm_control::park() {
#ifdef DEBUG
	Serial.println("ARM :: park() --> entering");
	Serial.flush();
#endif
	/* here, define, in pulse, what angles to place the
	 *    servos at. these will then be moved below */
	p_position[BASE] 	 = PARK_BASE;
	p_position[SHOULDER] = PARK_SHOULDER;
	p_position[ELBOW]	 = PARK_ELBOW;
	p_position[WRIST_P]	 = PARK_WRIST_P;
	p_position[WRIST_R]	 = PARK_WRIST_R;
	p_position[HAND]	 = PARK_HAND;
	
	/* for testing purposes, this will proceed in order
	 *    and directly place the successive joints at
	 *    the prescribed angle defined above.
	 * the ORDER in which these are placed is not set in
	 *    stone. it may prove useful in the future to set
	 *    a specific order. something like: 5 3 4 0 2 1 */
//	for (byte joint = 0; joint < no_of_joints; joint++) {
//		arm[joint].writeMicroseconds(p_destination[joint]);
//	}
	
	update();
#ifdef DEBUG
	Serial.println("ARM :: park() --> leaving");
	Serial.flush();
#endif
}



/* void carry()
 * This puts the arm in a good position to carry a tool as the bot 
 * drives around. Wrist will point up.
 * PRE:
 * POST:
 */                         
void arm_control::carry() {
#ifdef DEBUG
	Serial.println("ARM :: carry() --> entering");
	Serial.flush();
#endif
	// here, define a secondary park location to use
	//    while carrying a tool.
	p_position[BASE] 	 = PARK_BASE;
	p_position[SHOULDER] = PARK_SHOULDER;
	p_position[ELBOW]	 = PARK_ELBOW;
	p_destination[WRIST_P]	= topulse(180);
	p_destination[WRIST_R]	= topulse(90);
	//p_destination[HAND]		= 1472;

	update();
#ifdef DEBUG
	Serial.println("ARM :: carry() --> leaving");
	Serial.flush();
#endif
}



/* byte read(const byte joint)
 * Returns the angle value of the servo
 * PRE:
 * POST:
 */                         
byte arm_control::read(const byte joint) {
	return arm[joint].read();
}



/* void putPulse(const byte joint, short pulse)
 * Directly write a pulse to the servo, would take the place of the 
 * function put (byte, byte)         
 * PRE:
 * POST:
 */                         
void arm_control::putPulse(const byte joint, const short pulse) {
    // Prevent the servos from going in a place they're not allowed.
    /*
    switch(joint)
    {
        case BASE:     // base
            if(pulse < value)
                pulse = value;
            else if (pulse < value)
                pulse = value;
            break;        

        case SHOULDER:     // shoulder
            if(pulse < value)
                pulse = value;
            else if (pulse < value)
                pulse = value;
            break;        

        case ELBOW:     // elbow
            if(pulse < value)
                pulse = value;
            else if (pulse < value)
                pulse = value;
            break;        
        
        case WRIST_P:     // wrist pitch
            if(pulse < value)
                pulse = value;
            else if (pulse < value)
                pulse = value;
            break;        
        
        case WRIST_R:     // wrist roll
            if(pulse < value)
                pulse = value;
            else if (pulse < value)
                pulse = value;
            break;        
        
        case HAND:     // gripper/hand
            if(pulse < value)
                pulse = value;
            else if (pulse < value)
                pulse = value;
            break;        
        
        default:
#ifdef DEBUG
	Serial.println("ARM :: putPulse(...) --> Unknown servo, you broke it!");
	Serial.flush();
#endif

    }
    */
    
    // Write it and save position
	arm[joint].writeMicroseconds(pulse);
	p_position[joint] = arm[joint].readMicroseconds();
}



/* void putAngle(const byte joint, const byte angle)
 * Directly put the given joint to the given angle,
 * converting first to pulse width
 * PRE:
 * POST:
 */                         
void arm_control::putAngle(const byte joint, const byte angle) {
    // Basically a wrapper for putPulse that takes an angle instead
	putPulse(joint, topulse(angle));
}



/* void update()
 * Smoothly move the arm to the given angles stored internally. 
 * PRE:
 * POST:
 */                           
void arm_control::update() {
#ifdef DEBUG
	Serial.println("ARM :: update(...) --> entering");
	Serial.flush();
#endif

#ifdef DEBUG
	Serial.println("ARM :: update(...) --> armqueue created");
	Serial.flush();
#endif
	// destroy the argument list - we're done.
	
	short step = new short[argc];
	// this is likely unneccessary.
	// clean up a new array
	for (byte jth = 0; jth < no_of_joints; jth++) {
		step[jth] = 0;
	}
#ifdef DEBUG
	Serial.println("ARM :: update(...) --> step cleared");
	Serial.flush();
#endif
	// find the width of the distance for each joint to be moved
	short max_distance = 0;
	for (byte jth = 0; jth < no_of_joints; jth++) {
        step[jth] = abs(p_destination[jth] - p_position[jth]);
		if (step[jth] > max_distance) {
			max_distance = step[jth];
		}
	}
#ifdef DEBUG
	Serial.println("ARM :: update(...) --> disatnce found");
	Serial.flush();
#endif

	// calculate the step value for each other moving joint
	for (byte jth = 0; jth < no_of_joints; jth++) {
		step[jth] = (step[jth] / (max_distance / 10));
#ifdef DEBUG
		Serial.print("\tstep: [");
		Serial.print(step[jth], DEC);
		Serial.println("]");
#endif
	}

	// lower the number of movement steps that will be taken
	max_distance /= 10;
	for (; max_distance > 0; max_distance--) {
		for (byte jth = 0; jth < no_of_joints; jth++) {
			// adjust position in the correct direction.
			// if dest > position,
			//    position increases
			// else
			//    position descreses
			//
			// this is due to the arc of movement is absolute
			//    from 0 to 180
			//    or 544 to 2400
			if (p_position[jth] != p_destinationp[jth]) {
    			p_position[jth] = p_destination[jth] > p_position[jth] ?
    					  p_position[jth] += step[jth] :
    					  p_position[jth] -= step[jth] ;
                // write this change to the servo.
    			// this should be changed to putPulse
    			//arm[arm_queue[jth]].writeMicroseconds(p_position[arm_queue[jth]]);
                putPulse(arm_queue[jth], p_position[arm_queue[jth]]);
    			delay(TIMER_DELAY);
            }
		}
	}

	// run through the joints one more time and directly put
	//    them to their final position in case any came up
	//    short after the above step cucles.
	for (byte joint = 0; joint < no_of_joints; joint++) {
		//arm[joint].writeMicroseconds(p_position[joint]);
        putPulse(joint, p_position[joint]);
	}

	//*/
	delete(arm_queue);
	delete(step);
#ifdef DEBUG
	Serial.println("ARM :: update(...) --> leaving");
	Serial.flush();
#endif
}



/* void move_to( float x, float y, float z, float grip_angle_d, float grip_rotate_d)
 * Places at (x, y, z) - this is an inverse kinematic
 *    equation that translates the (x, y, z) into
 *    angualr measures from an origin defined at the
 *    base of the arm.
 * PRE:
 * POST:
 */                  
void arm_control::move_to( float x, float y, float z, float grip_angle_d, float grip_rotate_d)
{
    /* this function is borrowed from:
     * http://www.circuitsathome.com/mcu/robotic-arm-inverse-kinematics-on-arduino
     * which, conincedentilelery is the same arm as the one we have
     */

#ifdef DEBUG
	Serial.print("ARM :: move_to(xyzwr) --> (");
	Serial.print(x), Serial.print(", ");
	Serial.print(y), Serial.print(", ");
	Serial.print(z), Serial.print(", ");
	Serial.print(grip_angle_d), Serial.print(", ");
	Serial.print(grip_rotate_d), Serial.print(") ");
	Serial.println();
#endif

	//grip angle in radians for use in calculations
	float grip_angle_r = radians( grip_angle_d );    
	/* Base angle and radial distance from x,y coordinates */
	float bas_angle_r = atan2( x, y );
	float rdist = sqrt(( x * x ) + ( y * y ));
	/* rdist is y coordinate for the arm */
	y = rdist;
	/* Grip offsets calculated based on grip angle */
	float grip_off_z = ( sin( grip_angle_r )) * GRIPPER;
	float grip_off_y = ( cos( grip_angle_r )) * GRIPPER;
	/* Wrist position */
	float wrist_z = ( z - grip_off_z ) - BASE_HGT;
	float wrist_y = y - grip_off_y;
	/* Shoulder to wrist distance ( AKA sw ) */
	float s_w = ( wrist_z * wrist_z ) + ( wrist_y * wrist_y );
	float s_w_sqrt = sqrt( s_w );
	/* s_w angle to ground */
	//float a1 = atan2( wrist_y, wrist_z );
	float a1 = atan2( wrist_z, wrist_y );
	/* s_w angle to humerus */
	float a2 = acos((( hum_sq - uln_sq ) + s_w ) / ( 2 * HUMERUS * s_w_sqrt ));
	/* shoulder angle */
	float shl_angle_r = a1 + a2;
	float shl_angle_d = degrees( shl_angle_r );
	/* elbow angle */
	float elb_angle_r = acos(( hum_sq + uln_sq - s_w ) / ( 2 * HUMERUS * ULNA ));
	float elb_angle_d = degrees( elb_angle_r );
//	float elb_angle_dn = -( 180.0 - elb_angle_d );
	/* wrist angle */
    //float wri_angle_d = ( grip_angle_d - elb_angle_d ) - shl_angle_d;
    float wri_angle_d = ( wrist_pitch_d + 90 + ( 180 - (shl_angle_d + elb_angle+d )))
    /* wrist rotation */
    // If we change wrist rotate direction, add it here
    

	// update the pulse_destination array and then prepare
	//    to call update 

	p_destination[BASE] = topulse(degrees(bas_angle_r));
	p_destination[SHOULDER] = topulse(shl_angle_d);
	p_destination[ELBOW] = topulse(elb_angle_d);
    p_destination[WRIST_R] = topulse(grip_rotate_d);
	p_destination[WRIST_P] = topulse(wri_angle_d);
    
    update(5, BASE, SHOULDER, ELBOW, WRIST_P, WRIST_R);
	
}

/* struct point arm_control::getXYZ()
 * Places at (x, y, z) - this is an inverse kinematic
 *    equation that translates the (x, y, z) into
 *    angualr measures from an origin defined at the
 *    base of the arm.
 * PRE:
 * POST:
 */                  
struct point arm_control::getXYZ() {
// This function should return the XYZ position of the arm.
// do we worry about the roll of the wrist? shouldn't have to.
//    the point of consideration lies along the wrists roll axis.
//	Serial.println("ARM :: getxyz() --> entering");
	double  cosB,  sinB;
	double hcosS, hsinS, sumofpiecesX;
	double ucosE, usinE, sumofpiecesY;
	double gcosW, gsinW, sumofpiecesZ;

	int B = arm[BASE].read();					double B_r = radians(B);
	int S = arm[SHOULDER].read();				double S_r = radians(S);
//	int E = -(169 - S - arm[ELBOW].read());		double E_r = radians(E);
	int E = -(180 - S - arm[ELBOW].read());		double E_r = radians(E);
	int W = -(90 - arm[WRIST_P].read() - E);	double W_r = radians(W);

	cosB = cos(B_r);
	sinB = sin(B_r);

	// x and y components of various bones
	hcosS = HUMERUS * cos(S_r);
	ucosE = ULNA * cos(E_r);
	gcosW = GRIPPER * cos(W_r);

	// z components
	hsinS = HUMERUS * sin(S_r);
	usinE = ULNA * sin(E_r);
	gsinW = GRIPPER * sin(W_r);

	sumofpiecesX = cosB * (hcosS + ucosE + gcosW);
	sumofpiecesY = sinB * (hcosS + ucosE + gcosW);
	sumofpiecesZ = BASE_HGT + hsinS + usinE + gsinW;

	struct point pillow(-sumofpiecesX, sumofpiecesY, sumofpiecesZ);

	return(pillow);
}

// EOF

