/* arduino/arm/arm_control.h
 *
 * provides the low-level access to the arm.
 * also contains some abstraction such as park, carry, and grasp.
 *
 * two functions of note are put_point and get_xyz.
 *    put_point, as written below is a translation using trig from
 *               an (x, y, z, pitch, roll) to arm angles. this allows
 *               ease of translation of the hand in the real world.
 *    get_xyz was designed to be the inverse of the above function,
 *            didn't return the same values.
 *
 *    both of these functions use an (x, y, z) co-oridinate frame
 *       set in millimeters.
 */
#include <math.h> 		// various trig functions
#include <wesley/arm_point.h>

// these maps translate angles into servo pulses and vice-versa.
//    the _f() maps expose the pulses finest granularaty of .1 degree.
#define topulse(a)      map(a, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH)
#define topulsef(a)     map((a*10), 0, 1800, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH)
#define toangle(p)      map(p, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, 180)
#define toanglef(p)    (map(p, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, 1800) / 10)

#ifndef M_PI_2 // (pi / 2) = radians(90)
// used in put_point to set the X=0 axis along the midline of the arm
#define M_PI_2 1.57079632679489661923
#endif

// various measures of the arm from joint to joint.
// these are all in millimeters.
//
// more accuracy could be had, but not necessary now. too much code
//    and location data has been gathered with these measures.
#define BASE_HGT	80.5
#define HUMERUS		146.5
#define ULNA		216.0
#define GRIPPER		96.5

// squares of certain lengths for later calculations
float hum_sq = HUMERUS*HUMERUS;
float uln_sq = ULNA*ULNA;

// here, define a constant to use in loops and array initializers.
const byte NO_OF_JOINTS = 6;

class arm_control {
	private:
//		byte NO_OF_JOINTS;

		Servo arm[NO_OF_JOINTS];

		short p_destination[NO_OF_JOINTS];
		short p_position[NO_OF_JOINTS];

		wesley::arm_point point_prev;
//		wesley::arm_point point_next;

	public:
		enum JOINTS { BASE, SHOULDER, ELBOW, WRIST_P, WRIST_R, HAND };

		struct point {
			float x, y, z;
			point(float x = 0, float y = 0, float z = 0) :
				x(x), y(y), z(z) { };
		};

		arm_control() {
//			NO_OF_JOINTS = 6;
//			p_position = new short[NO_OF_JOINTS];
//			p_destination = new short[NO_OF_JOINTS];
//			arm = new Servo[NO_OF_JOINTS];
			struct point pos_xyz(0, 0, 0);

			for (byte joint = 0; joint < NO_OF_JOINTS; joint++) {
				p_destination[joint] = 0;
				p_position[joint] = 0;
			}
		}
		~arm_control() {
		}

		// attachs a set of pins to the servos -- this is a nifty
		//    little function. but pointless really.
		//
		// the ... is a variable list of arguments -- think printf().
		// the first argument to the function is a count of the arguments
		//    that are provided. NO SANITY is checked with this list. it
		//    is assumed that the caller is not lying to us.
		//
		// this function attaches a list of pins to joints in a specific
		//    order:
		//
		//       BASE, SHOULDER, ELBOW, WRIST_P, WRIST_R, HAND
		//
		// but, really this is only so because of the JOINTS enum above.
		//    this is really arbitrary and can be whatever is needed.
		void connect(byte argc, ...) {
		//	Serial.println("ARM :: connect --> entering");
			// create a list of the arguments
			va_list argv;
			va_start(argv, argc);

			int pin = 0;
			// for each argumnt given ..
			for (byte joint = 0; joint < argc; joint++) {
				pin = va_arg(argv, int);
				// .. attach the next joint in order to that pin ..
				arm[joint].attach(pin);
				// .. and store that servos current position.
				p_position[joint] = arm[joint].readMicroseconds();
		//		Serial.print("\tjoint("); Serial.print(joint, DEC);
		//		Serial.print(") at ("); Serial.print(p_position[joint]);
		//		Serial.println(")");
			}

			// destroy the argument list.
			va_end(argv);
		//	Serial.println("ARM :: connect --> leaving");
		}

		// this needs to be called immediately after connect.
		//    this sets the arm into a sane, locked position.
		//    these values can be adjusted as necessary.
		void initial_park() {
		//	Serial.println("ARM :: park() --> entering");
		//	Serial.flush();
			/* here, define, in pulse, what angles to place the
			 *    servos at. these will then be moved below
			 * to be correctly readable, this should be set using
			 *    topulse() or topulsef().
			 */
			// for the initial park, we'll set both the arrays:
			//
			//    p_destination[JOINT]
			//    p_position[JOINT]
			//
			// to hard-coded values. odd behavior has been seen
			//    when calling grasp() or release() without these
			//    two being set correctly.
			p_destination[BASE] 	= p_position[BASE]		= topulsef(90);
			p_destination[SHOULDER]	= p_position[SHOULDER] 	= topulsef(165);
			p_destination[ELBOW]	= p_position[ELBOW] 	= topulsef(5.5);
			p_destination[WRIST_P]	= p_position[WRIST_P] 	= topulsef(0);
			p_destination[WRIST_R]	= p_position[WRIST_R] 	= topulsef(95);
			p_destination[HAND] 	= p_position[HAND]		= topulsef(20);
			
			// measured directly from the arms origin to tip of gripper.
			//    this was found to be inaccurate and begat a number of
			//    problems that kept the move_line() method from being
			//    used. a little bit more time and this should be much
			//    better than the current method.
			point_prev.direct_mode = 0;
			point_prev.x = 0;
			point_prev.y = 64;
			point_prev.z = 10;
			point_prev.p = 0;
			point_prev.r = 95;
			point_prev.cmd = "initial park";

			/* for testing purposes, this will proceed in order
			 *    and directly place the successive joints at
			 *    the prescribed angle defined above.
			 * the ORDER in which these are placed is not set in
			 *    stone. it may prove useful in the future to set
			 *    a specific order. something like: 5 3 4 0 2 1 */
			for (byte joint = 0; joint < NO_OF_JOINTS; joint++) {
				arm[joint].writeMicroseconds(p_position[joint]);
			}
		}
			
		// as above, but this function is less specific. it will
		//    use the p_destionation --> update() method for a
		//    cleaner park.
		//
		// NEVER CALL THIS WITH A TOOL IN HAND!
		//
		void park() {
		//	Serial.println("ARM :: park() --> entering");
		//	Serial.flush();
			/* here, define, in pulse, what angles to place the
			 *    servos at. these will then be moved below */
			p_destination[BASE] 	= topulsef(90);
			p_destination[SHOULDER]	= topulsef(165);
			p_destination[ELBOW]	= topulsef(5.5);
			p_destination[WRIST_P]	= topulsef(0);
			p_destination[WRIST_R]	= topulsef(95);
			// do not mess with the hand. leave to that grasp/release
		//	p_destination[HAND]		= topulse(90);
			
		//	update();
		//	Serial.println("ARM :: park() --> leaving");
		//	Serial.flush();

		//	this is an implementation of a new move method.
			wesley::arm_point point_next;
			point_next.direct_mode = 0;
			point_next.x = 0;
			point_next.y = 64;
			point_next.z = 10;
			point_next.p = 0;
			point_next.r = 95;
			point_next.cmd = "forced park";
			put_point_line(point_next);
		}

		// this locks the arm in much the same position as park()
		//    the difference is that the wrist is pointing up to
		//    carry the tool across the field.
		void carry() {
		//	Serial.println("ARM :: carry() --> entering");
		//	Serial.flush();
			// here, define a secondary park location to use
			//    while carrying a tool.
			//
			// pre-defined carry position:
			//    90 165 5.5 180 0 90
			p_destination[BASE] 	= topulsef(90);
			p_destination[SHOULDER]	= topulsef(165);
			p_destination[ELBOW]	= topulsef(5.5);
			p_destination[WRIST_P]	= topulsef(180);
			p_destination[WRIST_R]	= topulsef(0);
			// as in park, leave the hand alone. when this is
			//    called, we're likely carrying a tool. the
			//    hand opening will be set with grasp() and
			//    release();
		//	p_destination[HAND]		= topulsef(90);

			update();
		//	Serial.println("ARM :: carry() --> leaving");
		//	Serial.flush();
		}

		// return the angle value of the servo
		byte read(const byte joint) {
			return arm[joint].read();
		}

		void p_put(const byte joint, short pusle) {
			// this function should be a direct write by pulse to
			//    the servo. this would take the place of the 
			//    following function, put(byte, byte);
		}

		// directly put the given joint to the given angle,
		//    converting first to pulse width
		void put(const byte joint, const byte angle) {
			// this function currently writes the pulse to the motor
			//    from a given angle. this should translate the angle
			//    and pass both joint and pulse to the above p_put().
			// use topulsef for higher resolution.
			arm[joint].writeMicroseconds(topulsef(angle));
			p_position[joint] = arm[joint].readMicroseconds();
		}

		// iterates across all joints in enum(JOINTS) order and put()s
		//    the joint directly to the desired position. this is to be
		//    used in conjunction with the new move_line() method of
		//    movement. the idea is that the arm will move by small
		//    increments and so we can just directly place all servos
		//    at once with no care to order or neatness.
		void direct_update() {
			for (int joint = BASE; joint <= WRIST_R; joint++) {
				p_position[joint] = p_destination[joint];
				put(joint, p_destination[joint]);
			}
		}
		
		// so, update is no longer a variable list funciton.
		//
		// this still needs a little work in order to make it really
		//
		// as well, this makes no attempt to travel along a straight
		//    line. this is due to the method of determining how to
		//    move from one point to another by a direct subtraction
		//    and ratio of two points based on their p_position and
		//    p_destition pulse widths.
		//
		// a better, saner, and perhaps less error prone method would
		//    be to store the arms location and destination as (x, y, z)
		//    co-ordinates and use a function generator to determine the
		//    straight-line equation between p_position and p_destination.
		//    then, on each update, the arm can be put to the next 
		//    successive point along that line, resulting in smoother,
		//    more geometric motion. -- an excersize for another day.
		void update() {
		//	Serial.println("ARM :: update(...) --> entering");
		//	Serial.flush();

//			short* step = new short[NO_OF_JOINTS];
			short step[NO_OF_JOINTS];
			// this is likely unneccessary.
			// clean up a new array
			for (byte jth = 0; jth < NO_OF_JOINTS; jth++) {
				step[jth] = 0;
			}
		//	Serial.println("ARM :: update(...) --> step cleared");
		//	Serial.flush();
			// find the width of the distance for each joint to be moved
			short max_distance = 0;
			for (byte jth = 0; jth < NO_OF_JOINTS; jth++) {
				step[jth] = abs(p_destination[jth] \
								- p_position[jth]);
				if (step[jth] > max_distance) {
					max_distance = step[jth];
				}
			}
		//	Serial.println("ARM :: update(...) --> disatnce found");
		//	Serial.flush();

			// calculate the step value for each other moving joint
			for (byte jth = 0; jth < NO_OF_JOINTS; jth++) {
				step[jth] = (step[jth] / (max_distance / 10));
		//		Serial.print("\t"); Serial.print(jth, DEC);
		//		Serial.print(") step: [");
		//		Serial.print(step[jth], DEC);
		//		Serial.println("]");
			}

			// lower the number of movement steps that will be taken
			max_distance /= 10;
			for (; max_distance > 0; max_distance--) {
				for (byte jth = 0; jth < NO_OF_JOINTS; jth++) {
					// adjust position in the correct direction.
					// if dest > position,
					//    position increases
					// else
					//    position descreses
					//
					// this is due to the arc of movement is absolute
					//    from 0 to 180
					//    or 544 to 2400
					if (p_position[jth] != p_destination[jth]) {
						p_position[jth] = p_destination[jth] > p_position[jth] ?
										  p_position[jth] += step[jth] :
										  p_position[jth] -= step[jth] ;
					// write this change to the servo.
					// this should be changed to p_put
						arm[jth].writeMicroseconds(p_position[jth]);
						delay(3);
					}
				}
			}

			// run through the joints one more time and directly put
			//    them to their final position in case any came up
			//    short after the above step cucles.
			for (byte joint = 0; joint < NO_OF_JOINTS; joint++) {
				arm[joint].writeMicroseconds(p_destination[joint]);
			}

		//	Serial.println("ARM :: update(...) --> leaving");
		//	Serial.flush();
		}

		// this is for the topic /arm/put/angle. it was handy during testing
		//    and proved to be marginally useful in production. call this
		//    function with the desired angles of all servos and we'll go
		//    there using the smooth update() function.
		void put_angle( unsigned short B, unsigned short S, unsigned short E, unsigned short WP, unsigned short WR, unsigned short H) {
		//	Serial.println("ARM --> put_angle :: entering");
			p_destination[BASE] = topulse(B);
			p_destination[SHOULDER] = topulse(S);
			p_destination[ELBOW] = topulse(E);
			p_destination[WRIST_P] = topulse(WP);
			p_destination[WRIST_R] = topulse(WR);
			p_destination[HAND] = topulse(H);

			update();
		}

		/* place at (x, y, z) - this is an inverse kinematic
		 *    equation that translates the (x, y, z) into
		 *    angualr measures from an origin defined at the
		 *    base of the arm.
		 *
		 * this function is borrowed from:
		 *    http://www.circuitsathome.com/mcu/robotic-arm-inverse-kinematics-on-arduino
		 * which, conincedentilelery is the same arm as the one we have
		 * */
		void put_point( float x, float y, float z, float wrist_pitch_d, float wrist_roll_d )
		{
		//	Serial.print("ARM :: put(xyz) --> (");
		//	Serial.print(x), Serial.print(", ");
		//	Serial.print(y), Serial.print(", ");
		//	Serial.print(z), Serial.print(") ");
		//	Serial.print(wrist_pitch_d); Serial.println();
			//grip angle in radians for use in calculations
			float wrist_pitch_r = radians( wrist_pitch_d );    
			/* Base angle and radial distance from x,y coordinates */
			float bas_angle_r = M_PI_2 - atan2( -x, y );
			float rdist = sqrt(( x * x ) + ( y * y ));
			/* rdist is y coordinate for the arm */
			y = rdist;
			/* Grip offsets calculated based on grip angle */
			float grip_off_z = ( sin( wrist_pitch_r )) * GRIPPER;
			float grip_off_y = ( cos( wrist_pitch_r )) * GRIPPER;
			/* Wrist position */
			float wrist_z = ( z - grip_off_z ) - BASE_HGT;
			float wrist_y = y - grip_off_y;
			/* Shoulder to wrist distance ( AKA sw ) */
			float s_w_sq = ( wrist_z * wrist_z ) + ( wrist_y * wrist_y );
			float s_w = sqrt( s_w_sq );
			/* s_w angle to ground */
			//float a1 = atan2( wrist_y, wrist_z );
			float a1 = atan2( wrist_z, wrist_y );
			/* s_w angle to humerus */
			float a2 = acos((( hum_sq - uln_sq ) + s_w_sq ) / ( 2 * HUMERUS * s_w ));
			/* shoulder angle */
			float shl_angle_r = a1 + a2;
			float shl_angle_d = degrees( shl_angle_r );
			/* elbow angle */
			float elb_angle_r = acos(( hum_sq + uln_sq - s_w_sq ) / ( 2 * HUMERUS * ULNA ));
			float elb_angle_d = degrees( elb_angle_r );
		//	float elb_angle_dn = -( 180.0 - elb_angle_d );
			/* wrist angle */
			// changed wri_angle_d to match math done at home
			float wri_angle_d = ( wrist_pitch_d + 90 + ( 180 - ( shl_angle_d + elb_angle_d ) ) );
		//	float wri_angle_d = ( wrist_pitch_d - elb_angle_d ) - shl_angle_d;

			// update the pulse_destination array and in preparation
			//    to call update()
			p_destination[BASE] = topulsef(degrees(bas_angle_r));
			p_destination[WRIST_P] = topulsef(wri_angle_d);
			p_destination[WRIST_R] = topulsef(wrist_roll_d);
			p_destination[SHOULDER] = topulsef(shl_angle_d);
			p_destination[ELBOW] = topulsef(elb_angle_d);

			update();
			// direct_update() is used in the newer move_line() method.
		//	direct_update();
		}

		struct point getxyz() {
			// this function should return the XYZ position of the arm.
			// do we worry about the roll of the wrist? shouldn't have to.
			//    the point of consideration lies along the wrist's roll axis.
		//	Serial.println("ARM :: getxyz() --> entering");
			double  cosB,  sinB;
			double hcosS, hsinS, sumofpiecesX;
			double ucosE, usinE, sumofpiecesY;
			double gcosW, gsinW, sumofpiecesZ;

		//	int B = arm[BASE].read();					double B_r = radians(B);
		//	int S = arm[SHOULDER].read();				double S_r = radians(S);
		//	int E = -(180 - S - arm[ELBOW].read());		double E_r = radians(E);
		//	int W = -(90 - arm[WRIST_P].read() - E);	double W_r = radians(W);
			float B = toanglef(arm[BASE].readMicroseconds());
				double B_r = radians(B);
			float S = toanglef(arm[SHOULDER].readMicroseconds());
				double S_r = radians(S);
			float E = -(180 - S - (toanglef(arm[ELBOW].readMicroseconds())));
				double E_r = radians(E);
			float W = -(90 - toanglef(arm[WRIST_P].readMicroseconds()) - E);
				double W_r = radians(W);

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

		//	Serial.print("\tresult:");
		//	Serial.print(" x: "); Serial.println(sumofpiecesX);
		//	Serial.print("\t\ty: "); Serial.println(sumofpiecesY);
		//	Serial.print("\t\tz: "); Serial.println(sumofpiecesZ);
			struct point pillow(-sumofpiecesX, sumofpiecesY, sumofpiecesZ);

			return(pillow);
		}

		float grasp(char tool) {
			switch(tool) {
				case 's':	// grap the square
					p_destination[HAND] = topulse(120);
					break;
				case 't':	// grab the triangle
					p_destination[HAND] = topulse(140);
					break;
				case 'c':	// grab the circle
					p_destination[HAND] = topulse(110);
					break;
			}
			// calling update isn't needed, but we'll do it for consistancy.
			update();
			return(p_position[HAND]);
		}

		void release() {
			// return the hand to the default open position.
			p_destination[HAND] = topulse(20);
			update();
		}

		// new implementation of movement. this solves an equation
		//    and increments along the longest difference between
		//    point_prev and point_next
		
		/** this method shows great promise, but currently doesn't work.
		 * what this does is:
		 * 
		 *    - the arm now keeps track of it's current position via
		 *      point_prev being updated everytime the arm moves.
		 *    - when given a new point find the length needed to move
		 *      along each axis.
		 *    - from among those 3 values, find the longest difference.
		 *      this will be our guide for solving our equations.
		 * 
		 * this was great in theory and in practice showed promise, but
		 *    i found that the initial value given does not match where
		 *    the arm thinks it's at. this led to crashes and confusion
		 *    when trying to move the arm.
		 * another issue is that the pitch and roll of the wrist is not
		 *    considered in this function. a more polished function will
		 *    adjust both .p and .r appropriately to continue smoothness.
		 * 
		 **/
		void put_point_line(wesley::arm_point point_next) {

			wesley::arm_point diff;
			diff.direct_mode = false;
			diff.cmd = "difference between two points.";

			// find the differences between where we are and where we
			//    want to end up.
			diff.x = point_next.x - point_prev.x;
			diff.y = point_next.y - point_prev.y;
			diff.z = point_next.z - point_prev.z;
			diff.p = point_next.p - point_prev.p;
			diff.r = point_next.r - point_prev.r;

			// from the differences of X, Y, and Z find the longest travel
			//    distance - this will be the guide for solving our functions.
			// we use the longest traversal so that we can have quantize the
			//    movement as small as possible. this allows for more updates
			//    along the line than the shorter traverals would allow for
			//    our quanta of movement.
			char guiding_light = 0;
			float longest_gap = 0;
			bool increasing = false;
			// floats don't allow normal absolute value. it was found that
			//    fabs(float) provided that functionality. (in math.h)
			// we'll check them all in turn. which ever is the longest
			//    will appear at the end of all the checks.
			if (fabs(diff.x) > longest_gap) {
				longest_gap = fabs(diff.x);
				guiding_light = 'x';
				increasing = (point_next.x > point_prev.x);
			}
			if (fabs(diff.y) > longest_gap) {
				longest_gap = fabs(diff.y);
				guiding_light = 'y';
				increasing = (point_next.y > point_prev.y);
			}
			if (fabs(diff.z) > longest_gap) {
				longest_gap = fabs(diff.z);
				guiding_light = 'z';
				increasing = (point_next.z > point_prev.z);
			}

			// if after all three of the above checks the gap is still 0
			//    it means that we were given the same point that we're
			//    already at; no need to move. the better function will
			//    have allowances for wrist pitch and roll here.
			if (longest_gap == 0) {
				return;
			} else {
				move_line(guiding_light, increasing, point_next);
			}
			
			/** it dawns on me now that another way to do this is to
			 *     have our point of consideration (tip) be at the end
			 *     of the wrist, not the end of the gripper. this would
			 *     allow movement of the hand to be completely independent
			 *     of the arm. extra care would need to be taken to make
			 *     sure that the hand doesn't bind or run into anything.
			 **/
		}

		/** this function solves a line between the previous point (our current one),
		 *     and the next one (given on /arm/put/point). the form of this eqaution is:
		 * 
		 *      X - x1       Y - y1       Z - z1       (note the capitals)
		 *     --------  =  --------  =  --------
		 *     x2 - x1      y2 - y1      y2 - y1
		 * 
		 *  here, we'll know one of the capitals and can then solve the other
		 *     two equalites with that value. for example, consider X is our
		 *     guiding light:
		 * 
		 *      X - x1
		 *     --------  =  a known value, k when X is provided.
		 *     x2 - x1 
		 * 
		 *      we then solve for the unknown Y and Z by setting the appropriate
		 *      term in the above equality to k
		 * 
		 *      Y - y1
		 *     --------  =  k  -->   k(y2 - y1) = Y - y1  -->  k(y2 - y1) + y1 = Y
		 *     y2 - y1
		 * 
		 *      Z - z1
		 *     --------  =  k  -->   k(z2 - z1) = Z - z1  -->  k(z2 - z1) + z1 = Z
		 *     z2 - z1
		 *
		 *  when the guiding light changes, we just solve for the other two unknowns
		 *     in a similar matter.
		 * 
		 *  after the two unknowns are solved we'll have our intermediate
		 *     point (X, Y, Z), where the guide is incremented by our quanta
		 *     of 1 mm (in the scale of the robot's reference frame)
		 * 
		 **/
		void move_line(char guide, bool increasing, wesley::arm_point& point_next) {
			// this function runs through a for loop while longest_gap != 0;
			// for each iteration it increases/decreases the guide by 1 mm
			//    and solves a 3-space line equation for the other two points
			//    and then called self.put_on_line(x, y, z, p, r) to move the
			//    arm into that direct point. this could call self.put_point

			float x = 0.0f;
			float y = 0.0f;
			float z = 0.0f;

			float k = 0.0f;

			switch (guide) {
				case 'x':
					for (x = point_prev.x; x != point_next.x;) {
						k = (x - point_prev.x) / (point_next.x - point_prev.x);
						y = (k * (point_next.y - point_prev.y)) + point_prev.y;
						z = (k * (point_next.z - point_prev.z)) + point_prev.z;
					
						put_point(x, y, z, point_next.p, point_next.r);
						if (increasing) {
							x += 1;
						} else {
							x -= 1;
						}
					}
					break;
				case 'y':
					for (y = point_prev.y; y != point_next.y;) {
						k = (y - point_prev.y) / (point_next.y - point_prev.y);
						x = (k * (point_next.x - point_prev.x)) + point_prev.x;
						z = (k * (point_next.z - point_prev.z)) + point_prev.z;
					
						put_point(x, y, z, point_next.p, point_next.r);
						if (increasing) {
							y += 1;
						} else {
							y -= 1;
						}
					}
					break;
				case 'z':
					for (z = point_prev.z; z != point_next.z;) {
						k = (z - point_prev.z) / (point_next.z - point_prev.z);
						x = (k * (point_next.x - point_prev.x)) + point_prev.x;
						y = (k * (point_next.y - point_prev.y)) + point_prev.y;
					
						put_point(x, y, z, point_next.p, point_next.r);
						if (increasing) {
							z += 1;
						} else {
							z -= 1;
						}
					}
					break;
				default:
					break;
			}
			// final put to catch the last point.
			put_point(point_next.x, point_next.y, point_next.z, point_next.p, point_next.r);
			point_prev = point_next;
		}
};
