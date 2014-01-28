/* ARM_CONTROL
 * written by: Eric M Gonzalez
 * date: 18-01-2014
 *
 * PURPOSE: Define and describe a control interface to the robotic arm.
 *
 * Recommended viewing at 100 coloumns
 *
 */

#include <Servo.h>
#include <stdio.h>

#define BASE_HGT 69.85      // base hight 2 3/4"
#define HUMERUS 146.05      // shoulder-to-elbow 5 3/4"
#define ULNA 187.325        // elbow-to-wrist 7 3/8"
#define GRIPPER 88.9        // gripper length 3 1/2"
                            // this gripper measure is to the outside
                            // screw-hole, not the tip of the hand.

/* pre-calculations */
float hum_sq = HUMERUS*HUMERUS;
float uln_sq = ULNA*ULNA;

class arm_control {
	private:
		byte* position;
		byte* destination;

		Servo* arm;
		byte no_of_joints;

		short pulse_width(short angle) {
			return(map(angle, 0, 1800, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
		}

		void store() {
			for (int joint = 0; joint < no_of_joints; joint++) {
				position[joint] = arm[joint].read();
			//	Serial.print("joint: "); Serial.print(joint, DEC);
			//	Serial.print(" position thought: ");
			//	Serial.print(position[joint]);
			//	Serial.print(" position actual: ");
			//	Serial.print(arm[joint].read());
			//	Serial.println();
			}
		}
	
	public:
		enum joints { BASE, SHOULDER, ELBOW, WRIST_P, WRIST_R, HAND };
		arm_control() {
			// default number of servo-joints in AL5D
			no_of_joints = 6;
			position = new byte[no_of_joints];
			destination = new byte[no_of_joints];

			arm = new Servo[no_of_joints];
		//	store();

			//arm_control(6) ?? just call another constructor?
		}
		arm_control(const byte joints) {
			no_of_joints = joints;
			position = new byte[no_of_joints];
			destination = new byte[no_of_joints];

			arm = new Servo[no_of_joints];
		//	store();
		}

		~arm_control() {
			delete(arm);
			delete(position);
			delete(destination);
		}

		void connect(int argc, ...) {
			int pin;
			va_list argv;
			va_start(argv, argc);

			for (int joint = 0; joint < argc; joint++) {
				pin = va_arg(argv, int);
			//	Serial.print(joint, DEC); Serial.print(" : ");
			//	Serial.println(pin, DEC);
				arm[joint].attach(pin);
			}

			va_end(argv);
		}

		byte read(const byte joint) {
			return(arm[joint].read());
		}

		void begin() {
			arm[BASE].write(80);
			arm[SHOULDER].write(160);
			arm[ELBOW].write(165);
			arm[WRIST_P].write(0);
			arm[WRIST_R].write(95);
			arm[HAND].write(90);
			store();
		}

		void park() {
			/* comment out - not working */
			destination[BASE] = 80;
			destination[SHOULDER] = 160;
			destination[ELBOW] = 165;
			destination[WRIST_P] = 0;
			destination[WRIST_R] = 95;
			destination[HAND] = 90;
			// need to update this each time a physical joint is added.
			put(no_of_joints, BASE, SHOULDER, ELBOW, \
							  WRIST_P, WRIST_R, HAND);
			//*/
		}
			

		void put(const byte joint, const byte angle) {
//			Serial.print("ARM :: called direct: ");
//			for (int tab = 0; tab < joint; tab++) {
//				Serial.print("\t\t");
//			}
//			Serial.print("put(");
//			Serial.print(joint, DEC);
//			Serial.print(", ");
//			Serial.print(angle);
//			Serial.println(")");
			arm[joint].write(angle);
		//	position[joint] = angle;
			// as opposed to..
			position[joint] = arm[joint].read();
		}

		void put(const byte joint, const float angle) {
			arm[joint].writeMicroseconds(pulse_width(angle * 10));
			position[joint] = arm[joint].read();
		}

		void put(byte argc, ...) {
//			Serial.println("ARM :: called put(...)");
//			Serial.flush();
			Serial.println("ARM :: put(...) --> going to:");
			Serial.print("\t"); Serial.print(destination[BASE], DEC);
			Serial.print(", "); Serial.print(destination[SHOULDER], DEC);
			Serial.print(", "); Serial.print(destination[ELBOW], DEC);
			Serial.print(", "); Serial.print(destination[WRIST_P], DEC);
			Serial.println();

			va_list argv;
			va_start(argv, argc);

			byte* arm_queue = new byte[argc];
			for (int jth = 0; jth < argc; jth++) {
				arm_queue[jth] = va_arg(argv, int);
			}
			byte max_distance = 0;
			for (byte jth = 0; jth < argc; jth++) {
				byte distance = destination[arm_queue[jth]] >= position[arm_queue[jth]] \
								? (destination[arm_queue[jth]] - position[arm_queue[jth]]) \
								: (position[arm_queue[jth]] - destination[arm_queue[jth]]);
				if (distance > max_distance) {
					max_distance = distance;
				}
			}
			
			short step[argc];
			for (int jth = 0; jth < argc; jth++) {
				// find the distance between where the joint is now and where it's going
				step[jth] = abs(destination[arm_queue[jth]] - position[arm_queue[jth]]);
				// divide out the max distance to find the width of a matching step
				step[jth] = (step[jth] * 10 ) / max_distance;
				// multiply by ten to increase the magnitude of the step, but
				//    retain it's ability to fit in an integer.
				Serial.print(step[jth], DEC); Serial.print(", ");
			}
			Serial.println();


			// divide out each distance to be moved by the number of steps
			//    required to move the longest
			// cycle through quanta of max_distance
			// ONLY AMONG THE JOINTS THAT WILL BE MOVED
			for (; max_distance > 0; max_distance--) {
				for (byte jth = 0; jth < argc; jth++) {
					if (destination[arm_queue[jth]] != position[arm_queue[jth]]) {
						destination[arm_queue[jth]] > position[arm_queue[jth]] \
							? position[arm_queue[jth]] += 1 \
							: position[arm_queue[jth]] -= 1;
						put(jth, position[jth]);
					//	destination[arm_queue[jth]] > position[arm_queue[jth]] \
					//		? p_put(arm_queue[jth],  step[jth]) \
					//		: p_put(arm_queue[jth], -step[jth]) ;
						delay(18);
					}
				}
			}


			delete (arm_queue);
			Serial.println("ARM :: leaving put(...)");
		}

		void p_put(const byte joint, const float step) {
			short angle_p = short( ( position[joint] + step ) * 10 );
			Serial.print("ARM :: p_put --> got (");
			Serial.print(joint, DEC); Serial.print(", ");
			Serial.print(angle_p, DEC); Serial.println(")");
			angle_p = map (angle_p, 0, 1800, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
			arm[joint].writeMicroseconds(angle_p);
			position[joint] = arm[joint].read();
		}

		/* The following movement functions were found on the internet.
		 * Original author: Oleg Mazurov
		 * LINK: http://www.circuitsathome.com/mcu/robotic-arm-inverse-kinematics-on-arduino
		 *
		 * An extended explanation can be found at the above link.
		 * In summary, the functions do:
		 *    put(x, y, z, a) : move the arm directly to a prescribed
		 *       Cartesian co-ordinate in a 3D-field with the origin
		 *       at the base of the robot.
		 *    zero_x() - moves arm along the y-axis
		 *    line() - moves arm along the x-axis
		 *    circle() - a circle in the y-z plane
		 */
		void put( float x, float y, float z, float grip_angle_d )
		{
			Serial.print("ARM :: put(xyz) --> (");
			Serial.print(x), Serial.print(", ");
			Serial.print(y), Serial.print(", ");
			Serial.print(z), Serial.print(") ");
			Serial.print(grip_angle_d); Serial.println();
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
			float elb_angle_dn = -( 180.0 - elb_angle_d );
			/* wrist angle */
			float wri_angle_d = ( grip_angle_d - elb_angle_dn ) - shl_angle_d;

			/* Servo pulses */
			float bas_servopulse = 1500.0 - (( degrees( bas_angle_r )) * 11.11 );
			float shl_servopulse = 1500.0 + (( shl_angle_d - 90.0 ) * 6.6 );
			float elb_servopulse = 1500.0 -  (( elb_angle_d - 90.0 ) * 6.6 );
			float wri_servopulse = 1500 + ( wri_angle_d  * 11.1 );

			/* Set Servos, using arm*
			put(BASE, degrees(bas_angle_r) );
			put(WRIST_P, wri_angle_d );
			put(SHOULDER, shl_angle_d );
			put(ELBOW, elb_angle_d );
			//*/

			destination[BASE] = degrees(bas_angle_r);
			destination[WRIST_P] = wri_angle_d;
			destination[SHOULDER] = shl_angle_d;
			destination[ELBOW] = elb_angle_d;

		//	put (4, BASE, WRIST_P, SHOULDER, ELBOW);
			put (4, BASE, SHOULDER, ELBOW, WRIST_P);
			
		}

		void zero_x() {
			for( double yaxis = 150.0; yaxis < 356.0; yaxis += 1 ) {
				put( 0, yaxis, 127.0, 0 );
				delay( 10 );
			}
			for( double yaxis = 356.0; yaxis > 150.0; yaxis -= 1 ) {
				put( 0, yaxis, 127.0, 0 );
			    delay( 10 );
			  }
		}
 
		/* moves arm in a straight line */
		void line() {
			for( double xaxis = -100.0; xaxis < 100.0; xaxis += 0.5 ) {
				put( xaxis, 250, 100, 0 );
				delay( 10 );
			}
			for( float xaxis = 100.0; xaxis > -100.0; xaxis -= 0.5 ) {
				put( xaxis, 250, 100, 0 );
				delay( 10 );
		    }
		}

		void circle() {
			#define RADIUS 80.0
			//float angle = 0;
			float zaxis,yaxis;
			for( float angle = 0.0; angle < 360.0; angle += 1.0 ) {
				yaxis = RADIUS * sin( radians( angle )) + 200;
				zaxis = RADIUS * cos( radians( angle )) + 200;
				put( 0, yaxis, zaxis, 0 );
				delay( 1 );
			}
		}

		void get(byte* angles, const byte size) {
			for (byte joint = 0; joint < size; joint++) {
				angles[joint] = arm[joint].read();
			}
		}

		double polar_distance(byte* angle) {
			double x_comp[4];
			double y_comp[4];
			byte angle_actual = 0;

			// position at the SHOULDER
			x_comp[0] = 0; y_comp[0] = 0;

			// position at the ELBOW
			angle_actual = arm[SHOULDER].read();
			x_comp[1] = HUMERUS * cos(radians(angle_actual));
			y_comp[1] = HUMERUS * sin(radians(angle_actual));

			// position at the WRIST
			angle_actual = (angle_actual - arm[ELBOW].read());
			x_comp[2] = ULNA * cos(radians(angle_actual));
			y_comp[2] = ULNA * sin(radians(angle_actual));

			// position at the GRIPPER POINT (outward screw)
			angle_actual = (angle_actual + (arm[WRIST_P].read() - 90));
			x_comp[3] = GRIPPER * cos(radians(angle_actual));
			y_comp[3] = GRIPPER * sin(radians(angle_actual));

			double y = y_comp[1] + y_comp[2] + y_comp[3];
			return(y);
		}

	// END OF CLASS (arm_control);
};
