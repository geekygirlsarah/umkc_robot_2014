/* ARM_CONTROL
 * written by: Eric M Gonzalez
 * date: 13-01-14
 *
 * PURPOSE: This class defines and describes the control interface
 *          to the AL5D arm.
 *
 * move_to function lifted from:
 * http://www.circuitsathome.com/mcu/robotic-arm-inverse-kinematics-on-arduino
 * and modified to fit this class
 *
 *
 * Servos work on the premise of angles from 0 to 180.
 * For the purposes of this code, CENTER is defiend as 90.
 *
 * Inverse Kinematic equations can be based on vectors and
 *    trigonometry.
 *
 *    |
 *    |
 *    |         ___________
 *    |        / α       γ \              alpha,
 *    |       /             \             gamma,
 *    |      /               \
 *    |     /                 \
 *    |    /
 *    |   /
 *    |  /
 *    | /  θ                              theta
 *    |/_____________________________
 *
 * the lengths between each axis/axel must be measured.
 */

#include <Servo.h>


#define BASE_HGT 47.625     //base hight 1 7/8"
#define HUMERUS 146.05      //shoulder-to-elbow 5 3/4"
#define ULNA 187.325        //elbow-to-wrist 7 3/8"
#define GRIPPER 8.89        //gripper length 3 1/2"

/* pre-calculations */
float hum_sq = HUMERUS*HUMERUS;
float uln_sq = ULNA*ULNA;

#define ftl(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))  //float to long conversion

class arm_control {
	private:
		Servo* arm;
		
		void attach_joint(const byte joint, const byte pin) {
			arm[joint].attach(pin);
		}

		void position(const byte joint, const byte angle) {
			arm[joint].write(angle);
		}

	public:
		enum joints { BASE, SHOULDER, ELBOW, WRIST_P, WRIST_R, HAND };
		arm_control() {
			arm = new Servo[6];
		};
		arm_control(const byte joints) {
			arm = new Servo[joints];
		}

		~arm_control() {
			delete(arm);
		}

		// connect(4, JOINT, JOINT, JOINT)
		// arm.connect(ELBOW)

		void connect( const byte joints, ...) {
			va_list args;
			va_start(args, joints);
			byte pin = 0;
			for (byte jth = 0; jth < joints; jth++) {
				pin = va_arg(args, byte);
				attach_joint(jth, pin);
			}
			va_end(args);
		}

		/* comment for testing */
		void connect( const byte b_jt, 		// base
					  const byte s_jt, 		// shoulder
					  const byte e_jt, 		// elbow
					  const byte p_jt, 		// wrist
					  const byte r_jt, 		// wrist
					  const byte h_jt ) {	// hand
			attach_joint(BASE,     b_jt);
			attach_joint(SHOULDER, s_jt);
			attach_joint(ELBOW,    e_jt);
			attach_joint(WRIST_P,  p_jt);
			attach_joint(WRIST_R,  r_jt);
			attach_joint(HAND,     h_jt);
		}	//*/

		void disconnect() {
			for (byte joint = BASE; joint <= HAND; joint++) {
				arm[joint].detach();
			}
		}

		void swivel(const byte angle) {
			arm[BASE].write(angle);
		//	Servo::refresh();
		}

		void put(const byte joint, const byte angle) {
			arm[joint].write(angle);
		}

		// return the arm to the parked position.
		// adjust these angles to change the park position.
		void park() {
			put(BASE, 90);
			put(SHOULDER, 167);
			put(ELBOW, 160);
			put(WRIST_P, 45);
			put(WRIST_R, 90);
			put(HAND, 90);
		}

		void move_to( float x, float y, float z, float grip_angle_d )
		{
			float grip_angle_r = radians( grip_angle_d );    //grip angle in radians for use in calculations
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

			/* Set Servos, using arm* */
			put(BASE, degrees(bas_angle_r) );
			put(WRIST_P, wri_angle_d );
			put(SHOULDER, shl_angle_d );
			put(ELBOW, elb_angle_d );
			//*/
			
			/* write out angle information to console */
			Serial.print("set arm to:");
			Serial.print(" base: "); Serial.print( degrees(bas_angle_r) );
			Serial.print(" shou: "); Serial.print( shl_angle_d );
			Serial.print(" elbo: "); Serial.print( elb_angle_d );
			Serial.print(" wriP: "); Serial.print( wri_angle_d );
			Serial.println();

			/* Set servos 
			servos.setposition( BAS_SERVO, ftl( bas_servopulse ));
			servos.setposition( WRI_SERVO, ftl( wri_servopulse ));
			servos.setposition( SHL_SERVO, ftl( shl_servopulse ));
			servos.setposition( ELB_SERVO, ftl( elb_servopulse ));
			//*/
		}
};
