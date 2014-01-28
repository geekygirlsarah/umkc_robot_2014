/* ARM_FLUID.H
 *
 * fluid arm movement from one position to the next
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

// MIN_ and MAX_PULSE_WIDTH are defined in <Servo.h>
#define topulse(a)		map((a*10), 0, 1800, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH)

/* pre-calculations */
float hum_sq = HUMERUS*HUMERUS;
float uln_sq = ULNA*ULNA;

class arm_control {
	private:
		byte* p_position;
		byte* p_destination;
		
		Servo* arm;
		byte no_of_joints;

		void p_store() {
			for (int joint = 0; joint < no_of_joints; joint++) {
				p_position[joint] = arm[joint].readMicroseconds();
			}
		}

	public:
		enum joints { BASE, SHOULDER, ELBOW, WRIST_P, WRIST_R, HAND };
		arm_control() {
			arm_control(6);		// default value of servos in the AL5D arm.
		}
		arm_control(const byte joints) {
			no_of_joints = joints;
			p_position = new byte[no_of_joints];
			p_destination = new byte[no_of_joints];

			arm = new Servo[no_of_joints];
		}
		~arm_control() {
			delete(arm);
			delete(p_position);
			delete(p_destination);
		}

		void connect(int argc, ...) {
			int pin;
			va_list argv;
			va_start(argv, argc);

			for (int joint = 0; joint < argc; joint++) {
				pin = va_arg(argv, int);
				arm[joint].attach(pin);
			}

			va_end(argv);
		}

		void begin() {
			arm[BASE].write(90);
			arm[SHOULDER].write(155);
			arm[ELBOW].write(150);
			arm[WRIST_P].write(10);
			arm[WRIST_R].write(95);
			arm[HAND].write(90);
			p_store();
		}
		

		void park() {
			/* comment out - not working
			p_destination[BASE] = 1472;
			p_destination[SHOULDER] = 2142;
			p_destination[ELBOW] = 2090;
			p_destination[WRIST_P] = 647;
			p_destination[WRIST_R] = 1523;
			p_destination[HAND] = 1472;
			// need to update this each time a physical joint is added.
			put(no_of_joints, BASE, SHOULDER, ELBOW, \
							  WRIST_P, WRIST_R, HAND);
			//*/
		}

		byte read(const byte joint) {
			return(arm[joint].read());
		}

		byte p_read(const byte joint) {
			return(arm[joint].readMicroseconds());
		}

		void put(const byte joint, short angle) {
			p_put(joint, topulse(angle));
			Serial.print("ARM :: put() --> got (");
			Serial.print(joint, DEC); Serial.print(", ");
			Serial.print(angle, DEC); Serial.print(");");
			Serial.print("\tpulse: (");
			Serial.print(topulse(angle)); Serial.println(")");
		}

		void p_put(const byte joint, short p_angle) {
			Serial.print("ARM :: p_put() --> putting joint [");
			Serial.print(joint, DEC); Serial.print("] to [");
			Serial.print(p_angle, DEC); Serial.println("]");
			arm[joint].writeMicroseconds(p_angle);
			p_position[joint] = arm[joint].readMicroseconds();
		}

};
