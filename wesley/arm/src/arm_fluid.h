/* ARM_FLUID.H
 *
 * fluid arm movement from one position to the next
 *
 */

#include <Arduino.h>
#include <Servo.h>
#include <stdio.h>

// These are in millimeters
#define BASE_HGT 69.85      // base hight 2 3/4"
#define HUMERUS 146.05      // shoulder-to-elbow 5 3/4"
#define ULNA 215.9          // elbow-to-wrist 8 1/2"
#define GRIPPER 88.9        // gripper length 3 1/2"
                            // this gripper measure is to the outside
                            // screw-hole, not the tip of the hand.

// MIN_ and MAX_PULSE_WIDTH are defined in <Servo.h>
#define topulse(a)		map((a*10), 0, 1800, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH)
#define f_toangle(p)	map((p   ), MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, 1800) / 10
#define toangle(p)		map((p+1 ), MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, 180)

/* pre-calculations */
float hum_sq = HUMERUS*HUMERUS;
float uln_sq = ULNA*ULNA;

class arm_control {
	private:
		short* p_position;
		short* p_destination;
		
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
			no_of_joints = 6;
			p_position = new short[no_of_joints];
			p_destination = new short[no_of_joints];

			arm = new Servo[no_of_joints];

		//	arm_control(6);		// default value of servos in the AL5D arm.
		}
		arm_control(const byte joints) {
			no_of_joints = joints;
			p_position = new short[no_of_joints];
			p_destination = new short[no_of_joints];

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
			arm[BASE].write(80);
			arm[SHOULDER].write(160);
			arm[ELBOW].write(165);
			arm[WRIST_P].write(0);
			arm[WRIST_R].write(95);
			arm[HAND].write(90);
			p_store();
		}
		

		void park() {
			/* comment out - not working */
			p_destination[BASE] = topulse(80); // = 1472;
			p_destination[SHOULDER] = topulse(160); //= 2142;
			p_destination[ELBOW] = topulse(165); // = 2090;
			p_destination[WRIST_P] = topulse(0); // = 647;
			p_destination[WRIST_R] = topulse(95); // = 1523;
			p_destination[HAND] = topulse(90); // = 1472;
			// need to update this each time a physical joint is added.
			put(no_of_joints, BASE, SHOULDER, ELBOW, \
							  WRIST_P, WRIST_R, HAND);
			//*/
		}

		void put(const byte argc, ...) {
			Serial.println("ARM :: put(...) --> entering:");
			va_list argv;
			va_start(argv, argc);

			byte* arm_queue = new byte[argc];
			for (byte jth = 0; jth < argc; jth++) {
				arm_queue[jth] = va_arg(argv, int);
				Serial.print(arm_queue[jth], DEC); Serial.print("\t");
			}
			Serial.println();

			short max_distance = 0;
			for (byte jth = 0; jth < argc; jth++) {
				short distance = abs(p_destination[arm_queue[jth]] - p_position[arm_queue[jth]]);
				Serial.print(jth, DEC); Serial.print(":\t");
				Serial.print(p_position[arm_queue[jth]], DEC); Serial.print(" ->\t");
				Serial.print(p_destination[arm_queue[jth]], DEC); Serial.print(" =\t");
				Serial.print(distance, DEC); Serial.println();
				if (distance > max_distance) {
					max_distance = distance;
				}
			}
			Serial.print("ARM :: put(...) --> max_distance: (");
			Serial.print(max_distance, DEC); Serial.println(")");

			short* step = new short[argc];
			for (byte ith = 0; ith < argc; ith++) {
				step[ith] = 0;
			}
			for (byte jth = 0; jth < argc; jth++) {
				step[jth] = abs(p_destination[arm_queue[jth]] - p_position[arm_queue[jth]]);
				if (step[jth] > 0) {
					// number of steps at 10 pulses per max_click
					step[jth] = step[jth] / 20;
					step[jth] = max_distance / step[jth];
				} else {
					step[jth] = 0;
				}
				Serial.print("ARM :: put(...) --> step("); Serial.print(jth, DEC); Serial.print("): at [");
				Serial.print(step[jth], DEC); Serial.println("]");
			}
			
			max_distance = max_distance / 5;
			for (; max_distance > 0; max_distance--) {
				for (byte jth = 0; jth < argc; jth++) {
					p_position[arm_queue[jth]] = p_destination[arm_queue[jth]] > p_position[arm_queue[jth]] ?
													p_position[arm_queue[jth]] + step[jth] :
													p_position[arm_queue[jth]] - step[jth] ;
					arm[arm_queue[jth]].writeMicroseconds(p_position[arm_queue[jth]]);
					delay(20);
				}
			}
			
			for (byte joint = 0; joint < no_of_joints; joint++) {
				arm[joint].writeMicroseconds(p_position[joint]);
			}

			delete(arm_queue);
			delete(step);
			Serial.println("ARM :: put(...) --> leaving");
		}

		byte read(const byte joint) {
			return(arm[joint].read());
		}

		byte p_read(const byte joint) {
			return(arm[joint].readMicroseconds());
		}

		void put(const byte joint, short angle) {
			Serial.print("ARM :: put() --> got (");
			Serial.print(joint, DEC); Serial.print(", ");
			Serial.print(angle, DEC); Serial.print(");");
			Serial.print("\tpulse: (");
			Serial.print(topulse(angle)); Serial.println(")");
			p_put(joint, topulse(angle));
		}

		void p_put(const byte joint, short p_angle) {
			Serial.print("ARM :: p_put() --> putting joint [");
			Serial.print(joint, DEC); Serial.print("] to [");
			Serial.print(p_angle, DEC); Serial.print("]");
			arm[joint].writeMicroseconds(p_angle);
			p_position[joint] = arm[joint].readMicroseconds();
			Serial.print("response: <"); Serial.print(p_position[joint], DEC);
			Serial.println();
		}

};
