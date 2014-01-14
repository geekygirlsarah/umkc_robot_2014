/* ARM_CONTROL
 * written by: Eric M Gonzalez
 * date: 13-01-14
 *
 */

#include <Servo.h>

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
		/*
		const byte BASE     = 0;
		const byte SHOULDER = 1;
		const byte ELBOW    = 2;
		const byte WRIST_P  = 3;
		const byte WRIST_R  = 4;
		const byte HAND     = 5;
		*/
		enum parts { BASE, SHOULDER, ELBOW, WRIST_P, WRIST_R, HAND };
		arm_control() {
			arm = new Servo[6];
		};
		arm_control(const byte joints) {
			arm = new Servo[joints];
		}

		~arm_control() {
			delete(arm);
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

		/* comment off */
		void park() {
			put(BASE, 90);
			put(SHOULDER, 167);
			put(ELBOW, 160);
			put(WRIST_P, 45);
			put(WRIST_R, 90);
			put(HAND, 90);
		} //*/
};
