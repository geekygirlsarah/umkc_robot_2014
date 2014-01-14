#include "arm_control.h"
#define console Serial

arm_control arm;

void setup()
{
	console.begin(9600);
	arm.connect(2, 3, 4, 6, 7, 8);
	arm.park();
}

	byte base = 90;
	byte shoulder = 167;
	byte elbow = 160;
	byte wrist_p = 45;
	byte wrist_r = 90;
	byte hand = 90;

void loop()
{
	/* commands:
	 * a - swivel left
	 * d - swivel right
	 */

	if (console.available() > 0) {
		byte cmd = console.read();
		switch(cmd) {
			case 'p':
				arm.park();
				base = 90;
				shoulder = 167;
				elbow = 160;
				wrist_p = 45;
				wrist_r = 90;
				hand = 90;
				break;

			case 'a':
				base -= 5;
				arm.put(arm.BASE, base);
				break;
			case 'd':
				base += 5;
				arm.put(arm.BASE, base);
				break;
			
			case 'w':
				shoulder -=5;
				arm.put(arm.SHOULDER, shoulder);
				break;
			case 's':
				shoulder +=5;
				arm.put(arm.SHOULDER, shoulder);
				break;

			case 'r':
				elbow -= 5;
				arm.put(arm.ELBOW, elbow);
				break;
			case 'f':
				elbow += 5;
				arm.put(arm.ELBOW, elbow);
				break;

			case 'q':
				wrist_r -= 5;
				arm.put(arm.WRIST_R, wrist_r);
				break;
			case 'e':
				wrist_r += 5;
				arm.put(arm.WRIST_R, wrist_r);
				break;
			case 't':
				wrist_p += 5;
				arm.put(arm.WRIST_P, wrist_p);
				break;
			case 'g':
				wrist_p -= 5;
				arm.put(arm.WRIST_P, wrist_p);
				break;

			default:
				break;
		}
	}
}
