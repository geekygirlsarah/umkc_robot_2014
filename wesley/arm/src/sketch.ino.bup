#include "arm_control.h"
#define console Serial

arm_control arm;

void setup()
{
	console.begin(9600);
	arm.connect(2, 3, 4, 6, 7, 8);
	arm.park();
	pinMode(9, INPUT);
}

byte base;
byte shoulder;
byte elbow;
byte wrist_p;
byte wrist_r;
byte hand;
byte pin;

void loop()
{
	/* commands:
	 * a - swivel left
	 * d - swivel right
	 */
	base = arm.get(arm.BASE);
	shoulder = arm.get(arm.SHOULDER);
	elbow = arm.get(arm.ELBOW);
	wrist_p = arm.get(arm.WRIST_P);
	wrist_r = arm.get(arm.WRIST_R);
	hand = arm.get(arm.HAND);
	if (console.available() > 0) {
		byte cmd = console.read();
		switch(cmd) {
			case 'p':
				arm.park();
				break;
			case 'k':
				arm.put(random(0, 180), random(0, 180), random(0, 180), random(0, 90));
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
		Serial.print(" base: "); Serial.print(base);
		Serial.print(" shou: "); Serial.print(shoulder);
		Serial.print(" elbo: "); Serial.print(elbow);
		Serial.print(" wriP: "); Serial.print(wrist_p);
		Serial.print(" wriR: "); Serial.print(wrist_r);
		Serial.println();
	}
}
