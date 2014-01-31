#include "arm_control.h"

#define console Serial

const byte NO_OF_JOINTS = 6;
arm_control arm(NO_OF_JOINTS);

void setup() {
	console.begin(9600);
	arm.connect(NO_OF_JOINTS, 2, 3, 4, 6, 7, 8);
	arm.initial_park();
}

byte base;
byte shoulder;
byte elbow;
byte wrist_p;
byte wrist_r;
byte hand;

void loop() {
	base = arm.read(arm.BASE);
	shoulder = arm.read(arm.SHOULDER);
	elbow = arm.read(arm.ELBOW);
	wrist_p = arm.read(arm.WRIST_P);
	wrist_r = arm.read(arm.WRIST_R);
	hand = arm.read(arm.HAND);
	
	if (console.available() > 0) {
		byte cmd = console.read();
		switch(cmd) {
			// inverse kinematic place ment. might need work
			case 'k':
				arm.move_to(random(0,180), random(0, 180),
						random(0,180), random(0, 180));
				break;

			// 'carry' position
			case 'c':
				arm.carry();
				break;
			// park position
			case 'p':
				arm.park();
				break;

			// base
			case 'a':
				base -= 1;
				arm.put(arm.BASE, base);
				break;
			case 'd':
				base += 1;
				arm.put(arm.BASE, base);
				break;

			// shoulder
			case 'w':
				shoulder -= 1;
				arm.put(arm.SHOULDER, shoulder);
				break;
			case 's':
				shoulder += 1;
				arm.put(arm.SHOULDER, shoulder);
				break;
			
			// elbow
			case 'r':
				elbow += 1;
				arm.put(arm.ELBOW, elbow);
				break;
			case 'f':
				elbow -= 1;
				arm.put(arm.ELBOW, elbow);
				break;

			// wrist pitch
			case 't':
				wrist_p += 1;
				arm.put(arm.WRIST_P, wrist_p);
				break;
			case 'g':
				wrist_p -= 1;
				arm.put(arm.WRIST_P, wrist_p);
				break;

			// wrist roll
			case 'q':
				wrist_r += 1;
				arm.put(arm.WRIST_R, wrist_r);
				break;
			case 'e':
				wrist_r -= 1;
				arm.put(arm.WRIST_R, wrist_r);
				break;

			// hand open/close
			case 'Q':
				hand -= 1;
				arm.put(arm.HAND, hand);
				break;
			case 'E':
				hand += 1;
				arm.put(arm.HAND, hand);
				break;

			// double check position with expected and actual
			case '?':
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
				break;

			deafult:
				break;
		}
	}
}
