#include "arm_control.h"
//#include "arm_fluid.h"
#define console Serial

const byte NO_OF_JOINTS = 6;

arm_control arm;

void setup() {
	console.begin(9600);
//	console.println("init - preparing arm"); console.flush();
	arm.connect(NO_OF_JOINTS, 2, 3, 4, 6, 7, 8);
	arm.begin();
}

byte base;
byte shoulder;
byte elbow;
byte wrist_p;
byte wrist_r;
byte hand;
byte pin;

void loop() {
	base = arm.read(arm.BASE);
	shoulder = arm.read(arm.SHOULDER);
	elbow = arm.read(arm.ELBOW);
	wrist_p = arm.read(arm.WRIST_P);
	wrist_r = arm.read(arm.WRIST_R);
	hand = arm.read(arm.HAND);
	if(console.available() > 0) {
		byte cmd = console.read();
		switch(cmd) {
			case 'p':
				arm.park();
				break;
			case 'k':
				arm.put(random(0, 180), random(0, 180), random(0, 180), random(0, 90));
				break;

			/* these don't work like you think.
			case 'c':
				arm.circle();
				break;
			case 'l':
				arm.line();
				break;
			case 'x':
				arm.zero_x();
				break;
			//*/

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
			case 'Q':
				hand -= 5;
				arm.put(arm.HAND, hand);
				break;
			case 'E':
				hand += 5;
				arm.put(arm.HAND, hand);
				break;

			case '?': {
				byte* pillow = new byte[4];
				arm.get(pillow, 4);
				console.println("MAIN :: get angles -->");
				console.print("\t");
				for (int ith = 0; ith < 4; ith++) {
					console.print(pillow[ith], DEC);
					console.print(", ");
				}
				console.println();
				delete(pillow);
			} // a bit confusing, but the 'new' needs to be in
			  //    a protected scope, hence the brackets.
				break;

			case 'z': {
				byte polar_angle;
				console.print("MAIN :: polar_distance --> ");
				console.print(arm.polar_distance(&polar_angle));
				console.println();
			}
				break;
			default:
				break;
		}
	}
}
