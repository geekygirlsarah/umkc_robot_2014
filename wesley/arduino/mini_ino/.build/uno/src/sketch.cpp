#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
void setup();
void loop();
#line 1 "src/sketch.ino"
//#include <ros.h>
//#include <geometry_msgs/Point.h>
//#include <std_msgs/Bool.h>

using namespace ros;

const byte BTN =  7;
const byte RED =  8;
const byte YLW =  9;
const byte GRN = 10;

void display_status(const geometry_msgs::Point& status) {
	digitalWrite(RED, status.x);
	digitalWrite(YLW, status.y);
	digitalWrite(GRN, status.z);
}

NodeHandle nh;
std_msgs::Bool running_state;

Publisher                        pub("/master/button", &running_state);
Subscriber<geometry_msgs::Point> sub("/master/leds",   &display_status);

void setup() {
	// status display LEDs
	pinMode(RED, OUTPUT);
	{
		digitalWrite(RED, HIGH);
		digitalWrite(YLW, LOW);
		digitalWrite(GRN, LOW);
	}
	pinMode(YLW, OUTPUT);
	pinMode(GRN, OUTPUT);

	// button and pull-up resistor
	pinMode(BTN, INPUT);
	digitalWrite(BTN, HIGH);

	nh.getHardware()->setBaud(19200);
	nh.initNode();
	nh.advertise(pub);
	nh.subscribe(sub);

	running_state.data = false;
	{
		delay(200);
		digitalWrite(RED, LOW);
		digitalWrite(YLW, HIGH);
		digitalWrite(GRN, LOW);
	}
}

bool published = false;

void loop() {
	if (!published) {
		if (running_state.data == false) {
			running_state.data = !digitalRead(BTN);
		} else {
			pub.publish(&running_state);
			{
				digitalWrite(RED, LOW);
				digitalWrite(YLW, LOW);
				digitalWrite(GRN, HIGH);
			}
			published = true;
		}
	}

	// once the button has been pressed, we can break out and
	//    just listen for status LED updates
	nh.spinOnce();
	delay(5);

}
