#include <Arduino.h>
#include <ros.h>					// basic ROS objects
#include <geometry_msgs/Point.h>	// a pre-generated 3 number message
#include <std_msgs/Bool.h>			// button: true / false
void setup();
void loop();
#line 1 "src/sketch.ino"
//#include <ros.h>					// basic ROS objects
//#include <geometry_msgs/Point.h>	// a pre-generated 3 number message
//#include <std_msgs/Bool.h>			// button: true / false

// majority of code lives in the ROS namespace
using namespace ros;

// set pins to easy-to-read names
const byte BTN =  7;
const byte RED =  8;
const byte YLW =  9;
const byte GRN = 10;

// subscriber call back. as soon as mini has latched into ROS, this topic
//    is available to be written to. receive a 3 number message and set
//    the status of individual pins based on what the status is.
//
// WESLEY has a 6 LED status block, and so we'll need a custom message.
//    or, just a byte message and then we can flag out the bits to find
//    out which LEDs need lit.
void display_status(const geometry_msgs::Point& status) {
	digitalWrite(RED, status.x);
	digitalWrite(YLW, status.y);
	digitalWrite(GRN, status.z);
}

// create the node handle for this arduino.
NodeHandle nh;

// keep track of running_state : this is altered by a button press.
std_msgs::Bool running_state;

Publisher                        pub("/master/button", &running_state);
Subscriber<geometry_msgs::Point> sub("/master/leds",   &display_status);

void setup() {
	// status display LEDs
	pinMode(RED, OUTPUT);
	pinMode(YLW, OUTPUT);
	pinMode(GRN, OUTPUT);
	{
		digitalWrite(RED, HIGH);
		digitalWrite(YLW, LOW);
		digitalWrite(GRN, LOW);
	}

	// button and pull-up resistor
	pinMode(BTN, INPUT);
	digitalWrite(BTN, HIGH);

	// through experiments, 19200 was discovered to be the slowest
	//    baud-rate that allowed this all to work. never sure why.
	nh.getHardware()->setBaud(19200);

	// initialize the node and attach the publication and subscription
	nh.initNode();
	nh.advertise(pub);
	nh.subscribe(sub);

	// set initial values of check.
	running_state.data = false;
	{
		delay(200);
		digitalWrite(RED, LOW);
		digitalWrite(YLW, HIGH);
		digitalWrite(GRN, LOW);
	}
}

// a check to see if we've published our message yet.
// this should be doable with only one boolean. my
//    logic is off somwhere.
//
// correctness now, completeness later.
bool published = false;

// main loop.
void loop() {
	// 1) if we haven't published our message
	if (!published) {
	// 2) if the previous button state was false
		if (running_state.data == false) {
	// 3) read the button again
			running_state.data = !digitalRead(BTN);
	// 4) else, if the previous button state was true
		} else {
	// 5) publish the state of our button
			pub.publish(&running_state);
			{
	// 6) TEMPORARY - the LEDs would be set by another node
				digitalWrite(RED, LOW);
				digitalWrite(YLW, LOW);
				digitalWrite(GRN, HIGH);
			}
	// 7) set master boolean so we never check this segment again.
			published = true;
		}
	}

	// 8) sit and spin, checking each round for:
	//    a) a message in on /master/leds
	//    b) or a message waiting to go out on /master/button
	nh.spinOnce();
	delay(5);

}

