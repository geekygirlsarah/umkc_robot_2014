#include <ros.h>					// basic ROS objects
#include <std_msgs/Byte.h>			// leds are condensed into one byte
#include <std_msgs/Bool.h>			// button: true / false
#include <std_msgs/Int8.h
#include <libaries/battery_checker.h>
// majority of code lives in the ROS namespace
using namespace ros;

// set pins to easy-to-read names
const byte BTN =  7;
const byte RED1 = A5;
const byte RED2 = A4;
const byte YLW1 = A3;
const byte YLW2 = A2;
const byte GRN1 = A1;
const byte GRN2 = A0;

// subscriber call back. as soon as mini has latched into ROS, this topic
//    is available to be written to. receive a 3 number message and set
//    the status of individual pins based on what the status is.
//
// WESLEY has a 6 LED status block, and so we'll need a custom message.
//    or, just a byte message and then we can flag out the bits to find
//    out which LEDs need lit.
//void display_status(const geometry_msgs::Point& status) {
void display_status(const std_msgs::Byte& msg) {
	bool status[6] = {
		msg.data & 0x01,
		msg.data & 0x02,
		msg.data & 0x04,
		msg.data & 0x08,
		msg.data & 0x10,
		msg.data & 0x20,
	};

	digitalWrite(RED1, status[0]);
	digitalWrite(RED2, status[1]);
	digitalWrite(YLW1, status[2]);
	digitalWrite(YLW2, status[3]);
	digitalWrite(GRN1, status[4]);
	digitalWrite(GRN2, status[5]);
}

// create the node handle for this arduino.
NodeHandle nh;

// keep track of running_state : this is altered by a button press.
std_msgs::Bool running_state;
std_msgs::int8 panic;
std_msgs::Byte set_leds;

Publisher                  pub("/master/button", &running_state);
Subscriber<std_msgs::Byte> sub("/master/leds",   &display_status);
Publisher panic_pub("/master/panic",1000); 
BatteryChecker batteryChecker;
void setup() {
	// status display LEDs
	pinMode(RED1, OUTPUT);
	pinMode(RED2, OUTPUT);
	pinMode(YLW1, OUTPUT);
	pinMode(YLW2, OUTPUT);
	pinMode(GRN1, OUTPUT);
	pinMode(GRN2, OUTPUT);
	set_leds.data = 0x03;
	display_status(set_leds);

	// button and pull-up resistor
	pinMode(BTN, INPUT);
	digitalWrite(BTN, HIGH);

	// through experiments, 19200 was discovered to be the slowest
	//    baud-rate that allowed this all to work. never sure why.
	nh.getHardware()->setBaud(19200);

	// initialize the node and attach the publication and subscription
	nh.initNode();
	nh.advertise(pub);
	nh.advertise(panic_pub);
	nh.subscribe(sub);

	batteryChecker.init();

	// set initial values of check.
	running_state.data = false;
}

// a check to see if we've published our message yet.
// this should be doable with only one boolean. my
//    logic is off somwhere.
//
// correctness now, completeness later.
bool published = false;
bool reset_held = false;
unsigned long reset_wait = 0;
unsigned long release_time = 0;

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
//	// 6) TEMPORARY - the LEDs would be set by another node
	 		set_leds.data = 0x30;
			display_status(set_leds);
	// 7) set master boolean so we never check this segment again.
			published = true;
		}
	} else {
		if (reset_held == false) {
			reset_held = !digitalRead(BTN);
			reset_wait = millis();
		} else {
			do {
				release_time = millis();
			} while((!digitalRead(BTN)) && (release_time - reset_wait < 4000));
			if ((!digitalRead(BTN)) && (release_time >= 4000)) {
				set_leds.data = 0x3F;
				display_status(set_leds);
				published = false;
				running_state.data = false;
	                        // wait until button is let go.
        	                while(!digitalRead(BTN));
                	        set_leds.data = 0x03;
                        	display_status(set_leds);
			} else {
				reset_held = false;
			}
			reset_held = false;
			release_time = 0;
			reset_wait = 0;
		}
	}

	// 8) sit and spin, checking each round for:
	//    a) a message in on /master/leds
	//    b) or a message waiting to go out on /master/button
	batteryChecker.update();
	panic.msg = (batteryChecker.isSafe())? 0 : 1;
	if(panic.msg == 1){
		panic_pub.publish(panic);
	}
	nh.spinOnce();
	delay(5);

}
