#include <ros.h>				// basic ROS objects
#include <std_msgs/Byte.h>			// leds are condensed into one byte
#include <std_msgs/Bool.h>			// button: true / false

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
		// an array of booleans indicate which LEDs to turn on.
		// these are gathered by masking out the individual bit
		//    as condesed by LedNotifier.
		msg.data & 0x01,	// RED 1
		msg.data & 0x02,	// RED 2
		msg.data & 0x04,	// YELLOW 1
		msg.data & 0x08,	// YELLOW 2
		msg.data & 0x10,	// GREEN 1
		msg.data & 0x20,	// GREEN 2
	};

	// now that the byte has been decompressed, set the LEDs
	//    according to the value given.
	// please see notify_id.txt for status values.
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
std_msgs::Byte set_leds;

Publisher                  pub("/master/button", &running_state);
Subscriber<std_msgs::Byte> sub("/master/leds",   &display_status);

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
	nh.subscribe(sub);

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
	// this else chunk is checked after the message has been published.
	// the idea here is that we want to be able to do a software reset
	//    of the mini so that we can return to a state that will push
	//    the 'go' status when the button is pressed again.
	// the overall button check logic is similar to the above code. it
	//    has been altered sligthly to to allow for accidental and
	//    momentary pushes of the button to do nothing.
		if (reset_held == false) {
			reset_held = !digitalRead(BTN);
			// we just checked the button so keep track of that
			//    time so we can set a hold delay below.
			reset_wait = millis();
		} else {
		// the button has been pressed. while the button is still
		//    being held wait until the required time has elapsed.
			do {
				release_time = millis();
			} while((!digitalRead(BTN)) && (release_time - reset_wait < 4000));
			// a final check to make sure we're still held down and
			//    the appropriate time has been met
			if ((!digitalRead(BTN)) && (release_time >= 4000)) {
				// turn on all the lights to indicate a change
				set_leds.data = 0x3F;
				display_status(set_leds);
				// reset all internal state check booleans
				published = false;
				running_state.data = false;
	                        // wait until button is let go.
        	                while(!digitalRead(BTN));
        	                // return the LEDs to the inital state (RED 1 & 2 on)
                	        set_leds.data = 0x03;
                        	display_status(set_leds);
			} else {
				// didn't meet the requirements for reset.
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
	nh.spinOnce();
	delay(5);

}
