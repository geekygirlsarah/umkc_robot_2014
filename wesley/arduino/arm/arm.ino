/* arduino/arm/arm.ino
 *
 * when porting this file from compilation with 'ino' to the
 *    Arduino 1.0 IDE, the libraries MUST BE included in the
 *    sketch, not the header that uses them. this is for ease
 *    of use (sic).
 *
 */

#include <Servo.h> 				// for use by arm_control.h
#include <ros.h>
//#include <ArduinoHardware.h>
#include <wesley/arm_angle.h> 	// custom message to put arm at specific angles
#include <wesley/arm_point.h> 	// custom message for put_point
#include <std_msgs/Empty.h> 	// cheap, tiny, response message

#include "arm_control.h"

// console makes more sense when using it for debug printing.
#define console Serial
arm_control arm;

// globally needed ros objects
ros::NodeHandle nh;
wesley::arm_point res;
ros::Publisher pub("/arm/response", &res);

// callback for response for /arm/put/point
//
// arm point contains a boolean: direct_mode.
//    if this value is true, call put_point with the 5 values
//       provided by the message.
//    if direct_mode is false, compare the string 'cmd' with
//       some pre-defined values for specific commands.
void arm_put_point(const wesley::arm_point& msg){
	if (msg.direct_mode == true) {
		arm.put_point(msg.x, msg.y, msg.z, msg.p, msg.r);
	} else {
		if (strcmp(msg.cmd, "park") == 0) {
			arm.park();
		} else
		if (strcmp(msg.cmd, "carry") == 0) {
			arm.carry();
		} else
		if (strcmp(msg.cmd, "grasp") == 0) {
			arm.grasp();
		} else
		if (strcmp(msg.cmd, "release") == 0) {
			arm.release();
		}
	}
	// wait a small time to let arm settle.
	// 20 doesn't seem long enough.
	delay(20);

	// ideally, we want to ask the arm where it ended up,
	//    but this value is inconsisitent with the inverse
	//    kinematic. so instead, in order to keep things
	//    witihin one co-ordinate frame, just return the
	//    message that we received. this data is used in
	//    id_tool to find the translated point 'p' to
	//    pick the tool up at.
//	arm_control::point arm_at = arm.getxyz();
//	res.direct_mode = msg.direct_mode;
//	res.x = arm_at.x;
//	res.y = arm_at.y;
//	res.z = arm_at.z;
//	res.p = arm.read(arm.WRIST_P) - 270 + (arm.read(arm.SHOULDER) + arm.read(arm.ELBOW));
//	res.r = arm.read(arm.WRIST_R);
//	res.cmd = msg.cmd;
//	pub.publish(&res);
	pub.publish(&msg);
}
// assign the subscriber to the above call_back
ros::Subscriber<wesley::arm_point> sub_point("/arm/put/point", &arm_put_point);

// callback response to /arm/put/angle
//
// the angle message contains 6 integers. these integers are
//    directly input to the arm joints.
void arm_put_angle(const wesley::arm_angle& msg) {
//	arm.put_angle(90, 90, 90, 90, 90, 90);
	arm.put_angle(msg.base,
				  msg.shoulder,
				  msg.elbow,
				  msg.wrist_pitch,
				  msg.wrist_roll,
				  msg.hand);
	// here, asking the arm is useful. this return value will
	//    allow us to get close in the future.
	arm_control::point arm_at = arm.getxyz();
	res.direct_mode = true;
	res.x = arm_at.x;
	res.y = arm_at.y;
	res.z = arm_at.z;
	res.p = arm.read(arm.WRIST_P) - 270 + (arm.read(arm.SHOULDER) + arm.read(arm.ELBOW));
	res.r = arm.read(arm.WRIST_R);
	res.cmd = "put_angle";
	pub.publish(&res);
}
// assign another subscriber for the above callback
ros::Subscriber<wesley::arm_angle> sub_angle("/arm/put/angle", &arm_put_angle);

// basic arduino setup.
void setup() {
//	console.begin(9600);
	// alter the pins that the joints connect to here.
	// ORDER IS IMPORTANT!
	//
	// BASE, SHOULDER, ELBOW, WRIST_PITCH, WRIST_ROLL, HAND
	//
	arm.connect(NO_OF_JOINTS, 2, 3, 4, 6, 7, 8);
	// this tells the arm to park. other wise, the servos
	//    default to 90 degrees.
	arm.initial_park();

	// initialize the nodehandle, publisher and two subscribers
	nh.initNode();
	nh.advertise(pub);
	nh.subscribe(sub_point);
	// in production, it may be beneficial to remove this subscription.
	nh.subscribe(sub_angle);

}

void loop() {
	nh.spinOnce();
	// that is all. this arduino is only an interface between
	//    ROS and the arm, therefore all the magic happens in
	//    the above callbacks and the arm_control class.
}
