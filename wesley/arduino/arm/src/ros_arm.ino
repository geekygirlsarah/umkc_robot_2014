/* this file contains ___ functions created by Eric Gonzalez to help control the arm. */
//#include <ArduinoHardware.h>
#include <ros.h>
#include <wesley/arm_angle.h>
#include <wesley/arm_point.h>

#include "arm_control.h"

#define console Serial
const byte NO_OF_JOINTS = 6;
arm_control arm(NO_OF_JOINTS);

// ROS node control
ros::NodeHandle nh;
wesley::arm_point res;
ros::Publisher pub("/arm/response", &res);

void arm_put_point(const wesley::arm_point& msg){
	if (msg.direct_mode == true) {
		arm.put_point(msg.x, msg.y, msg.z, msg.p, msg.r);
	} else {
		if (strcmp(msg.cmd, "park") == 0) {
			arm.park();
		} else
		if (strcmp(msg.cmd, "carry") == 0) {
			arm.carry();
		}
	}
	arm_control::point arm_at = arm.getxyz();
	res.direct_mode = msg.direct_mode;
	res.x = arm_at.x;
	res.y = arm_at.y;
	res.z = arm_at.z;
	res.p = arm.read(arm.WRIST_P) - 270 + (arm.read(arm.SHOULDER) + arm.read(arm.ELBOW));
	res.r = arm.read(arm.WRIST_R);
	res.cmd = msg.cmd;
	pub.publish(&res);
}
ros::Subscriber<wesley::arm_point> sub_point("/arm/put/point", &arm_put_point);

void arm_put_angle(const wesley::arm_angle& msg) {
//	arm.put_angle(90, 90, 90, 90, 90, 90);
	arm.put_angle(msg.base,
				  msg.shoulder,
				  msg.elbow,
				  msg.wrist_p,
				  msg.wrist_r,
				  msg.hand);
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
ros::Subscriber<wesley::arm_angle> sub_angle("/arm/put/angle", &arm_put_angle);

void setup() {
	arm.connect(NO_OF_JOINTS, 2, 3, 4, 6, 7, 8);
	arm.initial_park();

	// initialize subscriber - lines 20, 21
	nh.initNode();
	nh.advertise(pub);
	// one of these two topics. sub_point is preferred
	nh.subscribe(sub_point);
	// sub_angle is nice to have around, but not neccessary.
	//nh.subscribe(sub_angle);
}

void loop() {
	base = arm.read(arm.BASE);
	shoulder = arm.read(arm.SHOULDER);
	elbow = arm.read(arm.ELBOW);
	wrist_p = arm.read(arm.WRIST_P);
	wrist_r = arm.read(arm.WRIST_R);
	hand = arm.read(arm.HAND);

	nh.spinOnce();
	// when using the ROS topic, remove all references to Serial or console.
	// the ROS and arduino Serial topics clash and will not interoperate.
}
