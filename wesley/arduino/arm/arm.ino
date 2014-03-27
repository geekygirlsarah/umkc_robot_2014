#include <Servo.h>

#include <ros.h>
//#include <ArduinoHardware.h>
#include <wesley/arm_angle.h>
#include <wesley/arm_point.h>
#include <std_msgs/Empty.h>

#include "arm_control.h"

#define console Serial
arm_control arm;

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
		} else
		if (strcmp(msg.cmd, "grasp") == 0) {
			arm.grasp();
		} else
		if (strcmp(msg.cmd, "release") == 0) {
			arm.release();
		}
	}
	// wait a small time to let arm settle.
	delay(20);

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
ros::Subscriber<wesley::arm_point> sub_point("/arm/put/point", &arm_put_point);

void arm_put_angle(const wesley::arm_angle& msg) {
//	arm.put_angle(90, 90, 90, 90, 90, 90);
	arm.put_angle(msg.base,
				  msg.shoulder,
				  msg.elbow,
				  msg.wrist_pitch,
				  msg.wrist_roll,
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

void arm_park(const std_msgs::Empty& msg) {
	arm.park();
}
ros::Subscriber<std_msgs::Empty> sub_park("/arm/park", &arm_park);

void arm_carry(const std_msgs::Empty& msg) {
	arm.carry();
}
ros::Subscriber<std_msgs::Empty> sub_carry("/arm/carry", &arm_carry);


void setup() {
//	console.begin(9600);
	arm.connect(NO_OF_JOINTS, 2, 3, 4, 6, 7, 8);
	arm.initial_park();

	// initialize subscriber - lines 20, 21
	nh.initNode();
	nh.advertise(pub);
	nh.subscribe(sub_point);
	nh.subscribe(sub_angle);

}

arm_control::point point_xyz(0, 0, 0);

void loop() {
	nh.spinOnce();
}
