/** ALIGN.CPP
 *  by:  Eric M Gonzalez
 * 
 *  PURPOSE: This code is intened to hold the alignment code to insert the
 *           tool into the rig.
 */

#include <ros/ros.h>
#include <wesley/arm_point.h>

#include <iostream>


// a macro that will spin and wait for the arm to unblock.
//    nothing fancy, this just consolidates 3 or 4 lines into one.
#define wait_on_arm() {		\
	waiting = true;		\
	do { ros::spinOnce() } while waiting;

bool waiting = true;
void block_arm_wait(const wesley::arm_point& msg) {
	ROS_INFO("ALIGN :: got response from arm -- unblocking.");
	waiting = false;
}
	
int main(int argc, char* argv[]) {
	ros::init(argc, argv, "align");

	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<wesley::arm_point>("/arm/put/point", 1000);
	ros::Subscriber sub = nh.subscribe("/arm/response", 1000, &block_arm_wait);

	if (argc != 2) {
		ROS_ERROR("ALIGN --> incorrect arguments. please pass one of 1, 2, or 3 in relation to tool held.");
		return(60);
	}

	short tool = atoi(argv[1][0]);
	
	short wrist_angle[4] = {
		0,			// NONE
		90,			// SQUARE
		0,			// TRIANGLE
		90,			// CIRCLE
	};

	// wait for the publisher to settle before proceeding.
	while(pub.getNumSubscribers <= 0);
	ROS_INFO("ALIGN :: publisher ready. proceeding.");

	// position the arm up and to the right side of the vehicle
	welsey::arm_point pos;
	pos.direct_mode = true;
	pos.x = -216;
	pos.y = 0;
	pos.z = 227;
	pos.p = 0;
	pos.r = wrist_angle[tool];
	pos.cmd = "align, up";
	ROS_INFO("ALIGN --> holding tool up and out to left. (-x)");
	pub.publish(pos);
	wait_on_arm();

	// lower the arm
	pos.z = -30;
	pos.cmd = "align, down";
	ROS_INFO("ALIGN --> setting tool down to rig-hole height");
	pub.publish(pos);
	wait_on_arm();

	// stick the arm out, hopefully hitting the mark.
	pos.x -= 130;
	pos.cmd = "align, insert";
	ROS_INFO("ALIGN --> inserting tool.");
	pub.publish(pos);
	wait_on_arm();

	// leave.
	ROS_INFO("ALIGN --> tool inserted. done.");
	return(0);
}
