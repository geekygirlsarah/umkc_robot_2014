#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "notifier.h"

bool waiting = true;
void button_wait(const std_msgs::Bool &msg) {
	ROS_INFO("BTN_WAIT --> got response from mini::button. proceeding.");
	if (msg.data == true) {
		waiting = false;
	} else {
		ROS_WARN("BTN_WAIT --> got a FALSE response. not fatal, but not good.");
	}
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "button_wait");

	// start a lednotifier and set both yellow
	//    lights on to indicate wait status.
	LedNotifier ledn(0, 0, 1, 1, 0, 0);

	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/master/button", 1000, &button_wait);

	do {
		ros::spinOnce();
	} while(waiting);

	ledn.throwLedCode("ready_set_go");

	return(0);
}
