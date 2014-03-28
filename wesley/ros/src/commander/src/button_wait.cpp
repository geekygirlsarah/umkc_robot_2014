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
	ROS_INFO("BTN --> intializing - waiting for user input.");
	ros::NodeHandle nh;

	// start a lednotifier and set both yellow
	//    lights on to indicate wait status.
	LedNotifier ledn(&nh, 0, 0, 1, 1, 0, 0);
	string status_code_file = "/home/umkc/wesley/umkc_robot_2014_arduino/wesley/config/notify_id.txt";
	if (!ledn.parse(status_code_file.c_str())) {
		ROS_ERROR("BTN_WAIT :: parse --> unable to parse file. please check validity. non-fatal, bailing.");
		ROS_ERROR("BTN_WAIT :: parse --> given: [ %s ]", status_code_file.c_str());
		return(99);
	}

	string codes = ledn.cat_codes();
	ROS_INFO("BTN_WAIT --> known LED codes: [%s]", codes.c_str());

	ros::Subscriber sub = nh.subscribe("/master/button", 1000, &button_wait);

	do {
		ros::spinOnce();
	} while(waiting);

//	the following throw call is handled by the an exithandler.
//	ledn.throwLedCode("ready_set_go");
//	ledn.lightLeds(1, 1, 0, 0, 0, 0);

	ROS_INFO("BTN --> START ME UP!");
	return(0);
}
