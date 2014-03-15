#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ros/console.h>

#include <sstream>

#include "mega_caretaker/mega_caretaker.h"

//message handles
#include "mega_caretaker/MegaPacket.h"

using namespace mega_gatekeeper;


void MegaGatekeeper::heardFromMegaSimple(const std_msgs::Int8 &packet)	{
	ROS_INFO("Heard from the mega!1");

	//stupid simple. if it hears HEY from the mega, it will send back an ack
	//then send back a nother stop when 90 degrees change has been reached
	if(packet.data == 1)	{
		ROS_INFO("msg1");
	}
	else if(packet.data == 2)	{
		ROS_INFO("msg2");	
	}
				

}

void MegaGatekeeper::heardFromOrientation(const std_msgs::String &packet)	{
	ROS_INFO("Heard from the IMU!");
}

void MegaGatekeeper::setup()	{
//	motorCommandTopic = n.subscribe(geometry_msgs/
	megaTalker = node.advertise<std_msgs::String>("boardToArduino", 10);
	megaListener = node.subscribe("arduinoToBoard", 10, &MegaGatekeeper::heardFromMegaSimple, this);

	orientationListener = node.subscribe("Orientation_data", 10, &MegaGatekeeper::heardFromOrientation, this);
}

void MegaGatekeeper::init(ros::NodeHandle n)	{
	node = n;
	setup();
}

int main(int argc, char** argv)	{
	ros::init(argc, argv, "mega_gatekeeper");
	ros::NodeHandle n;
	MegaGatekeeper m;
	m.init(n);
	//m.run();
}
