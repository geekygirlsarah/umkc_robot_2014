#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include "mega_caretaker/mega_caretaker.h"

//message handles
#include "mega_caretaker/MegaPacket.h"

using namespace mega_gatekeeper;


void MegaGatekeeper::setup()	{
//	motorCommandTopic = n.subscribe(geometry_msgs/
//	megaTalker = node.advertise<MegaPacket>("boardToArduino", 10);
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
