#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ros/console.h>

#include <sstream>

#include "mega_caretaker/mega_caretaker.h"

//message handles
#include "mega_caretaker/MegaPacket.h"

//srv handle - getting yaw
#include <imu_filter_madgwick/imu_yaw.h>
using namespace mega_caretaker;

//Called when mega requests it.
//Will send proper command to turn to motors,
//wait until its 90 degrees from the IMU, 
//then tell the motors to stop.
//Then it will send an ack back to indicate it's done.
void MegaCaretaker::make90DegreeTurn()	{

	//this will assume the mega already stopped stuff before requesting the eturn 90 
	//get current orientation...
	
	//tell mega to keep turning 90 degrees		
	mega_caretaker::MegaPacket packet;
	packet.msgType = 3;		//command 
	packet.payload = 10;	//turn
	megaTalker.publish(packet);

	//if there is no ack, keep sending it until there is an ack?? (future ack-ing just assume that it goes through)
	//
	//once it IS 90 degrees... tell the mega to STOP ! we're done
	//
	packet.msgType = 3;		//command 
	packet.payload = 0;	//STOP RIGHT NOW
	megaTalker.publish(packet);


}

void MegaCaretaker::heardFromMega(const mega_caretaker::MegaPacket &packet)	{
	ROS_INFO("Heard from the mega!1");

	//stupid simple. if it hears HEY from the mega, it will send back an ack
	//then send back a nother stop when 90 degrees change has been reached
	
	//HEY LET"S START THE 90 degree thing! Tell them motors what to do!
	
	if(packet.msgType == 0)	{
		//tell the mega i'm taking control
			mega_caretaker::MegaPacket packet;
			packet.msgType = 1;		//ros_control started
			megaTalker.publish(packet);


		make90DegreeTurn();			

		packet.msgType = 2;	//ros control finished
		megaTalker.publish(packet);
	}
}



void MegaCaretaker::heardFromMegaSimple(const std_msgs::Int8 &packet)	{
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


void MegaCaretaker::heardFromOrientation(const std_msgs::String &packet)	{
	ROS_INFO("Heard from the IMU!");
}

void MegaCaretaker::setup()	{
//	motorCommandTopic = n.subscribe(geometry_msgs/
	megaTalker = node.advertise<std_msgs::String>("boardToArduino", 10);
	megaListener = node.subscribe("arduinoToBoard", 10, &MegaCaretaker::heardFromMegaSimple, this);

	orientationListener = node.subscribe("Orientation_data", 10, &MegaCaretaker::heardFromOrientation, this);
	client = node.serviceClient<imu_filter_madgwick::imu_yaw>("getCurrentYaw");
}

void MegaCaretaker::init(ros::NodeHandle n)	{
	node = n;
	setup();
}

int main(int argc, char** argv)	{
	ros::init(argc, argv, "mega_gatekeeper");
	ros::NodeHandle n;
	MegaCaretaker m;
	m.init(n);
	//m.run();
}
