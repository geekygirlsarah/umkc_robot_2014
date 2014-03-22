#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/console.h>

#include <math.h> //fabs

#include "mega_caretaker/mega_caretaker.h"

//message handles
#include "mega_caretaker/MegaPacket.h"

//srv handle - getting yaw
#include <imu_filter_madgwick/imu_yaw.h>

#include "mega_caretaker/redux_mega_packet_defs.h"
using namespace mega_caretaker;

//Called when mega requests it.
//Will send proper command to turn to motors,
//wait until its 90 degrees from the IMU, 
//then tell the motors to stop.
//Then it will send an ack back to indicate it's done.
//--> will only take care of TURNING and back. should be able to work with both turnign CW and CCW, only looking for the CHANGE
void MegaCaretaker::make90DegreeTurn()	{

	//this will assume the mega already stopped stuff before requesting the eturn 90 
	//get current orientation...
	
	/*	
	imu_filter_madgwick::imu_yaw srv; 
	double init_yaw;
	if(client.call(srv))	{
		ROS_INFO("Mega:: Current Yaw: %f", srv.response.yaw);
		init_yaw = srv.response.yaw;
	}
	else	{
		ROS_INFO("Mega:: unsuccessful call for yaw");
	}

	*/
	//tell mega to keep turning 90 degrees		
	mega_caretaker::MegaPacket packet;
	packet.msgType = MSGTYPE_MOTORCOM;		//command 
	packet.payload = PL_TURNCW;	//turn
	megaTalker.publish(packet);
	
	/*
	//keep checking until it's 90
	bool turned90 = false;
	while(!turned90)	{
		if(client.call(srv))	{
			turned90 = (fabs(init_yaw - srv.response.yaw) > 90);
		}
	}
*/
	//
	//once it IS 90 degrees... tell the mega to STOP ! we're done
	//
	packet.msgType = MSGTYPE_MOTORCOM;		//command 
	packet.payload = PL_STOP;	//STOP RIGHT NOW
	megaTalker.publish(packet);


}

void MegaCaretaker::heardFromMega(const mega_caretaker::MegaPacket &packet)	{
	ROS_INFO("Heard from the mega!1");

	//stupid simple. if it hears HEY from the mega, it will send back an ack
	//then send back a nother stop when 90 degrees change has been reached
	
	//HEY LET"S START THE 90 degree thing! Tell them motors what to do!

	//mega wants board to help with 90 degree thing
	if(packet.msgType == MSGTYPE_HEY)	{
		ROS_INFO(" MEGA needs help");
		if(packet.payload == PL_START_TURNING_90)	{
			ROS_INFO("care:: turning 90 degreees!");
			mega_caretaker::MegaPacket packet;
			packet.msgType = MSGTYPE_ACK;		//ack to mega
			packet.payload = PL_GENERAL_ACK;
			megaTalker.publish(packet);


			make90DegreeTurn();			

			packet.msgType = MSGTYPE_ACK;	//ros control finished
			packet.payload = PL_FINISHED_TURNING_90;
			megaTalker.publish(packet);
		}
	}

	else if (packet.msgType == MSGTYPE_ACK)	{
		if(packet.payload == PL_GENERAL_ACK)	{
			ROS_INFO("care:: Mega acked");
		}
		else if (packet.payload == PL_FINISHED_WAVE_CROSSING)	{
			ROS_INFO("care:: Mega is done with wave crossing!");
		}
	}
}



void MegaCaretaker::heardFromOrientation(const std_msgs::String &packet)	{
	ROS_INFO("Heard from the IMU!");
}


void MegaCaretaker::startWaveCrossing()	{

	//send hey to mega...
	mega_caretaker::MegaPacket packet;
	packet.msgType = MSGTYPE_ACK;
	packet.payload = PL_START_WAVE_CROSSING;
	megaTalker.publish(packet);
	ROS_INFO("care:: told mega to start wave crossing");

}


void MegaCaretaker::setup()	{
//	motorCommandTopic = n.subscribe(geometry_msgs/
	megaTalker = node.advertise<mega_caretaker::MegaPacket>("boardToArduino", 10);
	megaListener = node.subscribe("arduinoToBoard", 10, &MegaCaretaker::heardFromMega, this);

	orientationListener = node.subscribe("Orientation_data", 10, &MegaCaretaker::heardFromOrientation, this);
	client = node.serviceClient<imu_filter_madgwick::imu_yaw>("getCurrentYaw");
}


void MegaCaretaker::run()	{

	startWaveCrossing();
	ros::spin();


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
	m.run();
}
