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
	
	
	imu_filter_madgwick::imu_yaw srv; 
	double init_yaw;

	if(withIMU)	{
			if(client.call(srv))	{
				ROS_INFO("Mega:: Current Yaw: %f", srv.response.yaw);
				init_yaw = srv.response.yaw;
			}
			else	{
				ROS_INFO("Mega:: unsuccessful call for yaw");
			}
	}
	else	{
		ROS_INFO("Mega:: not using IMU");
	}
	
	//tell mega to keep turning 90 degrees		
	
	ROS_INFO("board->mega:: Now turn 90 degrees");
	mega_caretaker::MegaPacket packet;
	packet.msgType = MSGTYPE_MOTORCOM;		//command 
	packet.payload = PL_TURNCW;	//turn
	megaTalker.publish(packet);
	

	if(withIMU)	{
			//keep checking until it's 90
			bool turned90 = false;
			while(!turned90)	{
				if(client.call(srv))	{
					turned90 = (fabs(init_yaw - srv.response.yaw) > 90);
				}
			}
	}
	else	{
		ROS_INFO("Mega:: not using IMU");	
	}

	//
	//once it IS 90 degrees... tell the mega to STOP ! we're done
	//
	ROS_INFO("current yaw: %f", srv.response.yaw);
	ROS_INFO("board->mega:: it's 90 deg stop!!");
	packet.msgType = MSGTYPE_MOTORCOM;		//command 
	packet.payload = PL_STOP;	//STOP RIGHT NOW
	megaTalker.publish(packet);


}

void MegaCaretaker::heardFromMega(const mega_caretaker::MegaPacket &packet)	{

	//stupid simple. if it hears HEY from the mega, it will send back an ack
	//then send back a nother stop when 90 degrees change has been reached
	
	//HEY LET"S START THE 90 degree thing! Tell them motors what to do!

	//mega wants board to help with 90 degree thing
	if(packet.msgType == MSGTYPE_HEY)	{
		if(packet.payload == PL_START_TURNING_90)	{
			ROS_INFO("mega->board:: please help turn 90 degrees");
			mega_caretaker::MegaPacket packet;
			packet.msgType = MSGTYPE_ACK;		//ack to mega
			packet.payload = PL_GENERAL_ACK;
			megaTalker.publish(packet);
			ROS_INFO("board->mega:: turning 90 degreees!");

			make90DegreeTurn();			

			packet.msgType = MSGTYPE_ACK;	//ros control finished
			packet.payload = PL_FINISHED_TURNING_90;
			megaTalker.publish(packet);
			ROS_INFO("board->mega:: sending finished turning 90");
		}
	}

	else if (packet.msgType == MSGTYPE_ACK)	{
		if(packet.payload == PL_GENERAL_ACK)	{
			ROS_INFO("mega->board:: Mega acked");
		}
		else if (packet.payload == PL_FINISHED_WAVE_CROSSING)	{
			ROS_INFO("mega->board:: Mega is done with wave crossing!");
		}
	}
	else if(packet.msgType == MSGTYPE_STATE)	{
		if(packet.payload == PL_WAITING)	{
			ROS_INFO("mega->board:: mega ready for commands");
		}
		else if(packet.payload == PL_TURNING_CW_INIT)	{
			ROS_INFO("mega->board:: State turningCw start");
		}
		else if(packet.payload == PL_LOOKING_FOR_GAP)	{
			ROS_INFO("mega->board:: Looking for gap");
		}
	}
	else if(packet.msgType == MSGTYPE_HANDSHAKE)	{
		if(packet.payload == PL_SYN_ACK)	{
			ROS_INFO("mega->board:: Received syn-ack");
			megaConnectionOK = true;
		}
	}
}



void MegaCaretaker::heardFromOrientation(const std_msgs::String &packet)	{
	ROS_INFO("Heard from the IMU!");
}


void MegaCaretaker::startWaveCrossing()	{

	//send hey to mega...
	//KEEP SENDING THIS if you don't hear back mate
	mega_caretaker::MegaPacket packet;
	packet.msgType = MSGTYPE_HEY;
	packet.payload = PL_START_WAVE_CROSSING;
	megaTalker.publish(packet);
	ROS_INFO("board->mega:: start wave crossing");

}


void MegaCaretaker::setup()	{
//	motorCommandTopic = n.subscribe(geometry_msgs/
	ROS_INFO("care:: setting up subscribers + publishers");
	megaTalker = node.advertise<mega_caretaker::MegaPacket>("boardToArduino", 10);
	megaListener = node.subscribe("arduinoToBoard", 10, &MegaCaretaker::heardFromMega, this);

	orientationListener = node.subscribe("Orientation_data", 10, &MegaCaretaker::heardFromOrientation, this);
	client = node.serviceClient<imu_filter_madgwick::imu_yaw>("getCurrentYaw");
}


void MegaCaretaker::run()	{

	//tell it to DO STUFF!! and if it doesn't hear an ack KEEP TELLING IT STUFF SD:FKLJSDL:FKJSDL:kj
	//dont do ANYTHING until tthere's a subscriber listening!!!!

	//need to wait for mega to be in command state!
startWaveCrossing();
	ros::spin();

}

void MegaCaretaker::attemptMegaConnection()	{
	ROS_INFO("Attempting to connect with the Mega...");
	mega_caretaker::MegaPacket packet;
	packet.msgType = MSGTYPE_HANDSHAKE;
	packet.payload = PL_SYN;
	megaTalker.publish(packet);

	//need to wait until you hear the syn-ack from mega
	while(!megaConnectionOK)	{
		ros::spinOnce();
	}

	packet.msgType = MSGTYPE_HANDSHAKE;
	packet.payload = PL_ACK;
	megaTalker.publish(packet);

	ROS_INFO("Connection established with mega.");//TODO TODO make this be a timeout thing so it doesn't block here
}

void MegaCaretaker::init(ros::NodeHandle n)	{
	megaConnectionOK = false;
	node = n;
	setup();
	
	//get param for using IMU default to true
	n.param("useIMU", withIMU, true);

	//certain things this node needs
	//make sure arduino is actually listening
	ros::Rate poll_rate(100);
	while(megaTalker.getNumSubscribers() == 0) 	{
		poll_rate.sleep();
	}

	//make sure the IMU service is up
	if(withIMU)	{
		ros::service::waitForService("getCurrentYaw", 5000);
	}
	else	{
		ROS_INFO("Mega:: not using IMU!");
	}


	//need to see if mega is actually responding... if not keep trying
	attemptMegaConnection();
	
}

int main(int argc, char** argv)	{
	ros::init(argc, argv, "mega_gatekeeper");
	ros::NodeHandle n;

	
	MegaCaretaker m;
	m.init(n);
	m.run();
}
