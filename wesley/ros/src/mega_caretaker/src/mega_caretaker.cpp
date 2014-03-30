#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/console.h>
#include <string.h>
#include <math.h> //fabs

#include "mega_caretaker/mega_caretaker.h"

//message handles
#include "mega_caretaker/MegaPacket.h"

//srv handle - getting yaw
#include <imu_filter_madgwick/imu_yaw.h>

//message protocol stuff
#include "mega_caretaker/redux_mega_packet_defs.h"
#include "mega_caretaker/master_mega_packet_defs.h"
using namespace mega_caretaker;

//Called when mega requests it.
//Will send proper command to turn to motors,
//wait until its 90 degrees from the IMU, 
//then tell the motors to stop.
//Then it will send an ack back to indicate it's done.
//--> will only take care of TURNING and back. should be able to work with both turnign CW and CCW, only looking for the CHANGE
void MegaCaretaker::make90DegreeTurn(int8_t given_payload)	{

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
	
	//need to go to either 0 or 90 degrees ish
	mega_caretaker::MegaPacket packet;
	packet.msgType = MSGTYPE_MOTORCOM;		//command 
	if(given_payload == PL_START_TURNING_90_CW)	{
			ROS_INFO("board->mega:: Now turn 90 degrees CW");
			packet.payload = PL_TURNCW;	//turn
	}
	else {
			ROS_INFO("board->mega:: Now turn 90 degrees CCW");
			packet.payload = PL_TURNCCW;	//turn
	}
	megaTalker.publish(packet);
	

	if(withIMU)	{
			//keep checking until it's 90
			bool turned90 = false;
			bool x_aligned = false;
			bool y_aligned = false;
			double diff = 1;	//within 90 or 0 +- this diff
			while(!turned90)	{
				if(client.call(srv))	{
					turned90 = (fabs(init_yaw - srv.response.yaw) > 90);
//					if(given_payload == PL_START_TURNING_90_CCW)	{
//						turned90 = (srv.response.yaw > (90 - diff) && srv.response.yaw< (0 + diff));
//					}
//					else	{
//						turned90 = (srv.response.yaw > (0 - diff) && srv.response.yaw < (0 + diff));	
//					}
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


void MegaCaretaker::heardFromMaster(const mega_caretaker::MasterPacket &packet)	{
	//DEBUGGing stuff! yay!... more easy


	if(packet.msgType == MASTER_MSGTYPE_COMMAND)	{
		if(packet.payload == MASTER_PL_GO_TO_TOOLS)	{
			
			//tell mega to go to tools
			startGoToTools();
	
			//Ack to rest of board
		
			/*
			mega_caretaker::MasterPacket temp;
			temp.msgType = MASTER_MSGTYPE_STATE;
			temp.payload = MASTER_PL_GO_TO_TOOLS_ACK;
			commandTalker.publish(temp);
			*/
		}
		else if (packet.payload == MASTER_PL_CROSS_WAVES)	{
			//tell mega
			startWaveCrossing();
			
			//ACk to rest of board
			/*
			mega_caretaker::MasterPacket temp;
			temp.msgType = MASTER_MSGTYPE_ACK;
			temp.payload = MASTER_PL_CROSS_WAVES_ACK;
			commandTalker.publish(temp);
			*/
		}
	}
	
}

void MegaCaretaker::heardFromMega(const mega_caretaker::MegaPacket &packet)	{

	//stupid simple. if it hears HEY from the mega, it will send back an ack
	//then send back a nother stop when 90 degrees change has been reached
	
	//HEY LET"S START THE 90 degree thing! Tell them motors what to do!

	//mega wants board to help with 90 degree thing
//	ROS_INFO("HEARD FROM MEGA");
	bool msg_understood = false;
	if(packet.msgType == 99)	{
		ROS_INFO("HELLLLLLPPPPPP MEEEEEEEEEE");
	}
	if(packet.msgType == MSGTYPE_HEY)	{
		if(packet.payload == PL_START_TURNING_90_CW || packet.payload == PL_START_TURNING_90_CCW)	{
			ROS_INFO("mega->board:: please help turn 90 degrees");
			mega_caretaker::MegaPacket temp;
			/*
			temp.msgType = MSGTYPE_ACK;		//ack to mega
			temp.payload = PL_GENERAL_ACK;
			megaTalker.publish(temp);
			*/
			ROS_INFO("board->mega:: turning 90 degreees!");

			make90DegreeTurn(packet.payload);			

			temp.msgType = MSGTYPE_FINISHED;	//ros control finished - no need to modify payload, cw and ccw are still same
			temp.payload = packet.payload;
			megaTalker.publish(temp);
			ROS_INFO("board->mega:: sending finished turning 90");

			msg_understood = true;
		}
	}

	else if (packet.msgType == MSGTYPE_FINISHED)	{
		
		if (packet.payload == PL_FINISHED_WAVE_CROSSING)	{
			ROS_INFO("mega->board:: Mega is done with wave crossing!");
			informFinishedWaveCrossing();
			msg_understood = true;
		}
		else if (packet.payload == PL_FINISHED_GO_TO_TOOLS)	{
			ROS_INFO("mega->board:: Mega is done with go to tools!");
			informFinishedGoToTools();
			msg_understood = true;
		

			//TES:TING TESTINGSL:DKFJKL:SDFJKL:SDFJ:SDKLFJSDKL:FJSDL:FJL:
			//AS SOON AS IT IS FINISEHD GOING TO TOOLS GO TO WAVE CROSSING 
			//TODODODOTOTODOTODOTODO
			//startWaveCrossing();
		}
	}
	else if(packet.msgType == MSGTYPE_STATE)	{
		if(packet.payload == PL_WAITING)	{
			megaReady = true;	
		}
		printStateInfo(packet.payload);
		msg_understood = true;
	}
	else if(packet.msgType == MSGTYPE_HANDSHAKE)	{
		if(packet.payload == PL_SYN_ACK)	{
			ROS_INFO("mega->board:: Received syn-ack");
			megaConnectionOK = true;
			msg_understood = true;
		}
	}

	if(!msg_understood)	{
		ROS_INFO("mega->board:: MSG NOT UNDERSTOOD %d|%d", packet.msgType, packet.payload);
	}
	
}

void MegaCaretaker::printStateInfo(int8_t payload)	{
	ROS_INFO("-------------------------------------");
	ROS_INFO("MEGA STATE ChANGE!");
	switch(payload)	{
			case 	PL_WAITING:
			ROS_INFO("mega->board:: mega ready for commands");
			break;
			case	PL_LOOKING_FOR_GAP: 
			ROS_INFO("mega->board:: Looking for gap");
			break;
			case	PL_TURNING_CW_INIT :
			ROS_INFO("mega->board:: State turningCw start");
			break;
			case	PL_TURNING_CCW_INIT:
			ROS_INFO("mega->board:: State turningCCw start");
			break;
			case	PL_GAP_FOUND:
			ROS_INFO("mega->board:: Gap Found!");
			break;
			case	PL_CROSSING_GAP: 
			ROS_INFO("mega->board:: Crossing gap!");
			break;
			case	PL_GAP_CROSSED:
			ROS_INFO("mega->board:: Gap has been crossed!");
			break;
			case	PL_FINDING_EDGE:
			ROS_INFO("mega->board:: Reversing and finding edge...");
			break;
			case    PL_TRANSITION_1_2:
			ROS_INFO("mega->board:: Transitioning from finding tools to crossing waves");
			break;
	}
	ROS_INFO("-------------------------------------");
}


void MegaCaretaker::heardFromOrientation(const std_msgs::String &packet)	{
	ROS_INFO("Heard from the IMU!");
}

void MegaCaretaker::startGoToTools()	{
	mega_caretaker::MegaPacket packet;
	packet.msgType = MSGTYPE_HEY;
	packet.payload = PL_START_GO_TO_TOOLS;
	megaTalker.publish(packet);
	ROS_INFO("===================================");
	ROS_INFO("mega_caretaker:: START GO TO TOOLS!");
	ROS_INFO("===================================");
}



void MegaCaretaker::startWaveCrossing()	{

	//send hey to mega...
	//KEEP SENDING THIS if you don't hear back mate
	mega_caretaker::MegaPacket packet;
	packet.msgType = MSGTYPE_HEY;
	packet.payload = PL_START_WAVE_CROSSING;
	megaTalker.publish(packet);
	ROS_INFO("===================================");
	ROS_INFO("mega_caretaker:: START WAVE CROSSING!");
	ROS_INFO("===================================");

}

//tell teh rest of board i'm done
void MegaCaretaker::informFinishedGoToTools()	{
	mega_caretaker::MasterPacket temp;
	temp.msgType = MASTER_MSGTYPE_STATE;
	temp.payload = MASTER_PL_GO_TO_TOOLS_FIN;
	commandTalker.publish(temp);
	ROS_INFO("===================================");
	ROS_INFO("mega_caretaker:: FINISHED GO TO TOOLS");
	ROS_INFO("===================================");

}

//tell teh rest of board i'm done
void MegaCaretaker::informFinishedWaveCrossing()	{
	mega_caretaker::MasterPacket temp;
	temp.msgType = MASTER_MSGTYPE_STATE;
	temp.payload = MASTER_PL_CROSS_WAVES_FIN;
	commandTalker.publish(temp);
	ROS_INFO("===================================");
	ROS_INFO("mega_caretaker:: FINISHED WAVE CROSSING");
	ROS_INFO("===================================");


}
void MegaCaretaker::setup()	{
//	motorCommandTopic = n.subscribe(geometry_msgs/
	ROS_INFO("care:: setting up subscribers + publishers");
	megaTalker = node.advertise<mega_caretaker::MegaPacket>("/mega_caretaker/boardToArduino", 10);
	megaListener = node.subscribe("/mega_caretaker/arduinoToBoard", 10, &MegaCaretaker::heardFromMega, this);

	orientationListener = node.subscribe("Orientation_data", 10, &MegaCaretaker::heardFromOrientation, this);
	client = node.serviceClient<imu_filter_madgwick::imu_yaw>("getCurrentYaw");
	
	commandListener = node.subscribe("/mega/command", 10, &MegaCaretaker::heardFromMaster, this);
	commandTalker = node.advertise<mega_caretaker::MasterPacket>("/mega/response", 10);
}


void MegaCaretaker::run()	{

	//tell it to DO STUFF!! and if it doesn't hear an ack KEEP TELLING IT STUFF SD:FKLJSDL:FKJSDL:kj
	//dont do ANYTHING until tthere's a subscriber listening!!!!

	//need to wait for mega to be in command state!
	//startGoToTools();
	//startWaveCrossing();
	while(ros::ok())	{
		ros::spinOnce();
		ROS_INFO("spinning?");
	}
}

void MegaCaretaker::attemptMegaConnection()	{

	// looooopiness here!! must loop here! 
	// TODO TODO TODO
	ROS_INFO("Attempting to connect with the Mega...");

	mega_caretaker::MegaPacket packet;
	//need to wait until you hear the syn-ack from mega
	//need some sort of a time out here to try again some times... 
	while(!megaConnectionOK)	{
			packet.msgType = MSGTYPE_HANDSHAKE;
			packet.payload = PL_SYN;
			megaTalker.publish(packet);
			ROS_DEBUG_THROTTLE(.5, "connecting...");
			ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(.5));	//waits every X sec. 
	}

	packet.msgType = MSGTYPE_HANDSHAKE;
	packet.payload = PL_ACK;
	megaTalker.publish(packet);

	ROS_INFO("Connection established with mega.");//TODO TODO make this be a timeout thing so it doesn't block here

	//Need to wait for mega to say that it is READY to receive comands before issuing commands.
	ROS_INFO("Waiting for Mega to be ready to receive commands...");	
	megaReady = false;
	while(!megaReady)	{
		//block????	
		ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(.5));	//waits every X sec. 
	}
	ROS_INFO("Mega is ready to receive commands.");

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
