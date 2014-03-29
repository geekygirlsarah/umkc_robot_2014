#ifndef MEGA_CARETAKER_H 
#define MEGA_CARETAKER_H 

#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <ros/console.h>
//msg includes
#include "mega_caretaker/MegaPacket.h"
#include "mega_caretaker/MasterPacket.h"



namespace mega_caretaker	{

		class MegaCaretaker	{
			private:
					
					ros::NodeHandle node;
					bool withIMU;	//used for debugging - no imu input
					bool megaConnectionOK;	//starts as false. have i establshed comms with mega yet?

					//talking to the arduino
					ros::Publisher megaTalker;		//publishes to boardToArduino
					ros::Subscriber megaListener;	//subscribes to arduinoToBoard


					//interfacing with the rest of the board
					ros::Publisher commandTalker;	//inform the rest of board what the mega is doing
					ros::Subscriber commandListener;	//tell the mega what to do! from the outside 
					ros::Subscriber orientationListener;	//subscribes to /Orientation_data
					ros::ServiceClient client;				//client for getCurrentYaw service

					void setup();
					void attemptMegaConnection();

					//callbacks galore
					void heardFromMega(const mega_caretaker::MegaPacket &packet);
					void heardFromMaster(const mega_caretaker::MasterPacket &packet);
					void heardFromOrientation(const std_msgs::String &packet);


					//talking to rest of board
					void informFinishedWaveCrossing();
					void informFinishedGoToTools();

					void printStateInfo(int8_t payload);
					//logic functions
					void make90DegreeTurn(int8_t payload);

					//talk to  mega
					void startWaveCrossing();
					void startGoToTools();


			public:
					void init(ros::NodeHandle n);
					void run();
		};


}

#endif
