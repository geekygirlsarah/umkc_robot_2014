#ifndef MEGA_CARETAKER_H 
#define MEGA_CARETAKER_H 

#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <ros/console.h>
//msg includes
#include "mega_caretaker/MegaPacket.h"



namespace mega_caretaker	{

		class MegaCaretaker	{
			private:
					
					ros::NodeHandle node;
					bool withIMU;	//used for debugging - no imu input

					//talking to the arduino
					ros::Publisher megaTalker;		//publishes to boardToArduino
					ros::Subscriber megaListener;	//subscribes to arduinoToBoard


					//interfacing with the rest of the board
					ros::Subscriber orientationListener;	//subscribes to /Orientation_data
					ros::ServiceClient client;				//client for getCurrentYaw service

					void setup();

					//callbacks galore
					void heardFromMega(const mega_caretaker::MegaPacket &packet);
					void heardFromOrientation(const std_msgs::String &packet);

					//logic functions
					void make90DegreeTurn();
					void startWaveCrossing();


			public:
					void init(ros::NodeHandle n);
					void run();
		};


}

#endif
