#ifndef MEGA_CARETAKER_H 
#define MEGA_CARETAKER_H 

#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <ros/console.h>
//msg includes
#include "mega_caretaker/MegaPacket.h"



namespace mega_gatekeeper	{

		class MegaGatekeeper	{
			private:
					
					ros::NodeHandle node;

					//talking to the arduino
					ros::Publisher megaTalker;		//publishes to boardToArduino
					ros::Subscriber megaListener;	//subscribes to arduinoToBoard


					//interfacing with the rest of the board
					ros::Subscriber orientationListener;	//subscribes to /Orientation_data

					void setup();
					//void heardFromMega(const mega_caretaker::MegaPacket &packet);
					void heardFromMegaSimple(const std_msgs::Int8 &packet);
					void heardFromOrientation(const std_msgs::String &packet);
			public:
					void init(ros::NodeHandle n);
					void run();
		};


}

#endif
