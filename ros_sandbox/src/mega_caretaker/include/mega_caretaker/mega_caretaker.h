#ifndef MEGA_CARETAKER_H 
#define MEGA_CARETAKER_H 

#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"
#include <ros/console.h>
//msg includes
#include "mega_caretaker/MegaPacket.h"



namespace mega_gatekeeper	{

		class MegaGatekeeper	{
			private:
					
					ros::NodeHandle node;
					ros::Publisher megaTalker;
					ros::Subscriber megaListener;

					void setup();
					//void heardFromMega(const mega_caretaker::MegaPacket &packet);
					void heardFromMegaSimple(const std_msgs::String &packet);
			public:
					void init(ros::NodeHandle n);
					void run();
		};


}

#endif
