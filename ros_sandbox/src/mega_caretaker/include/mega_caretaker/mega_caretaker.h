#ifndef MEGA_CARETAKER_H 
#define MEGA_CARETAKER_H 

#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"

//msg includes
//#include "mega_caretaker/Mega.h"



namespace mega_gatekeeper	{

		class MegaGatekeeper	{
			private:
					
					ros::NodeHandle node;
					ros::Publisher megaTalker;

					void setup();
			public:
					void init(ros::NodeHandle n);
		};


}

#endif
