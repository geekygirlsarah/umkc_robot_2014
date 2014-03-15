#include "logger.h"
#include <cstdlib>
#include <iostream>
using std::rand;
using std::ostream;
Logger::Logger(){
	ros::init(0,NULL,"logger");
	pub = nh.advertise<std_msgs::string>("/master/logger",1000);
	ID = rand();
	messageNumber = 0;
	name = "Commander";
}

logStatus(string message){
	std_msgs::string msg;
	string outString = "";
	outstring += name + " ";
	//cast int to string for ID
	outString += static_cast<ostringstream*>( &(ostringstream() << ID) )->str(); 
	outString += " ";
	//Cast int to string for messageNumber
	outString += static_cast<ostringstream*>( &(ostringstream() << messageNumber) )->str(); 
	outstring += message;

	msg.data = outString;

	pub.publish(msg);

	ros::spinOnce();
}
