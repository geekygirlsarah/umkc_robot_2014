#include <ros/ros.h>
#include <string>
using std::string;

#ifndef LOGGER_H
#define LOGGER_H
/**
 * Class to log status to status node. 
 */
class Logger{
private:
	ros::NodeHandle nh;
	ros::Publisher pub;
	//Used to put the messages in order
	long messageNumber;
	//A randomly chosen ID for this logging session
	int ID;
	//The name of this logging unit
	string name;
public:
	Logger();
	void logStatus(string message);
};

#endif
