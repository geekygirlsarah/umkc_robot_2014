// Subcriber to write to disk - Chase Peterson// 


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <fstream>
#include <ctime>
#include <string>
using std::string;

std::ofstream fout;
using std::endl;

void loggerCallBack(const std_msgs::String::ConstPtr& msg)
{
	fout << msg->data << endl;
}



int main(int argc, char **argv)
{
	std::time_t date;
	
	struct tm * timeinfo;
	
	time(&date);
	timeinfo = localtime(&date);
	
	
	// filename has format: 
	//   DATE-TIME.log
	//   2014-03-14_1502.log
	string extension = ".log";
	string filename = asctime(timeinfo) + extension;
	fout.open (filename.c_str());
	
	ros::init(argc,argv,"speaker_for_the_dead");
	
	ros::NodeHandle nh;
	
	ros::Subscriber subber = nh.subscribe("/master/logger", 1000, loggerCallBack);
	
	ros::spin();
	
	fout.close();
	return 0;
}
