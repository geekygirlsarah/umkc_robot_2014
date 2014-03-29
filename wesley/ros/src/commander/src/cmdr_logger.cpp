// Subcriber to write to disk - Chase Peterson// 


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <fstream>
#include <ctime>
#include <string>
#include <sstream>
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
	
	
	string prefix = "/home/umkc/wesley/logs";
	if (argc == 2) {	// this means that someone provided 
						//    a path prefix for the log file.
		prefix = argv[1];
	}

	std::stringstream ss;
	// filename has format: 
	//   DATE-TIME.log
	//   2014-03-14_1502.log
	//
	// i don't make this up.. read 'man 3 localtime' and see how the struct is set up.
	ss << prefix << "/" <<   timeinfo->tm_year  + 1900
				 << "-" << ((timeinfo->tm_mon + 1 < 10) ? "0" : "") << timeinfo->tm_mon + 1
				 << "-" <<   timeinfo->tm_mday
				 << "_" << ((timeinfo->tm_hour    < 10) ? "0" : "") << timeinfo->tm_hour
						<< ((timeinfo->tm_min     < 10) ? "0" : "") << timeinfo->tm_min
						<< ".log";
	string filename = ss.str();
	fout.open (filename.c_str());
	
	ros::init(argc,argv,"speaker_for_the_dead");
	
	ros::NodeHandle nh;
	
	ros::Subscriber subber = nh.subscribe("/master/logger", 1000, loggerCallBack);
	
	ros::spin();
	
	fout.close();
	return 0;
}
