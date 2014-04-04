#include <ros/ros.h>
#include <std_msg/builtin_int8.h>

//Call pm-shutdown
void callback(const std_msg::int8& msg){
	FILE * f = popen("pm-shutdown","r");
	
	//Eror while opening file stream
	if(f == 0){
		return -2;
	}
	return 0;
}
int main(int argc, char* argv[]) {

	ros::init(argc, argv, "cmdr");
	ros::NodeHandle nh;
	ROS_INFO("CMDR :: main --> powerman  initializing");
	ros::Subscriber sub = nh.subscribe("/master/panic",1000,callback);
}
